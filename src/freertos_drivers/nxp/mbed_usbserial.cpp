/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file mbed_usbserial.cpp
 * This file implements an USB-Serial driver on top of the mbed
 * library. Currently it is tested on the LPC23xx and LPC17xx processors.
 *
 * @author Balazs Racz
 * @date 4 May 2013
 */

#include <algorithm>

#include "mbed.h"
#include "USBSerial.h"
#include "Serial.hxx"
#include "os/os.h"
#include "utils/macros.h"
#include "portmacro.h"

#ifdef TARGET_LPC2368
#endif

/// Number of bytes in the send buffer.
#define TX_DATA_SIZE 64
/// Number of bytes in the receive buffer.
#define RX_DATA_SIZE 64

#include <stdlib.h>

/// C++ operator (not sure why this is needed).
void* operator new(size_t size);
/// C++ operator (not sure why this is needed).
void* operator new[](size_t size);
/// C++ operator (not sure why this is needed).
void operator delete(void* ptr);
/// C++ operator (not sure why this is needed).
void operator delete[](void* ptr);

/// not sure why this is needed.
__extension__ typedef int __guard __attribute__((mode(__DI__)));

/// C++ operator (not sure why this is needed).
extern "C" int __cxa_guard_acquire(__guard*);
/// C++ operator (not sure why this is needed).
extern "C" void __cxa_guard_release(__guard*);
/// C++ operator (not sure why this is needed).
extern "C" void __cxa_guard_abort(__guard*);
/// C++ operator (not sure why this is needed).
extern "C" void __cxa_pure_virtual(void);

/** This class is an empty wrapper around MBed's USB CDC class. The difference
    between this and mbed::USBSerial is that this class does not have any
    buffering and no interaction with stdio, whereas mbed::USBSerial has the
    following buffering:
    * it has a 128-byte receive buffer.
    * it has an fd
    * it has a FILE* with a default-sized send/receive buffer
    * it requires mbed's custom glue code in the open(2) method, without which
      it crashes.
 */
class MbedRawUSBSerial : public USBCDC, public ::Serial
{
public:
    /// Constructor.
    /// @param name device path to export (e.g. /dev/usbser0).
    /// @param vendor_id USB vendor ID to report
    /// @param product_id USB device ID to report
    /// @param product_release USB product version to report.
    MbedRawUSBSerial(const char* name, uint16_t vendor_id = 0x1f00,
                     uint16_t product_id = 0x2012,
                     uint16_t product_release = 0x0001)
        : USBCDC(vendor_id, product_id, product_release),
          Serial(name),
          txPending(false)
    {
        os_sem_init(&rxSem, 0);
        os_thread_t thread;
        os_thread_create(&thread, "usbserial.rx", 3, 1024, &_RxThread, this);
    }

    ~MbedRawUSBSerial()
    {
        os_sem_destroy(&rxSem);
    }

protected:
    /// Callback when EP2_OUT is active
    bool EP2_OUT_callback() override
    {
        //HASSERT(IsEpPending());
        // and wake up the RX thread.
        int woken;
        os_sem_post_from_isr(&rxSem, &woken);
        return false;
    }

    /// Callback when EP2_IN is active
    bool EP2_IN_callback() override
    {
        configASSERT(txPending);
        int woken = 0;
        if (TxHelper()) {
          txBuf->signal_condition_from_isr();
        }
        if (woken)
        {
#ifdef TARGET_LPC1768
            portYIELD();
#elif defined(TARGET_LPC2368)
/** @todo(balazs.racz): need to find a way to yield on ARM7. The builtin
 * portYIELD_FROM_ISR assumes that we have entered the ISR with context
 * saving, which we didn't. */
#else
#error define how to yield on your CPU.
#endif
        }
        return true;
    }

private:
    void enable() override
    {
    } /**< function to enable device */
    void disable() override
    {
    } /**< function to disable device */

    /** function to try and transmit a character */
    void tx_char() override
    {
        // Without this critical section there were cases when we deadlocked
        // with txPending == true but no interrupt coming in to clear it.
        taskENTER_CRITICAL();
        if (txPending)
        {
            taskEXIT_CRITICAL();
            return;
        }
        txPending = true;
        if (TxHelper()) {
            txBuf->signal_condition();
        }
        taskEXIT_CRITICAL();
    }

    /// What's the maximum packet length for transmit.
    static const unsigned MAX_TX_PACKET_LENGTH = 64;
    /// What's the maximum packet length for receive.
    static const unsigned MAX_RX_PACKET_LENGTH = 64;

    /** Transmits count bytes from the txData buffer. Sets txPending and
        bytesLost as needed. @return true if we consumed some data from the tx
        buffer. */
    bool TxHelper()
    {
        size_t count;
        bool signal = false;
        for (count = 0; count < MAX_TX_PACKET_LENGTH;)
        {
            uint8_t *data;
            size_t max_count = std::min(txBuf->data_read_pointer(&data),
                                        MAX_TX_PACKET_LENGTH - count);
            if (max_count)
            {
                memcpy(&txData[count], data, max_count);
                count += max_count;
                txBuf->consume(max_count);
                signal = true;
            }
            else
            {
                break;
            }
        }
        if (!count)
        {
            txPending = false;
            return signal;
        }
        if (!configured())
        {
            // An error occured, data was lost.
            txPending = false;
            overrunCount += count;
            return signal;
        }
        txPending = true;
        sendNB(txData, count);
        return signal;
    }

    void RxThread()
    {
        while (1)
        {
            os_sem_wait(&rxSem);
            portENTER_CRITICAL();
            // we read the packet received to our assembly buffer
            bool result = readEP_NB(rxData, &rxSize);
            portEXIT_CRITICAL();
            if (!result)
            {
                diewith(0x80000CCC);
            }
            for (uint32_t i = 0; i < rxSize; i++)
            {
              /// @todo (balazs.racz) this needs to be replaced with something
              /// that works in the new select based model.
              //os_mq_send(rxQ, rxData + i);
            }
            rxSize = 0;
            // We reactivate the endpoint to receive next characters
            // readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);
        }
    }

    /// Entry point to receiving thread. @param arg is the UsbServial object to
    /// run upon. @return null.
    static void* _RxThread(void* arg)
    {
        ((MbedRawUSBSerial*)arg)->RxThread();
        return NULL;
    }

    uint8_t txData[MAX_TX_PACKET_LENGTH]; /**< packet assemby buffer to host */
    uint32_t rxSize; /**< number of valid characters in rxData */
    uint8_t rxData
        [MAX_RX_PACKET_LENGTH]; /**< packet assembly buffer from host */
    bool txPending;             /**< transmission currently pending */
    /// Semaphore to wake up receiving thread.
    os_sem_t rxSem;
};

void* operator new(size_t size)
{
    return malloc(size);
}

void* operator new [](size_t size)
{ return malloc(size); } void
operator delete(void* ptr) noexcept(true)
{
    free(ptr);
}

void operator delete [](void* ptr) noexcept(true)
{ free(ptr); } int __cxa_guard_acquire(__guard* g)
{
    return !*(char*)(g);
};
void __cxa_guard_release(__guard* g)
{
    *(char*)g = 1;
};
void __cxa_guard_abort(__guard*) {};

void __cxa_pure_virtual(void) {};


/// Global object for the mbed usb serial.
MbedRawUSBSerial g_mbed_usb_serial("/dev/serUSB0");
