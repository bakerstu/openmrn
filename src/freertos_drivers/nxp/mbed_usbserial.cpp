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
 * library. Currently it is tested on the LPC23xx processors.
 *
 * @author Balazs Racz
 * @date 4 May 2013
 */

#include "mbed.h"
#include "USBSerial.h"
#include "Serial.hxx"
#include "os/os.h"
#include "utils/macros.h"
#include "portmacro.h"

#ifdef TARGET_LPC2368
#endif

#define TX_DATA_SIZE 64
#define RX_DATA_SIZE 64

#include <stdlib.h>

void* operator new(size_t size);
void* operator new[](size_t size);
void operator delete(void* ptr);
void operator delete[](void* ptr);

__extension__ typedef int __guard __attribute__((mode(__DI__)));

extern "C" int __cxa_guard_acquire(__guard*);
extern "C" void __cxa_guard_release(__guard*);
extern "C" void __cxa_guard_abort(__guard*);

extern "C" void __cxa_pure_virtual(void);

extern DigitalOut d2;

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
    virtual bool EP2_OUT_callback()
    {
        //HASSERT(IsEpPending());
        // and wake up the RX thread.
        os_sem_post_from_isr(&rxSem);
        return false;
    }

    virtual bool EP2_IN_callback()
    {
        int count;
        int woken = 0;
        configASSERT(txPending);
        for (count = 0; count < MAX_TX_PACKET_LENGTH; count++)
        {
            if (os_mq_receive_from_isr(txQ, &txData[count],
                                       &woken) != OS_MQ_NONE)
            {
                /* no more data left to transmit */
                break;
            }
        }
        TxHelper(count);
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
    void enable()
    {
    } /**< function to enable device */
    void disable()
    {
    } /**< function to disable device */

    /** function to try and transmit a character */
    void tx_char()
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
        int count;
        for (count = 0; count < TX_DATA_SIZE; count++)
        {
            if (os_mq_timedreceive(txQ, txData + count, 0) !=
                OS_MQ_NONE)
            {
                /* no more data left to transmit */
                break;
            }
        }
        TxHelper(count);
        taskEXIT_CRITICAL();
    }

    static const int MAX_TX_PACKET_LENGTH = 64;
    static const int MAX_RX_PACKET_LENGTH = 64;

    /** Transmits count bytes from the txData buffer. Sets txPending and
        bytesLost as needed. */
    void TxHelper(int count)
    {
        if (!count)
        {
            txPending = false;
            return;
        }
#ifdef TARGET_LPC1768
        d2 = !d2;
#endif
        if ((!configured()) || (!terminal_connected))
        {
            // An error occured, data was lost.
            txPending = false;
            overrunCount += count;
            return;
        }
        txPending = true;
        sendNB(txData, count);
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
                os_mq_send(rxQ, rxData + i);
            }
            rxSize = 0;
            // We reactivate the endpoint to receive next characters
            // readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);
        }
    }

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
    os_sem_t rxSem;
};

void* operator new(size_t size)
{
    return malloc(size);
}

void* operator new [](size_t size)
{ return malloc(size); } void
operator delete(void* ptr)
{
    free(ptr);
}

void operator delete [](void* ptr)
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


MbedRawUSBSerial g_mbed_usb_serial("/dev/serUSB0");
