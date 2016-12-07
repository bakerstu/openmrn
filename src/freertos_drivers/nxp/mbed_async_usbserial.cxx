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
 * \file mbed_async_usbserial.cxx
 * This file implements an USB-Serial driver on top of the mbed
 * library. Currently it is tested on the LPC23xx and LPC17xx processors.
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
#include "utils/Atomic.hxx"
#include "executor/Notifiable.hxx"

#ifdef TARGET_LPC2368
#endif

/// Number of bytes in the send buffer.
#define TX_DATA_SIZE 64
/// Number of bytes in the receive buffer.
#define RX_DATA_SIZE 64

#include <stdlib.h>

/// C++ operator (not sure why this is needed).
void *operator new(size_t size);
/// C++ operator (not sure why this is needed).
void *operator new[](size_t size);
/// C++ operator (not sure why this is needed).
void operator delete(void *ptr);
/// C++ operator (not sure why this is needed).
void operator delete[](void *ptr);

/// not sure why this is needed.
__extension__ typedef int __guard __attribute__((mode(__DI__)));

/// C++ operator (not sure why this is needed).
extern "C" int __cxa_guard_acquire(__guard *);
/// C++ operator (not sure why this is needed).
extern "C" void __cxa_guard_release(__guard *);
/// C++ operator (not sure why this is needed).
extern "C" void __cxa_guard_abort(__guard *);
/// C++ operator (not sure why this is needed).
extern "C" void __cxa_pure_virtual(void);

/// Mutex for the USB-serial object.
static Atomic critical_lock;

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
class MbedAsyncUSBSerial : public USBCDC, public ::NonBlockNode
{
public:
    /// Constructor.
    ///
    /// @param name device path to export (e.g. /dev/usbser0).
    /// @param vendor_id USB vendor ID to report
    /// @param product_id USB device ID to report
    /// @param product_release USB product version to report.
    MbedAsyncUSBSerial(const char *name, uint16_t vendor_id = 0x1f00,
        uint16_t product_id = 0x2012, uint16_t product_release = 0x0001)
        : USBCDC(vendor_id, product_id, product_release)
        , NonBlockNode(name)
        , overrunCount_(0)
        , txCount_(0)
        , rxBegin_(0)
        , rxEnd_(0)
        , txPending_(false)
        , rxPending_(false)
    {
    }

    ~MbedAsyncUSBSerial()
    {
    }

protected:
    /// Callback when EP2_OUT is ready.
    bool EP2_OUT_callback() override
    {
        // HASSERT(IsEpPending());
        // and wake up the RX thread.
        rxPending_ = 1;
        if (readableNotify_)
        {
            readableNotify_->notify_from_isr();
            readableNotify_ = nullptr;
        }
        return false;
    }

    /// Callback when EP2_IN is ready.
    bool EP2_IN_callback() override
    {
        configASSERT(txPending_);
        TxHelper();
        if (writableNotify_)
        {
            writableNotify_->notify_from_isr();
            writableNotify_ = nullptr;
        }
        return true;
    }

private:
    void enable() OVERRIDE
    {
    }
    void disable() OVERRIDE
    {
    }
    void flush_buffers() OVERRIDE
    {
    }

    bool has_rx_buffer_data() OVERRIDE
    {
        return (rxPending_ || rxBegin_ < rxEnd_);
    }

    bool has_tx_buffer_space() OVERRIDE
    {
        return (!txPending_ || txCount_ < sizeof(txData_));
    }

    /** Read from a file or device.
    * @param file file reference for this device
    * @param buf location to place read data
    * @param count number of bytes to read
    * @return number of bytes read upon success, -1 upon failure with errno
    * containing the cause
    */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE
    {
        uint8_t *rbuf = static_cast<uint8_t *>(buf);
        ssize_t result = 0;
        while (true)
        {
            {
                OSMutexLock l(&lock_);
                while (result < (int)count && rxBegin_ < rxEnd_)
                {
                    *rbuf++ = rxData_[rxBegin_++];
                    ++result;
                }
                if (rxBegin_ >= rxEnd_ && rxPending_)
                {
                    bool result = true;
                    {
                        AtomicHolder h(&critical_lock);
                        if (!rxPending_)
                        {
                            continue;
                        }
                        uint32_t rxSize = 0;
                        // we read the packet received to our assembly buffer
                        result = readEP_NB(rxData_, &rxSize);
                        rxPending_ = 0;
                        HASSERT(rxSize >= 0 && rxSize <= 255);
                        rxEnd_ = rxSize;
                        rxBegin_ = 0;
                    }
                    if (!result)
                    {
                        diewith(0x80000CCC);
                    }
                    continue;
                }
                if (result > 0)
                {
                    return result;
                }
            } // lock
            // Now: we have no data to give back.
            if (file->flags & O_NONBLOCK)
            {
                return result;
            }
            Notifiable *n = nullptr;
            {
                AtomicHolder h(&critical_lock);
                if (rxPending_ || rxBegin_ < rxEnd_)
                {
                    continue;
                }
                // Will be called at the end if non-null.
                n = readableNotify_;
                if (n == &readSync_)
                {
                    DIE("This serial driver does not support having multiple "
                        "threads execute blocking reads concurrently.");
                }
                readableNotify_ = &readSync_;
            }
            readSync_.wait_for_notification();
            /** If we got a notification, we automatically notify the others in
             * line. This handles the case if there are multiple threads
             * blocked on the same input. */
            if (n)
            {
                n->notify();
            }
        }
    }

    /** Write to a file or device.
    * @param file file reference for this device
    * @param buf location to find write data
    * @param count number of bytes to write
    * @return number of bytes written upon success, -1 upon failure with errno
    * containing the cause
    */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE
    {
        const uint8_t *wbuf = static_cast<const uint8_t *>(buf);
        ssize_t result = 0;
        while (true)
        {
            {
                OSMutexLock l(&lock_);
                {
                    AtomicHolder h(&critical_lock);
                    while (result < (int)count && txCount_ < sizeof(txData_))
                    {
                        txData_[txCount_++] = *wbuf++;
                        ++result;
                    }
                    if (txCount_ > 0 && !txPending_)
                    {
                        TxHelper();
                        continue;
                    }
                } // end atomic
            }     // end mutex
            // Now: we have either result == count or tx buffer full and still
            // having data to write.
            if (result > 0)
            {
                return result;
            }
            // Now: we couldn't yet write anything.
            if (file->flags & O_NONBLOCK)
            {
                return result;
            }
            // Now: let's wait for some space in the buffer to be available.
            Notifiable *n = nullptr;
            {
                AtomicHolder h(&critical_lock);
                if (txCount_ < sizeof(txData_) || !txPending_)
                {
                    continue;
                }
                Notifiable *n = writableNotify_;
                if (n == &writeSync_)
                {
                    DIE("This serial driver does not support having multiple "
                        "threads execute blocking writes concurrently.");
                }
                writableNotify_ = &writeSync_;
            }
            writeSync_.wait_for_notification();
            /** If we got a notification, we automatically notify the others in
             * line. This handles the case if there are multiple threads
             * blocked on the same input. */
            if (n)
            {
                n->notify();
            }
        } // while trying to write
    }

    /// What's the maximum packet length for transmit.
    static const int MAX_TX_PACKET_LENGTH = 64;
    /// What's the maximum packet length for receive.
    static const int MAX_RX_PACKET_LENGTH = 64;

    /** Transmits txCount_ bytes from the txData_ buffer. Sets txPending and
        bytesLost as needed. Must be called from a critical section or ISR,
        when the previous pending transmit operation has finished. */
    void TxHelper()
    {
        if (!txCount_)
        {
            txPending_ = false;
            return;
        }
        if (!configured())
        {
            // An error occured, data was lost.
            txPending_ = false;
            overrunCount_ += txCount_;
            return;
        }
        txPending_ = true;
        sendNB(txData_, txCount_);
        txCount_ = 0;
    }

    /// How many times have we had to drop data to the floor due to buffer
    /// overrun.
    unsigned overrunCount_;
    /** packet assemby buffer from app to device */
    uint8_t txData_[MAX_TX_PACKET_LENGTH];
    /** packet assemby buffer from device to app */
    uint8_t rxData_[MAX_RX_PACKET_LENGTH];
    /** number of valid characters in txData */
    uint8_t txCount_;
    /** First valid character in rxData */
    uint8_t rxBegin_;
    /** 1 + Last valid character in rxData. */
    uint8_t rxEnd_;
    uint8_t txPending_ : 1; /**< transmission currently pending */
    uint8_t rxPending_ : 1; /**< there is a packet in the USB block waiting */
    /// Helper object to block reader.
    SyncNotifiable readSync_;
    /// Helper object to block writer.
    SyncNotifiable writeSync_;
};

void *operator new(size_t size)
{
    return malloc(size);
}

void *operator new [](size_t size)
{ return malloc(size); } void
operator delete(void *ptr) noexcept(true)
{
    free(ptr);
}

void operator delete [](void *ptr) noexcept(true)
{ free(ptr); } int __cxa_guard_acquire(__guard *g)
{
    return !*(char *)(g);
};
void __cxa_guard_release(__guard *g)
{
    *(char *)g = 1;
};
void __cxa_guard_abort(__guard *) {};

void __cxa_pure_virtual(void)
{
    configASSERT(0);
};

/// Global object for the mbed usb serial.
MbedAsyncUSBSerial g_mbed_usb_serial("/dev/serUSB0");
