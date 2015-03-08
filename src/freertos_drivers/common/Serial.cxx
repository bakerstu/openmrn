/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file Serial.cxx
 * This file implements a generic serial device driver layer.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#include <cstdint>
#include <fcntl.h>
#include "Devtab.hxx"
#include "Serial.hxx"
#include "can_ioctl.h"

/** Flush the receive and transmit buffers for this device.
 */
void Serial::flush_buffers()
{
    txBuf->flush();
    rxBuf->flush();
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t Serial::read(File *file, void *buf, size_t count)
{
    unsigned char *data = (unsigned char*)buf;
    ssize_t result = 0;
    
    while (count)
    {
        portENTER_CRITICAL();
        /* We limit the amount of bytes we read with each iteration in order
         * to limit the amount of time that interrupts are disabled and
         * preserve our real-time performance.
         */
        size_t bytes_read = rxBuf->get(data, count < 64 ? count : 64);
        portEXIT_CRITICAL();

        if (bytes_read == 0)
        {
            /* no more data to receive */
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                /* wait for data to come in, this call will release the
                 * critical section lock.
                 */
                rxBuf->block_until_condition(file, true);
            }
        }

        count -= bytes_read;
        result += bytes_read;
        data += bytes_read;
    }

    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t Serial::write(File *file, const void *buf, size_t count)
{
    const unsigned char *data = (const unsigned char*)buf;
    ssize_t result = 0;
    
    while (count)
    {
        portENTER_CRITICAL();
        /* We limit the amount of bytes we write with each iteration in order
         * to limit the amount of time that interrupts are disabled and
         * preserve our real-time performance.
         */
        size_t bytes_written = txBuf->put(data, count < 64 ? count : 64);
        portEXIT_CRITICAL();

        if (bytes_written == 0)
        {
            /* no more data to receive */
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                /* wait for space to be available, this call will release the
                 * critical section lock.
                 */
                txBuf->block_until_condition(file, false);
            }
        }
        tx_char();

        count -= bytes_written;
        result += bytes_written;
        data += bytes_written;
    }
    
    return result;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int Serial::ioctl(File *file, unsigned long int key, unsigned long data)
{
    return -1;
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool Serial::select(File* file, int mode)
{
    bool retval = false;
    switch (mode)
    {
        case FREAD:
            portENTER_CRITICAL();
            if (rxBuf->pending() > 0)
            {
                retval = true;
            }
            else
            {
                rxBuf->select_insert();
            }
            portEXIT_CRITICAL();
            break;
        case FWRITE:
            portENTER_CRITICAL();
            if (txBuf->space() > 0)
            {
                retval = true;
            }
            else
            {
                txBuf->select_insert();
            }
            portEXIT_CRITICAL();
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }
    return retval;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int USBSerial::ioctl(File *file, unsigned long int key, unsigned long data)
{
    return -1;
}

/** Flush the receive and transmit buffers for this device.
 */
void USBSerial::flush_buffers()
{
}

ssize_t USBSerialNode::read(File *file, void *buf, size_t count)
{
    if (!count) return 0;
    auto h = rxBlock_.holder();
    ssize_t ret = 0;
    uint8_t *dst = static_cast<uint8_t *>(buf);
    while (true)
    {
        bool pass_on_notify = false;
        // Checks if we have some RX job to do.
        {
            auto hh = h.critical();
            if ((rxQBegin_ >= rxQEnd_) && (rxPending_))
            {
                rxQBegin_ = 0;
                rxQEnd_ = rx_packet_irqlocked(rxQ_);
                rxPending_ = 0;
            }
            // Tries to copy data from the buffer.
            while (rxQBegin_ < rxQEnd_ && ret < (ssize_t)count)
            {
                *dst++ = rxQ_[rxQBegin_++];
                ++ret;
            }
            if (rxQBegin_ < rxQEnd_)
            {
                pass_on_notify = true;
            }
        }
        if (pass_on_notify)
        {
            // We have a partial read. Wake up someone else trying to read.
            h.notify_next();
            // We lost the lock at notify_next, so we return immediately.
            return ret;
        }
        if (ret > 0 || file->flags & O_NONBLOCK)
        {
            return ret;
        }
        // No data received now, and no data in the queue. Blocks the current
        // thread and tries the whole dance again.
        h.wait_for_notification();
    }
}


ssize_t USBSerialNode::write(File *file, const void *buf, size_t count)
{
    if (!count) return 0;
    const uint8_t *data = static_cast<const uint8_t*>(buf);
    ssize_t ret = 0;

    auto h = txBlock_.holder();
    bool pass_on_notify = false;
    while (count)
    {
        {
            auto hh = h.critical();
            while (has_tx_buffer_free() && count)
            {
                add_to_tx_buffer(*data++);
                --count;
                ++ret;
            }
            if (has_tx_buffer_free()) pass_on_notify = true;
            if (!txPending_)
            {
                if (tx_packet_irqlocked(txQ_, txQEnd_))
                {
                    mark_tx_buffer_sent();
                    txPending_ = 1;
                } else {
                    // packet send failed -- we'll re-try in a next iteration
                    pass_on_notify = true;
                }
            }
        }
        if (file->flags & O_NONBLOCK) {
            break;
        }
        if (count && !pass_on_notify)
        {
            h.wait_for_notification();
        }
    }
    if (pass_on_notify)
    {
        h.notify_next();
        // must not use locked fields after this
        return ret;
    }
    return ret;
}

int USBSerialNode::ioctl(File *file, unsigned long int key, unsigned long data)
{
    /* sanity check to be sure we have a valid key for this device */
    HASSERT(IOC_TYPE(key) == CAN_IOC_MAGIC);

    // Will be called at the end if non-null.
    Notifiable *n = nullptr;

    if (IOC_SIZE(key) == NOTIFIABLE_TYPE)
    {
        n = reinterpret_cast<Notifiable *>(data);
    }

    switch (key)
    {
        default:
            return -EINVAL;
        case CAN_IOC_READ_ACTIVE:
        {
            auto h = rxBlock_.holder();
            auto hh = h.critical();
            if (rxQBegin_ >= rxQEnd_) {
                n = rxBlock_.register_notifiable(n);
                log_.log(0xC8);
            } else {
                log_.log(0xC8);
            }
            break;
        }
        case CAN_IOC_WRITE_ACTIVE:
        {
            auto h = txBlock_.holder();
            auto hh = h.critical();
            if (!has_tx_buffer_free()) {
                n = txBlock_.register_notifiable(n);
                log_.log(0xCA);
            } else {
                log_.log(0xCB);
            }
            break;
        }
    }
    if (n)
        n->notify();
    return 0;
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool USBSerialNode::select(File* file, int mode)
{
#if 0
    switch (mode)
    {
        case FREAD:
            if (os_mq_num_pending(rxQ) > 0)
            {
                return true;
            }
            select_insert(&selInfoRd);
            break;
        case FWRITE:
            if (has_tx_buffer_free())
            {
                return true;
            }
            select_insert(&selInfoWr);
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }
#endif
    return false;
}

void USBSerialNode::flush_buffers() OVERRIDE {
    {
        auto h = txBlock_.holder();
        // Since this was called after disable() we don't need the critical
        // section anymore.
        txQEnd_ = 0;
        txPending_ = 0; // is this dangerous?
    }
    {
        auto h = rxBlock_.holder();
        rxQBegin_ = rxQEnd_ = 0;
        if (rxPending_) {
            rx_packet_irqlocked(rxQ_);
            rxPending_ = 0;
        }
    }
}

void USBSerialNode::tx_finished_from_isr()
{
    if (txQEnd_) {
        if (tx_packet_from_isr(txQ_, txQEnd_))
        {
            txQEnd_ = 0;
            txPending_ = 1;
        }
        else
        {
            // We failed ot send the next packet. We'll wake up a writer
            // (hopefully) that will re-try sending the packet. However, it is
            // possible that there is no writer thread around and there will be
            // data stuck in the queue here.
            txPending_ = 0;
        }
    }
    else
    {
        txPending_ = 0;
        log_.log(0x20);
    }
    txBlock_.notify_from_isr();
}

void *USBSerialNode::try_read_packet_from_isr()
{
    if (rxQBegin_ >= rxQEnd_)
    {
        rxQBegin_ = rxQEnd_ = 0;
    }
    if (static_cast<size_t>(rxQEnd_) + USB_SERIAL_PACKET_SIZE <= sizeof(rxQ_))
    {
        return &rxQ_[rxQEnd_];
    }
    else
    {
        return nullptr;
    }
}

void USBSerialNode::set_rx_finished_from_isr(uint8_t size)
{
    rxQEnd_ += size;
    rxBlock_.notify_from_isr();
}
