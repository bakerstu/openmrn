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
    if (txBuf)
    {
        txBuf->flush();
    }
    if (rxBuf)
    {
        rxBuf->flush();
    }
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno
 * containing the cause
 */
ssize_t Serial::read(File *file, void *buf, size_t count)
{
    unsigned char *data = (unsigned char *)buf;
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

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno
 * containing the cause
 */
ssize_t Serial::write(File *file, const void *buf, size_t count)
{
    const unsigned char *data = (const unsigned char *)buf;
    ssize_t result = 0;

    while (count)
    {
        portENTER_CRITICAL();
        /* We limit the amount of bytes we write with each iteration in order
         * to limit the amount of time that interrupts are disabled and
         * preserve our real-time performance.
         */
        size_t bytes_written = txBuf->put(data, count < 64 ? count : 64);

        if (bytes_written == 0)
        {
            portEXIT_CRITICAL();
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
        else
        {
            tx_char();
            portEXIT_CRITICAL();
            count -= bytes_written;
            result += bytes_written;
            data += bytes_written;
        }
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
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
bool Serial::select(File *file, int mode)
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
