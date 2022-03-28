/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file Can.cxx
 * This file implements a generic can device driver layer.
 *
 * @author Stuart W. Baker
 * @date 28 December 2012
 */

#include <cstdint>
#include <algorithm>
#include <fcntl.h>
#include "Devtab.hxx"
#include "Can.hxx"
#include "can_frame.h"

unsigned Can::numReceivedPackets_{0};
unsigned Can::numTransmittedPackets_{0};

/** Flush the receive and transmit buffers for this device.
 */
void Can::flush_buffers()
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
ssize_t Can::read(File *file, void *buf, size_t count)
{
    HASSERT((count % sizeof(struct can_frame)) == 0);

    struct can_frame *data = (struct can_frame*)buf;
    ssize_t result = 0;
    
    count /= sizeof(struct can_frame);

    while (count)
    {
        portENTER_CRITICAL();
        /* We limit the amount of bytes we read with each iteration in order
         * to limit the amount of time that interrupts are disabled and
         * preserve our real-time performance.
         */
        size_t frames_read = rxBuf->get(data, count < 8 ? count : 8);
        portEXIT_CRITICAL();

        if (frames_read == 0)
        {
            /* no more data to receive */
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                /* wait for data to come in */
                rxBuf->block_until_condition(file, true);
            }
        }

        count -= frames_read;
        result += frames_read;
        data += frames_read;
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result * sizeof(struct can_frame);
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t Can::write(File *file, const void *buf, size_t count)
{
    HASSERT((count % sizeof(struct can_frame)) == 0);

    const struct can_frame *data = (const struct can_frame*)buf;
    ssize_t result = 0;

    count /= sizeof(struct can_frame);

    while (count)
    {
        portENTER_CRITICAL();
        /* We limit the amount of bytes we write with each iteration in order
         * to limit the amount of time that interrupts are disabled and
         * preserve our real-time performance.
         */
        size_t frames_written = txBuf->put(data, count < 8 ? count : 8);

        if (frames_written == 0)
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
            tx_msg();
            portEXIT_CRITICAL();
            result += frames_written;
            count -= frames_written;
            data += frames_written;
        }
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result * sizeof(struct can_frame);
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool Can::select(File* file, int mode)
{
    portENTER_CRITICAL();
    bool retval = false;
    switch (mode)
    {
        case FREAD:
            if (rxBuf->pending() > 0)
            {
                retval = true;
            }
            else
            {
                rxBuf->select_insert();
            }
            break;
        case FWRITE:
            if (txBuf->space() > 0)
            {
                retval = true;
            }
            else
            {
                txBuf->select_insert();
            }
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }
    portEXIT_CRITICAL();

    return retval;
}
