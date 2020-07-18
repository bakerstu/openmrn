/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file SPI.cxx
 * This file implements a generic SPI device driver layer.
 *
 * @author Stuart W. Baker
 * @date 29 May 2016
 */

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include <cstdint>
#include <fcntl.h>
#include "Devtab.hxx"
#include "SPI.hxx"

/** Conduct multiple message transfers with one stop at the end.
 * @param msgs array of messages to transfer
 * @param num number of messages to transfer
 * @return total number of bytes transfered, -errno upon failure
 */
__attribute__((optimize("-O3")))
int SPI::transfer_messages(struct spi_ioc_transfer *msgs, int num)
{
    //HASSERT(num > 0);

    int count = 0;
    int result;

    lock_.lock();
    bus_lock();
    for (int i = 0; i < num; ++i, ++msgs)
    {
        count += msgs->len;
        csAssert();
        result = transfer(msgs);
        if (UNLIKELY(result < 0))
        {
            /* something bad happened, reset the bus and bail */
            csDeassert();
            bus_unlock();
            lock_.unlock();
            return result;
        }
        if (msgs->cs_change)
        {
            if (UNLIKELY(msgs->delay_usec))
            {
                usleep(msgs->delay_usec);
            }
            csDeassert();
        }
    }
    bus_unlock();
    lock_.unlock();

    return count;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -errno upon failure
 */
__attribute__((optimize("-O3")))
ssize_t SPI::read(File *file, void *buf, size_t count)
{
    int result;

    if ((file->flags & O_ACCMODE) == O_WRONLY)
    {
        return -EBADF;
    }

    struct spi_ioc_transfer msg;
    msg.tx_buf = 0;
    msg.rx_buf = (uint64_t)buf;
    msg.len = count;
    msg.speed_hz = speedHz;
    msg.delay_usec = 0;
    msg.bits_per_word = bitsPerWord;
    msg.cs_change = true;

    lock_.lock();
    bus_lock();
    csAssert();
    result = transfer(&msg);
    csDeassert();
    bus_unlock();
    lock_.unlock();

    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -errno upon failure
 */
__attribute__((optimize("-O3")))
ssize_t SPI::write(File *file, const void *buf, size_t count)
{
    int result;

    if ((file->flags & O_ACCMODE) == O_RDONLY)
    {
        return -EBADF;
    }

    struct spi_ioc_transfer msg;
    msg.tx_buf = (uint64_t)buf;
    msg.rx_buf = 0;
    msg.len = count;
    msg.speed_hz = speedHz;
    msg.delay_usec = 0;
    msg.bits_per_word = bitsPerWord;
    msg.cs_change = true;

    lock_.lock();
    bus_lock();
    csAssert();
    result = transfer(&msg);
    csDeassert();
    bus_unlock();
    lock_.unlock();

    return result;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param key ioctl key
 * @param data key data
 * @return 0 upon success, -errno upon failure
 */
__attribute__((optimize("-O3")))
int SPI::ioctl(File *file, unsigned long int key, unsigned long data)
{
    HASSERT(IOC_TYPE(key) == SPI_IOC_MAGIC);

    if (IOC_NR(key) == 0)
    {
        /* message transfer */
        struct spi_ioc_transfer *msgs = (struct spi_ioc_transfer*)data;
        int result = transfer_messages(msgs, IOC_SIZE(key) /
                                             sizeof(struct spi_ioc_transfer));
        if (result < 0)
        {
            return result;
        }
        return 0;
    }
    switch (key)
    {
        default:
            return -EINVAL;
        case SPI_IOC_GET_OBJECT_REFERENCE:
        {
            SPI **self = (SPI**)data;
            *self = this;
            return 0;
        }
        case SPI_IOC_RD_MODE:
        {
            uint8_t *m = (uint8_t*)data;
            *m = mode;
            break;
        }
        case SPI_IOC_WR_MODE:
        {
            uint8_t *m = (uint8_t*)data;
            HASSERT(*m <= SPI_MODE_3);
            mode = *m;
            break;
        }
        case SPI_IOC_RD_LSB_FIRST:
        {
            uint8_t *lsbf = (uint8_t*)data;
            *lsbf = lsbFirst;
            break;
        }
        case SPI_IOC_WR_LSB_FIRST:
        {
            uint8_t *lsbf = (uint8_t*)data;
            lsbFirst = *lsbf;
            break;
        }
        case SPI_IOC_RD_BITS_PER_WORD:
        {
            uint8_t *bpw = (uint8_t*)data;
            *bpw = bitsPerWord;
            break;
        }
        case SPI_IOC_WR_BITS_PER_WORD:
        {
            uint8_t *bpw = (uint8_t*)data;
            bitsPerWord = *bpw;
            break;
        }
        case SPI_IOC_RD_MAX_SPEED_HZ:
        {
            uint32_t *hz = (uint32_t*)data;
            *hz = speedHz;
            break;
        }
        case SPI_IOC_WR_MAX_SPEED_HZ:
        {
            uint32_t *hz = (uint32_t*)data;
            speedHz = *hz;
            break;
        }
    }

    lock_.lock();
    bus_lock();
    int result = update_configuration();
    lock_.unlock();
    bus_unlock();

    return result;
}

