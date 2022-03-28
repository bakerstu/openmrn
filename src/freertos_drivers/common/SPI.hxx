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
 * \file SPI.hxx
 * This file implements a generic SPI device driver layer.
 *
 * @author Stuart W. Baker
 * @date 29 May 2016
 */

#ifndef _FREERTOS_DRIVERS_COMMON_SPI_HXX_
#define _FREERTOS_DRIVERS_COMMON_SPI_HXX_

#include <unistd.h>
#include <compiler.h>

#include "BlockOrWakeUp.hxx"
#include "SimpleLog.hxx"
#include "Devtab.hxx"
#include "nmranet_config.h"
#include "os/OS.hxx"
#include "spi/spidev.h"

/** Private data for an SPI device.  Unlike the Linux model this interface is
 * based on, speed, bits per word, mode, and lsb first will always be the same
 * for both read and write.  Changing one of these parameters for read will also
 * reflect the change in write and vice versa.
 */
class SPI : public Node
{
public:
    /** Function point for the chip select assert and deassert methods */
    typedef void (*ChipSelectMethod)();

    /** Method to transmit/receive the data.  Chip select will be asserted at
     * start of the transfer and deasserted at the end of the transfer.
     * @param msg message(s) to transact.
     * @return bytes transfered upon success, -errno upon failure
     */
    __attribute__((optimize("-O3")))
    int transfer_with_cs_assert(struct spi_ioc_transfer *msg)
    {
        csAssert();
        int result = transfer(msg);
        csDeassert();
        return result;
    }

    /** Method to transmit/receive the data.  Chip select will be asserted at
     * start of the transfer and deasserted at the end of the transfer.  This
     * will always be a polled transaction no matter what.
     * @param msg message(s) to transact.
     * @param num number of messages to transfer
     * @return bytes transfered upon success, -errno upon failure
     */
    __attribute__((optimize("-O3")))
    int transfer_with_cs_assert_polled(struct spi_ioc_transfer *msgs,
                                       int num = 1)
    {
        int count = 0;
        int result;

        csAssert();
        for (int i = 0; i < num; ++i, ++msgs)
        {
            result = transfer_polled(msgs);
            if (UNLIKELY(result < 0))
            {
                // something bad happened, reset the bus and bail
                csDeassert();
                return result;
            }
            count += msgs->len;
        }
        csDeassert();
        return count;
    }

    /** Conduct multiple message transfers with one stop at the end.
     * @param msgs array of messages to transfer
     * @param num number of messages to transfer
     * @return total number of bytes transfered, -errno upon failure
     */
    __attribute__((optimize("-O3")))
    int transfer_messages(struct spi_ioc_transfer *msgs, int num);

protected:
    /** Constructor
     * @param name device name in file system
     * @param cs_assert function pointer to a method that asserts chip select
     * @param cs_deassert function pointer to a method that deasserts chip
     *                    select
     * @param bus_lock the user must provide a shared mutex if the device
     *                 instance represents more than one chip select on the
     *                 same bus interface.
     */
    SPI(const char *name, ChipSelectMethod cs_assert,
        ChipSelectMethod cs_deassert, OSMutex *bus_lock = nullptr)
        : Node(name)
        , csAssert(cs_assert)
        , csDeassert(cs_deassert)
        , speedHz(1000000)
        , bitsPerWord(8)
        , mode(SPI_MODE_0)
        , lsbFirst(false)
        , busLock(bus_lock)
    {
    }

    /** Destructor.
     */
    ~SPI()
    {
        HASSERT(0);
    }

    /** Lock the bus shared by many chip selects. */
    void bus_lock()
    {
        if (busLock)
        {
            busLock->lock();
        }
    }

    /** Unlock the bus shared by many chip selects. */
    void bus_unlock()
    {
        if (busLock)
        {
            busLock->unlock();
        }
    }

    /** Method to transmit/receive the data.
     * @param msg message(s) to transact.
     * @return bytes transfered upon success, -errno upon failure
     */
    virtual int transfer(struct spi_ioc_transfer *msg) = 0;

    /** Method to transmit/receive the data, but always in polled mode.
     * @param msg message(s) to transact.
     * @return bytes transfered upon success, -errno upon failure
     */
    virtual int transfer_polled(struct spi_ioc_transfer *msg) = 0;

    /** Update the configuration of the bus.
     * @return >= 0 upon success, -errno upon failure
     */
    virtual int update_configuration() = 0;

    /** Request an ioctl transaction.
     * @param file file reference for this device
     * @param key ioctl key
     * @param data key data
     * @return >= 0 upon success, -errno upon failure
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) override;

    /** function pointer to a method that asserts chip select. */
    ChipSelectMethod csAssert;

    /** function pointer to a method that deasserts chip select. */
    ChipSelectMethod csDeassert;

    /** Max default speed in Hz */
    uint32_t speedHz;

    /** number of bits per word transaction */
    uint8_t bitsPerWord;

    /** one of four SPI modes */
    uint8_t mode;

    /** transmit LSB first if true */
    bool lsbFirst;

private:
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -errno upon failure
     */
    ssize_t read(File *file, void *buf, size_t count) override;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -errno upon failure
     */
    ssize_t write(File *file, const void *buf, size_t count) override;

    /** Discards all pending buffers. Called after disable(). */
    void flush_buffers() override {}

    /** Mutual exclusion for the bus among many chip selects */
    OSMutex *busLock;

    /** Default constructor.
     */
    SPI();

    DISALLOW_COPY_AND_ASSIGN(SPI);
};

#endif /* _FREERTOS_DRIVERS_COMMON_SPI_HXX_ */
