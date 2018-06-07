/** @copyright
 * Copyright (c) 2018, Stuart W Baker
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
 * @file TivaCanNullTx.hxx
 * This file implements a test driver for CAN on Tiva.
 *
 * @author Stuart W. Baker
 * @date 6 June 2018
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVACANNULLTX_HXX_
#define _FREERTOS_DRIVERS_TI_TIVACANNULLTX_HXX_

#include "TivaDev.hxx"

/** Specialization of Tiva CAN driver for testing purposes.
 */
class TivaCanNullTx : public TivaCan
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     */
    TivaCanNullTx(const char *name, unsigned long base, uint32_t interrupt)
        : TivaCan(name, base, interrupt)
        , readTimeFirst_(0)
        , readTime10000_(0)
        , readCount_(0)
    {
    }

    /** Destructor.
     */
    ~TivaCanNullTx()
    {
    }

    /** Get the latest performance time stamp.
     * @param count the number of messages that have been received in total
     * @return if count < @ref MESSAGE_COUNT, the time since the first message
     *         was received, else, the time between the first message received
     *         and the 10,000th message received.
     */
    long long get_timestamp_and_count(unsigned *count)
    {
        long long result;

        portENTER_CRITICAL();
        *count = readCount_;
        if (readCount_ >= MESSAGE_COUNT)
        {
            result = readTime10000_ - readTimeFirst_;
        }
        else
        {
            result = OSTime::get_monotonic() - readTimeFirst_;
        }
        portEXIT_CRITICAL();

        return result;
    }

private:
    static constexpr size_t MESSAGE_COUNT = 10000;

    /** Read from a file or device.
    * @param file file reference for this device
    * @param buf location to place read data
    * @param count number of bytes to read
    * @return number of bytes read upon success, -1 upon failure with errno
    *         containing the cause
    */
    ssize_t read(File *file, void *buf, size_t count) override
    {
        if (readCount_ == 0)
        {
            readTimeFirst_ = OSTime::get_monotonic();
        }

        ssize_t result = Can::read(file, buf, count);

        if (result > 0)
        {
            readCount_ += result / sizeof(struct can_frame);

            if (readCount_ >= MESSAGE_COUNT && readTime10000_ == 0)
            {
                readTime10000_ = OSTime::get_monotonic();
            }
        }

        return result;
    }

    /** Write to a file or device.
    * @param file file reference for this device
    * @param buf location to find write data
    * @param count number of bytes to write
    * @return number of bytes written upon success, -1 upon failure with errno
    *         containing the cause
    */
    ssize_t write(File *file, const void *buf, size_t count) override
    {
        /* drop all the write data on the floor */
        return count;
    }

    long long readTimeFirst_; /**< timestamp of first read in nsec */
    long long readTime10000_; /**< timestamp of 10,000th read in nsec */

    size_t readCount_; /**< running count of all the reads */

    DISALLOW_COPY_AND_ASSIGN(TivaCanNullTx);
};

#endif /* _FREERTOS_DRIVERS_TI_TIVACANNULLTX_HXX_ */
