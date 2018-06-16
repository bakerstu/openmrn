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
 * @file BenchmarkCan.hxx
 * This file implements a fake CAN driver used for benchmarking the stack.
 *
 * @author Stuart W. Baker
 * @date 6 June 2018
 */

#ifndef _FREERTOS_DRIVERS_COMMON_BENCHMARKCAN_HXX_
#define _FREERTOS_DRIVERS_COMMON_BENCHMARKCAN_HXX_

#include "freertos_drivers/common/Can.hxx"
#include "utils/Atomic.hxx"

/** Generic CAN driver for throughput testing purposes.
 */
class BenchmarkCan : public Can, private Atomic
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     */
    BenchmarkCan(const char *name)
        : Can(name)
        , readTimeLast_(0)
        , readCount_(0)
    {
    }

    /** Destructor.
     */
    ~BenchmarkCan()
    {
    }

    /** Start a benchmarking run. Causes the same frame from being injected
     * count times as if it was received from the CAN-bus.
     * @param frame is the frame to inject to the stack.
     * @param count how many copies of this frame to inject.
     */
    void start_benchmark(const struct can_frame *frame, unsigned count)
    {
        bool need_signal = false;
        {
            AtomicHolder h(this);
            packet_ = *frame;
            readCount_ = count;
            need_signal = refill_locked();
        }
        if (need_signal)
        {
            rxBuf->signal_condition();
        }
    }

    /** Get the latest performance time stamp.
     * @param count the number of messages still left from the performance run,
     * zero if the performance run is completed.
     * @return the time when the last message was read by the stack.
     */
    long long get_timestamp(unsigned *count)
    {
        *count = readCount_;
        return readTimeLast_;
    }

private:
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno
     *         containing the cause
     */
    ssize_t read(File *file, void *buf, size_t count) override
    {
        ssize_t result = Can::read(file, buf, count);

        bool need_signal = false;
        if (result > 0)
        {
            AtomicHolder lock(this);
            readTimeLast_ = OSTime::get_monotonic();
            unsigned num_frames = result / sizeof(struct can_frame);
            if (readCount_ > num_frames)
            {
                readCount_ -= num_frames;
            }
            else
            {
                readCount_ = 0;
            }
            need_signal = refill_locked();
        }
        if (need_signal)
        {
            rxBuf->signal_condition();
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
        /* drop all the written data on the floor */
        return count;
    }

    /** Refills the rxBuf from the packet_ and readCount. Must be called within
     * a critical section lock. @return true if the condition needs to be
     * signaled. */
    bool refill_locked()
    {
        bool need_signal = false;
        while ((rxBuf->pending() < readCount_) && (rxBuf->space()))
        {
            if (rxBuf->pending() == 0)
            {
                need_signal = true;
            }
            rxBuf->put(&packet_, 1);
        }
        return need_signal;
    }

    void enable() override
    {
    }
    void disable() override
    {
    }
    void tx_msg() override
    {
    }

    struct can_frame packet_; /**< packet to inject. */
    long long readTimeLast_;  /**< timestamp of last read in OS time */
    size_t readCount_;        /**< count of packets still to inject */

    DISALLOW_COPY_AND_ASSIGN(BenchmarkCan);
};

#endif /* _FREERTOS_DRIVERS_COMMON_BENCHMARKCAN_HXX_ */
