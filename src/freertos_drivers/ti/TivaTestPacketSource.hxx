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
 * \file TivaTestPacketSource.hxx
 *
 * A dummy device driver that spews out a lot of GC packets when a button is
 * held. Useful for debugging buffer usage.
 *
 * @author Balazs Racz
 * @date 11 Jul 2016
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVATESTPACKETSOURCE_HXX_
#define _FREERTOS_DRIVERS_TI_TIVATESTPACKETSOURCE_HXX_

#include "freertos_drivers/common/Devtab.hxx"
#include <fcntl.h>
#include <sys/select.h>
#include "hardware.hxx"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "utils/Ewma.hxx"

/// Example hardware specification for the TivaTestPacketSource device driver.
struct TivaTestPktDefHw
{
    /// Which timer resource to use. The driver always uses the TIMER_A+B in
    /// concatenated mode.
    static constexpr auto TIMER_BASE = TIMER3_BASE;
    /// Peripheral to enable the clock output to.
    static constexpr auto TIMER_PERIPH = SYSCTL_PERIPH_TIMER3;
    /// GPIO pin structure that turns on the input feed.
    using ENABLE_Pin = SW1_Pin;
    /// Defines whether the enabe pin is active-high or active-low.
    static constexpr auto ENABLE_PIN_TRUE = false;
    /// Interrupt number of the timer resource.
    static constexpr auto TIMER_INTERRUPT = INT_TIMER3A;
    /// How may bytes maximum can be accumulated as pending if the reader is
    /// not consuming data fast enough.
    static constexpr unsigned MAX_BUFFER_VALUE = 20000;
    /// How many bytes to generate in one go.
    static constexpr unsigned PACKET_SIZE = 1500;
    /// Bytes/sec to generate in total as input.
    static constexpr unsigned BANDWIDTH = 30000 * 29 / 7; ///< in bytes/sec
    /// Number of clock cycles to wait between generating two packets.
    static constexpr unsigned TIMER_PERIOD = UINT64_C(80000000) * PACKET_SIZE / BANDWIDTH;
};

/// A dummy device driver that spews out a lot of GC packets when a button is
/// held. Useful for debugging buffer usage.
template <class HW> class TivaTestPacketSource : public Node
{
public:
    /// Constructor. @param name is the filesystem device name
    /// (e.g. "/dev/load")
    TivaTestPacketSource(const char *name)
        : Node(name)
    {
        MAP_SysCtlPeripheralEnable(HW::TIMER_PERIPH);
        MAP_TimerDisable(HW::TIMER_BASE, TIMER_A);
        reset_packet();
    }

    /// Call this function from extern "C" void timer3a_interrupt_handler()
    void timer_isr()
    {
        MAP_TimerIntClear(HW::TIMER_BASE, TIMER_TIMA_TIMEOUT);
        int woken;
        if (HW::ENABLE_Pin::get() == HW::ENABLE_PIN_TRUE &&
            bufferedBytes_ < HW::MAX_BUFFER_VALUE)
        {
            bufferedBytes_ += HW::PACKET_SIZE;
            Device::select_wakeup_from_isr(&readSelect_, &woken);
        }
    }

    /// @return the total number of bytes produced so far.
    unsigned absolute_offset() {
        return absoluteOffset_;
    }

protected:
    /// Turns on the device.
    void enable() override
    {
        MAP_SysCtlPeripheralEnable(HW::TIMER_PERIPH);
        MAP_TimerDisable(HW::TIMER_BASE, TIMER_A);

        MAP_TimerClockSourceSet(HW::TIMER_BASE, TIMER_CLOCK_SYSTEM);
        MAP_TimerConfigure(HW::TIMER_BASE, TIMER_CFG_PERIODIC);

        MAP_TimerLoadSet(HW::TIMER_BASE, TIMER_A, HW::TIMER_PERIOD);

        MAP_IntDisable(HW::TIMER_INTERRUPT);
        MAP_IntPrioritySet(
            HW::TIMER_INTERRUPT, configKERNEL_INTERRUPT_PRIORITY);
        MAP_TimerIntEnable(HW::TIMER_BASE, TIMER_TIMA_TIMEOUT);
        MAP_TimerEnable(HW::TIMER_BASE, TIMER_A);
        MAP_IntEnable(HW::TIMER_INTERRUPT);
    }

    /// Turns off the device.
    void disable() override
    {
        MAP_TimerDisable(HW::TIMER_BASE, TIMER_A);
    }

    /// Unused.
    void flush_buffers() override
    {
    }

    /// Unused.
    ssize_t write(File *, const void *, size_t len) override
    {
        // Swallow all data.
        return len;
    }

    /// Performs generating the fake data and/or blocking the caller as needed.
    ssize_t read(File *file, void *vtgt, size_t len) override
    {
        char *tgt = (char *)vtgt;
        unsigned rd = 0;
        while (rd < len)
        {
            if (bufferedBytes_ == 0)
            {
                if (file->flags & O_NONBLOCK)
                {
                    if (rd > 0)
                        return rd;
                    return -EAGAIN;
                }
                fd_set fds;
                FD_ZERO(&fds);
                int fd = Device::fd_lookup(file);
                FD_SET(fd, &fds);

                ::select(fd + 1, &fds, NULL, NULL, NULL);
            }
            portENTER_CRITICAL();
            if (bufferedBytes_ == 0)
            {
                portEXIT_CRITICAL();
                continue;
            }
            while (rd < len && bufferedBytes_ > 0)
            {
                unsigned tocp = std::min(len - rd, inputPacketLen_ - offset_);
                tocp = std::min(tocp, bufferedBytes_);
                memcpy(tgt, packet_ + offset_, tocp);
                offset_ += tocp;
                rd += tocp;
                bufferedBytes_ -= tocp;
                tgt += tocp;
                absoluteOffset_ += tocp;
                if (offset_ >= inputPacketLen_)
                {
                    reset_packet();
                }
            }
            portEXIT_CRITICAL();
        }
        if (bufferedBytes_)
        {
            select_wakeup(&readSelect_);
        }
        return rd;
    }

    /// Implementation of device-specific select() functionality.
    bool select(File *file, int mode) override
    {
        switch (mode)
        {
            case FWRITE:
                return true;
            case FREAD:
                portENTER_CRITICAL();
                if (bufferedBytes_)
                {
                    portEXIT_CRITICAL();
                    return true;
                }
                select_insert(&readSelect_);
                portEXIT_CRITICAL();
                return false;
        }
        return true;
    }

    /// Helper function to clear the input packet buffer and fill in with the
    /// next packet in line.
    void reset_packet()
    {
        offset_ = 0;
        snprintf(packet_, sizeof(packet_), ":X1F555444N%016d;\n", cnt_);
        ++cnt_;
        inputPacketLen_ = strlen(packet_);
    }

    /// Helper structure for select() support.
    SelectInfo readSelect_;
    /// Which is the next byte from the INPUT_PACKET to send back.
    unsigned offset_{0};
    /// How many bytes we have accumulated to send back.
    unsigned bufferedBytes_{0};
    /// Index of the next packet.
    unsigned cnt_{0};
    /// Text-rendered form of the next packet.
    char packet_[40];
    /// Length of packet_;
    unsigned inputPacketLen_;
    /// How many bytes in total have been read from this driver since boot.
    uint32_t absoluteOffset_{0};
};

#endif // _FREERTOS_DRIVERS_TI_TIVATESTPACKETSOURCE_HXX_
