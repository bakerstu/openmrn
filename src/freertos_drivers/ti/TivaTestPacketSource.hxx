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

struct TivaTestPktDefHw
{
    static constexpr auto TIMER_BASE = TIMER3_BASE;
    static constexpr auto TIMER_PERIPH = SYSCTL_PERIPH_TIMER3;
    using ENABLE_Pin = SW1_Pin;
    static constexpr auto ENABLE_PIN_TRUE = false;
    static constexpr auto TIMER_INTERRUPT = INT_TIMER3A;
    static constexpr unsigned MAX_BUFFER_VALUE = 20000;
    static constexpr unsigned PACKET_SIZE = 1500;
    static constexpr unsigned BANDWIDTH = 30000 * 29 / 7; ///< in bytes/sec
    static constexpr unsigned TIMER_PERIOD = UINT64_C(80000000) * PACKET_SIZE / BANDWIDTH;
};

template <class HW> class TivaTestPacketSource : public Node
{
public:
    TivaTestPacketSource(const char *name)
        : Node(name)
    {
        MAP_SysCtlPeripheralEnable(HW::TIMER_PERIPH);
        MAP_TimerDisable(HW::TIMER_BASE, TIMER_A);
        reset_packet();
    }

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

protected:
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

    void disable() override
    {
        MAP_TimerDisable(HW::TIMER_BASE, TIMER_A);
    }

    void flush_buffers() override
    {
    }

    ssize_t write(File *, const void *, size_t len) override
    {
        // Swallow all data.
        return len;
    }
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

    void reset_packet()
    {
        offset_ = 0;
        snprintf(packet_, sizeof(packet_), ":X1F555444N%016d;\n", cnt_);
        ++cnt_;
        inputPacketLen_ = strlen(packet_);
    }

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
};

#endif // _FREERTOS_DRIVERS_TI_TIVATESTPACKETSOURCE_HXX_
