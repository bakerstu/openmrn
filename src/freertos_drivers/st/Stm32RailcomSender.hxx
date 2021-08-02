/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file Stm32RailcomSender.hxx
 *
 * Implements a RailcomDriver that sends outgoing railcom data through a UART
 * TX on an STM32 target. Designed to be invoked from the priority zero
 * interrupt of the DCC decoder.
 *
 * @author Balazs Racz
 * @date July 10, 2021
 */

#include "freertos_drivers/common/RailcomDriver.hxx"
#include "dcc/railcom.h"
#include "freertos_drivers/st/Stm32Uart.hxx"

class Stm32RailcomSender : public RailcomDriver, protected Stm32Uart
{
public:
    Stm32RailcomSender(
        const char *name, USART_TypeDef *base, IRQn_Type interrupt)
        : Stm32Uart(name, base, interrupt)
    {
    }

public:
    /// Specifies what packet should be sent for the channel1 cutout. It is
    /// okay to specify the same packet pointer for ch1 and ch2 cutout.
    /// @param ch1_pkt the RailCom packet. Only the ch1 data will be read from
    /// this packet. This pointer must stay alive until the next DCC packet
    /// comes. The FeedbackKey in this packet must be correct for the current
    /// DCC packet or else the data will not be sent.
    void send_ch1(const DCCFeedback *ch1_pkt) override
    {
        ch1Pkt_ = ch1_pkt;
    }

    /// Specifies what packet should be sent for the channel2 cutout. It is
    /// okay to specify the same packet pointer for ch1 and ch2 cutout.
    /// @param ch2_pkt the RailCom packet. Only the ch2 data will be read from
    /// this packet. This pointer must stay alive until the next DCC packet
    /// comes. The FeedbackKey in this packet must be correct for the current
    /// DCC packet or else the data will not be sent.
    void send_ch2(const DCCFeedback *ch2_pkt) override
    {
        ch2Pkt_ = ch2_pkt;
    }

    ssize_t write(File *file, const void *buf, size_t count) override
    {
        // We do not support writing through the regular posix API.
        return -EIO;
    }

    ssize_t read(File *file, void *buf, size_t count) override
    {
        // We do not support reading through the regular posix API.
        return -EIO;
    }

private:
    // ======= DCC driver API ========
    /// No implementation needed.
    void feedback_sample() override
    {
    }
    /// Called at the beginning of the first window.
    void start_cutout() override;
    /// Called at the beginning of the middle window.
    void middle_cutout() override;
    /// Called after the cutout is over.
    void end_cutout() override {
        // We throw away the packets that we got given.
        //ch1Pkt_ = nullptr;
        //ch2Pkt_ = nullptr;
    }
    /// Called instead of start/mid/end-cutout at the end of the current packet
    /// if there was no cutout requested.
    void no_cutout() override
    {
        // We throw away the packets that we got given.
        //ch1Pkt_ = nullptr;
        //ch2Pkt_ = nullptr;
    }
    /// Feedback key is set by the DCC decoder driver. The feedback packet must
    /// carry the same feedback key or else it will not be transmitted.
    void set_feedback_key(uint32_t key) override
    {
        expectedFeedbackKey_ = key;
    }

private:
    /// What should be the feedback key in the packet. This value comes from
    /// the DCC driver and is compared to the RailCom packets we should be
    /// sending at the beginning of the cutout windows.
    uintptr_t expectedFeedbackKey_ = 0;

    /// The packet to send in channel 1. Externally owned.
    const DCCFeedback *ch1Pkt_ = nullptr;
    /// The packet to send in channel 2. Externally owned.
    const DCCFeedback *ch2Pkt_ = nullptr;
};
