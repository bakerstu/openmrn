/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file mbed_can.cpp
 * This file implements a CAN driver on top of the mbed library. Currently it
 * is tested on the LPC23xx processors.
 *
 * @author Balazs Racz
 * @date 12 April 2013
 */

#include "mbed.h"
#include "Can.hxx"
#ifdef TARGET_LPC2368
#include "lpc23xx.h"
#endif

#ifdef TARGET_LPC1768
#include "LPC17xx.h"
#endif

class MbedCanDriver : public Can
{
public:
    enum Instance {
        CAN1,
        CAN2
    };
    MbedCanDriver(Instance instance, const char* dev, int frequency)
        : Can(dev),
          mbedCan_(instance == CAN1 ? P0_0 : P0_4,
                   instance == CAN1 ? P0_1 : P0_5),
          SR_(instance == CAN1 ? &LPC_CAN1->SR : &LPC_CAN2->SR) {
        mbedCan_.frequency(frequency);
        mbedCan_.attach(this, &MbedCanDriver::interrupt);
    }
private:
    virtual void enable() {}; /**< function to enable device */
    virtual void disable() {}; /**< function to disable device */

    void interrupt();
    virtual void tx_msg();

    mbed::CAN mbedCan_;
    // Status register.
    volatile uint32_t* SR_;
    char txPending_;
};

/** Try and transmit a message. Does nothing if there is no message to transmit
 *  or no write buffers to transmit via.
 * @param dev device to transmit message on
 */
void MbedCanDriver::tx_msg()
{
    if (txPending_)
        return;
    if (!(*SR_ & 0x4))
        return; // TX buffer is holding a packet

    struct can_frame can_frame;
    /** @todo (balazs.racz): think about how we could do with a shorter
     critical section. The problem is that an ISR might decide to send off the
     next frame ahead of us. */
    taskENTER_CRITICAL();
    if (!get_tx_msg(&can_frame))
    {
        taskEXIT_CRITICAL();
        return;
    }
    CANMessage msg(can_frame.can_id, (const char*)can_frame.data,
                   can_frame.can_dlc, can_frame.can_rtr ? CANRemote : CANData,
                   can_frame.can_eff ? CANExtended : CANStandard);
    if (!mbedCan_.write(msg))
    {
        // NOTE(balazs.racz): This means that the CAN layer didn't find an
        // available TX buffer to send the CAN message. However, since
        // txPending == 0 at this point, that can only happen if someone else
        // was also writing frames to this CAN device. We won't handle that
        // case now.
        overrunCount++;
    }
    txPending_ = 1;
    taskEXIT_CRITICAL();
}

/** Handler for CAN device. Called from the mbed irq handler. */
void MbedCanDriver::interrupt()
{
    int woken = 0;
    CANMessage msg;
    if (mbedCan_.read(msg))
    {
        struct can_frame can_frame;
        can_frame.can_id = msg.id;
        can_frame.can_rtr = msg.type == CANRemote ? 1 : 0;
        can_frame.can_eff = msg.format == CANStandard ? 0 : 1;
        can_frame.can_err = 0;
        can_frame.can_dlc = msg.len;
        memcpy(can_frame.data, msg.data, msg.len);
        put_rx_msg_from_isr(can_frame);
    }
#if defined(TARGET_LPC2368) || defined(TARGET_LPC1768)
    if (*SR_ & 0x4)
    {
        // Transmit buffer 1 empty => transmit finished.
        struct can_frame can_frame;
        if (get_tx_msg_from_isr(&can_frame))
        {
            CANMessage msg(can_frame.can_id, (const char*)can_frame.data,
                           can_frame.can_dlc,
                           can_frame.can_rtr ? CANRemote : CANData,
                           can_frame.can_eff ? CANExtended : CANStandard);
            if (mbedCan_.write(msg))
            {
                txPending_ = 1;
            }
            else
            {
                // NOTE(balazs.racz): This is an inconsistency -- if *SR&0x4
                // then TX1 buffer is empty, so if write fails, then... a task
                // switch occured while serving an interrupt handler?
                overrunCount++;
                txPending_ = 0;
            }
        }
        else
        {
            txPending_ = 0;
        }
    }
#else
#error you need to define how to figure out whether the transmit buffer is empty.
#endif
    /** @todo (balazs.racz): need to see what needs to be done for acking the
        interrupt, depending on what the interrupt fnction attributes say. */
    if (woken)
    {
#ifdef TARGET_LPC1768
        portYIELD();
#elif defined(TARGET_LPC2368)
/** @todo(balazs.racz): need to find a way to yield on ARM7. The builtin
 * portYIELD_FROM_ISR assumes that we have entered the ISR with context
 * saving, which we didn't. */
#else
#error define how to yield on your CPU.
#endif
    }
}

/** The TCH baseboard for the mbed has CAN1 and CAN2 mixed up. */
MbedCanDriver can0(MbedCanDriver::CAN2, "/dev/can0", config_nmranet_can_bitrate());
MbedCanDriver can1(MbedCanDriver::CAN1, "/dev/can1", config_nmranet_can_bitrate());
