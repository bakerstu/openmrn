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
 * \file 11cxx_async_can.cxx
 * This file implements an asynchronous CAN driver for the LPC11Cxx
 * microcontrollers, using the builtin ROM drivers.
 *
 * @author Balazs Racz
 * @date 14 Dec 2013
 */

#ifdef TARGET_LPC11Cxx

#include "11CXX_rom_driver_CAN.h"
#include "nmranet_config.h"

#include "executor/StateFlow.hxx"
#include "utils/Hub.hxx"

typedef struct _ROM
{
    const unsigned p_usbd;
    const unsigned p_clib;
    const CAND *pCAND;
} ROM;

/** Pointer to the ROM call structures. */
const ROM *const *const rom = (ROM **)0x1fff1ff8;

namespace lpc11cxx
{

#define FIRST_TX_OBJ 1
#define FIRST_RX_OBJ 16

class CanPipeMember;
class CanRxFlow;

/* Static instances of the CAN drivers */
CanPipeMember *g_tx_instance = nullptr;
CanRxFlow *g_rx_instance = nullptr;
CanHubFlow *g_pipe = nullptr;

class CanPipeMember : public CanHubPort
{
public:
    CanPipeMember(Service *s)
        : CanHubPort(s)
        , freeTxBuffers_(0xFFFFFFFF)
        , frameToWrite_(nullptr)
        , bufferFull_(0)
    {
    }

    Action entry() OVERRIDE
    {
        AtomicHolder h(this);
        HASSERT(!frameToWrite_);
        frameToWrite_ = message()->data()->mutable_frame();
        return call_immediately(STATE(try_send));
    }

    Action try_send()
    {
        HASSERT(frameToWrite_);
        const struct can_frame *frame = nullptr;
        uint8_t buf_num = FIRST_TX_OBJ;
        {
            AtomicHolder h(this);

            // Looks for a free tx buffer.
            while (buf_num < FIRST_RX_OBJ &&
                   (!(freeTxBuffers_ & (1 << buf_num))))
            {
                buf_num++;
            }
            if (buf_num >= FIRST_RX_OBJ)
            {
                // Wait for ISR to wake us up.
                bufferFull_ = 1;
                return wait();
            }

            freeTxBuffers_ &= ~(1 << buf_num);

            // We decided to send the frame.
            frame = frameToWrite_;
            frameToWrite_ = nullptr; // no callbacks from ISR.
        }
        CAN_MSG_OBJ msg_obj;
        msg_obj.msgobj = buf_num;
        msg_obj.mode_id = frame->can_id |
                          (frame->can_rtr ? CAN_MSGOBJ_RTR : 0) |
                          (frame->can_eff ? CAN_MSGOBJ_EXT : 0);
        msg_obj.mask = 0x0;
        msg_obj.dlc = frame->can_dlc;
        memcpy(msg_obj.data, frame->data, frame->can_dlc);
        (*rom)->pCAND->can_transmit(&msg_obj);

        // The incoming frame is no longer needed.
        return release_and_exit();
    }

    void TxFinishedFromIsr(uint8_t buf_num)
    {
        HASSERT(!(freeTxBuffers_ & (1 << buf_num)));
        freeTxBuffers_ |= (1 << buf_num);
        if (frameToWrite_ && bufferFull_)
        {
            bufferFull_ = 0;
            service()->executor()->add_from_isr(this, priority());
        }
    }

private:
    uint32_t freeTxBuffers_;
    const struct can_frame *frameToWrite_;
    // 1 if we are waiting for a free tx buffer.
    unsigned bufferFull_ : 1;
};

class CanRxFlow : public StateFlowBase, private Atomic
{
public:
    CanRxFlow(Service *s)
        : StateFlowBase(s)
        , bufFull_(0)
        , rxPending_(1)
        , frameLost_(0)
    {
        /* Configures msgobj NN to receive all extended frames. */
        CAN_MSG_OBJ msg_obj;
        msg_obj.msgobj = FIRST_RX_OBJ;
        msg_obj.mode_id = 0x000 | CAN_MSGOBJ_EXT;
        msg_obj.mask = 0x000;
        msg_obj.dlc = 0x000;
        (*rom)->pCAND->config_rxmsgobj(&msg_obj);
        start_flow(STATE(wait_or_copy));
    }

    // Callback inside the ISR context. @param buf_num is the number of the
    // message object in the hardware.
    void isr(uint8_t buf_num)
    {
        if (bufFull_)
        {
            // Another interrupt came in before we have cleared the buffer.
            frameLost_ = 1;
            return;
        }
        else
        {
            bufFull_ = 1;
        }
        if (!rxPending_)
        {
            rxPending_ = 1;
            // highest priority
            service()->executor()->add_from_isr(this, 0);
        }
    }

    Action wait_or_copy()
    {
        {
            AtomicHolder h(this);
            if (!bufFull_)
            {
                rxPending_ = 0;
                return wait();
            }
        }
        return allocate_and_call(g_pipe, STATE(allocation_complete));
    }

    // Scheduled by the ISR when the frame has arrived.
    Action allocation_complete()
    {
        HASSERT(bufFull_);
        auto *b = get_allocation_result(g_pipe);
        {
            AtomicHolder h(this);
            copy_hardware_frame(FIRST_RX_OBJ, b->data()->mutable_frame());
        }
        b->data()->skipMember_ = g_tx_instance;
        g_pipe->send(b);
        return call_immediately(STATE(wait_or_copy));
    }

private:
    // Has to run either in ISR or in a critical section. Retrieves a hardware
    // buffer and copies it to the class-local can_frame.
    void copy_hardware_frame(uint8_t buf_num, struct can_frame* frame)
    {
        HASSERT(bufFull_);
        CAN_MSG_OBJ msg_obj;
        /* Determine which CAN message has been received */
        msg_obj.msgobj = buf_num;

        /* Now load up the msg_obj structure with the CAN message */
        (*rom)->pCAND->can_receive(&msg_obj);

        // Here we need to be in a critical section; otherwise another CAN
        // interrupt might come in between these two calls.
        bufFull_ = 0;
        rxPending_ = 0;

        frame->can_id = msg_obj.mode_id & ((1 << 29) - 1);
        // JMRI crashes the node here.
        // HASSERT(frame->can_id & 0xfff);
        frame->can_rtr = (msg_obj.mode_id & CAN_MSGOBJ_RTR) ? 1 : 0;
        frame->can_eff = (msg_obj.mode_id & CAN_MSGOBJ_EXT) ? 1 : 0;
        frame->can_err = 0;
        frame->can_dlc = msg_obj.dlc;
        memcpy(frame->data, msg_obj.data, msg_obj.dlc);
    }

    // 1 if the hardware object for RX has a frame.
    unsigned bufFull_ : 1;
    // 1 if the state flow is in operation, 0 if the state flow can be
    // scheduled.
    unsigned rxPending_ : 1;
    // 1 if we have lost a frame.
    unsigned frameLost_ : 1;
};

/** CAN receive callback. Called by the ROM can driver in an ISR context.
    @param msg_obj_num the number of CAN buffer that has the new frame.
*/
void CAN_rx(uint8_t msg_obj_num)
{
    HASSERT(g_rx_instance);
    g_rx_instance->isr(msg_obj_num);
    // We always assume there is someone woken up. This is not nice.
    portYIELD();
}

/** CAN transmit callback. Called by the ROM can driver in an ISR context.
    @param msg_obj_num the number of CAN buffer that finished transmission.
*/
void CAN_tx(uint8_t msg_obj_num)
{
    HASSERT(g_tx_instance);
    g_tx_instance->TxFinishedFromIsr(msg_obj_num);
    // We always assume there is someone woken up. This is not nice.
    portYIELD();
}

/** CAN error callback. Called by the ROM can driver in an ISR context.
    @param error_info defines what kind of error occured on the bus.
*/
void CAN_error(uint32_t error_info)
{
    return;
}

/** Function pointer table to pass to the ROM drivers with callbacks. */
static const CAN_CALLBACKS callbacks = {CAN_rx, CAN_tx, CAN_error, NULL,
                                        NULL,   NULL,   NULL,      NULL, };

/**  Clock initialization constants for 125 kbaud */
static const uint32_t ClkInitTable125[2] = {0x00000000UL, // CANCLKDIV
                                            0x00001C57UL  // CAN_BTR
};

/**  Clock initialization constants for 250 kbaud */
static const uint32_t ClkInitTable250[2] = {0x00000000UL, // CANCLKDIV
                                            0x00001C4BUL  // CAN_BTR
};

void CreateCanDriver(CanHubFlow *parent)
{
    if (config_nmranet_can_bitrate() == 250000) {
        /* Initialize the CAN controller */
        (*rom)->pCAND->init_can((uint32_t *)&ClkInitTable250[0], 1);
    } else if ((config_nmranet_can_bitrate() == 125000)) {
        /* Initialize the CAN controller */
        (*rom)->pCAND->init_can((uint32_t *)&ClkInitTable125[0], 1);
    } else {
        DIE("Unknown can bitrate.");
    }
    /* Configure the CAN callback functions */
    (*rom)->pCAND->config_calb((CAN_CALLBACKS *)&callbacks);

    g_pipe = parent;
    g_rx_instance = new CanRxFlow(parent->service());
    g_tx_instance = new CanPipeMember(parent->service());
    parent->register_port(g_tx_instance);

    /* Enable the CAN Interrupt */
    NVIC_SetPriority(CAN_IRQn, 1);
    NVIC_EnableIRQ(CAN_IRQn);
}

} // namespace lpc11cxx

extern "C" {
/** Overrides the system's weak interrupt handler and calls the builtin ROM
 * interrupt handler. */
void CAN_IRQHandler(void)
{
    (*rom)->pCAND->isr();
}
}

#else
#error You need to define TARGET_LPC11Cxx if you want to compiple its rom driver.
#endif
