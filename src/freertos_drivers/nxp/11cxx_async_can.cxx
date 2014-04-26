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

#if 0
#ifdef TARGET_LPC11Cxx

#include "11CXX_rom_driver_CAN.h"
#include "can.h"

#include "executor/control_flow.hxx"
#include "utils/pipe.hxx"

typedef struct _ROM
{
    const unsigned p_usbd;
    const unsigned p_clib;
    const CAND* pCAND;
} ROM;

/** Pointer to the ROM call structures. */
const ROM* const* const rom = (ROM**)0x1fff1ff8;

static const int RX_MSG_OBJ_NUM = 1;
static const int TX_MSG_OBJ_NUM = 2;

extern Executor g_executor;

namespace lpc11cxx
{

#define FIRST_TX_OBJ 1
#define FIRST_RX_OBJ 16

class CanPipeMember;
class CanRxFlow;

/* Static instances of the CAN drivers */
CanPipeMember* g_tx_instance = nullptr;
CanRxFlow* g_rx_instance = nullptr;
Pipe* g_pipe = nullptr;

class CanPipeMember : public PipeMember, public Executable, private Lockable
{
public:
    CanPipeMember()
        : freeTxBuffers_(0xFFFFFFFF), frameToWrite_(nullptr), done_(nullptr), bufferFull_(0)
    {
        lock_.TypedRelease(this);
        released_ = 1;
        allocatorAsked_ = 0;
    }

    virtual AllocatorBase* get_allocator()
    {
        allocatorAsked_++;
        return &lock_;
    }

    virtual void write(const void* buf, size_t count)
    {
        HASSERT(0);
    }

    virtual void async_write(const void* buf, size_t count, Notifiable* done)
    {
        released_ = 0;
        allocatorAsked_--;
        HASSERT(count == sizeof(struct can_frame));
        HASSERT(!frameToWrite_);
        done_ = done;
        LockHolder h(this);
        frameToWrite_ = static_cast<const struct can_frame*>(buf);
        g_executor.Add(this);
    }

    virtual void Run()
    {
        HASSERT(frameToWrite_);
        const struct can_frame* frame = nullptr;
        Notifiable* done = nullptr;
        uint8_t buf_num = FIRST_TX_OBJ;
        {
            LockHolder h(this);

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
                return;
            }

            freeTxBuffers_ &= ~(1 << buf_num);

            // We decided to send the frame.
            frame = frameToWrite_;
            frameToWrite_ = nullptr;  // no callbacks from ISR.
            done = done_;
            done_ = nullptr;
        }
        bufferFull_ = 0;
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
        done->notify();
        released_ = 1;
        lock_.TypedReleaseBack(this);
    }

    void TxFinishedFromIsr(uint8_t buf_num)
    {
        HASSERT(!(freeTxBuffers_ & (1 << buf_num)));
        freeTxBuffers_ |= (1 << buf_num);
        if (frameToWrite_ && !g_executor.IsRunning(this))
        {
            g_executor.AddFromIsr(this);
        }
    }

private:
    uint32_t freeTxBuffers_;
    const struct can_frame* frameToWrite_;
    Notifiable* done_;
    TypedAllocator<PipeMember> lock_;
    // 1 if a run method returned because it didn't find a free tx buffer.
    unsigned bufferFull_ : 1;
    unsigned released_ : 1;
    unsigned allocatorAsked_ : 4;
};

class CanRxFlow : public AllocationResult
{
public:
    CanRxFlow() : bufFull_(0), rxPending_(0), frameLost_(0)
    {
        /* Configures msgobj NN to receive all extended frames. */
        CAN_MSG_OBJ msg_obj;
        msg_obj.msgobj = FIRST_RX_OBJ;
        msg_obj.mode_id = 0x000 | CAN_MSGOBJ_EXT;
        msg_obj.mask = 0x000;
        msg_obj.dlc = 0x000;
        (*rom)->pCAND->config_rxmsgobj(&msg_obj);
    }

    // Callback inside the ISR context. @param buf_num is the number of the
    // message object in the hardware.
    void isr(uint8_t buf_num)
    {
        if (rxPending_)
        {
            frameLost_ = 1;
            // Since rxPending is already 1, we'll let the executor deal with
            // copying the frame to the buffer.
            return;
        }
        else
        {
            rxPending_ = 1;
        }
        if (!bufFull_)
        {
            copy_hardware_frame(buf_num);
            g_executor.AddFromIsr(this);
        }
    }

    // Scheduled by the ISR when the frame has arrived.
    virtual void Run()
    {
        HASSERT(bufFull_);
        g_can_alloc.AllocateEntry(this);
    }

    virtual void AllocationCallback(QueueMember* entry)
    {
        HASSERT(bufFull_);
        CanPipeBuffer* buf = g_can_alloc.cast_result(entry);
        buf->Reset();
        memcpy(&buf->frame, &frame_, sizeof(frame_));
        buf->pipe_buffer.skipMember = g_tx_instance;
        g_pipe->SendBuffer(&buf->pipe_buffer);
        {
            portSET_INTERRUPT_MASK();
            bufFull_ = 0;
            if (rxPending_)
            {
                copy_hardware_frame(FIRST_RX_OBJ);
                // will crash if this is already added
                g_executor.Add(this);
            }
            portCLEAR_INTERRUPT_MASK();
        }
    }

private:
    // Has to run either in ISR or in a critical section. Retrieves a hardware
    // buffer and copies it to the class-local can_frame.
    void copy_hardware_frame(uint8_t buf_num)
    {
        HASSERT(rxPending_);
        HASSERT(!bufFull_);
        CAN_MSG_OBJ msg_obj;
        /* Determine which CAN message has been received */
        msg_obj.msgobj = buf_num;

        /* Now load up the msg_obj structure with the CAN message */
        (*rom)->pCAND->can_receive(&msg_obj);

        // Here we need to be in a critical section; otherwise another CAN
        // interrupt might come in between these two calls.
        bufFull_ = 1;
        rxPending_ = 0;

        frame_.can_id = msg_obj.mode_id & ((1 << 29) - 1);
        // JMRI crashes the node here.
        // HASSERT(frame_.can_id & 0xfff);
        frame_.can_rtr = (msg_obj.mode_id & CAN_MSGOBJ_RTR) ? 1 : 0;
        frame_.can_eff = (msg_obj.mode_id & CAN_MSGOBJ_EXT) ? 1 : 0;
        frame_.can_err = 0;
        frame_.can_dlc = msg_obj.dlc;
        memcpy(frame_.data, msg_obj.data, msg_obj.dlc);
    }

    struct can_frame frame_;
    // 1 if the struct can_frame in here is full.
    unsigned bufFull_ : 1;
    // 1 if the can buffer in the hardware object is full.
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
const uint32_t ClkInitTable125[2] = {0x00000000UL, // CANCLKDIV
                                     0x00001C57UL  // CAN_BTR
};

/**  Clock initialization constants for 250 kbaud */
static const uint32_t ClkInitTable250[2] = {0x00000000UL, // CANCLKDIV
                                            0x00001C4BUL  // CAN_BTR
};

void CreateCanDriver(Pipe* parent)
{
    /* Initialize the CAN controller */
    (*rom)->pCAND->init_can((uint32_t*)&ClkInitTable250[0], 1);
    /* Configure the CAN callback functions */
    (*rom)->pCAND->config_calb((CAN_CALLBACKS*)&callbacks);

    g_pipe = parent;
    g_rx_instance = new CanRxFlow();
    g_tx_instance = new CanPipeMember();
    parent->RegisterMember(g_tx_instance);

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
#endif // if 0
