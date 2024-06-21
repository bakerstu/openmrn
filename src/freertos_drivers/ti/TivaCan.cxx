/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * \file TivaCan.cxx
 * This file implements a can device driver layer specific to TivaWare.
 *
 * @author Stuart W. Baker
 * @date 6 May 2014
 */

#include <stdint.h>

#include "can_ioctl.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "nmranet_config.h"

#include "TivaDev.hxx"

/** Instance pointers help us get context from the interrupt handler(s) */
static TivaCan *instances[2] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 * @param interrupt interrupt number of this device
 */
TivaCan::TivaCan(const char *name, unsigned long base, uint32_t interrupt)
    : Can(name)
    , base(base)
    , interrupt(interrupt)
    , txPending(false)
    , canState(CAN_STATE_STOPPED)
{
    switch (base)
    {
        default:
            HASSERT(0);
        case CAN0_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
            instances[0] = this;
            break;
        case CAN1_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
            instances[1] = this;
            break;
    }

    MAP_CANInit(base);

    uint32_t ftq = config_nmranet_can_bitrate() * 16;
    // If this fails, the CAN bit timings do not support this CPU clock. The
    // CPU clock has to be an even number of MHz.
    HASSERT(cm3_cpu_clock_hz % ftq == 0);
    
    /* Nominal 2 MHz quantum clock
     * SyncSeg = 1 TQ
     * PropSeg = 7 TQ
     * PS1 = 4 TQ
     * PS2 = 4 TQ
     * Bit total = 16 TQ
     * Baud = 125 kHz
     * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
     * SJW = 4 TQ
     *
     * Oscillator Tolerance:
     *     4 / (2 * ((13 * 16) - 4)) = 0.980%
     *     4 / (20 * 16) = 1.250%
     *     = 0.980%
     */
    tCANBitClkParms clk_params = {
        .ui32SyncPropPhase1Seg = 11, // Sum of PropSeg and PS1 in #TQ
        .ui32Phase2Seg = 4, // PS2 in #TQ
        .ui32SJW = 4,
        .ui32QuantumPrescaler = cm3_cpu_clock_hz / ftq
    };
    MAP_CANBitTimingSet(base, &clk_params);
    MAP_CANIntEnable(base, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    tCANMsgObject can_message;
    can_message.ui32MsgID = 0;
    can_message.ui32MsgIDMask = 0;
    can_message.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    can_message.ui32MsgLen = 8;
    MAP_CANMessageSet(base, 1, &can_message, MSG_OBJ_TYPE_RX);
}

//
// TCAN4550Can::ioctl()
//
int TivaCan::ioctl(File *file, unsigned long int key, unsigned long data)
{
    if (key == SIOCGCANSTATE)
    {
        *((can_state_t *)data) = canState;
        return 0;
    }
    return -EINVAL;
}

/** Enable use of the device.
 */
void TivaCan::enable()
{
    MAP_CANBitRateSet(base, cm3_cpu_clock_hz, config_nmranet_can_bitrate());
    MAP_IntEnable(interrupt);
    // The priority of CAN interrupt is as high as possible while maintaining
    // FreeRTOS compatibility.
    MAP_IntPrioritySet(interrupt, configKERNEL_INTERRUPT_PRIORITY);
    MAP_CANEnable(base);
    // Wait for a successful RX or TX before moving to CAN_STATE_ACTIVE.
    canState = CAN_STATE_STOPPED;
}

/** Disable use of the device.
 */
void TivaCan::disable()
{
    canState = CAN_STATE_STOPPED;
    MAP_IntDisable(interrupt);
    MAP_CANDisable(base);
}

/* Try and transmit a message.
 */
void TivaCan::tx_msg()
{
    if (txPending == false || canState != CAN_STATE_ACTIVE)
    {
        struct can_frame *can_frame;

        if (txBuf->data_read_pointer(&can_frame))
        {
            /* load the next message to transmit */
            tCANMsgObject can_message;
            can_message.ui32MsgID = can_frame->can_id;
            can_message.ui32MsgIDMask = 0;
            can_message.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
            if (can_frame->can_eff)
            {
                can_message.ui32Flags |= MSG_OBJ_EXTENDED_ID;
            }
            if (can_frame->can_rtr)
            {
                can_message.ui32Flags |= MSG_OBJ_REMOTE_FRAME;
            }
            can_message.ui32MsgLen = can_frame->can_dlc;
            /* zero copy data */
            can_message.pui8MsgData = can_frame->data;

            MAP_CANMessageSet(base, 2, &can_message, MSG_OBJ_TYPE_TX);
            txPending = true;
        }
    }
    if (canState != CAN_STATE_ACTIVE)
    {
        txBuf->flush();
        txBuf->signal_condition();
    }
}

/** Common interrupt handler for all CAN devices.
 */
void TivaCan::interrupt_handler()
{
    uint32_t status = MAP_CANIntStatus(base, CAN_INT_STS_CAUSE);
    /// @todo(balazs.racz) make this a static variable in os.c
    int woken = false;

    if (status == CAN_INT_INTID_STATUS)
    {
        bool cancel_queue = false;
        
        status = MAP_CANStatusGet(base, CAN_STS_CONTROL);
        /* some error occured */
        if (status & CAN_STATUS_BUS_OFF)
        {
            /* bus off error condition */
            ++busOffCount;
            canState = CAN_STATE_BUS_OFF;

            cancel_queue = true;

            /* attempt recovery */
            MAP_CANEnable(base);
        }
        if (status & CAN_STATUS_EWARN)
        {
            /* One of the error counters has exceded a value of 96 */
            ++softErrorCount;
        }
        if (status & CAN_STATUS_EPASS)
        {
            /* In error passive state */
            canState = CAN_STATE_BUS_PASSIVE;
            cancel_queue = true;
        }
        if (status & CAN_STATUS_LEC_STUFF)
        {
            /* bit stuffing error occured */
        }
        if (status & CAN_STATUS_LEC_FORM)
        {
            /* format error occured in the fixed format part of the message */
        }
        if (status & CAN_STATUS_LEC_ACK)
        {
            /* a transmit message was not acked */
        }
        if (status & CAN_STATUS_LEC_CRC)
        {
            /* CRC error detected in received message */
        }

        if (cancel_queue)
        {
            /* flush data in the tx pipeline */
            txBuf->flush();
            txPending = false;
            txBuf->signal_condition_from_isr();
        }
    }
    else if (status == 1)
    {
        /* rx data received */
        canState = CAN_STATE_ACTIVE;

        struct can_frame *can_frame;
        if (rxBuf->data_write_pointer(&can_frame))
        {
            /* we have space remaining to buffer up this incoming message */
            tCANMsgObject can_message;
            can_message.pui8MsgData = can_frame->data;
            /* Read a message from CAN and clear the interrupt source */
            MAP_CANMessageGet(base, 1, &can_message, 1 /* clear interrupt */);

            can_frame->can_id = can_message.ui32MsgID;
            can_frame->can_rtr = (can_message.ui32Flags & MSG_OBJ_REMOTE_FRAME) ? 1 : 0;
            can_frame->can_eff = (can_message.ui32Flags & MSG_OBJ_EXTENDED_ID) ? 1 : 0;
            can_frame->can_err = 0;
            can_frame->can_dlc = can_message.ui32MsgLen;
            rxBuf->advance(1);
            ++numReceivedPackets_;
            rxBuf->signal_condition_from_isr();

            /** @todo (Stuart Baker) remove notify logic once we switch over to
             * select()
             */
            if (readableNotify_)
            {
                readableNotify_->notify_from_isr();
                readableNotify_ = nullptr;
            }
        }
        else
        {
            /* ran out of space to buffer, flush incoming message */
            ++overrunCount;
            tCANMsgObject can_message;
            can_message.pui8MsgData = can_frame->data;
            /* Read a message from CAN and clear the interrupt source */
            MAP_CANMessageGet(base, 1, &can_message, 1 /* clear interrupt */);
        }
    }
    else if (status == 2)
    {
        /* tx complete */
        MAP_CANIntClear(base, 2);
        canState = CAN_STATE_ACTIVE;

        /* previous (zero copy) message from buffer no longer needed */
        txBuf->consume(1);
        ++numTransmittedPackets_;
        txBuf->signal_condition_from_isr();

        /** @todo (Stuart Baker) remove notify logic once we switch over to
         * select()
         */
        if (writableNotify_)
        {
            writableNotify_->notify_from_isr();
            writableNotify_= nullptr;
        }

        struct can_frame *can_frame;

        if (txBuf->data_read_pointer(&can_frame))
        {
            /* load the next message to transmit */
            tCANMsgObject can_message;
            can_message.ui32MsgID = can_frame->can_id;
            can_message.ui32MsgIDMask = 0;
            can_message.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
            if (can_frame->can_eff)
            {
                can_message.ui32Flags |= MSG_OBJ_EXTENDED_ID;
            }
            if (can_frame->can_rtr)
            {
                can_message.ui32Flags |= MSG_OBJ_REMOTE_FRAME;
            }
            can_message.ui32MsgLen = can_frame->can_dlc;
            /* zero copy data */
            can_message.pui8MsgData = can_frame->data;

            MAP_CANMessageSet(base, 2, &can_message, MSG_OBJ_TYPE_TX);
        }
        else
        {
            /* no more messages pending transmission */
            txPending = false;
        }
    }
    os_isr_exit_yield_test(woken);
}

extern "C" {
/** This is the interrupt handler for the can0 device.
 */
void can0_interrupt_handler(void)
{
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}

/** This is the interrupt handler for the can1 device.
 */
void can1_interrupt_handler(void)
{
    if (instances[1])
    {
        instances[1]->interrupt_handler();
    }
}

} // extern "C"
