/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file StellarisCan.cxx
 * This file implements a can device driver layer specific to stellarisware.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_can.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "StellarisDev.hxx"

/** Instance pointers help us get context from the interrupt handler(s) */
static StellarisCan *instances[2] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 */
StellarisCan::StellarisCan(const char *name, unsigned long base)
    : Can(name),
      base(base),
      interrupt(0),
      txPending(false)
{
    switch (base)
    {
        default:
            HASSERT(0);
        case CAN0_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
            interrupt = INT_CAN0;
            instances[0] = this;
            break;
        case CAN1_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
            interrupt = INT_CAN1;
            instances[1] = this;
            break;
    }
    
    MAP_CANInit(base);
    MAP_CANBitRateSet(base, MAP_SysCtlClockGet(), 125000);
    MAP_CANIntEnable(base, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    
    tCANMsgObject can_message;
    can_message.ulMsgID = 0;
    can_message.ulMsgIDMask = 0;
    can_message.ulFlags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    can_message.ulMsgLen = 8;
    MAP_CANMessageSet(base, 1, &can_message, MSG_OBJ_TYPE_RX);
}

/** Enable use of the device.
 */
void StellarisCan::enable()
{
    MAP_IntEnable(interrupt);
    MAP_CANEnable(base);
}

/** Disable use of the device.
 */
void StellarisCan::disable()
{
    MAP_IntDisable(interrupt);
    MAP_CANDisable(base);
}

/* Try and transmit a message.
 */
void StellarisCan::tx_msg()
{
    if (txPending == false)
    {
        struct can_frame can_frame;
        if (os_mq_timedreceive(txQ, &can_frame, 0) == OS_MQ_NONE)
        {
            /* load the next message to transmit */
            tCANMsgObject can_message;
            can_message.ulMsgID = can_frame.can_id;
            can_message.ulMsgIDMask = 0;
            can_message.ulFlags = MSG_OBJ_TX_INT_ENABLE;
            if (can_frame.can_eff)
            {
                can_message.ulFlags |= MSG_OBJ_EXTENDED_ID;
            }
            if (can_frame.can_rtr)
            {
                can_message.ulFlags |= MSG_OBJ_REMOTE_FRAME;
            }
            can_message.ulMsgLen = can_frame.can_dlc;
            can_message.pucMsgData = data;
            memcpy(data, can_frame.data, can_frame.can_dlc);
            
            MAP_IntDisable(interrupt);
            MAP_CANMessageSet(base, 2, &can_message, MSG_OBJ_TYPE_TX);
            txPending = true;
            MAP_IntEnable(interrupt);
        }
    }
}

/** Common interrupt handler for all CAN devices.
 */
void StellarisCan::interrupt_handler()
{
    uint32_t status = MAP_CANIntStatus(base, CAN_INT_STS_CAUSE);
    int woken = false;

    if (status == CAN_INT_INTID_STATUS)
    {
        status = MAP_CANStatusGet(base, CAN_STS_CONTROL);
        /* some error occured */
        if (status & CAN_STATUS_BUS_OFF)
        {
            /* bus off error condition */
        }
        if (status & CAN_STATUS_EWARN)
        {
            /* One of the error counters has exceded a value of 96 */
        }
        if (status & CAN_STATUS_EPASS)
        {
            /* In error passive state */
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
    }
    else if (status == 1)
    {
        /* rx data received */
        tCANMsgObject can_message;
        uint8_t data[8];
        can_message.pucMsgData = data;

        /* Read a message from CAN and clear the interrupt source */
        MAP_CANMessageGet(base, 1, &can_message, 1 /* clear interrupt */);
        
        struct can_frame can_frame;
        can_frame.can_id = can_message.ulMsgID;
        can_frame.can_rtr = (can_message.ulFlags & MSG_OBJ_REMOTE_FRAME) ? 1 : 0;
        can_frame.can_eff = (can_message.ulFlags & MSG_OBJ_EXTENDED_ID) ? 1 : 0;
        can_frame.can_err = 0;
        can_frame.can_dlc = can_message.ulMsgLen;
        memcpy(can_frame.data, data, can_message.ulMsgLen);
        if (os_mq_send_from_isr(rxQ, &can_frame, &woken) == OS_MQ_FULL)
        {
            overrunCount++;
        }
        /* wakeup anyone waiting for read active */
        if (read_callback)
        {
            read_callback(readContext, &woken);
            read_callback = NULL;
            readContext = NULL;
        }
    }
    else if (status == 2)
    {
        /* tx complete */
        MAP_CANIntClear(base, 2);

        struct can_frame can_frame;
        if (os_mq_receive_from_isr(txQ, &can_frame, &woken) == OS_MQ_NONE)
        {
            /* load the next message to transmit */
            tCANMsgObject can_message;
            can_message.ulMsgID = can_frame.can_id;
            can_message.ulMsgIDMask = 0;
            can_message.ulFlags = MSG_OBJ_TX_INT_ENABLE;
            if (can_frame.can_eff)
            {
                can_message.ulFlags |= MSG_OBJ_EXTENDED_ID;
            }
            if (can_frame.can_rtr)
            {
                can_message.ulFlags |= MSG_OBJ_REMOTE_FRAME;
            }
            can_message.ulMsgLen = can_frame.can_dlc;
            can_message.pucMsgData = data;
            memcpy(data, can_frame.data, can_frame.can_dlc);
            
            MAP_CANMessageSet(base, 2, &can_message, MSG_OBJ_TYPE_TX);
            /* wakeup anyone waiting for write active */
            if (write_callback)
            {
                write_callback(writeContext, &woken);
                write_callback = NULL;
                writeContext = NULL;
            }
        }
        else
        {
            /* no more messages pending transmission */
            txPending = false;
        }
    }
    os_isr_exit_yield_test(woken);
}

/** This is the interrupt handler for the can0 device.
 */
void can0_interrupt_handler(void)
{
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}

#if 0
/** This is the interrupt handler for the can0 device.
 */
void can1_interrupt_handler(void)
{
    can_interrupt_handler(&can1);
}
#endif


