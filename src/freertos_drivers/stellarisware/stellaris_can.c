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
 * \file stellaris_can.c
 * This file implements a can device driver layer specific to stellarisware.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#include <stdlib.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_can.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "can.h"

/* prototypes */
static int stellaris_can_init(devtab_t *dev);
static void stellaris_can_enable(devtab_t *dev);
static void stellaris_can_disable(devtab_t *dev);
static int stellaris_can_tx_msg(devtab_t *dev, uint32_t id, char eff, char rtr, uint8_t dlc, const uint8_t *data);
static void stellaris_can_lock(devtab_t *);
static void stellaris_can_unlock(devtab_t *);

/** Private data for this implementation of CAN
 */
typedef struct stellaris_can_priv
{
    CanPriv canPriv; /**< common private data */
    unsigned long base; /**< base address of this device */
    unsigned long interrupt; /**< interrupt of this device */
    uint8_t data[8]; /**< transmit data */
    char txPending; /**< transmission currently pending */
} StellarisCanPriv;

/** private data for the can device */
static StellarisCanPriv can_private[2] =
{
    {
        .base = CAN0_BASE,
        .interrupt = INT_CAN0,
        .txPending = 0
    },
    {
        .base = CAN1_BASE,
        .interrupt = INT_CAN1,
        .txPending = 0
    }
};

/** Device table entry for can device */
CAN_DEVTAB_ENTRY(can0, "/dev/can0", stellaris_can_init, &can_private[0]);
/** Device table entry for can device */
CAN_DEVTAB_ENTRY(can1, "/dev/can1", stellaris_can_init, &can_private[1]);

/** intitailize the device 
 * @parem dev device to initialize
 * @return 0 upon success
 */
static int stellaris_can_init(devtab_t *dev)
{
    StellarisCanPriv *priv = dev->priv;
    
    switch (priv->base)
    {
        default:
            return -1;
        case CAN0_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
            break;
    }
    
    MAP_CANInit(priv->base);
    MAP_CANBitRateSet(priv->base, MAP_SysCtlClockGet(), 125000);
    MAP_CANIntEnable(priv->base, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    
    tCANMsgObject can_message;
    can_message.ulMsgID = 0;
    can_message.ulMsgIDMask = 0;
    can_message.ulFlags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    can_message.ulMsgLen = 8;
    MAP_CANMessageSet(priv->base, 1, &can_message, MSG_OBJ_TYPE_RX);
    
    
    
    priv->canPriv.enable = stellaris_can_enable;
    priv->canPriv.disable = stellaris_can_disable;
    priv->canPriv.tx_msg = stellaris_can_tx_msg;
    priv->canPriv.lock = stellaris_can_lock;
    priv->canPriv.unlock = stellaris_can_unlock;
    
    return can_init(dev);
}

/** Enable use of the device interrupts.
 * @param dev device to enable
 */
static void stellaris_can_enable(devtab_t *dev)
{
    StellarisCanPriv *priv = dev->priv;
    MAP_IntEnable(priv->interrupt);
    MAP_CANEnable(priv->base);
}

/** Disable use of the device interrupts.
 * @param dev device to disable
 */
static void stellaris_can_disable(devtab_t *dev)
{
    StellarisCanPriv *priv = dev->priv;
    MAP_CANDisable(priv->base);
    MAP_IntDisable(priv->interrupt);
}

/** Mutual exclusion lock for device.
 * @param dev device to lock
 */
static void stellaris_can_lock(devtab_t *dev)
{
    StellarisCanPriv *priv = dev->priv;
    MAP_IntDisable(priv->interrupt);
}

/** Mutual exclusion unlock for device.
 * @param dev device to unlock
 */
static void stellaris_can_unlock(devtab_t *dev)
{
    StellarisCanPriv *priv = dev->priv;
    MAP_IntEnable(priv->interrupt);
}

/* Try and transmit a message.  Call with device locked.
 * @param dev device to transmit message on
 * @param id CAN identifier
 * @param eff set if an extended frame format, else 0
 * @param rtr set if a remote frame, else 0
 * @param dlc data length code, number of data bytes
 * @param data pointer to an array of data
 * @return 1 if message put into transmit buffer, else 0
 */
static int stellaris_can_tx_msg(devtab_t *dev, uint32_t id, char eff, char rtr, uint8_t dlc, const uint8_t *data)
{
    StellarisCanPriv *priv = dev->priv;
    
    if (priv->txPending == 0)
    {
        tCANMsgObject can_message;
        can_message.ulMsgID = id;
        can_message.ulMsgIDMask = 0;
        can_message.ulFlags = MSG_OBJ_TX_INT_ENABLE;
        can_message.ulMsgLen = dlc;
        can_message.pucMsgData = priv->data;
        memcpy(priv->data, data, dlc);
        
        MAP_CANMessageSet(priv->base, 2, &can_message, MSG_OBJ_TYPE_TX);
        priv->txPending = 1;

        return 1;
    }
    else
    {
        return 0;
    }
    
}

/** Common interrupt handler for all CAN devices.
 * @param dev device to handle and interrupt for
 */
static void can_interrupt_handler(devtab_t *dev)
{
    StellarisCanPriv *priv = dev->priv;
    
    uint32_t status = MAP_CANIntStatus(priv->base, CAN_INT_STS_CAUSE);

    if (status == CAN_INT_INTID_STATUS)
    {
        status = MAP_CANStatusGet(priv->base, CAN_STS_CONTROL);
        /* some error occured */
    }
    if (status == 1)
    {
        /* rx data received */
        tCANMsgObject can_message;
        uint8_t data[8];
        can_message.pucMsgData = data;

        /* Read a message from CAN and clear the interrupt source */
        MAP_CANMessageGet(priv->base, 1, &can_message, 1);
        
        can_rx_msg(dev,
                   can_message.ulMsgID,
                   can_message.ulFlags & MSG_OBJ_EXTENDED_ID,
                   can_message.ulFlags & MSG_OBJ_REMOTE_FRAME,
                   can_message.ulMsgLen,
                   data);
    }
    if (status == 2)
    {
        /* tx complete */
        MAP_CANIntClear(priv->base, 2);
        priv->txPending = 0;
        can_tx_ready(dev);
    }
}

/** This is the interrupt handler for the can0 device.
 */
void can0_interrupt_handler(void)
{
    can_interrupt_handler(&can0);
}

/** This is the interrupt handler for the can0 device.
 */
void can1_interrupt_handler(void)
{
    can_interrupt_handler(&can1);
}

