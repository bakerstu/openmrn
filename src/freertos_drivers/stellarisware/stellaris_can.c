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
 * \file stellaris_can.c
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

#include "can.h"

/* prototypes */
static int stellaris_can_init(devtab_t *dev);
static void stellaris_can_enable(devtab_t *dev);
static void stellaris_can_disable(devtab_t *dev);
static void stellaris_can_tx_msg(devtab_t *dev);

/** Private data for this implementation of CAN
 */
typedef struct stellaris_can_priv {
  CanPriv canPriv;         /**< common private data */
  unsigned long base;      /**< base address of this device */
  unsigned long interrupt; /**< interrupt of this device */
  uint8_t data[8];         /**< transmit data */
  char txPending;          /**< transmission currently pending */
} StellarisCanPriv;

/** private data for the can device */
static StellarisCanPriv can_private[2] = {
    {.base = CAN0_BASE, .interrupt = INT_CAN0, .txPending = 0},
    {.base = CAN1_BASE, .interrupt = INT_CAN1, .txPending = 0}};

/** Device table entry for can device */
static CAN_DEVTAB_ENTRY(can0, "/dev/can0", stellaris_can_init, &can_private[0]);
/** Device table entry for can device */
// static CAN_DEVTAB_ENTRY(can1, "/dev/can1", stellaris_can_init,
// &can_private[1]);

/** intitailize the device
 * @parem dev device to initialize
 * @return 0 upon success
 */
static int stellaris_can_init(devtab_t *dev) {
  StellarisCanPriv *priv = dev->priv;

  switch (priv->base) {
    default:
      return -1;
    case CAN0_BASE:
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
      break;
    case CAN1_BASE:
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
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

  return can_init(dev);
}

/** Enable use of the device.
 * @param dev device to enable
 */
static void stellaris_can_enable(devtab_t *dev) {
  StellarisCanPriv *priv = dev->priv;
  MAP_IntEnable(priv->interrupt);
  MAP_CANEnable(priv->base);
}

/** Disable use of the device.
 * @param dev device to disable
 */
static void stellaris_can_disable(devtab_t *dev) {
  StellarisCanPriv *priv = dev->priv;
  MAP_IntDisable(priv->interrupt);
  MAP_CANDisable(priv->base);
}

/* Try and transmit a message.
 * @param dev device to transmit message on
 */
static void stellaris_can_tx_msg(devtab_t *dev) {
  StellarisCanPriv *priv = dev->priv;

  if (priv->txPending == 0) {
    struct can_frame can_frame;
    if (os_mq_timedreceive(priv->canPriv.txQ, &can_frame, 0) == OS_MQ_NONE) {
      /* load the next message to transmit */
      tCANMsgObject can_message;
      can_message.ulMsgID = can_frame.can_id;
      can_message.ulMsgIDMask = 0;
      can_message.ulFlags = MSG_OBJ_TX_INT_ENABLE;
      if (can_frame.can_eff) {
        can_message.ulFlags |= MSG_OBJ_EXTENDED_ID;
      }
      if (can_frame.can_rtr) {
        can_message.ulFlags |= MSG_OBJ_REMOTE_FRAME;
      }
      can_message.ulMsgLen = can_frame.can_dlc;
      can_message.pucMsgData = priv->data;
      memcpy(priv->data, can_frame.data, can_frame.can_dlc);

      MAP_IntDisable(priv->interrupt);
      MAP_CANMessageSet(priv->base, 2, &can_message, MSG_OBJ_TYPE_TX);
      priv->txPending = 1;
      MAP_IntEnable(priv->interrupt);
    }
  }
}

/** Common interrupt handler for all CAN devices.
 * @param dev device to handle and interrupt for
 */
static void can_interrupt_handler(devtab_t *dev) {
  StellarisCanPriv *priv = dev->priv;
  int woken = false;

  uint32_t status = MAP_CANIntStatus(priv->base, CAN_INT_STS_CAUSE);

  if (status == CAN_INT_INTID_STATUS) {
    status = MAP_CANStatusGet(priv->base, CAN_STS_CONTROL);
    /* some error occured */
  }
  if (status == 1) {
    /* rx data received */
    tCANMsgObject can_message;
    uint8_t data[8];
    can_message.pucMsgData = data;

    /* Read a message from CAN and clear the interrupt source */
    MAP_CANMessageGet(priv->base, 1, &can_message, 1);

    struct can_frame can_frame;
    can_frame.can_id = can_message.ulMsgID;
    can_frame.can_rtr = (can_message.ulFlags & MSG_OBJ_REMOTE_FRAME) ? 1 : 0;
    can_frame.can_eff = (can_message.ulFlags & MSG_OBJ_EXTENDED_ID) ? 1 : 0;
    can_frame.can_err = 0;
    can_frame.can_dlc = can_message.ulMsgLen;
    memcpy(can_frame.data, data, can_message.ulMsgLen);
    if (os_mq_send_from_isr(priv->canPriv.rxQ, &can_frame, &woken) ==
        OS_MQ_FULL) {
      priv->canPriv.overrunCount++;
    }
  }
  if (status == 2) {
    /* tx complete */
    MAP_CANIntClear(priv->base, 2);

    struct can_frame can_frame;
    if (os_mq_receive_from_isr(priv->canPriv.txQ, &can_frame, &woken) ==
        OS_MQ_NONE) {
      /* load the next message to transmit */
      tCANMsgObject can_message;
      can_message.ulMsgID = can_frame.can_id;
      can_message.ulMsgIDMask = 0;
      can_message.ulFlags = MSG_OBJ_TX_INT_ENABLE;
      if (can_frame.can_eff) {
        can_message.ulFlags |= MSG_OBJ_EXTENDED_ID;
      }
      if (can_frame.can_rtr) {
        can_message.ulFlags |= MSG_OBJ_REMOTE_FRAME;
      }
      can_message.ulMsgLen = can_frame.can_dlc;
      can_message.pucMsgData = priv->data;
      memcpy(priv->data, can_frame.data, can_frame.can_dlc);

      MAP_CANMessageSet(priv->base, 2, &can_message, MSG_OBJ_TYPE_TX);
    } else {
      /* no more messages pending transmission */
      priv->txPending = 0;
    }
  }
}

/** This is the interrupt handler for the can0 device.
 */
void can0_interrupt_handler(void) { can_interrupt_handler(&can0); }

#if 0
/** This is the interrupt handler for the can0 device.
 */
void can1_interrupt_handler(void)
{
    can_interrupt_handler(&can1);
}
#endif
