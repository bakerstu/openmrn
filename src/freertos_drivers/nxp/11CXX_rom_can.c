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
 * \file 11CXX_rom_can.c
 * This file implements a CAN driver on top of the mbed library. Currently it
 * is tested on the LPC23xx processors.
 *
 * @author Balazs Racz
 * @date 29 April 2013
 */


#ifdef TARGET_LPC11Cxx

#include "freertos_drivers/nxp/11CXX_rom_driver_CAN.h"
#include "can.h"

typedef	struct _ROM {
   const unsigned p_usbd;
   const unsigned p_clib;
   const CAND *pCAND;
} ROM;

/** Pointer to the ROM call structures. */
ROM **rom = (ROM **)0x1fff1ff8;

static const int RX_MSG_OBJ_NUM = 1;
static const int TX_MSG_OBJ_NUM = 2;

/** Private data for this implementation of CAN */
typedef struct lpc11crom_can_priv
{
    CanPriv canPriv; /**< common private data */
    char txPending; /**< transmission currently pending */
} LPC11CRomCanPriv;

/** private data for the can device */
static LPC11CRomCanPriv can_private[1] =
{
    {
        .txPending = 0
    }
};

/** Overrides the system's weak interrupt handler and calls the builtin ROM interrupt handler. */
void CAN_IRQHandler (void){
  (*rom)->pCAND->isr();
}

static int lpc11crom_can_init(devtab_t *dev);
static void ignore_dev_function(devtab_t *dev);
static void lpc11crom_can_tx_msg(devtab_t *dev);

static void CAN_rx(uint8_t msg_obj_num);
static void CAN_tx(uint8_t msg_obj_num);
static void CAN_error(uint32_t error_info);

/** Function pointer table to pass to the ROM drivers with callbacks. */
static const CAN_CALLBACKS callbacks = {
   CAN_rx,
   CAN_tx,
   CAN_error,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
};


/**  Clock initialization constants for 125 kbaud */
const uint32_t ClkInitTable125[2] = {
    0x00000000UL, // CANCLKDIV
    0x00001C57UL  // CAN_BTR
};

/**  Clock initialization constants for 250 kbaud */
static const uint32_t ClkInitTable250[2] = {
    0x00000000UL, // CANCLKDIV
    0x00001C4BUL  // CAN_BTR
};

/** initialize the device 
 * @param dev device to initialize
 * @return 0 upon success
 */
static int lpc11crom_can_init(devtab_t *dev)
{
    /* Initialize the CAN controller */
    (*rom)->pCAND->init_can((uint32_t*) &ClkInitTable250[0], 1);
    /* Configure the CAN callback functions */
    (*rom)->pCAND->config_calb((CAN_CALLBACKS*) &callbacks);

    /* Enable the CAN Interrupt */
    NVIC_EnableIRQ(CAN_IRQn);

    /* Configures msgobj 1 to receive all extended frames. */
    CAN_MSG_OBJ msg_obj;
    msg_obj.msgobj = RX_MSG_OBJ_NUM;
    msg_obj.mode_id = 0x000 | CAN_MSGOBJ_EXT;
    msg_obj.mask = 0x000;
    msg_obj.dlc = 0x000;
    (*rom)->pCAND->config_rxmsgobj(&msg_obj);

    LPC11CRomCanPriv *priv = (LPC11CRomCanPriv*)dev->priv;
    priv->canPriv.enable = ignore_dev_function;
    priv->canPriv.disable = ignore_dev_function;
    priv->canPriv.tx_msg = lpc11crom_can_tx_msg;
    return can_init(dev);
}

/** Empty device function. Does nothing.
 * @param dev device
 */
static void ignore_dev_function(devtab_t *dev) {}

static void send_frame(struct can_frame *can_frame)
{
    CAN_MSG_OBJ msg_obj;
    msg_obj.msgobj  = TX_MSG_OBJ_NUM;
    msg_obj.mode_id = can_frame->can_id |
        (can_frame->can_rtr ? CAN_MSGOBJ_RTR : 0) |
        (can_frame->can_eff ? CAN_MSGOBJ_EXT : 0);
    msg_obj.mask    = 0x0;
    msg_obj.dlc     = can_frame->can_dlc;
    memcpy(msg_obj.data, can_frame->data, can_frame->can_dlc);
    (*rom)->pCAND->can_transmit(&msg_obj);
}

/** Try and transmit a message. Does nothing if there is no message to transmit
 *  or no write buffers to transmit via.
 * @param dev device to transmit message on
 */
static void lpc11crom_can_tx_msg(devtab_t *dev)
{
    LPC11CRomCanPriv *priv = (LPC11CRomCanPriv*)dev->priv;
    if (priv->txPending) return;

    struct can_frame can_frame;
    taskENTER_CRITICAL();
    if (os_mq_timedreceive(priv->canPriv.txQ, &can_frame, 0) != OS_MQ_NONE)
    {
        return;
    }
    priv->txPending = 1;
    send_frame(&can_frame);
    taskEXIT_CRITICAL();
}

/** CAN receive callback. Called by the ROM can driver in an ISR context.
    @param msg_obj_num the number of CAN buffer that has the new frame.
*/
void CAN_rx(uint8_t msg_obj_num)
{
    CAN_MSG_OBJ msg_obj;
    /* Determine which CAN message has been received */
    msg_obj.msgobj = msg_obj_num;

    /* Now load up the msg_obj structure with the CAN message */
    (*rom)->pCAND->can_receive(&msg_obj);

    struct can_frame can_frame;
    can_frame.can_id = msg_obj.mode_id & ((1<<30) - 1);
    can_frame.can_rtr = (msg_obj.mode_id & CAN_MSGOBJ_RTR) ? 1 : 0;
    can_frame.can_eff = (msg_obj.mode_id & CAN_MSGOBJ_EXT) ? 1 : 0;
    can_frame.can_err = 0;
    can_frame.can_dlc = msg_obj.dlc;
    memcpy(can_frame.data, msg_obj.data, msg_obj.dlc);
    int woken = 0;
    if (os_mq_send_from_isr(can_private[0].canPriv.rxQ, &can_frame, &woken)
        == OS_MQ_FULL)
    {
        can_private[0].canPriv.overrunCount++;
    }
    if (woken)
    {
      portYIELD();
    }
}

/** CAN transmit callback. Called by the ROM can driver in an ISR context.
    @param msg_obj_num the number of CAN buffer that finished transmission.
*/
void CAN_tx(uint8_t msg_obj_num){
    // If we don't know the msg object number, let's not do anything.
    if (msg_obj_num != TX_MSG_OBJ_NUM) return;
    struct can_frame can_frame;
    int woken = 0;
    if (os_mq_receive_from_isr(can_private[0].canPriv.txQ, &can_frame, &woken)
        != OS_MQ_NONE)
    {
        can_private[0].txPending = 0;
        return;
    }
    send_frame(&can_frame);

    if (woken)
    {
      portYIELD();
    }
}

/** CAN error callback. Called by the ROM can driver in an ISR context.
    @param error_info defines what kind of error occured on the bus.
*/
void CAN_error(uint32_t error_info)
{
    return;
}

/** Device table entry for can device */
static CAN_DEVTAB_ENTRY(can0, "/dev/can0", lpc11crom_can_init, &can_private[0]);

#else
#error You need to define TARGET_LPC11Cxx if you want to compiple its rom driver.
#endif
