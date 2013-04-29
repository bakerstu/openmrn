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
 * @date 29 April 2013
 */


#ifdef TARGET_LPC11Cxx

#include "11CXX_rom_driver_CAN.h"

/** Pointer to the ROM call structures. */
ROM **rom = (ROM **)0x1fff1ff8;

/** Private data for this implementation of CAN */
typedef struct lpc11crom_can_priv
{
    CanPriv canPriv; /**< common private data */
    char txPending; /**< transmission currently pending */
    char can_data[8];
    void interrupt();
} LPC11CRomCanPriv;

/** private data for the can device */
static LPC11CRomCanPriv can_private[1] =
{
    {
	CanPriv(),
        0
    }
};

static int lpc11crom_can_init(devtab_t *dev);
static void ignore_dev_function(devtab_t *dev) {}
static void lpc11crom_can_tx_msg(devtab_t *dev);

/** Overrides the system's weak interrupt handler and calls the builtin ROM interrupt handler. */
void CAN_IRQHandler (void){
  (*rom)->pCAND->isr();
}

/** Function pointer table to pass to the ROM drivers with callbacks. */
CAN_CALLBACKS callbacks = {
   CAN_rx,
   CAN_tx,
   CAN_error,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
};


/*  125 kbaud */
 uint32_t ClkInitTable125[2] = {
  0x00000000UL, // CANCLKDIV
  0x00001C57UL  // CAN_BTR
};

// 250 kbaud
uint32_t ClkInitTable250[2] = {
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
    (*rom)->pCAND->init_can(&ClkInitTable250[0], 1);
    /* Configure the CAN callback functions */
    (*rom)->pCAND->config_calb(&callbacks);

    /* Enable the CAN Interrupt */
    NVIC_EnableIRQ(CAN_IRQn);

    /* Configures msgobj 1 to receive all extended frames. */
    CAN_MSG_OBJ msg_obj;
    msg_obj.msgobj = 1;
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


static void send_frame(const struct can_frame &can_frame)
{
    CAN_MSG_OBJ msg_obj;
    msg_obj.msgobj  = 2;
    msg_obj.mode_id = can_frame.can_id |
        (can_frame.can_rtr ? CAN_MSGOBJ_RTR : 0) |
        (can_frame.can_eff ? CAN_MSGOBJ_EXT : 0);
    msg_obj.mask    = 0x0;
    msg_obj.dlc     = can_frame.can_dlc;
    memcpy(msg_obj.data, can_frame.data, can_frame.dlc);
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
    // TODO(balazs.racz): think about how we could do with a shorter critical
    // section. The problem is that an ISR might decide to send off the next
    // frame ahead of us.
    taskENTER_CRITICAL();
    if (os_mq_timedreceive(priv->canPriv.txQ, &can_frame, 0) != OS_MQ_NONE)
    {
	return;
    }
    priv->txPending = 1;
    taskEXIT_CRITICAL();
    send_frame(can_frame);
}


void CAN_rx(uint8_t msg_obj_num)
{
    CAN_MSG_OBJ msg_obj;
    /* Determine which CAN message has been received */
    msg_obj.msgobj = msg_obj_num;

    /* Now load up the msg_obj structure with the CAN message */
    (*rom)->pCAND->can_receive(&msg_obj);

    struct can_frame can_frame;
    can_frame.can_id = msg_obj.;
    can_frame.can_rtr = msg.type == CANRemote ? 1 : 0;
    can_frame.can_eff = msg.format == CANStandard ? 0 : 1;
    can_frame.can_err = 0;
    can_frame.can_dlc = msg.len;
    memcpy(can_frame.data, msg.data, msg.len);
    if (os_mq_send_from_isr(canPriv.rxQ, &can_frame) == OS_MQ_FULL)
    {
        canPriv.overrunCount++;
    }
  
    if (msg_obj_num == 1)
    {
        /* Simply transmit CAN frame (echo) with with ID +0x100 via buffer 2 */
        msg_obj.msgobj = 2;
        msg_obj.mode_id += 0x100;
        (*rom)->pCAND->can_transmit(&msg_obj);
    }

  return;
}

/*  CAN transmit callback */
/*  Function is executed by the Callback handler after
    a CAN message has been transmitted */
void CAN_tx(uint8_t msg_obj_num){
  return;
}

/*  CAN error callback */
/*  Function is executed by the Callback handler after
    an error has occured on the CAN bus */
void CAN_error(uint32_t error_info){
  LPC_GPIO0->DATA ^= (1<<7);
  return;
}



#else

#error You need to define TARGET_LPC11Cxx if you want to compiple its rom driver.

#endif

#ifdef TARGET_LPC2368
/** mbed CAN implementation object */
CAN can2(P0_4, P0_5);
#endif


/** initialize the device 
 * @param dev device to initialize
 * @return 0 upon success
 */
static int mbed_can_init(devtab_t *dev)
{
    MbedCanPriv *priv = (MbedCanPriv*)dev->priv;
    //priv->can->frequency(125000);
    priv->can->frequency(250000);
    priv->can->attach(priv, &MbedCanPriv::interrupt);
    
    priv->canPriv.enable = ignore_dev_function;
    priv->canPriv.disable = ignore_dev_function;
    priv->canPriv.tx_msg = mbed_can_tx_msg;
    return can_init(dev);
}

/** Empty device function. Does nothing.
 * @param dev device
 */
static void ignore_dev_function(devtab_t *dev) {}

/** Try and transmit a message. Does nothing if there is no message to transmit
 *  or no write buffers to transmit via.
 * @param dev device to transmit message on
 */
static void mbed_can_tx_msg(devtab_t *dev)
{
    MbedCanPriv *priv = (MbedCanPriv*)dev->priv;
    if (priv->txPending) return;
#ifdef TARGET_LPC2368
    if (!(*priv->SR & 0x4)) return;
#endif

    struct can_frame can_frame;
    // TODO(balazs.racz): think about how we could do with a shorter critical
    // section. The problem is that an ISR might decide to send off the next
    // frame ahead of us.
    taskENTER_CRITICAL();
    if (os_mq_timedreceive(priv->canPriv.txQ, &can_frame, 0) != OS_MQ_NONE)
    {
	return;
    }
    CANMessage msg(can_frame.can_id,
		   (const char*) can_frame.data,
		   can_frame.can_dlc,
		   can_frame.can_rtr ? CANRemote : CANData,
		   can_frame.can_eff ? CANExtended : CANStandard);
    if (!priv->can->write(msg))
    {
	// NOTE(balazs.racz): This means that the CAN layer didn't find an
	// available TX buffer to send the CAN message. However, since
	// txPending == 0 at this point, that can only happen if someone else
	// was also writing frames to this CAN device. We won't handle that
	// case now.
	priv->canPriv.overrunCount++;
    }
    priv->txPending = 1;
    taskEXIT_CRITICAL();
}

/** Handler for CAN device. Called from the mbed irq handler. */
void MbedCanPriv::interrupt() {
    CANMessage msg;
    if (can->read(msg))
    {
        struct can_frame can_frame;
        can_frame.can_id = msg.id;
        can_frame.can_rtr = msg.type == CANRemote ? 1 : 0;
        can_frame.can_eff = msg.format == CANStandard ? 0 : 1;
        can_frame.can_err = 0;
        can_frame.can_dlc = msg.len;
        memcpy(can_frame.data, msg.data, msg.len);
        if (os_mq_send_from_isr(canPriv.rxQ, &can_frame) == OS_MQ_FULL)
        {
            canPriv.overrunCount++;
        }
    }
#ifdef TARGET_LPC2368
    if (*SR & 0x4)
    {
	// Transmit buffer 1 empty => transmit finished.
        struct can_frame can_frame;
        if (os_mq_receive_from_isr(canPriv.txQ, &can_frame) == OS_MQ_NONE)
	{
	    CANMessage msg(can_frame.can_id,
			   (const char*) can_frame.data,
			   can_frame.can_dlc,
			   can_frame.can_rtr ? CANRemote : CANData,
			   can_frame.can_eff ? CANExtended : CANStandard);
	    if (can->write(msg))
	    {
		txPending = 1;
	    }
	    else
	    {
		// NOTE(balazs.racz): This is an inconsistency -- if *SR&0x4
		// then TX1 buffer is empty, so if write fails, then... a task
		// switch occured while serving an interrupt handler?
		canPriv.overrunCount++;
		txPending = 0;
	    } 
	}
	else
	{
	    txPending = 0;
	}
    }
#else
#error you need to define how to figure out whether the transmit buffer is empty.
#endif
    // TODO(balazs.racz): need to see what needs to be done for acking the interrupt, depending on what the interrupt fnction attributes say.
} 

/** Device table entry for can device */
static CAN_DEVTAB_ENTRY(can1, "/dev/can1", mbed_can_init, &can_private[1]);
