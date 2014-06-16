/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file Stm32Can.cxx
 *
 * This file implements a CAN driver for the STM32F10x microcontrollers using
 * the STM32 peripheral library.
 *
 * @author Balazs Racz
 * @date 15 June 2014
 */

#include "stm32f10x.h"
#include "Can.hxx"

#define USE_OLIMEXINO_STM32

#ifdef USE_OLIMEXINO_STM32
#define RCC_APB2Periph_GPIO_CAN1 RCC_APB2Periph_GPIOB
#define GPIO_Remapping_CAN1 GPIO_Remap1_CAN1
#define GPIO_CAN1 GPIOB
#define GPIO_Pin_CAN1_RX GPIO_Pin_8
#define GPIO_Pin_CAN1_TX GPIO_Pin_9
#endif

extern "C" {
void USB_HP_CAN1_TX_IRQHandler(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
}

class Stm32CanDriver : public Can
{
public:
    Stm32CanDriver(CAN_TypeDef *instance, const char *dev)
        : Can(dev)
        , instance_(instance)
    {
        HASSERT(instance == CAN1);
        init_can_gpio();
        init_can_ctrl();
    }

private:
    void init_can_gpio()
    {
        // Turns on clock for the GPIO port of the canbus.
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CAN1, ENABLE);

        // Configures CAN RX pin
        GPIO_InitTypeDef gp;
        GPIO_StructInit(&gp);
        gp.GPIO_Pin = GPIO_Pin_CAN1_RX;
        gp.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIO_CAN1, &gp);

        // Configures CAN TX pin
        gp.GPIO_Pin = GPIO_Pin_CAN1_TX;
        gp.GPIO_Mode = GPIO_Mode_AF_PP;
        gp.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIO_CAN1, &gp);

        GPIO_PinRemapConfig(GPIO_Remapping_CAN1, ENABLE);

        // Turns on clock for the CAN controller
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    }

    void init_can_ctrl()
    {
        CAN_DeInit(instance_);

        CAN_InitTypeDef CAN_InitStructure;
        CAN_StructInit(&CAN_InitStructure);
        CAN_InitStructure.CAN_TTCM = DISABLE;
        CAN_InitStructure.CAN_ABOM = DISABLE;
        CAN_InitStructure.CAN_AWUM = DISABLE;
        CAN_InitStructure.CAN_NART = DISABLE;
        CAN_InitStructure.CAN_RFLM = DISABLE;
        CAN_InitStructure.CAN_TXFP = DISABLE;
        CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

        /* CAN Baudrate = 1MBps / requested_rate*/
        CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
        CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
        CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
        CAN_InitStructure.CAN_Prescaler =
            4 * 1000000 / config_nmranet_can_bitrate();
        HASSERT(4 * 1000000 / CAN_InitStructure.CAN_Prescaler ==
                config_nmranet_can_bitrate());
        HASSERT(CAN_Init(instance_, &CAN_InitStructure) ==
                CAN_InitStatus_Success);
    }

    void init_can_filter()
    {
        CAN_FilterInitTypeDef CAN_FilterInitStructure;
        CAN_FilterInitStructure.CAN_FilterNumber = 0;
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
        CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
        CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
        CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
        CAN_FilterInit(&CAN_FilterInitStructure);
    }

    void enable() OVERRIDE
    {
        init_can_filter();
        // Sets the CAN interrupt priorities to be compatible with FreeRTOS.
        NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, configKERNEL_INTERRUPT_PRIORITY);
        NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, configKERNEL_INTERRUPT_PRIORITY);
        CAN_ITConfig(instance_, CAN_IT_TME, DISABLE);
        NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
        NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
        CAN_ITConfig(instance_, CAN_IT_FMP0, ENABLE);
    };
    void disable() OVERRIDE
    {
        CAN_ITConfig(instance_, CAN_IT_FMP0, DISABLE);
        CAN_ITConfig(instance_, CAN_IT_TME, DISABLE);
        NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
        NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    };

    void tx_interrupt();
    void rx_interrupt();
    void tx_msg() OVERRIDE;

    friend void USB_HP_CAN1_TX_IRQHandler(void);
    friend void USB_LP_CAN1_RX0_IRQHandler(void);

    CAN_TypeDef *instance_;

    static const uint32_t CAN_TSR_TME_ANY =
        CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;

    /** Converts between our frame format and STM32 peripheral library's frame
     * format. */
    static void fill_tx_message(const struct can_frame &frame, CanTxMsg *msg);
    /** Converts between our frame format and STM32 peripheral library's frame
     * format. */
    static void parse_rx_message(const CanRxMsg &msg, struct can_frame *frame);
};

// static
void Stm32CanDriver::fill_tx_message(const struct can_frame &frame,
                                     CanTxMsg *msg)
{
    if (IS_CAN_FRAME_EFF(frame))
    {
        msg->IDE = CAN_Id_Extended;
        msg->ExtId = GET_CAN_FRAME_ID_EFF(frame);
    }
    else
    {
        msg->IDE = CAN_Id_Standard;
        msg->StdId = GET_CAN_FRAME_ID(frame);
    }
    if (IS_CAN_FRAME_RTR(frame))
    {
        msg->RTR = CAN_RTR_Remote;
    }
    else
    {
        msg->RTR = CAN_RTR_Data;
    }
    msg->DLC = frame.can_dlc;
    memcpy(msg->Data, frame.data, frame.can_dlc);
}

// static
void Stm32CanDriver::parse_rx_message(const CanRxMsg &msg,
                                      struct can_frame *frame)
{
    if (msg.IDE == CAN_Id_Standard)
    {
        CLR_CAN_FRAME_EFF(*frame);
        SET_CAN_FRAME_ID(*frame, msg.StdId);
    }
    else
    {
        SET_CAN_FRAME_EFF(*frame);
        SET_CAN_FRAME_ID_EFF(*frame, msg.ExtId);
    }
    if (msg.RTR == CAN_RTR_Data)
    {
        CLR_CAN_FRAME_RTR(*frame);
    }
    else
    {
        SET_CAN_FRAME_RTR(*frame);
    }
    CLR_CAN_FRAME_ERR(*frame);
    frame->can_dlc = msg.DLC;
    memcpy(frame->data, msg.Data, msg.DLC);
}

/** Try and transmit a message. Does nothing if there is no message to transmit
 *  or no write buffers to transmit via.
 * @param dev device to transmit message on
 */
void Stm32CanDriver::tx_msg()
{
    if (instance_->IER & CAN_IT_TME)
    {
        // We are waiting for a transmit interrupt to come in. That interrupt
        // will send off our frame.
        return;
    }
    // Now: no transmit interrupt is pending. This means that there must be a
    // transmit message buffer free.
    HASSERT(instance_->TSR & CAN_TSR_TME_ANY);

    struct can_frame can_frame;
    if (!get_tx_msg(&can_frame))
    {
        // No frame -- probably our frame was sent off by a racing ISR.
        return;
    }
    CanTxMsg msg;
    fill_tx_message(can_frame, &msg);
    if (CAN_Transmit(instance_, &msg) == CAN_TxStatus_NoMailBox)
    {
        DIE("Internal inconsistency: no interrupt pending but no transmit "
            "buffer free");
    }
    if ((instance_->TSR & CAN_TSR_TME_ANY) == 0)
    {
        // No more free buffers --> enable transmit IRQ.
        instance_->IER |= CAN_IT_TME;
    }
}

/** Handler for CAN device's transmit interrupt. */
void Stm32CanDriver::tx_interrupt()
{
    struct can_frame can_frame;
    while (instance_->TSR & CAN_TSR_TME_ANY)
    {
        if (!get_tx_msg_from_isr(&can_frame))
        {
            // No new frame but we have a txbuffer -> disable tx interrupt.
            instance_->IER &= ~CAN_IT_TME;
            return;
        }
        CanTxMsg msg;
        fill_tx_message(can_frame, &msg);
        if (CAN_Transmit(instance_, &msg) == CAN_TxStatus_NoMailBox)
        {
            DIE("Internal inconsistency: free xmit buffer disappeared.");
        }
    }
}

void Stm32CanDriver::rx_interrupt()
{
    HASSERT(instance_->RF0R & 3);
    while (instance_->RF0R & 3) {
        CanRxMsg msg;
        CAN_Receive(instance_, CAN_FIFO0, &msg);
        struct can_frame frame;
        parse_rx_message(msg, &frame);
        put_rx_msg_from_isr(frame);
    }
}

Stm32CanDriver can0(CAN1, "/dev/can0");

extern "C" {

void USB_HP_CAN1_TX_IRQHandler(void) {
    can0.tx_interrupt();
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
    can0.rx_interrupt();
}

}

