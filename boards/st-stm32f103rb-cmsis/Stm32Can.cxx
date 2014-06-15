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

class Stm32CanDriver : public Can
{
public:
    Stm32CanDriver(CAN_TypeDef *instance, const char *dev, int frequency)
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
        gp.GPIO_Pin = GPIO_Pin_CAN_RX;
        gp.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIO_CAN, &gp);

        // Configures CAN TX pin
        gp.GPIO_Pin = GPIO_Pin_CAN_TX;
        gp.GPIO_Mode = GPIO_Mode_AF_PP;
        gp.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIO_CAN, &gp);

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

    void enable() OVERRIDE {
        init_can_filter();
        // Sets the CAN interrupt priorities to be compatible with FreeRTOS.
        NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, configKERNEL_INTERRUPT_PRIORITY);
        NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, configKERNEL_INTERRUPT_PRIORITY);
        NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
        NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
        CAN_ITConfig(instance_, CAN_IT_FMP0, ENABLE);
    };
    void disable() OVERRIDE {
        CAN_ITConfig(instance_, CAN_IT_FMP0, DISABLE);
        NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
        NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    };

    void interrupt();
    void tx_msg() OVERRIDE;

    CAN_TypeDef *instance_;
    // Status register.
    volatile uint32_t *SR_;
    char txPending_;
};

/** Try and transmit a message. Does nothing if there is no message to transmit
 *  or no write buffers to transmit via.
 * @param dev device to transmit message on
 */
void Stm32CanDriver::tx_msg()
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
    CANMessage msg(can_frame.can_id, (const char *)can_frame.data,
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
void Stm32CanDriver::interrupt()
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
            CANMessage msg(can_frame.can_id, (const char *)can_frame.data,
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
Stm32CanDriver can0(Stm32CanDriver::CAN2, "/dev/can0",
                    config_nmranet_can_bitrate());
Stm32CanDriver can1(Stm32CanDriver::CAN1, "/dev/can1",
                    config_nmranet_can_bitrate());
