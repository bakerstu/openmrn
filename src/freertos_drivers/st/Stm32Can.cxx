/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * @file Stm32Can.cxx
 * This file implements a can device driver layer specific to STM32F0xx devices.
 *
 * @author Stuart W. Baker
 * @date 3 May 2015
 */

#if (!defined(ARDUINO)) || defined(ARDUINO_ARCH_STM32)

#include "Stm32Can.hxx"

#include <stdint.h>

#include "can_ioctl.h"

#include "stm32f_hal_conf.hxx"

#if defined (STM32F072xB) || defined (STM32F091xC)

#include "stm32f0xx_hal_cortex.h"
#define CAN1_IRQN CEC_CAN_IRQn
#define CAN1 CAN
#define CAN_CLOCK cpu_clock_hz

#elif defined (STM32F103xB)

#include "stm32f1xx_hal_cortex.h"
#define SPLIT_INT
#define CAN1_TX_IRQN USB_HP_CAN1_TX_IRQn
#define CAN1_IRQN CAN1_TX_IRQN
#define CAN1_SECOND_IRQN USB_LP_CAN1_RX0_IRQn
#define CAN1_THIRD_IRQN CAN1_SCE_IRQn

#ifdef CAN2
#define CAN2_TX_IRQN CAN2_TX_IRQn
#define CAN2_IRQN CAN2_TX_IRQN
#define CAN2_SECOND_IRQN CAN2_RX0_IRQn
#define CAN2_THIRD_IRQN CAN2_SCE_IRQn
#endif

#define CAN_CLOCK (cm3_cpu_clock_hz >> 1)

#elif defined (STM32F303xC) || defined (STM32F303xE)

#include "stm32f3xx_hal_cortex.h"
#define SPLIT_INT
#define CAN1_TX_IRQN USB_HP_CAN_TX_IRQn
#define CAN1_IRQN CAN1_TX_IRQN
#define CAN1_SECOND_IRQN USB_LP_CAN_RX0_IRQn
#define CAN1_THIRD_IRQN CAN_SCE_IRQn
#define CAN1 CAN
#define CAN_CLOCK (cm3_cpu_clock_hz >> 1)

#elif defined (STM32L431xx) || defined (STM32L432xx)

#include "stm32l4xx_hal_cortex.h"
#define SPLIT_INT
#define CAN1_TX_IRQN CAN1_TX_IRQn
#define CAN1_IRQN CAN1_TX_IRQN
#define CAN1_SECOND_IRQN CAN1_RX0_IRQn
#define CAN1_THIRD_IRQN CAN1_SCE_IRQn
#define CAN_CLOCK (cm3_cpu_clock_hz)

#ifdef CAN2
#define CAN2_TX_IRQN CAN2_TX_IRQn
#define CAN2_IRQN CAN2_TX_IRQN
#define CAN2_SECOND_IRQN CAN2_RX0_IRQn
#define CAN2_THIRD_IRQN CAN2_SCE_IRQn
#endif

#elif defined (STM32F767xx)

#include "stm32f7xx_hal_cortex.h"
#define SPLIT_INT
#define CAN1_TX_IRQN CAN1_TX_IRQn
#define CAN1_IRQN CAN1_TX_IRQN
#define CAN1_SECOND_IRQN CAN1_RX0_IRQn
#define CAN1_THIRD_IRQN CAN1_SCE_IRQn
#define CAN_CLOCK (cm3_cpu_clock_hz >> 2) // 54 MHz, sysclk/4

#ifdef CAN2
#define CAN2_TX_IRQN CAN2_TX_IRQn
#define CAN2_IRQN CAN2_TX_IRQN
#define CAN2_SECOND_IRQN CAN2_RX0_IRQn
#define CAN2_THIRD_IRQN CAN2_SCE_IRQn
#endif

#ifdef CAN3
#define CAN3_TX_IRQN CAN3_TX_IRQn
#define CAN3_IRQN CAN3_TX_IRQN
#define CAN3_SECOND_IRQN CAN3_RX0_IRQn
#define CAN3_THIRD_IRQN CAN3_SCE_IRQn
#endif

#else
#error Dont know what STM32 chip you have.
#endif

Stm32Can *Stm32Can::instances[MAXCANIFS] = {NULL,NULL,NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 */
Stm32Can::Stm32Can(const char *name, uint8_t index)
    : Can(name)
    , state_(CAN_STATE_STOPPED)
{
#if defined(CAN3)
    HASSERT(index < 3);
#elif defined(CAN2)
    HASSERT(index < 2);
#else
    HASSERT(index < 1);
#endif
    
    /* only one instance per interface allowed */
    HASSERT(instances[index] == NULL);

    instances[index] = this;
    switch (index)
    {
        case 0: /* CAN1... */
            can_ = CAN1;
            canIrqn_ = CAN1_IRQN;
#ifdef SPLIT_INT
            canSecondIrqn_ = CAN1_SECOND_IRQN;
            canThirdIrqn_ = CAN1_THIRD_IRQN;
#endif
            break;
#ifdef CAN2
        case 1: /* CAN2... */
            can_ = CAN2;
            canIrqn_ = CAN2_IRQN;
#ifdef SPLIT_INT
            canSecondIrqn_ = CAN2_SECOND_IRQN;
            canThirdIrqn_ = CAN2_THIRD_IRQN;
#endif
            break;
#endif
#ifdef CAN3
        case 2: /* CAN3... */
            can_ = CAN3;
            canIrqn_ = CAN3_IRQN;
#ifdef SPLIT_INT
            canSecondIrqn_ = CAN3_SECOND_IRQN;
            canThirdIrqn_ = CAN3_THIRD_IRQN;
#endif
            break;
#endif
        default:
            break;
    }

    /* should already be disabled, but just in case */
    HAL_NVIC_DisableIRQ(canIrqn_);

#if defined (STM32F030x6) || defined (STM32F031x6) || defined (STM32F038xx) \
 || defined (STM32F030x8) || defined (STM32F030xC) || defined (STM32F042x6) \
 || defined (STM32F048xx) || defined (STM32F051x8) || defined (STM32F058xx) \
 || defined (STM32F070x6) || defined (STM32F070xB) || defined (STM32F071xB) \
 || defined (STM32F072xB) || defined (STM32F078xx) \
 || defined (STM32F091xC) || defined (STM32F098xx)
#else
    /* The priority of CAN interrupt is as high as possible while maintaining
     * FreeRTOS compatibility.
     */
    SetInterruptPriority(canIrqn_, configKERNEL_INTERRUPT_PRIORITY);

#ifdef SPLIT_INT
    HAL_NVIC_DisableIRQ(canSecondIrqn_);
    SetInterruptPriority(canSecondIrqn_, configKERNEL_INTERRUPT_PRIORITY);
    HAL_NVIC_DisableIRQ(canThirdIrqn_);
    SetInterruptPriority(canThirdIrqn_, configKERNEL_INTERRUPT_PRIORITY);
#endif
#endif
}

#if !defined(ARDUINO) 
//
// Stm32Can::ioctl()
//
int Stm32Can::ioctl(File *file, unsigned long int key, unsigned long data)
{
    if (key == SIOCGCANSTATE)
    {
        *((can_state_t*)data) = state_;
        return 0;
    }
    return -EINVAL;
}
#endif // !ARDUINO

/** Enable use of the device.
 */
void Stm32Can::enable()
{
    /* disable sleep, enter init mode */
    can_->MCR = CAN_MCR_INRQ;

    /* Time triggered tranmission off
     * Bus off state is left automatically
     * Auto-Wakeup mode disabled
     * automatic re-transmission enabled
     * receive FIFO not locked on overrun
     * TX FIFO mode on
     */
    can_->MCR |= (CAN_MCR_ABOM | CAN_MCR_TXFP);

    /* Setup timing.
     * 125,000 Kbps = 8 usec/bit
     */
    can_->BTR = (CAN_BS1_5TQ | CAN_BS2_2TQ | CAN_SJW_1TQ |
                ((CAN_CLOCK / 1000000) - 1));

    /* enter normal mode */
    can_->MCR &= ~CAN_MCR_INRQ;

    /* Enter filter initialization mode.  Filter 0 will be used as a single
     * 32-bit filter, ID Mask Mode, we accept everything, no mask.
     */
    can_->FMR |= CAN_FMR_FINIT;
    can_->FM1R = 0;
    can_->FS1R = 0x000000001;
    can_->FFA1R = 0;
    can_->sFilterRegister[0].FR1 = 0;
    can_->sFilterRegister[0].FR2 = 0;

    /* Activeate filter and exit initialization mode. */
    can_->FA1R = 0x000000001;
    can_->FMR &= ~CAN_FMR_FINIT;

    state_ = CAN_STATE_ACTIVE;

    /* enable interrupts */
    can_->IER = (CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_EWGIE); // errors
    can_->IER |= (CAN_IER_ERRIE | CAN_IER_FMPIE0); // error + receive
    HAL_NVIC_EnableIRQ(canIrqn_);
#ifdef SPLIT_INT
    HAL_NVIC_EnableIRQ(canSecondIrqn_);
    HAL_NVIC_EnableIRQ(canThirdIrqn_);
#endif
}

/** Disable use of the device.
 */
void Stm32Can::disable()
{
    HAL_NVIC_DisableIRQ(canIrqn_);
#ifdef SPLIT_INT
    HAL_NVIC_DisableIRQ(canSecondIrqn_);
    HAL_NVIC_DisableIRQ(canThirdIrqn_);
#endif
    can_->IER = 0;

    state_ = CAN_STATE_STOPPED;

    /* disable sleep, enter init mode */
    can_->MCR = CAN_MCR_INRQ;
}

/* Try and transmit a message.
 */
void Stm32Can::tx_msg()
{
    // If we are error passive, the last transmission ended with an error, and
    // there are no free TX mailboxes, then we flush the input queue. This is a
    // workaround because the STM32 CAN controller can get stuck in this state
    // and never get to bus off if the TX attempts end up with no-ack (meaning
    // the controller is alone on the bus).
    if ((can_->ESR & CAN_ESR_EPVF) && ((can_->ESR & CAN_ESR_LEC_Msk) != 0) &&
        ((can_->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0))
    {
        txBuf->flush();
        txBuf->signal_condition();
        return;
    }

    /* see if we can send anything out */
    struct can_frame *can_frame;

    size_t msg_count = txBuf->data_read_pointer(&can_frame);
    unsigned i;

    for (i = 0; i < msg_count; ++i, ++can_frame)
    {
        volatile CAN_TxMailBox_TypeDef *mailbox;
        if (can_->TSR & CAN_TSR_TME0)
        {
            mailbox = can_->sTxMailBox + 0;
        }
        else if (can_->TSR & CAN_TSR_TME1)
        {
            mailbox = can_->sTxMailBox + 1;
        }
        else if (can_->TSR & CAN_TSR_TME2)
        {
            mailbox = can_->sTxMailBox + 2;
        }
        else
        {
            /* no empty mailboxes left to fill */
            break;
        }

        /* setup frame */
        if (can_frame->can_eff)
        {
            mailbox->TIR = (can_frame->can_id << 3) | CAN_TI0R_IDE;
        }
        else
        {
            mailbox->TIR = can_frame->can_id << 21;
        }
        if (can_frame->can_rtr)
        {
            mailbox->TIR |= CAN_TI0R_RTR;
        }
        else
        {
            mailbox->TDTR = can_frame->can_dlc;
            mailbox->TDLR = (can_frame->data[0] <<  0) |
                            (can_frame->data[1] <<  8) |
                            (can_frame->data[2] << 16) |
                            (can_frame->data[3] << 24);
            mailbox->TDHR = (can_frame->data[4] <<  0) |
                            (can_frame->data[5] <<  8) |
                            (can_frame->data[6] << 16) |
                            (can_frame->data[7] << 24);
        }

        /* request transmission */
        mailbox->TIR |= CAN_TI0R_TXRQ;
    }

    if (i)
    {
        txBuf->consume(i);
        txBuf->signal_condition();
    }

    /* enable transmit interrupt */
    can_->IER |= CAN_IER_TMEIE;
}

/** Handle an interrupt.
 */
void Stm32Can::rx_interrupt_handler()
{
    unsigned msg_receive_count = 0;

    while (can_->RF0R & CAN_RF0R_FMP0)
    {
        /* rx data received */
        struct can_frame *can_frame;
        size_t msg_count = rxBuf->data_write_pointer(&can_frame);
        if (msg_count)
        {
            if (can_->sFIFOMailBox[0].RIR & CAN_RI0R_IDE)
            {
                /* extended frame */
                can_frame->can_id = can_->sFIFOMailBox[0].RIR >> 3;
                can_frame->can_eff = 1;
            }
            else
            {
                /* standard frame */
                can_frame->can_id = can_->sFIFOMailBox[0].RIR >> 21;
                can_frame->can_eff = 0;
            }
            if (can_->sFIFOMailBox[0].RIR & CAN_RI0R_RTR)
            {
                /* remote frame */
                can_frame->can_rtr = 1;
                can_frame->can_dlc = 0;
            }
            else
            {
                /* data frame */
                can_frame->can_rtr = 0;
                can_frame->can_dlc = can_->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC;
                can_frame->data[0] = (can_->sFIFOMailBox[0].RDLR >>  0) & 0xFF;
                can_frame->data[1] = (can_->sFIFOMailBox[0].RDLR >>  8) & 0xFF;
                can_frame->data[2] = (can_->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
                can_frame->data[3] = (can_->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
                can_frame->data[4] = (can_->sFIFOMailBox[0].RDHR >>  0) & 0xFF;
                can_frame->data[5] = (can_->sFIFOMailBox[0].RDHR >>  8) & 0xFF;
                can_frame->data[6] = (can_->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
                can_frame->data[7] = (can_->sFIFOMailBox[0].RDHR >> 24) & 0xFF;
            }
        }
        else
        {
            ++overrunCount;
        }
        /* release FIFO */
        can_->RF0R |= CAN_RF0R_RFOM0;
        ++msg_receive_count;
    }

    if (msg_receive_count)
    {
        /* advance the "zero copy" buffer by the number of messages received */
        rxBuf->advance(msg_receive_count);
        rxBuf->signal_condition_from_isr();
        state_ = CAN_STATE_ACTIVE;
    }
}

void Stm32Can::tx_interrupt_handler()
{
    if (can_->TSR & (CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2))
    {
        /* transmit request completed, should be able to send another */
        struct can_frame *can_frame;

        size_t msg_count = txBuf->data_read_pointer(&can_frame);
        if (msg_count)
        {
            /* try and send some more CAN frames */
            unsigned i;

            for (i = 0; i < msg_count; ++i, ++can_frame)
            {
                volatile CAN_TxMailBox_TypeDef *mailbox;
                if (can_->TSR & CAN_TSR_TME0)
                {
                    mailbox = can_->sTxMailBox + 0;
                }
                else if (can_->TSR & CAN_TSR_TME1)
                {
                    mailbox = can_->sTxMailBox + 1;
                }
                else if (can_->TSR & CAN_TSR_TME2)
                {
                    mailbox = can_->sTxMailBox + 2;
                }
                else
                {
                    /* no empty mailboxes left to fill */
                    break;
                }

                /* setup frame */
                if (can_frame->can_eff)
                {
                    mailbox->TIR = (can_frame->can_id << 3) | CAN_TI0R_IDE;
                }
                else
                {
                    mailbox->TIR = can_frame->can_id << 21;
                }
                if (can_frame->can_rtr)
                {
                    mailbox->TIR |= CAN_TI0R_RTR;
                }
                else
                {
                    mailbox->TDTR = can_frame->can_dlc;
                    mailbox->TDLR = (can_frame->data[0] <<  0) |
                                    (can_frame->data[1] <<  8) |
                                    (can_frame->data[2] << 16) |
                                    (can_frame->data[3] << 24);
                    mailbox->TDHR = (can_frame->data[4] <<  0) |
                                    (can_frame->data[5] <<  8) |
                                    (can_frame->data[6] << 16) |
                                    (can_frame->data[7] << 24);
                }

                /* request transmission */
                mailbox->TIR |= CAN_TI0R_TXRQ;
            }
            txBuf->consume(i);
        }
        else
        {
            /* no more data left to transmit */
            can_->IER &= ~CAN_IER_TMEIE;
        }
        txBuf->signal_condition_from_isr();
        state_ = CAN_STATE_ACTIVE;
    }
}

/*
 * Stm32Can::sce_interrupt_handler()
 */
void Stm32Can::sce_interrupt_handler()
{
    if (can_->MSR & CAN_MSR_ERRI)
    {
        /* error interrupt has occured */
        can_->MSR |= CAN_MSR_ERRI; // clear flag

        bool cancel_queue = false;
        
        if (can_->ESR & CAN_ESR_EWGF)
        {
            /* error warning condition */
            state_ = CAN_STATE_BUS_WARNING;
        }
        if (can_->ESR & CAN_ESR_EPVF)
        {
            /* error passive condition */
            ++softErrorCount;
            state_ = CAN_STATE_BUS_PASSIVE;
            cancel_queue = true;
        }
        if (can_->ESR & CAN_ESR_BOFF)
        {
            /* bus off error condition */
            ++busOffCount;
            state_ = CAN_STATE_BUS_OFF;
            cancel_queue = true;
        }
        if (cancel_queue)
        {
            can_->TSR |= CAN_TSR_ABRQ2;
            can_->TSR |= CAN_TSR_ABRQ1;
            can_->TSR |= CAN_TSR_ABRQ0;
            can_->IER &= ~CAN_IER_TMEIE;
            txBuf->flush();
            txBuf->signal_condition_from_isr();
        }
    }
}

extern "C" {
/** This is the interrupt handler for the can device.
 */

//----------------------------------------------------------------------------
//
// F072xB & F091xC
//
//----------------------------------------------------------------------------

#if defined (STM32F072xB) || defined (STM32F091xC)
void cec_can_interrupt_handler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
    Stm32Can::instances[0]->tx_interrupt_handler();
    Stm32Can::instances[0]->sce_interrupt_handler();
}
#elif defined (STM32F103xB) || defined (STM32F303xC) || defined (STM32F303xE)
//----------------------------------------------------------------------------
//
// F103xB, F303xC and F303E
//
//----------------------------------------------------------------------------

void usb_hp_can1_tx_interrupt_handler(void)
{
    Stm32Can::instances[0]->tx_interrupt_handler();
}

void usb_lp_can1_rx0_interrupt_handler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
}

void can1_sce_interrupt_handler(void)
{
    Stm32Can::instances[0]->sce_interrupt_handler();
}

#ifdef CAN2
void can2_tx_interrupt_handler(void)
{
    Stm32Can::instances[1]->tx_interrupt_handler();
}

void can2_rx0_interrupt_handler(void)
{
    Stm32Can::instances[1]->rx_interrupt_handler();
}

void can2_sce_interrupt_handler(void)
{
    Stm32Can::instances[1]->sce_interrupt_handler();
}

#endif

#elif defined(STM32F767xx) || defined(STM32L431xx) || defined(STM32L432xx)
//----------------------------------------------------------------------------
//
// F767xx L431xx & L432xx
//
//----------------------------------------------------------------------------

void can1_tx_interrupt_handler(void)
{
    Stm32Can::instances[0]->tx_interrupt_handler();
}

void can1_rx0_interrupt_handler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
}

void can1_sce_interrupt_handler(void)
{
    Stm32Can::instances[0]->sce_interrupt_handler();
}

#ifdef CAN2
void can2_tx_interrupt_handler(void)
{
    Stm32Can::instances[1]->tx_interrupt_handler();
}

void can2_rx0_interrupt_handler(void)
{
    Stm32Can::instances[1]->rx_interrupt_handler();
}

void can2_sce_interrupt_handler(void)
{
    Stm32Can::instances[1]->sce_interrupt_handler();
}
#endif
#ifdef CAN3
void can3_tx_interrupt_handler(void)
{
    Stm32Can::instances[2]->tx_interrupt_handler();
}

void can3_rx0_interrupt_handler(void)
{
    Stm32Can::instances[2]->rx_interrupt_handler();
}

void can3_sce_interrupt_handler(void)
{
    Stm32Can::instances[2]->sce_interrupt_handler();
}
#endif

#else
#error Dont know what STM32 chip you have.
#endif

} // extern "C"

#endif // !ARDUINO

#if defined(ARDUINO_ARCH_STM32)

#include "stm32_def.h"
#include "PinAF_STM32F1.h"
#include <PeripheralPins.h>

void arduino_can_pinmap(PinName tx_pin, PinName rx_pin) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    void* can_tx = pinmap_peripheral(tx_pin, PinMap_CAN_TD);
    void* can_rx = pinmap_peripheral(rx_pin, PinMap_CAN_RD);
    void* can = pinmap_merge_peripheral(can_tx, can_rx);
    if (can == NP) {
        DIE("Could not find CAN peripheral");
    }
        
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Pin       = STM_GPIO_PIN(tx_pin);
    auto fn = pinmap_function(tx_pin, PinMap_CAN_TD);
    GPIO_InitStruct.Mode      = STM_PIN_MODE(fn);
    GPIO_InitStruct.Pull      = STM_PIN_PUPD(fn);
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
#ifdef STM32F1xx
    pin_SetF1AFPin(STM_PIN_AFNUM(fn));
#else
    GPIO_InitStruct.Alternate = STM_PIN_AFNUM(fn);
#endif /* STM32F1xx */
    HAL_GPIO_Init(set_GPIO_Port_Clock(STM_PORT(tx_pin)), &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = STM_GPIO_PIN(rx_pin);
    fn = pinmap_function(rx_pin, PinMap_CAN_RD);
    GPIO_InitStruct.Mode      = STM_PIN_MODE(fn);
    GPIO_InitStruct.Pull      = STM_PIN_PUPD(fn);
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
#ifdef STM32F1xx
    pin_SetF1AFPin(STM_PIN_AFNUM(fn));
#else
    GPIO_InitStruct.Alternate = STM_PIN_AFNUM(fn);
#endif /* STM32F1xx */
    HAL_GPIO_Init(set_GPIO_Port_Clock(STM_PORT(rx_pin)), &GPIO_InitStruct);
}

extern "C" {
void USB_HP_CAN_TX_IRQHandler(void)
{
    Stm32Can::instances[0]->tx_interrupt_handler();
}

void USB_LP_CAN_RX0_IRQHandler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
}
void CEC_CAN_IRQHandler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
    Stm32Can::instances[0]->tx_interrupt_handler();
    Stm32Can::instances[0]->sce_interrupt_handler();
}
void CAN1_TX_IRQHandler(void)
{
    Stm32Can::instances[0]->tx_interrupt_handler();
}
void CAN1_RX0_IRQHandler(void)
{
    Stm32Can::instances[0]->rx_interrupt_handler();
}
void CAN1_SCE_IRQHandler(void)
{
    Stm32Can::instances[0]->sce_interrupt_handler();
}
} // extern "C"

#endif // ARDUINO_ARCH_STM32
