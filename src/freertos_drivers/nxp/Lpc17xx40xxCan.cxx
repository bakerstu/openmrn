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
 * \file Lpc17xx40xxCan.cxx
 * This file implements a can device driver layer specific to LPC17xx and
 * LPC40xx devices.
 *
 * @author Stuart W. Baker
 * @date 18 April 2015
 */

#include <stdint.h>

#include "Lpc17xx40xxCan.hxx"

#include "chip.h"
#include "nmranet_config.h"

LpcCan *LpcCan::instances[2] = {NULL};
unsigned int LpcCan::intCount = 0;

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 */
LpcCan::LpcCan(const char *name, LPC_CAN_T *base)
    : Can(name)
    , base(base)
{
    if (base == LPC_CAN1)
    {
        instances[0] = this;
    }
    else if (base == LPC_CAN2)
    {
        instances[1] = this;
    }
    else
    {
        HASSERT(0);
    }

    /* should already be disabled, but just in case */
    NVIC_DisableIRQ(CAN_IRQn);

    /* The priority of CAN interrupt is as high as possible while maintaining
     * FreeRTOS compatibility.
     */
    NVIC_SetPriority(CAN_IRQn, configKERNEL_INTERRUPT_PRIORITY);
}

/** Enable use of the device.
 */
void LpcCan::enable()
{
    Chip_CAN_Init(base, LPC_CANAF, LPC_CANAF_RAM);
    if (base == LPC_CAN2)
    {
        Chip_CAN_SetBitRate(base, config_nmranet_can_bitrate());
    }
    else if (base == LPC_CAN1)
    {
        Chip_CAN_SetBitRate(base, config_can2_bitrate());
    }
    else
    {
        DIE("Unknown CAN base address.");
    }
    Chip_CAN_SetAFMode(LPC_CANAF, CAN_AF_BYBASS_MODE);
    Chip_CAN_EnableInt(base, CAN_IER_BITMASK & 
                             ~(CAN_IER_TIE2 | CAN_IER_TIE3 | CAN_ICR_IDI));

    if (intCount++ == 0)
    {
        NVIC_EnableIRQ(CAN_IRQn);
    }
}

/** Disable use of the device.
 */
void LpcCan::disable()
{
    if (--intCount == 0)
    {
        NVIC_DisableIRQ(CAN_IRQn);
    }
    Chip_CAN_DisableInt(base, CAN_IER_BITMASK &
                             ~(CAN_IER_TIE2 | CAN_IER_TIE3 | CAN_ICR_IDI));
    Chip_CAN_DeInit(base);
}

/* Try and transmit a message.
 */
void LpcCan::tx_msg()
{
    if (Chip_CAN_GetStatus(base) & CAN_SR_TBS(0))
    {
        struct can_frame *can_frame;

        if (txBuf->data_read_pointer(&can_frame))
        {
            /* The LPC Chip drivers perform a lot of extra copying, which is
             * stupid.  We can do significantly better by using the structure
             * members in the can_17xx_40xx.h header directly.
             */

            /* load the next message to transmit */
            base->TX[0].TFI = 0;
            if (can_frame->can_eff)
            {
                base->TX[0].TFI |= CAN_TFI_FF;
                base->TX[0].TID = CAN_TID_ID29(can_frame->can_id);
            }
            else
            {
                base->TX[0].TID = CAN_TID_ID11(can_frame->can_id);
            }
            if (can_frame->can_rtr)
            {
                base->TX[0].TFI |= CAN_TFI_RTR;
            }
            else
            {
                base->TX[0].TFI |= CAN_TFI_DLC(can_frame->can_dlc);
                for (unsigned i = 0; i < (CAN_MSG_MAX_DATA_LEN + 3) / 4; ++i)
                {
                    base->TX[0].TD[i] = (can_frame->data[4 * i + 0] <<  0) |
                                        (can_frame->data[4 * i + 1] <<  8) |
                                        (can_frame->data[4 * i + 2] << 16) |
                                        (can_frame->data[4 * i + 3] << 24);
                }
            }

            Chip_CAN_SetCmd(base, CAN_CMR_STB(0) | CAN_CMR_TR);
            txBuf->consume(1);
            txBuf->signal_condition();
        }
    }
}

/** Common interrupt handler for all CAN devices.
 * @param status interrupt source status
 */
void LpcCan::interrupt_handler(uint32_t status)
{
    if (status & CAN_ICR_EI)
    {
        ++softErrorCount;
        /* flush and data in the tx pipeline */
        Chip_CAN_SetCmd(base, CAN_CMR_TR);
        txBuf->flush();
        txBuf->signal_condition_from_isr();
    }
    if (status & CAN_ICR_DOI)
    {
        /* receive buffer overrun */
        ++overrunCount;
    }
    if (status & CAN_ICR_WUI)
    {
        /* wakeup from activity */
    }
    if (status & CAN_ICR_EPI)
    {
        /* In error passive state */
    }
    if (status & CAN_ICR_ALI)
    {
        /* Arbitration lost */
    }
    if (status & CAN_ICR_BEI)
    {
        /* bus off error condition */
        ++busOffCount;
        /* flush and data in the tx pipeline */
        Chip_CAN_SetCmd(base, CAN_CMR_TR);
        txBuf->flush();
        txBuf->signal_condition_from_isr();
        /* reset controller */
        Chip_CAN_SetMode(base, CAN_RESET_MODE, DISABLE);
    }
    if (status & CAN_ICR_IDI)
    {
        /* transmission successful or aborted */
    }
    if (status & CAN_ICR_RI)
    {
        /* rx data received */
        struct can_frame *can_frame;
        if (rxBuf->data_write_pointer(&can_frame))
        {
            if (base->SR & CAN_SR_RBS(0))
            {
                can_frame->can_id = (base->RX.RFS & CAN_RFS_FF) ?
                                    CAN_RID_ID_29(base->RX.RID) :
                                    CAN_RID_ID_11(base->RX.RID);
                can_frame->can_rtr = (base->RX.RFS & CAN_RFS_RTR) ? 1 : 0;
                can_frame->can_err = 0;
                if (base->RX.RFS & CAN_RFS_FF)
                {
                    can_frame->can_eff = 1;
                    can_frame->can_id = CAN_RID_ID_29(base->RX.RID);
                }
                else
                {
                    can_frame->can_eff = 0;
                    can_frame->can_id = CAN_RID_ID_11(base->RX.RID);
                }
                can_frame->can_dlc = CAN_RFS_DLC(base->RX.RFS);
                if (base->RX.RFS & CAN_RFS_RTR)
                {
                    can_frame->can_rtr = 1;
                }
                else
                {
                    can_frame->can_rtr = 0;
                    can_frame->data[0] = (base->RX.RD[0] >>  0) & 0xFF;
                    can_frame->data[1] = (base->RX.RD[0] >>  8) & 0xFF;
                    can_frame->data[2] = (base->RX.RD[0] >> 16) & 0xFF;
                    can_frame->data[3] = (base->RX.RD[0] >> 24) & 0xFF;
                    can_frame->data[4] = (base->RX.RD[1] >>  0) & 0xFF;
                    can_frame->data[5] = (base->RX.RD[1] >>  8) & 0xFF;
                    can_frame->data[6] = (base->RX.RD[1] >> 16) & 0xFF;
                    can_frame->data[7] = (base->RX.RD[1] >> 24) & 0xFF;
                }
                Chip_CAN_SetCmd(base, CAN_CMR_RRB);
                rxBuf->advance(1);
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
        }
        else
        {
            /* ran out of space to buffer, flush incoming message */
            ++overrunCount;
            Chip_CAN_SetCmd(base, CAN_CMR_RRB);
        }
    }
    if (status & CAN_ICR_TI1)
    {
        struct can_frame *can_frame;

        if (txBuf->data_read_pointer(&can_frame))
        {
            /* The LPC Chip drivers perform a lot of extra copying, which is
             * stupid.  We can do significantly better by using the structure
             * members in the can_17xx_40xx.h header directly.
             */

            /* load the next message to transmit */
            base->TX[0].TFI = 0;
            if (can_frame->can_eff)
            {
                base->TX[0].TFI |= CAN_TFI_FF;
                base->TX[0].TID = CAN_TID_ID29(can_frame->can_id);
            }
            else
            {
                base->TX[0].TID = CAN_TID_ID11(can_frame->can_id);
            }
            if (can_frame->can_rtr)
            {
                base->TX[0].TFI |= CAN_TFI_RTR;
            }
            else
            {
                base->TX[0].TFI |= CAN_TFI_DLC(can_frame->can_dlc);
                for (unsigned i = 0; i < (CAN_MSG_MAX_DATA_LEN + 3) / 4; ++i)
                {
                    base->TX[0].TD[i] = (can_frame->data[4 * i + 0] <<  0) |
                                        (can_frame->data[4 * i + 1] <<  8) |
                                        (can_frame->data[4 * i + 2] << 16) |
                                        (can_frame->data[4 * i + 3] << 24);
                }
            }

            Chip_CAN_SetCmd(base, CAN_CMR_STB(0) | CAN_CMR_TR);
            txBuf->consume(1);
            txBuf->signal_condition_from_isr();

            /** @todo (Stuart Baker) remove notify logic once we switch over to
             * select()
             */
            if (writableNotify_)
            {
                writableNotify_->notify_from_isr();
                writableNotify_= nullptr;
            }
        }
    }
}

extern "C" {
/** This is the interrupt handler for the can device(s).
 */
void can_interrupt_handler(void)
{
    LpcCan::interrupt_handler();
}

} // extern "C"
