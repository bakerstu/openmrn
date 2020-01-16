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
 * \file Pic32mxCan.cxx
 *
 * This file implements a can device driver layer for the pic32mx using the
 * Microchip plib CAN library.
 *
 * @author Balazs Racz
 * @date 12 Aug 2013
 */

#include "freertos_drivers/pic32mx/Pic32mxCan.hxx"

#include "DeviceBuffer.hxx"
#include "nmranet_config.h"
#include "can_frame.h"
#include <fcntl.h>

Pic32mxCan::Pic32mxCan(CAN_MODULE module, const char *dev, unsigned irq_vector)
    : Node(dev)
    , hw_(module)
    , overrunCount_(0)
    , irqVector_(irq_vector)
{
    messageFifoArea_ = malloc(
        (config_can_tx_buffer_size() + config_can_rx_buffer_size()) * 16);
}

Pic32mxCan::~Pic32mxCan()
{
    disable();
    free(messageFifoArea_);
}

/// Translates a hardware buffer to a struct can_frame.
///
/// @param message hardware buffer.
/// @param can_frame output can frame.
///
static void pic_buffer_to_frame(const CANRxMessageBuffer *message,
                                struct can_frame *can_frame)
{
    uint32_t id = message->msgSID.SID;
    if (message->msgEID.IDE)
    {
        SET_CAN_FRAME_EFF(*can_frame);
        id <<= 18;
        id |= message->msgEID.EID;
        SET_CAN_FRAME_ID_EFF(*can_frame, id);
    }
    else
    {
        CLR_CAN_FRAME_EFF(*can_frame);
        SET_CAN_FRAME_ID(*can_frame, id);
    }
    if (message->msgEID.RTR)
    {
        SET_CAN_FRAME_RTR(*can_frame);
    }
    else
    {
        CLR_CAN_FRAME_RTR(*can_frame);
    }
    CLR_CAN_FRAME_ERR(*can_frame);

    can_frame->can_dlc = message->msgEID.DLC;
    memcpy(can_frame->data, message->data, can_frame->can_dlc);
}

/// Translates a struct can_frame to a hardware buffer.
///
/// @param can_frame frame to send
/// @param message hardware buffer to fill from the frame to send.
///
static void frame_to_pic_buffer(const struct can_frame *can_frame,
                                CANTxMessageBuffer *message)
{
    message->messageWord[0] = 0;
    message->messageWord[1] = 0;
    if (IS_CAN_FRAME_EFF(*can_frame))
    {
        uint32_t id = GET_CAN_FRAME_ID_EFF(*can_frame);
        message->msgEID.IDE = 1;
        message->msgSID.SID = id >> 18;
        message->msgEID.EID = id & ((1 << 19) - 1);

        // message->msgSID.SID = id & 0x7ff;
        // message->msgEID.EID = id >> 11;
    }
    else
    {
        message->msgSID.SID = GET_CAN_FRAME_ID(*can_frame);
        message->msgEID.IDE = 0;
    }
    if (IS_CAN_FRAME_RTR(*can_frame))
    {
        message->msgEID.RTR = 1;
    }
    else
    {
        message->msgEID.RTR = 0;
    }
    message->msgEID.DLC = can_frame->can_dlc;
    memcpy(message->data, can_frame->data, can_frame->can_dlc);
}

ssize_t Pic32mxCan::read(File *file, void *buf, size_t count)
{
    struct can_frame *can_frame = static_cast<struct can_frame *>(buf);
    ssize_t result = 0;

    int flags = -1;

    while (count >= sizeof(struct can_frame))
    {
        /*
           At the beginning of the iteration we are in a critical section.

           We need this critical section because the GetRxBuffer call will
           return the same buffer until the Update call is successful. We want
           to prevent multiple threads from receiving the same CAN frame
           however. */

        taskENTER_CRITICAL();
        CANRxMessageBuffer *message =
            (CANRxMessageBuffer *)CANGetRxMessage(hw_, CAN_CHANNEL1);
        if (message != NULL)
        {
            CANUpdateChannel(hw_, CAN_CHANNEL1);
        }
        if (flags == -1)
        {
            flags = file->flags;
        }
        taskEXIT_CRITICAL();

        /* Now let's take a look if we have actually found a message. */
        if (message != NULL)
        {
            pic_buffer_to_frame(message, can_frame);

            count -= sizeof(struct can_frame);
            result += sizeof(struct can_frame);
            can_frame++;
            continue;
        }

        /* We do not have a message. Return a short read or zero if we have a
         * non-blocking filedes. */

        if (result || (flags & O_NONBLOCK))
        {
            break;
        }

        /* Blocking read. Enable the receive interrupt and pend on the rx
           semaphore. Spurious interrupts and extra tokens in the semaphore are
           not a problem, they will just drop back here with no messages
           found.

           There is no race condition between checking the queue above and
           enabling the interrupt here. The interrupt pending flag is
           persistent, thus if there is already a message in the queue it will
           trigger the interrupt immediately.
        */

        CANEnableChannelEvent(hw_, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY,
                              TRUE);
        DeviceBufferBase::block_until_condition(file, true);
    }

    /* As a good-bye we wake up the interrupt handler once more to post on the
     * semaphore in case there is another thread waiting. */
    CANEnableChannelEvent(hw_, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY,
                          TRUE);

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }
    
    return result;
}

// static
ssize_t Pic32mxCan::write(File *file, const void *buf, size_t count)
{
    const struct can_frame *can_frame =
        static_cast<const struct can_frame *>(buf);
    ssize_t result = 0;

    int flags = -1;

    while (count >= sizeof(struct can_frame))
    {
        taskENTER_CRITICAL();
        CANTxMessageBuffer *message =
            CANGetTxMessageBuffer(hw_, CAN_CHANNEL0);
        if (message != NULL)
        {
            /* Unfortunately we have to fill the buffer in the critical section
             * or else we risk that another thread will call the FlushTxChannel
             * while our buffer is not fully completed. */
            frame_to_pic_buffer(can_frame, message);
            CANUpdateChannel(hw_, CAN_CHANNEL0);
        }
        if (flags == -1)
        {
            flags = file->flags;
        }
        taskEXIT_CRITICAL();

        /* Did we actually find a slot to transmit? */
        if (message != NULL)
        {
            CANFlushTxChannel(hw_, CAN_CHANNEL0);

            count -= sizeof(struct can_frame);
            result += sizeof(struct can_frame);
            can_frame++;
            continue;
        }

        /* We did not find a transmit slot. We purposefully do not execute a
         * short write here, although that would be an option. */
        if (flags & O_NONBLOCK)
        {
            break;
        }
        /* Blocking read. We enable the interrupt and wait for the
         * semaphore. There is no race condition here, because the TX buffer
         * not full interrupt is persistent, so if a buffer got free between
         * our check and now, the interrupt will trigger immediately and wake
         * us up. */
        CANEnableChannelEvent(hw_, CAN_CHANNEL0, CAN_TX_CHANNEL_NOT_FULL,
                              TRUE);
        DeviceBufferBase::block_until_condition(file, false);
    }

    /* As a good-bye we wake up the interrupt handler once more to post on the
     * semaphore in case there is another thread waiting. */
    CANEnableChannelEvent(hw_, CAN_CHANNEL0, CAN_TX_CHANNEL_NOT_FULL,
                          TRUE);

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result;
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool Pic32mxCan::select(File* file, int mode)
{
    portENTER_CRITICAL();
    bool retval = false;
    switch (mode)
    {
        case FREAD:
            if (CANGetChannelEvent(hw_, CAN_CHANNEL1) &
                CAN_RX_CHANNEL_NOT_EMPTY)
            {
                retval = true;
            }
            else
            {
                Device::select_insert(&rxSelect_);
            }
            break;
        case FWRITE:
            if (CANGetChannelEvent(hw_, CAN_CHANNEL0) & CAN_TX_CHANNEL_NOT_FULL)
            {
                retval = true;
            }
            else
            {
                Device::select_insert(&txSelect_);
            }
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }
    portEXIT_CRITICAL();

    return retval;
}

void Pic32mxCan::enable()
{
    CANEnableModule(hw_, TRUE);
    /* Step 1: Switch the CAN module
     * ON and switch it to Configuration
     * mode. Wait till the mode switch is
     * complete. */
    CANSetOperatingMode(hw_, CAN_CONFIGURATION);
    while (CANGetOperatingMode(hw_) != CAN_CONFIGURATION)
        ;

    /* Step 2: Configure the Clock.The
     * CAN_BIT_CONFIG data structure is used
     * for this purpose. The propagation segment,
     * phase segment 1 and phase segment 2
     * are configured to have 3TQ. SYSTEM_FREQ
     * and CAN_BUS_SPEED are defined in  */

    CAN_BIT_CONFIG canBitConfig;
    canBitConfig.phaseSeg2Tq = CAN_BIT_3TQ;
    canBitConfig.phaseSeg1Tq = CAN_BIT_3TQ;
    canBitConfig.propagationSegTq = CAN_BIT_3TQ;
    canBitConfig.phaseSeg2TimeSelect = TRUE;
    canBitConfig.sample3Time = TRUE;
    canBitConfig.syncJumpWidth = CAN_BIT_2TQ;

    CANSetSpeed(hw_, &canBitConfig, configCPU_CLOCK_HZ,
                config_nmranet_can_bitrate());

    /* Step 3: Assign the buffer area to the
     * CAN module.
     */

    CANAssignMemoryBuffer(
        hw_, messageFifoArea_,
        16 * (config_can_tx_buffer_size() + config_can_rx_buffer_size()));

    /* Step 4: Configure channel 0 for TX and size of tx_queue_len message
     * buffers with RTR disabled and low medium priority. Configure channel 1
     * for RX and size of rx_queue_len message buffers and receive the full
     * message.
     */

    /// @todo(balazs.racz) why is the tx buffer length 1?
    CANConfigureChannelForTx(hw_, CAN_CHANNEL0, 1, CAN_TX_RTR_DISABLED,
                             CAN_LOW_MEDIUM_PRIORITY);
    CANConfigureChannelForRx(hw_, CAN_CHANNEL1, config_can_rx_buffer_size(),
                             CAN_RX_FULL_RECEIVE);

    // We create a catch-all filter for channel 1.
    CANConfigureFilterMask(hw_, CAN_FILTER_MASK0, 0, CAN_EID, CAN_FILTER_MASK_ANY_TYPE);
    CANEnableFilter(hw_, CAN_FILTER0, FALSE);
    while(CANIsFilterDisabled(hw_, CAN_FILTER0) == FALSE);
    CANConfigureFilter(hw_, CAN_FILTER0, 0, CAN_EID);
    CANLinkFilterToChannel(hw_, CAN_FILTER0, CAN_FILTER_MASK0, CAN_CHANNEL1);
    CANEnableFilter(hw_, CAN_FILTER0, TRUE);

    CANEnableModuleEvent(hw_, CAN_RX_EVENT, TRUE);
    CANEnableModuleEvent(hw_, CAN_TX_EVENT, TRUE);

    /* Step 6: Enable interrupt and events.
     * The interrrupt peripheral library is used to enable
     * the CAN interrupt to the CPU. */

    INTSetVectorPriority(can_vector(), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(can_vector(), INT_SUB_PRIORITY_LEVEL_0);
    INTEnable(can_int(), INT_ENABLED);
    /* Step 7: Switch the CAN mode
     * to normal mode. */

    CANSetOperatingMode(hw_, CAN_NORMAL_OPERATION);
    while (CANGetOperatingMode(hw_) != CAN_NORMAL_OPERATION)
        ;
}

void Pic32mxCan::disable()
{
    /* If the transmit buffer is not empty, we should crash here. Otherweise it
     * is possible that the user sends some frames, then closes the device and
     * the frames never get sent really. */
    INTEnable(can_int(), INT_DISABLED);
    CANEnableModule(hw_, FALSE);
}

/// @todo: process receive buffer overflow flags.
