/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file Pic32mxCan.hxx
 *
 * Declaration for the CAN device driver layer for the pic32mx.
 *
 * @author Balazs Racz
 * @date 21 Oct 2018
 */

#ifndef _FREERTOS_DRIVERS_PIC32MX_PIC32MXCAN_HXX_
#define _FREERTOS_DRIVERS_PIC32MX_PIC32MXCAN_HXX_

#include "Devtab.hxx"

#include "GenericTypeDefs.h"
#include <xc.h>
extern "C" {
#include "peripheral/CAN.h"
#include "peripheral/int.h"
}

/// CAN-bus device driver for the Pic32MX.
///
/// This driver does not inherit from the shared Can driver, because the RX and
/// RX buffering is different. The shared CAN driver has a DeviceBuffer for
/// transmit and receive in the format of struct can_frame; whereas the PIC32
/// can write directly into the memory area of the CAN controller as the queues
/// are long enough and the CAN controller is a bus master.
class Pic32mxCan : public Node
{
public:
    /// Constructor.
    ///
    /// @param module defines which CAN hardware to use. (can0 or can1).
    /// @param dev filename of the device to create (e.g. "/dev/can0");
    /// @param irq_vector fill with can1_interrupt_vector_number or
    /// can2_interrupt_vector_number.
    Pic32mxCan(CAN_MODULE module, const char *dev, unsigned irq_vector);

    ~Pic32mxCan();

    /// Implementation of the interrupt handler.
    inline void isr()
    {
        int woken = 0;
        if ((CANGetModuleEvent(hw_) & CAN_RX_EVENT) != 0)
        //    if(CANGetPendingEventCode(hw_) == CAN_CHANNEL1_EVENT)
        {
            /* This means that channel 1 caused the event.
             * The CAN_RX_CHANNEL_NOT_EMPTY event is persistent. You
             * could either read the channel in the ISR
             * to clear the event condition or as done
             * here, disable the event source, and set
             * an application flag to indicate that a message
             * has been received. The event can be
             * enabled by the application when it has processed
             * one message.
             *
             * Note that leaving the event enabled would
             * cause the CPU to keep executing the ISR since
             * the CAN_RX_CHANNEL_NOT_EMPTY event is persistent (unless
             * the not empty condition is cleared.)
             * */
            CANEnableChannelEvent(
                hw_, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY, FALSE);
            Device::select_wakeup_from_isr(&rxSelect_, &woken);
        }
        if ((CANGetModuleEvent(hw_) & CAN_TX_EVENT) != 0)
        //    if(CANGetPendingEventCode(hw_) == CAN_CHANNEL0_EVENT)
        {
            /* Same with the TX event. */
            CANEnableChannelEvent(
                hw_, CAN_CHANNEL0, CAN_TX_CHANNEL_NOT_FULL, FALSE);
            Device::select_wakeup_from_isr(&txSelect_, &woken);
        }
        INTClearFlag(can_int());
    }

private:
    void enable();  /**< function to enable device */
    void disable(); /**< function to disable device */
    void flush_buffers() OVERRIDE {} /**< function to disable device */

    ssize_t read(File *file, void *buf, size_t count);
    ssize_t write(File *file, const void *buf, size_t count);
    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) OVERRIDE;

    /// @return the interrupt source as used by the internal tables of the
    /// peripheral library.
    INT_SOURCE can_int()
    {
        return (INT_SOURCE)(INT_SOURCE_CAN(hw_));
    }

    /// @return the interrupt vector enum as used by the internal tables of the
    /// peripheral library.
    INT_VECTOR can_vector()
    {
        return (INT_VECTOR)(INT_VECTOR_CAN(hw_));
    }

    /// Hardware (enumeration value).
    CAN_MODULE hw_;
    /// How many times did we drop a frame because we did not have enough
    /// hardware buffers.
    int overrunCount_;
    /// Hardware interrupt vector number. Do not delete!
    unsigned irqVector_;
    /// Points to the shared RAM area between the hardware and the driver.
    void *messageFifoArea_;
    // Select for the transmit buffers.
    SelectInfo txSelect_;
    // Select for the receive buffers.
    SelectInfo rxSelect_;

    DISALLOW_COPY_AND_ASSIGN(Pic32mxCan);
};


#endif // _FREERTOS_DRIVERS_PIC32MX_PIC32MXCAN_HXX_
