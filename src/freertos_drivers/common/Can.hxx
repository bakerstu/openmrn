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
 * \file Can.hxx
 * This file implements a generic can device driver layer.
 *
 * @author Stuart W. Baker
 * @date 28 December 2012
 */

#ifndef _FREERTOS_DRIVERS_COMMON_CAN_HXX_
#define _FREERTOS_DRIVERS_COMMON_CAN_HXX_

#include "Devtab.hxx"
#include "can_frame.h"
#include "nmranet_config.h"
#include "os/OS.hxx"
#include "executor/Notifiable.hxx"
#include "DeviceBuffer.hxx"

/** Private data for a can device */
class Can : public NonBlockNode
{
protected:
    /** Constructor
     * @param name device name in file system
     */
    Can(const char *name)
        : NonBlockNode(name)
        , txQ(os_mq_create(config_can_tx_buffer_size(),
                           sizeof(struct can_frame)))
        , rxQ(os_mq_create(config_can_rx_buffer_size(),
                           sizeof(struct can_frame)))
        , txBuf(DeviceBuffer<struct can_frame>::create(config_can_tx_buffer_size(), 
                                                       config_can_tx_buffer_size()/2))
        , rxBuf(DeviceBuffer<struct can_frame>::create(config_can_rx_buffer_size()))
        , overrunCount(0)
        , selInfoRd()
        , selInfoWr()
    {
    }    

    /** Destructor.
     */
    ~Can()
    {
        /** @todo (Stuart Baker) for completeness we should destroy the
         * txQ and rxQ here.
         */
    }

    virtual void enable() = 0; /**< function to enable device */
    virtual void disable() = 0; /**< function to disable device */
    virtual void tx_msg() = 0; /**< function to try and transmit a message */

    bool has_tx_buffer_space() OVERRIDE {
        return os_mq_num_pending(txQ) < config_can_tx_buffer_size();        
    }

    bool has_rx_buffer_data() OVERRIDE {
        return os_mq_num_pending(rxQ) != 0;
    }

    void flush_buffers() OVERRIDE; /**< called after disable */

    /** Sends an incoming frame to the receive buffer and notifies any waiting
     * process. Drops the frame if there is no receive buffer available. */
    void put_rx_msg_from_isr(const struct can_frame &f)
    {
        int woken;
        if (os_mq_send_from_isr(rxQ, &f, &woken) == OS_MQ_FULL)
        {
            overrunCount++;
        }
        if (readableNotify_) {
            readableNotify_->notify_from_isr();
            readableNotify_ = nullptr;
        }
    }

    /** Requests the next outgoing frame to be sent to the hardware.
     * @returns true is a frame was found.
     */
    bool get_tx_msg_from_isr(struct can_frame *f)
    {
        int woken;
        if (writableNotify_) {
            writableNotify_->notify_from_isr();
            writableNotify_= nullptr;
        }
        if (os_mq_receive_from_isr(txQ, f, &woken) == OS_MQ_NONE)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /** Requests the next outgoing frame to be sent to the hardware.
     * @returns true is a frame was found.
     */
    bool get_tx_msg(struct can_frame *f)
    {
        Notifiable* n = nullptr;
        portENTER_CRITICAL();
        if (writableNotify_) {
            n = writableNotify_;
            writableNotify_= nullptr;
        }
        portEXIT_CRITICAL();
        if (os_mq_timedreceive(txQ, f, 0) == OS_MQ_NONE)
        {
            if (n) n->notify();
            return true;
        }
        else
        {
            if (n) n->notify();
            return false;
        }
    }

    os_mq_t txQ; /**< transmit queue */
    os_mq_t rxQ; /**< receive queue */
    DeviceBuffer<struct can_frame> *txBuf; /**< transmit buffer */
    DeviceBuffer<struct can_frame> *rxBuf; /**< receive buffer */
    unsigned int overrunCount; /**< overrun count */

    SelectInfo selInfoRd; /**< select wakeup metadata for read active */
    SelectInfo selInfoWr; /**< select wakeup metadata for write active */

private:
    /** Read from a file or device.
    * @param file file reference for this device
    * @param buf location to place read data
    * @param count number of bytes to read
    * @return number of bytes read upon success, -1 upon failure with errno containing the cause
    */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;

    /** Write to a file or device.
    * @param file file reference for this device
    * @param buf location to find write data
    * @param count number of bytes to write
    * @return number of bytes written upon success, -1 upon failure with errno containing the cause
    */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) OVERRIDE;

    DISALLOW_COPY_AND_ASSIGN(Can);
};

#endif /* _FREERTOS_DRIVERS_COMMON_CAN_HXX_ */
