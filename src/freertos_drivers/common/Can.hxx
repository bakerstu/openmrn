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
public:
    static unsigned numReceivedPackets_;
    static unsigned numTransmittedPackets_;

protected:
    /** Constructor
     * @param name device name in file system
     * @param tx_buffer_size transmit buffer size in can frames
     * @param rx_buffer_size receive buffer size in can frames
     */
    Can(const char *name, size_t tx_buffer_size = config_can_tx_buffer_size(),
        size_t rx_buffer_size = config_can_rx_buffer_size())
        : NonBlockNode(name)
        , txBuf(DeviceBuffer<struct can_frame>::create(tx_buffer_size,
                                                       tx_buffer_size / 2))
        , rxBuf(DeviceBuffer<struct can_frame>::create(rx_buffer_size))
        , overrunCount(0)
        , busOffCount(0)
        , softErrorCount(0)
    {
    }

    /** Destructor.
     */
    ~Can()
    {
        txBuf->destroy();
        rxBuf->destroy();
    }

    void enable() override = 0; /**< function to enable device */
    void disable() override = 0; /**< function to disable device */
    virtual void tx_msg() = 0; /**< function to try and transmit a message */

    /** @todo (Stuart Baker) remove once we switch over to select().
     * @return true if there is space in the transmit buffer, false if transmit
     * buffers are full (i.e. a transmit would block).
     */
    bool has_tx_buffer_space() OVERRIDE {
        return txBuf->space();
    }

    /** @todo (Stuart Baker) remove once we switch over to select().
     * @return true if there is data in the receive buffer, false if all data
     * has been consumed.
     */
    bool has_rx_buffer_data() OVERRIDE {
        return rxBuf->pending();
    }

    void flush_buffers() OVERRIDE; /**< called after disable */

    DeviceBuffer<struct can_frame> *txBuf; /**< transmit buffer */
    DeviceBuffer<struct can_frame> *rxBuf; /**< receive buffer */
    unsigned int overrunCount; /**< overrun count */
    unsigned int busOffCount; /**< bus-off count */
    unsigned int softErrorCount; /**< soff error count */

protected:
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

private:
    DISALLOW_COPY_AND_ASSIGN(Can);
};

#endif /* _FREERTOS_DRIVERS_COMMON_CAN_HXX_ */
