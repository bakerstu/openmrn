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
 * \file Serial.hxx
 * This file implements a generic serial device driver layer.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#ifndef _FREERTOS_DRIVERS_COMMON_SERIAL_HXX_
#define _FREERTOS_DRIVERS_COMMON_SERIAL_HXX_

#include "BlockOrWakeUp.hxx"
#include "SimpleLog.hxx"
#include "Devtab.hxx"
#include "nmranet_config.h"
#include "os/OS.hxx"
#include "DeviceBuffer.hxx"

/** Private data for a serial device */
class Serial : public Node
{
protected:
    /** Constructor
     * @param name device name in file system
     * @param tx_buffer_size transmit buffer size in bytes
     * @param rx_buffer_size receive buffer size in bytes
     */
    Serial(const char *name,
           size_t tx_buffer_size = config_serial_tx_buffer_size(),
           size_t rx_buffer_size = config_serial_rx_buffer_size())
        : Node(name)
        , txBuf(tx_buffer_size ? DeviceBuffer<uint8_t>::create(tx_buffer_size,
                                                               tx_buffer_size/2)
                               : NULL)
        , rxBuf(rx_buffer_size ? DeviceBuffer<uint8_t>::create(rx_buffer_size)
                               : NULL)
        , overrunCount(0)
    {
    }    

    /** Destructor.
     */
    ~Serial()
    {
        if (txBuf)
        {
            txBuf->destroy();
        }
        if (rxBuf)
        {
            rxBuf->destroy();
        }
    }

    /** Function to try and transmit a character.
     */
    virtual void tx_char() = 0;

    DeviceBuffer<uint8_t> *txBuf; /**< transmit buffer */
    DeviceBuffer<uint8_t> *rxBuf; /**< receive buffer */
    unsigned int overrunCount; /**< overrun count */

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

    /** Request an ioctl transaction
     * @param file file reference for this device
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;
    
    /** Device select method. Default implementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) OVERRIDE;

    /** Discards all pending buffers. Called after disable(). */
    void flush_buffers() OVERRIDE;

private:
    DISALLOW_COPY_AND_ASSIGN(Serial);
};

#endif /* _FREERTOS_DRIVERS_COMMON_SERIAL_HXX_ */
