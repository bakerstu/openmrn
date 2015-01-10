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
 * \file I2C.hxx
 * This file implements a generic I2C device driver layer.
 *
 * @author Stuart W. Baker
 * @date 5 January 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_I2C_HXX_
#define _FREERTOS_DRIVERS_COMMON_I2C_HXX_

#include "BlockOrWakeUp.hxx"
#include "SimpleLog.hxx"
#include "Devtab.hxx"
#include "nmranet_config.h"
#include "os/OS.hxx"

/** Private data for an I2C device.  This device only supports blocking mode
 * transfers.
 */
class I2C : public Node
{
protected:
    /** Flags used to determine the kind of I2C transaction.
     */
    enum Flags
    {
        FLAG_START   = 0x01,
        FLAG_REPEAT_START = 0x02,
        FLAG_STOP    = 0x04,
        FLAG_RD      = 0x08
    };

    /** Metadata for an I2C transaction */
    struct Transaction
    {
        union
        {
            const unsigned char *tx_data;
            unsigned char *rx_data;
        };
        size_t count;
        int flags;
    };

    /** Constructor
     * @param name device name in file system
     */
    I2C(const char *name)
        : Node(name)
        , address(0)
    {
    }

    /** Destructor.
     */
    ~I2C()
    {
        HASSERT(0);
    }

    /** Method to transmit/receive the data.
     * @param t transaction to take place
     * @return 0 upon success or -1 with errno set
     */
    virtual int transfer(Transaction *t) = 0;

    /** Address of target I2C slave */
    long address;

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

    /** Request an ioctl transaction.
     * @param file file reference for this device
     * @param node node reference for this device
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;

    /** Discards all pending buffers. Called after disable(). */
    void flush_buffers() OVERRIDE {}

    DISALLOW_COPY_AND_ASSIGN(I2C);
};

#endif /* _FREERTOS_DRIVERS_COMMON_I2C_HXX_ */
