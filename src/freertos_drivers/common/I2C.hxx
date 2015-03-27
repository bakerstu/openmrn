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
#include "i2c.h"
#include "i2c-dev.h"

/** Private data for an I2C device.  This device only supports blocking mode
 * transfers.
 */
class I2C : public Node
{
protected:
    /** Constructor
     * @param name device name in file system
     */
    I2C(const char *name)
        : Node(name)
    {
    }

    /** Destructor.
     */
    ~I2C()
    {
        HASSERT(0);
    }

    /** Method to transmit/receive the data.
     * @param msg message to transact.
     * @param stop produce a stop condition at the end of the transfer
     * @return bytes transfered upon success or -1 with errno set
     */
    virtual int transfer(struct i2c_msg *msg, bool stop) = 0;

private:
    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -errno upon failure
     */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -errno upon failure
     */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    /** Request an ioctl transaction.
     * @param file file reference for this device
     * @param key ioctl key
     * @param data key data
     * @return >= 0 upon success, -errno upon failure
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;

    /** Conduct multiple message transfers with one stop at the end.
     * @param msgs array of messages to transfer
     * @param num number of messages to transfer
     * @return total number of bytes transfered, -errno upon failure
     */
    int transfer_messages(struct i2c_msg *msgs, int num);

    /** Request an smbus (ioctl) transaction.
     * @param file file reference for this device
     * @param data smbus data
     * @return 0 upon success, -errno upon failure
     */
    int smbus(File *file, unsigned long data);    

    /** Discards all pending buffers. Called after disable(). */
    void flush_buffers() OVERRIDE {}

    /** Default constructor.
     */
    I2C();

    DISALLOW_COPY_AND_ASSIGN(I2C);
};

#endif /* _FREERTOS_DRIVERS_COMMON_I2C_HXX_ */
