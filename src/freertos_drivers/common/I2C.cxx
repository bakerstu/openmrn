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
 * \file I2C.cxx
 * This file implements a generic I2C device driver layer.
 *
 * @author Stuart W. Baker
 * @date 5 January 2015
 */

#include <cstdint>
#include <fcntl.h>
#include "Devtab.hxx"
#include "I2C.hxx"

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -errno upon failure
 */
ssize_t I2C::read(File *file, void *buf, size_t count)
{
    int result;

    if ((file->flags & O_ACCMODE) == O_WRONLY)
    {
        return -EBADF;
    }

    struct i2c_msg msg;
    msg.addr = (uintptr_t)file->priv;
    msg.flags = I2C_M_RD;
    msg.len = count;
    msg.buf = (uint8_t*)buf;

    lock_.lock();
    result = transfer(&msg, true);
    lock_.unlock();

    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -errno upon failure
 */
ssize_t I2C::write(File *file, const void *buf, size_t count)
{
    int result;

    if ((file->flags & O_ACCMODE) == O_RDONLY)
    {
        return -EBADF;
    }

    struct i2c_msg msg;
    msg.addr = (uintptr_t)file->priv;
    msg.flags = 0;
    msg.len = count;
    msg.buf = (uint8_t*)buf;

    lock_.lock();
    result = transfer(&msg, true);
    lock_.unlock();

    return result;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param key ioctl key
 * @param data key data
 * @return 0 upon success, -errno upon failure
 */
int I2C::ioctl(File *file, unsigned long int key, unsigned long data)
{
    HASSERT(IOC_TYPE(key) == I2C_MAGIC);

    switch (key)
    {
        default:
            return -EINVAL;
        case I2C_SLAVE:
            file->priv = (void*)data;
            break;
        case I2C_RDWR:
        {
            struct i2c_rdwr_ioctl_data *rdwr_data = (struct i2c_rdwr_ioctl_data*)data;
            return transfer_messages(rdwr_data->msgs, rdwr_data->nmsgs);
        }
        case I2C_SMBUS:
            return smbus(file, data);
    }
    return 0;
}

/** Conduct multiple message transfers with one stop at the end.
 * @param msgs array of messages to transfer
 * @param num number of messages to transfer
 * @return total number of bytes transfered, -errno upon failure
 */
int I2C::transfer_messages(struct i2c_msg *msgs, int num)
{
    HASSERT(num > 0);

    int count = 0;
    int result;

    lock_.lock();
    for (int i = 0; i < num; ++i)
    {
        count += msgs[i].len;
        result = transfer(msgs + i, (num == (i + 1)));
        if (result < 0)
        {
            lock_.unlock();
            return result;
        }
    }
    lock_.unlock();

    return  count;
}

/** Request an smbus (ioctl) transaction.
 * @param file file reference for this device
 * @param data smbus data
 * @return 0 upon success, -errno upon failure
 */
int I2C::smbus(File *file, unsigned long data)
{
    struct i2c_smbus_ioctl_data *sm_data = (struct i2c_smbus_ioctl_data *)data;

    /* check that we have a valid transaction type */
    if ((sm_data->size != I2C_SMBUS_QUICK) &&
        (sm_data->size != I2C_SMBUS_BYTE) &&
        (sm_data->size != I2C_SMBUS_BYTE_DATA) &&
        (sm_data->size != I2C_SMBUS_WORD_DATA) &&
        (sm_data->size != I2C_SMBUS_PROC_CALL) &&
        (sm_data->size != I2C_SMBUS_BLOCK_DATA) &&
        (sm_data->size != I2C_SMBUS_I2C_BLOCK_BROKEN) &&
        (sm_data->size != I2C_SMBUS_BLOCK_PROC_CALL) &&
        (sm_data->size != I2C_SMBUS_I2C_BLOCK_DATA))
    {
        return -EINVAL;
    }

    /* check that we have a read or write */
    if ((sm_data->read_write != I2C_SMBUS_READ) &&
        (sm_data->read_write != I2C_SMBUS_WRITE))
    {
        return -EINVAL;
    }

    /// @todo need to finish implementation (Stuart Baker)
    return 0;
}  

