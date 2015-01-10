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
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t I2C::read(File *file, void *buf, size_t count)
{
    unsigned char *data = (unsigned char*)buf;
    bool result;

    Transaction t = {data, count, FLAG_START | FLAG_STOP | FLAG_RD};
    lock_.lock();
    result = transfer(&t);
    lock_.unlock();

    return result == 0 ? count : -1;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t I2C::write(File *file, const void *buf, size_t count)
{
    const unsigned char *data = (const unsigned char*)buf;
    bool result;

    Transaction t = {data, count, FLAG_START | FLAG_STOP};
    lock_.lock();
    result = transfer(&t);
    lock_.unlock();

    return result == 0 ? count : -1;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int I2C::ioctl(File *file, unsigned long int key, unsigned long data)
{
    HASSERT(IOC_TYPE(key) == I2C_MAGIC);

    switch (key)
    {
        default:
            errno = EINVAL;
            return -1;
        case I2C_SLAVE:
            address = data;
            break;
        case I2C_RDWR:
            /* not yet implemented */
            HASSERT(0 && "I2C_RDWR not yet implemented");
    }
    return 0;
}

