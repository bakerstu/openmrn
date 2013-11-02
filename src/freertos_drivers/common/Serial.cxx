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
 * \file Serial.cxx
 * This file implements a generic serial device driver layer.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#include <cstdint>
#include <fcntl.h>
#include "Devtab.hxx"
#include "Serial.hxx"

Devops Serial::ops = {Serial::open, Serial::close, Serial::read, Serial::write, Serial::ioctl};

/** Open a device.
 * @param file new file reference to this device
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode
 * @return 0 upon success, negative errno upon failure
 */
int Serial::open(File* file, const char *path, int flags, int mode)
{
    Serial *serial = (Serial*)file->dev->get_priv();

    serial->mutex.lock();
    file->node = serial;
    file->offset = 0;
    if (serial->references++ == 0)
    {
        serial->enable();
    }
    serial->mutex.unlock();
    
    return 0;
}

/** Close a device.
 * @param file file reference for this device
 * @param node node reference for this device
 * @return 0 upon success, negative errno upon failure
 */
int Serial::close(File *file, Node *node)
{
    Serial *serial = (Serial*)file->dev->get_priv();

    serial->mutex.lock();
    if (--serial->references == 0)
    {
        /* no more open references */
        serial->disable();
        /* flush the queues */
        int result;
        do
        {
            unsigned char data;
            result = os_mq_timedreceive(serial->txQ, &data, 0);
        } while (result == OS_MQ_NONE);
        do
        {
            unsigned char data;
            result = os_mq_timedreceive(serial->rxQ, &data, 0);
        } while (result == OS_MQ_NONE);
    }
    serial->mutex.unlock();
    
    return 0;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t Serial::read(File *file, void *buf, size_t count)
{
    Serial *serial = static_cast<Serial*>(file->node);
    unsigned char *data = (unsigned char*)buf;
    ssize_t result = 0;
    
    while (count)
    {
        if (file->flags & O_NONBLOCK)
        {
            if (os_mq_timedreceive(serial->rxQ, data, 0) == OS_MQ_TIMEDOUT)
            {
                /* no more data to receive */
                break;
            }
        }
        else
        {
            /* wait for data to come in */
            os_mq_receive(serial->rxQ, data);
        }

        count--;
        result++;
        data++;
    }
    
    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t Serial::write(File *file, const void *buf, size_t count)
{
    Serial *serial = static_cast<Serial*>(file->node);
    const unsigned char *data = (const unsigned char*)buf;
    ssize_t result = 0;
    
    while (count)
    {
        if (file->flags & O_NONBLOCK)
        {
            if (os_mq_timedsend(serial->txQ, data, 0) == OS_MQ_TIMEDOUT)
            {
                /* no more room in the buffer */
                break;
            }
        }
        else
        {
            /* wait for room in the queue */
            os_mq_send(serial->txQ, data);
        }
        serial->mutex.lock();
        serial->tx_char();
        serial->mutex.unlock();

        count--;
        result++;
        data++;
    }
    
    return result;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int Serial::ioctl(File *file, Node *node, unsigned long int key, unsigned long data)
{
    return 0;
}

