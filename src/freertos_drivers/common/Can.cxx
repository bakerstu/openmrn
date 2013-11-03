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
 * \file Can.cxx
 * This file implements a generic can device driver layer.
 *
 * @author Stuart W. Baker
 * @date 28 December 2012
 */

#include <cstdint>
#include <fcntl.h>
#include "Devtab.hxx"
#include "Can.hxx"
#include "nmranet_can.h"
#include "can_ioctl.h"

Devops Can::ops = {Can::open, Can::close, Can::read, Can::write, Can::ioctl};

/** Open a device.
 * @param file new file reference to this device
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode
 * @return 0 upon success, negative errno upon failure
 */
int Can::open(File* file, const char *path, int flags, int mode)
{
    Can *can = (Can*)file->dev->get_priv();

    can->mutex.lock();
    file->node = can;
    file->offset = 0;
    if (can->references++ == 0)
    {
        can->enable();
    }
    can->mutex.unlock();
    
    return 0;
}

/** Close a device.
 * @param file file reference for this device
 * @param node node reference for this device
 * @return 0 upon success, negative errno upon failure
 */
int Can::close(File *file, Node *node)
{
    Can *can = (Can*)file->dev->get_priv();

    can->mutex.lock();
    if (--can->references == 0)
    {
        can->disable();
    }
    can->mutex.unlock();
    
    return 0;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t Can::read(File *file, void *buf, size_t count)
{
    Can *can = static_cast<Can*>(file->node);
    struct can_frame *can_frame = (struct can_frame*)buf;
    ssize_t result = 0;
    
    while (count >= sizeof(struct can_frame))
    {
        if (file->flags & O_NONBLOCK)
        {
            if (os_mq_timedreceive(can->rxQ, can_frame, 0) == OS_MQ_TIMEDOUT)
            {
                /* no more data to receive */
                break;
            }
        }
        else
        {
            /* wait for data to come in */
            os_mq_receive(can->rxQ, can_frame);
        }

        count -= sizeof(struct can_frame);
        result += sizeof(struct can_frame);
        can_frame++;
    }
    
    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t Can::write(File *file, const void *buf, size_t count)
{
    Can *can = static_cast<Can*>(file->node);
    const struct can_frame *can_frame = (const struct can_frame*)buf;
    ssize_t result = 0;
        
    while (count >= sizeof(struct can_frame))
    {
        if (file->flags & O_NONBLOCK)
        {
            if (os_mq_timedsend(can->txQ, can_frame, 0) == OS_MQ_TIMEDOUT)
            {
                /* no more room in the buffer */
                break;
            }
        }
        else
        {
            /* wait for room in the queue */
            os_mq_send(can->txQ, can_frame);
        }
        can->mutex.lock();
        can->tx_msg();
        can->mutex.unlock();

        count -= sizeof(struct can_frame);
        result += sizeof(struct can_frame);
        can_frame++;
    }
    
    return result;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int Can::ioctl(File *file, Node *node, unsigned long int key, unsigned long data)
{
    Can *can = static_cast<Can*>(file->node);

    /* sanity check to be sure we have a valid key for this devide */
    HASSERT(IOC_TYPE(key) == CAN_IOC_MAGIC);
    HASSERT(IOC_SIZE(key) == sizeof(CanActiveCallback));

    const CanActiveCallback *can_active_callback = (CanActiveCallback*)data;

    switch (key)
    {
        default:
            return -EINVAL;
        case CAN_IOC_READ_ACTIVE:
            can->read_callback = can_active_callback->callback;
            can->readContext = can_active_callback->context;
            break;
        case CAN_IOC_WRITE_ACTIVE:
            can->write_callback = can_active_callback->callback;
            can->writeContext = can_active_callback->context;
            break;
    }

    return 0;
}

