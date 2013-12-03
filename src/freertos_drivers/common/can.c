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
 * \file can.c
 * This file implements a generic can device driver layer.
 *
 * @author Stuart W. Baker
 * @date 28 December 2012
 */

#include <stdint.h>
#include <fcntl.h>
#include "devtab.h"
#include "can.h"
#include "nmranet_can.h"

/* prototypes */
int can_init(devtab_t* dev);
static int can_open(file_t* file, const char* path, int flags, int mode);
static int can_close(file_t* file, node_t* node);
static ssize_t can_read(file_t* file, void* buf, size_t count);
static ssize_t can_write(file_t* file, const void* buf, size_t count);
static int can_ioctl(file_t* file, node_t* node, int key, void* data);

/** device operations for can */
DEVOPS(can_ops, can_open, can_close, can_read, can_write, can_ioctl);

/** intitailize the device
 * @parem dev device to initialize
 * @return 0 upon success
 */
int can_init(devtab_t* dev)
{
    CanPriv* priv = dev->priv;

    os_mutex_init(&priv->mutex);
    priv->node.references = 0;
    priv->node.priv = priv;
    priv->rxQ = os_mq_create(CAN_RX_BUFFER_SIZE, sizeof(struct can_frame));
    priv->txQ = os_mq_create(CAN_TX_BUFFER_SIZE, sizeof(struct can_frame));
    priv->overrunCount = 0;

    return 0;
}

/** Open a device.
 * @param file new file reference to this device
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode
 * @return 0 upon success, negative errno upon failure
 */
static int can_open(file_t* file, const char* path, int flags, int mode)
{
    CanPriv* priv = file->dev->priv;

    os_mutex_lock(&priv->mutex);
    file->node = &priv->node;
    file->offset = 0;
    if (priv->node.references++ == 0) {
        priv->enable(file->dev);
    }
    os_mutex_unlock(&priv->mutex);

    return 0;
}

/** Close a device.
 * @param file file reference for this device
 * @param node node reference for this device
 * @return 0 upon success, negative errno upon failure
 */
static int can_close(file_t* file, node_t* node)
{
    CanPriv* priv = file->dev->priv;

    os_mutex_lock(&priv->mutex);
    if (--node->references == 0) {
        priv->disable(file->dev);
    }
    os_mutex_unlock(&priv->mutex);

    return 0;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno
 * containing the cause
 */
static ssize_t can_read(file_t* file, void* buf, size_t count)
{
    CanPriv* priv = file->dev->priv;
    struct can_frame* can_frame = buf;
    ssize_t result = 0;

    os_mutex_lock(&priv->mutex);
    int flags = file->flags;
    os_mutex_unlock(&priv->mutex);

    while (count >= sizeof(struct can_frame)) {
        if (flags & O_NONBLOCK) {
            if (os_mq_timedreceive(priv->rxQ, can_frame, 0) == OS_MQ_TIMEDOUT) {
                /* no more data to receive */
                break;
            }
        } else {
            /* wait for data to come in */
            os_mq_receive(priv->rxQ, can_frame);
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
 * @return number of bytes written upon success, -1 upon failure with errno
 * containing the cause
 */
static ssize_t can_write(file_t* file, const void* buf, size_t count)
{
    CanPriv* priv = file->dev->priv;
    const struct can_frame* can_frame = buf;
    ssize_t result = 0;

    os_mutex_lock(&priv->mutex);
    int flags = file->flags;
    os_mutex_unlock(&priv->mutex);

    while (count >= sizeof(struct can_frame)) {
        if (flags & O_NONBLOCK) {
            if (os_mq_timedsend(priv->txQ, can_frame, 0) == OS_MQ_TIMEDOUT) {
                /* no more room in the buffer */
                break;
            }
        } else {
            /* wait for room in the queue */
            os_mq_send(priv->txQ, can_frame);
        }
        priv->tx_msg(file->dev);

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
 * @param ... key data
 */
static int can_ioctl(file_t* file, node_t* node, int key, void* data)
{
    return 0;
}
