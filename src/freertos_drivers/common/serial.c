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
 * \file serial.c
 * This file implements a generic serial device driver layer.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#include <stdint.h>
#include <fcntl.h>
#include "devtab.h"
#include "serial.h"

/* prototypes */
int serial_init(devtab_t* dev);
static int serial_open(file_t* file, const char* path, int flags, int mode);
static int serial_close(file_t* file, node_t* node);
static ssize_t serial_read(file_t* file, void* buf, size_t count);
static ssize_t serial_write(file_t* file, const void* buf, size_t count);
static int serial_ioctl(file_t* file, node_t* node, int key, void* data);

/** device operations for can */
DEVOPS(serial_ops, serial_open, serial_close, serial_read, serial_write,
       serial_ioctl);

/** intitailize the device
 * @parem dev device to initialize
 * @return 0 upon success
 */
int serial_init(devtab_t* dev)
{
    SerialPriv* priv = dev->priv;

    os_mutex_init(&priv->mutex);
    os_mutex_init(&priv->wrMutex);
    os_mutex_init(&priv->rdMutex);
    priv->node.references = 0;
    priv->node.priv = priv;
    priv->rxQ = os_mq_create(SERIAL_RX_BUFFER_SIZE, sizeof(unsigned char));
    priv->txQ = os_mq_create(SERIAL_TX_BUFFER_SIZE, sizeof(unsigned char));
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
static int serial_open(file_t* file, const char* path, int flags, int mode)
{
    SerialPriv* priv = file->dev->priv;

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
static int serial_close(file_t* file, node_t* node)
{
    SerialPriv* priv = file->dev->priv;

    os_mutex_lock(&priv->mutex);
    if (--node->references == 0) {
        /* no more open references */
        priv->disable(file->dev);
        /* flush the queues */
        int result;
        do {
            unsigned char data;
            result = os_mq_timedreceive(priv->txQ, &data, 0);
        } while (result == OS_MQ_NONE);
        do {
            unsigned char data;
            result = os_mq_timedreceive(priv->rxQ, &data, 0);
        } while (result == OS_MQ_NONE);
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
static ssize_t serial_read(file_t* file, void* buf, size_t count)
{
    SerialPriv* priv = file->dev->priv;
    unsigned char* data = buf;
    ssize_t result = 0;

    os_mutex_lock(&priv->rdMutex);
    while (count) {
        if (file->flags & O_NONBLOCK) {
            if (os_mq_timedreceive(priv->rxQ, data, 0) == OS_MQ_TIMEDOUT) {
                /* no more data to receive */
                break;
            }
        } else {
            /* wait for data to come in */
            os_mq_receive(priv->rxQ, data);
        }

        count--;
        result++;
        data++;
    }
    os_mutex_unlock(&priv->rdMutex);

    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno
 * containing the cause
 */
static ssize_t serial_write(file_t* file, const void* buf, size_t count)
{
    SerialPriv* priv = file->dev->priv;
    const unsigned char* data = buf;
    ssize_t result = 0;

    os_mutex_lock(&priv->wrMutex);
    while (count) {
        if (file->flags & O_NONBLOCK) {
            if (os_mq_timedsend(priv->txQ, data, 0) == OS_MQ_TIMEDOUT) {
                /* no more room in the buffer */
                break;
            }
        } else {
            /* wait for room in the queue */
            os_mq_send(priv->txQ, data);
        }
        priv->tx_char(file->dev);

        count--;
        result++;
        data++;
    }
    os_mutex_unlock(&priv->wrMutex);

    return result;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param ... key data
 */
static int serial_ioctl(file_t* file, node_t* node, int key, void* data)
{
    return 0;
}
