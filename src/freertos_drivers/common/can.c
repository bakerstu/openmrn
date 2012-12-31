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
 * This file implements the generic filio.
 *
 * @author Stuart W. Baker
 * @date 28 December 2012
 */

#include <stdint.h>
#include "devtab.h"
#include "nmranet_can.h"
#include "os/os.h"

#define CAN_RX_BUFFER_SIZE 4
#define CAN_TX_BUFFER_SIZE 4

/* prototypes */
static int can_init(devtab_t *dev);
static int can_open(file_t* file, const char *path, int flags, int mode);
static int can_close(file_t *file, node_t *node);
static ssize_t can_read(file_t *file, void *buf, size_t count);
static ssize_t can_write(file_t *file, const void *buf, size_t count);
static int can_ioctl(file_t *file, node_t *node, int key, void *data);

/** Private data for a can device */
typedef struct can_priv
{
    os_mutex_t mutex; /**< mutual exclusion for the device */
    node_t node;
    struct can_frame rxBuf[CAN_RX_BUFFER_SIZE];
    struct can_frame txBuf[CAN_TX_BUFFER_SIZE];
    unsigned int rxCount;
    unsigned int txCount;
    unsigned int rxRdIndex;
    unsigned int rxWrIndex;
    unsigned int txRdIndex;
    unsigned int txWrIndex;
} CanPriv;

/** private data for the can device */
CanPriv can_private[1];

/** device operations for can */
static DEVOPS(can_ops, can_open, can_close, can_read, can_write, can_ioctl);

/** device table entry for can device */
DEVTAB_ENTRY(can0, "/dev/can0", can_init, &can_ops, &can_private[0]);

/** intitailize the device 
 * @parem dev device to initialize
 * @return 0 upon success
 */
static int can_init(devtab_t *dev)
{
    CanPriv *priv = dev->priv;
    
    os_mutex_init(&priv->mutex);
    priv->node.references = 0;
    priv->node.priv = priv;
    priv->rxCount = 0;
    priv->txCount = 0;
    priv->rxRdIndex = 0;
    priv->rxWrIndex = 0;
    priv->txRdIndex = 0;
    priv->txWrIndex = 0;
    
    return 0;
}

/** Open a device.
 * @param file new file reference to this device
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode
 * @return 0 upon success, negative errno upon failure
 */
static int can_open(file_t* file, const char *path, int flags, int mode)
{
    CanPriv *priv = file->dev->priv;

    os_mutex_lock(&priv->mutex);
    file->node = &priv->node;
    file->offset = 0;
    priv->node.references++;
    os_mutex_unlock(&priv->mutex);
    
    return 0;
}

/** Close a device.
 * @param file file reference for this device
 * @param node node reference for this device
 * @return 0 upon success, negative errno upon failure
 */
static int can_close(file_t *file, node_t *node)
{
    CanPriv *priv = file->dev->priv;

    os_mutex_lock(&priv->mutex);
    node->references--;
    os_mutex_unlock(&priv->mutex);
    
    return 0;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
static ssize_t can_read(file_t *file, void *buf, size_t count)
{
    return 0;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
static ssize_t can_write(file_t *file, const void *buf, size_t count)
{
    return 0;
}

/** Request and ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param ... key data
 */
static int can_ioctl(file_t *file, node_t *node, int key, void *data)
{
    return 0;
}

