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
#include "devtab.h"
#include "can.h"
#include "nmranet_can.h"

#define CAN_RX_BUFFER_SIZE 4
#define CAN_TX_BUFFER_SIZE 4

/* prototypes */
int can_init(devtab_t *dev);
static int can_open(file_t* file, const char *path, int flags, int mode);
static int can_close(file_t *file, node_t *node);
static ssize_t can_read(file_t *file, void *buf, size_t count);
static ssize_t can_write(file_t *file, const void *buf, size_t count);
static int can_ioctl(file_t *file, node_t *node, int key, void *data);

/** device operations for can */
DEVOPS(can_ops, can_open, can_close, can_read, can_write, can_ioctl);

/** intitailize the device 
 * @parem dev device to initialize
 * @return 0 upon success
 */
int can_init(devtab_t *dev)
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
    priv->rxOverrun = 0;
    
    return 0;
}

/** Pass a received message from lower layer to upper layer.  The device is
 * assumed to be locked prior to calling this method.
 * assumed to be dissabled during this time.
 * @param dev device to receive message to
 * @param id CAN identifier
 * @param eff set if an extended frame format, else 0
 * @param rtr set if a remote frame, else 0
 * @param dlc data length code, number of data bytes
 * @param data pointer to an array of data
 */
void can_rx_msg(devtab_t *dev, uint32_t id, char eff, char rtr, uint8_t dlc, uint8_t *data)
{
    CanPriv *priv = dev->priv;
    
    if (priv->rxCount < CAN_RX_BUFFER_SIZE)
    {
        priv->rxBuf[priv->rxWrIndex].can_id = id;
        priv->rxBuf[priv->rxWrIndex].can_dlc = dlc;
        priv->rxBuf[priv->rxWrIndex].can_rtr = rtr;
        priv->rxBuf[priv->rxWrIndex].can_eff = eff;
        priv->rxBuf[priv->rxWrIndex].can_err = 0;
        memcpy(priv->rxBuf[priv->rxWrIndex].data, data, dlc);
        
        priv->rxCount++;
        priv->rxWrIndex++;
        if (priv->rxWrIndex >= CAN_TX_BUFFER_SIZE)
        {
            priv->rxWrIndex = 0;
        }
    }
    else
    {
        /* no room left, indicate an overrun */
        priv-> rxOverrun = 1;
    }
}

/** Device is ready for the next transmission.  The device is
 * assumed to be locked prior to calling this method.
 * @param dev device ready
 */
void can_tx_ready(devtab_t *dev)
{
    CanPriv *priv = dev->priv;

    if (priv->txCount)
    {
        int result = priv->tx_msg(dev,
                                  priv->txBuf[priv->txRdIndex].can_id,
                                  priv->txBuf[priv->txRdIndex].can_eff,
                                  priv->txBuf[priv->txRdIndex].can_rtr,
                                  priv->txBuf[priv->txRdIndex].can_dlc,
                                  priv->txBuf[priv->txRdIndex].data);
        if (result)
        {
            priv->txCount--;
            priv->txRdIndex++;
            if (priv->txRdIndex >= CAN_TX_BUFFER_SIZE)
            {
                priv->txRdIndex = 0;
            }
        }
    }
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
    if (priv->node.references++ == 0)
    {
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
static int can_close(file_t *file, node_t *node)
{
    CanPriv *priv = file->dev->priv;

    os_mutex_lock(&priv->mutex);
    if (--node->references == 0)
    {
        priv->disable(file->dev);
    }
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
    CanPriv *priv = file->dev->priv;
    struct can_frame *can_frame = buf;
    ssize_t result = 0;
    
    os_mutex_lock(&priv->mutex);
    
    while (count >= sizeof(struct can_frame))
    {
        priv->lock(file->dev);
        if (priv->rxCount)
        {
            /* grab message */
            memcpy(can_frame, &priv->txBuf[priv->rxRdIndex], sizeof(struct can_frame));
        
            priv->rxCount--;
            priv->rxRdIndex++;
            if (priv->rxRdIndex >= CAN_TX_BUFFER_SIZE)
            {
                priv->rxRdIndex = 0;
            }
        }
        priv->unlock(file->dev);

        count -= sizeof(struct can_frame);
        result += sizeof(struct can_frame);
        can_frame++;
    }
    
    os_mutex_unlock(&priv->mutex);
    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
static ssize_t can_write(file_t *file, const void *buf, size_t count)
{
    CanPriv *priv = file->dev->priv;
    const struct can_frame *can_frame = buf;
    ssize_t result = 0;
    
    os_mutex_lock(&priv->mutex);
    
    while (count >= sizeof(struct can_frame))
    {
        priv->lock(file->dev);
        if (priv->txCount >= CAN_TX_BUFFER_SIZE)
        {
            /* no room left in the buffer */
            priv->unlock(file->dev);
            break;
        }
        if (priv->txCount == 0)
        {
            /* first try placing message directly in the hardware */
            int result = priv->tx_msg(file->dev,
                                      can_frame->can_id,
                                      can_frame->can_eff,
                                      can_frame->can_rtr,
                                      can_frame->can_dlc,
                                      can_frame->data);
            if (result > 0)
            {
                /* message placed in hardware successfully */
                priv->unlock(file->dev);
                continue;
            }
        }
        
        /* buffer message */
        memcpy(&priv->txBuf[priv->txWrIndex], can_frame, sizeof(struct can_frame));
        
        priv->txCount++;
        priv->txWrIndex++;
        if (priv->txWrIndex >= CAN_TX_BUFFER_SIZE)
        {
            priv->txWrIndex = 0;
        }
        priv->unlock(file->dev);

        count -= sizeof(struct can_frame);
        result += sizeof(struct can_frame);
        can_frame++;
    }
    
    os_mutex_unlock(&priv->mutex);
    return result;
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

