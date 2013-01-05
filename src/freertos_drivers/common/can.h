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
 * \file can.h
 * This file implements a generic can device driver layer.
 *
 * @author Stuart W. Baker
 * @date 28 December 2012
 */

#ifndef _can_h_
#define _can_h_

#include "devtab.h"
#include "os/os.h"
#include "nmranet_can.h"

#define CAN_RX_BUFFER_SIZE 4
#define CAN_TX_BUFFER_SIZE 4

/** Private data for a can device */
typedef struct can_priv
{
    void (*enable)(devtab_t *); /**< function to enable device */
    void (*disable)(devtab_t *); /**< function to disable device */
    /** function to try and transmit a message */
    int (*tx_msg)(devtab_t *, uint32_t, char, char, uint8_t, const uint8_t *);
    void (*lock)(devtab_t *); /**< mutual exclusion lock for device */
    void (*unlock)(devtab_t *); /**< mutual esclusion unlock for device */
    os_mutex_t mutex; /**< mutual exclusion for the device */
    node_t node;
    struct can_frame rxBuf[CAN_RX_BUFFER_SIZE];
    struct can_frame txBuf[CAN_TX_BUFFER_SIZE];
    unsigned char rxCount;
    unsigned char txCount;
    unsigned char rxRdIndex;
    unsigned char rxWrIndex;
    unsigned char txRdIndex;
    unsigned char txWrIndex;
    char rxOverrun;
} CanPriv;

extern devops_t can_ops;
/** device table entry for can device */
#define CAN_DEVTAB_ENTRY(_label, _name, _init, _priv) \
    DEVTAB_ENTRY(_label, _name, _init, &can_ops, _priv)

/** intitailize the device 
 * @parem dev device to initialize
 * @return 0 upon success
 */
int can_init(devtab_t *dev);

/** Pass a received message from lower layer to upper layer.  The device is
 * assumed to be locked prior to calling this method.
 * @param dev device to receive message to
 * @param id CAN identifier
 * @param eff set if an extended frame format, else 0
 * @param rtr set if a remote frame, else 0
 * @param dlc data length code, number of data bytes
 * @param data pointer to an array of data
 */
void can_rx_msg(devtab_t *dev, uint32_t id, char eff, char rtr, uint8_t dlc, uint8_t *data);

/** Device is ready for the next transmission.  The device is
 * assumed to be locked prior to calling this method.
 * @param dev device ready
 */
void can_tx_ready(devtab_t *dev);

#endif /* _can_h_ */
