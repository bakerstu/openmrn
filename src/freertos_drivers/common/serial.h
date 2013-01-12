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
 * \file serial.h
 * This file implements a generic serial device driver layer.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#ifndef _serial_h_
#define _serial_h_

#include "devtab.h"
#include "os/os.h"

/** Number of receive characters that are buffered in the serial driver.
 */
extern const size_t SERIAL_RX_BUFFER_SIZE;

/** Number of transmit characters that are buffered in the serial driver.
 */
extern const size_t SERIAL_TX_BUFFER_SIZE;

/** Private data for a can device */
typedef struct serial_priv
{
    void (*enable)(devtab_t *); /**< function to enable device */
    void (*disable)(devtab_t *); /**< function to disable device */
    void (*tx_char)(devtab_t *); /**< function to try and transmit a message */
    os_mutex_t mutex; /**< mutual exclusion for the device */
    os_mutex_t wrMutex; /**< mutual exclusion for reading the device */
    os_mutex_t rdMutex; /**< mutual exclusion for writing the device */
    node_t node;
    os_mq_t txQ;
    os_mq_t rxQ;
    unsigned int overrunCount;
} SerialPriv;

extern devops_t serial_ops;
/** device table entry for can device */
#define SERIAL_DEVTAB_ENTRY(_label, _name, _init, _priv) \
    DEVTAB_ENTRY(_label, _name, _init, &serial_ops, _priv)

/** intitailize the device 
 * @parem dev device to initialize
 * @return 0 upon success
 */
int serial_init(devtab_t *dev);

#endif /* _serial_h_ */
