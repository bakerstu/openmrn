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

#ifdef __cplusplus
extern "C" {
#endif

/** Number of receive CAN messages that are buffered in the CAN driver.
 */
extern const size_t CAN_RX_BUFFER_SIZE;

/** Number of transmit CAN messages that are buffered in the CAN driver.
 */
extern const size_t CAN_TX_BUFFER_SIZE;

/** Private data for a can device */
typedef struct can_priv {
  void (*enable)(devtab_t *);  /**< function to enable device */
  void (*disable)(devtab_t *); /**< function to disable device */
  void (*tx_msg)(devtab_t *);  /**< function to try and transmit a message */
  os_mutex_t mutex;            /**< mutual exclusion for the device */
  node_t node;
  os_mq_t txQ;
  os_mq_t rxQ;
  unsigned int overrunCount;
} CanPriv;

extern devops_t can_ops;
/** device table entry for can device */
#define CAN_DEVTAB_ENTRY(_label, _name, _init, _priv) \
  DEVTAB_ENTRY(_label, _name, _init, &can_ops, _priv)

/** initialize the device
 * @param dev device to initialize
 * @return 0 upon success
 */
int can_init(devtab_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* _can_h_ */
