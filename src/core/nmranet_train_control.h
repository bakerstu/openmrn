/** \copyright
 * Copyright (c) 2012, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file nmranet_train_control.h
 * This file handles the NMRAnet datagram protocol for train control.
 *
 * @author Stuart W. Baker
 * @date 11 November 2012
 */

#ifndef _nmranet_train_control_h_
#define _nmranet_train_control_h_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Type of velocity inquiry or report. */
enum tc_velocity_kind
{
    VELOCITY_KIND_COMMANDED = 0x00, /**< commanded velocity */
    VELOCITY_KIND_INTENDED  = 0x01, /**< intended velocity */
    VELOCITY_KIND_ACTUAL    = 0x03 /**< actual velocity */
};

/** Process the datagram automatically for train control.
 * @param node node handle the datagram was received from
 * @param datagram datagram to process
 */
void nmranet_train_control_process(node_t node, Datagram *datagram);

/** Callback type for handling emergency stop of a node */
typedef void (*TCEstopCallback)(node_t node, node_id_t from);

/** Callback type for handling velocity of a node */
typedef void (*TCVelocityCallback)(node_t node, node_id_t from, float velocity);

/** Callback type for handling velocity inquire of a node */
typedef void (*TCVelocityInquireCallback)(node_t node, node_id_t from, int kind);

/** Callback type for handling velocity report of a node */
typedef void (*TCVelocityReportCallback)(node_t node, node_id_t from, int kind, float velocity);


#ifdef __cplusplus
}
#endif

#endif /* _nmranet_train_control_h_ */
