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
 * \file nmranet_lo_if.c
 * This file defines the NMRAnet local or loopback interface.
 *
 * @author Stuart W. Baker
 * @date 24 September 2012
 */

#include <stdlib.h>
#include "if/nmranet_if.h"
#include "core/nmranet_node.h"
#include "core/nmranet_event.h"

/** Information about the interface. */
typedef struct
{
    NMRAnetIF nmranetIF; /**< generic NMRAnet interface info */
} NMRAnetLoIF;

/** Write a message onto the local bus.
 * @param nmranet_if interface to write message to
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
static int lo_write(NMRAnetIF *nmranet_if, uint16_t mti, node_id_t src, node_id_t dst, const void *data)
{
    switch (mti)
    {
        default:
            /* we don't care about these MTI's */
            break;
        case MTI_VERIFY_NODE_ID_ADDRESSED:
            /* fall through */
        case MTI_VERIFY_NODE_ID_GLOBAL:
            nmranet_node_packet(mti, src, dst, data);
            break;
        case MTI_EVENT_REPORT:
            nmranet_event_packet(mti, src, dst, data);
            break;
    }
    return 0;
}

/** Initialize the local loopback interface.
 * @return initialized interface, NULL upon failure
 */
NMRAnetIF *nmranet_lo_if_init(void)
{
    NMRAnetLoIF *lo_if = malloc(sizeof(NMRAnetLoIF));
    
    lo_if->nmranetIF.write = lo_write;

    return &lo_if->nmranetIF;
}

