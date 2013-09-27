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
 * \file NMRAnetIf.hxx
 * This file provides the generic NMRAnet interface.
 *
 * @author Stuart W. Baker
 * @date 18 September 2013
 */

#ifndef _NMRAnetIf_hxx_
#define _NMRAnetIf_hxx_

#include <cstdint>

#include "utils/macros.h"

namespace NMRAnet
{

typedef uint64_t NodeID;
typedef uint16_t NodeAlias;

struct NodeHandle
{
    NodeID id;
    NodeAlias alias;
};

/** The generic interface for NMRAnet network interfaces
 */
class If
{
public:
    /** Constructor.
     */
    If(NodeID node_id)
        : nodeID(node_id)
    {
    }

    /** Known Message type indicators.
     */
    enum MTI
    {
        MTI_NONE                      = 0x0000, /**< invalid MTI */
        MTI_INITIALIZATION_COMPLETE   = 0x0100, /**< initialization complete */
        MTI_VERIFY_NODE_ID_ADDRESSED  = 0x0488, /**< verify a Node ID */
        MTI_VERIFY_NODE_ID_GLOBAL     = 0x0490, /**< verify a Node ID globally */
        MTI_VERIFIED_NODE_ID_NUMBER   = 0x0170, /**< respond to a verify Node ID request */
        MTI_OPTIONAL_INTERACTION_REJECTED = 0x0068, /**< rejected request */
        MTI_TERMINATE_DUE_TO_ERROR    = 0x00A8, /**< terminate due to some error */
        MTI_PROTOCOL_SUPPORT_INQUIRY  = 0x0828, /**< inquire on supported protocols */
        MTI_PROTOCOL_SUPPORT_REPLY    = 0x0668, /**< reply with supported protocols */
        MTI_CONSUMER_IDENTIFY         = 0x08F4, /**< query about consumers */
        MTI_CONSUMER_IDENTIFY_RANGE   = 0x04A4, /**< query about a range of consumers */
        MTI_CONSUMER_IDENTIFY_UNKNOWN = 0x04C7, /**< consumer broadcast, validity unknown */
        MTI_CONSUMER_IDENTIFY_VALID   = 0x04C4, /**< consumer broadcast, valid state */
        MTI_CONSUMER_IDENTIFY_INVALID = 0x04C5, /**< consumer broadcast, invalid state */
        MTI_CONSUMER_IDENTIFY_RESERVED = 0x04C6, /**< reserved for future use */
        MTI_PRODUCER_IDENTIFY         = 0x0914, /**< query about producers */
        MTI_PRODUCER_IDENTIFY_RANGE   = 0x0524, /**< query about a range of producers */
        MTI_PRODUCER_IDENTIFY_UNKNOWN = 0x0547, /**< producer broadcast, validity unknown */
        MTI_PRODUCER_IDENTIFY_VALID   = 0x0544, /**< producer broadcast, valid state */
        MTI_PRODUCER_IDENTIFY_INVALID = 0x0545, /**< producer broadcast, invalid state */
        MTI_PRODUCER_IDENTIFY_RESERVED = 0x0546, /**< reserved for future use */
        MTI_EVENTS_IDENTIFY_ADDRESSED = 0x0968, /**< request identify all of a node's events */
        MTI_EVENTS_IDENTIFY_GLOBAL    = 0x0970, /**< request identify all of every node's events */
        MTI_LEARN_EVENT               = 0x0594, /**< */
        MTI_EVENT_REPORT              = 0x05B4, /**< */
        MTI_XPRESSNET                 = 0x09C0, /**< */
        MTI_IDENT_INFO_REQUEST        = 0x0DE8, /**< request node identity */
        MTI_IDENT_INFO_REPLY          = 0x0A08, /**< node identity reply */
        MTI_DATAGRAM                  = 0x1C48, /**< datagram */
        MTI_DATAGRAM_OK               = 0x0A28, /**< datagram received okay */
        MTI_DATAGRAM_REJECTED         = 0x0A48, /**< datagram rejected by receiver */
        MTI_STREAM_DATA               = 0x1F88, /**< stream data */
    };

    /** Write a message onto the interface.
     * @param mti Message Type Indicator
     * @param src source node ID, 0 if unavailable
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    virtual int write(uint16_t MTI, NodeID src, NodeHandle dst, const void *data);

protected:
    ~If();

private:
    /** 48-bit NMRAnet node id associated with this interface */
    NodeID nodeID;

    DISALLOW_COPY_AND_ASSIGN(If);
};

}; /* namespace NMRAnet */

#endif /* _NMRAnetIf_hxx_ */
