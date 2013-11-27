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
 * \file NMRAnetIf.cxx
 * This file provides the generic NMRAnet interface.
 *
 * @author Stuart W. Baker
 * @date 2 October 2013
 */

#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/NMRAnetNode.hxx"

namespace NMRAnet
{

/** Process receive data.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 */
void If::rx_data(MTI mti, NodeHandle src, NodeID dst, Buffer *data)
{
    const uint8_t *bytes = (uint8_t*)data->start();

    Node::mutex.lock();
    if (dst != 0)
    {
        /* !!! Because the message is addressed, the downstream protocol is
         * !!! responsible for freeing any data buffers.
         */
        RBTree <NodeID, Node*>::Node *id_node = Node::idTree.find(dst);
        if (id_node)
        {
            Node *node = id_node->value;
            HASSERT(node->nmranetIf == this);
            /* we own this id */
            if (node->state != Node::UNINITIALIZED)
            {
                /* the node is initialized */
                switch (mti)
                {
                    default:
                        if (data)
                        {
                            data->free();
                        }
                        break;
                    case MTI_PROTOCOL_SUPPORT_INQUIRY:
                        node->protocol_support_reply(src);
                        break;
                    case MTI_VERIFY_NODE_ID_ADDRESSED:
                        node->verify_id_number();
                        break;
                    case MTI_IDENT_INFO_REQUEST:
                        node->ident_info_reply(src);
                        break;
                    case MTI_DATAGRAM_REJECTED: /* fall through */
                    case MTI_DATAGRAM_OK:       /* fall through */
                    case MTI_DATAGRAM:
                        node->Datagram::packet(mti, src, data);
                        break;
                    case MTI_STREAM_INITIATE_REQUEST:
                    case MTI_STREAM_INITIATE_REPLY:
                    case MTI_STREAM_DATA:
                    case MTI_STREAM_PROCEED:
                    case MTI_STREAM_COMPLETE:
                        node->Stream::packet(mti, src, data);
                        break;
                    case MTI_EVENTS_IDENTIFY_ADDRESSED:
                        HASSERT(data == NULL);
                        //nmranet_event_packet_addressed(mti, id_node, data);
                        break;
                }
            }
        }
        else
        {
            if (data)
            {
                data->free();
            }
        }
    }
    else
    {
        /* global message, handle subscribe based protocols first */
        switch (mti)
        {
            case MTI_CONSUMER_IDENTIFY:            /* fall through */
            case MTI_CONSUMER_IDENTIFY_RANGE:      /* fall through */
            case MTI_CONSUMER_IDENTIFIED_UNKNOWN:  /* fall through */
            case MTI_CONSUMER_IDENTIFIED_VALID:    /* fall through */
            case MTI_CONSUMER_IDENTIFIED_INVALID:  /* fall through */
            case MTI_CONSUMER_IDENTIFIED_RESERVED: /* fall through */
            case MTI_PRODUCER_IDENTIFY:            /* fall through */
            case MTI_PRODUCER_IDENTIFY_RANGE:      /* fall through */
            case MTI_PRODUCER_IDENTIFIED_UNKNOWN:  /* fall through */
            case MTI_PRODUCER_IDENTIFIED_VALID:    /* fall through */
            case MTI_PRODUCER_IDENTIFIED_INVALID:  /* fall through */
            case MTI_PRODUCER_IDENTIFIED_RESERVED: /* fall through */
            case MTI_EVENTS_IDENTIFY_GLOBAL:       /* fall through */
            case MTI_EVENT_REPORT:
                //nmranet_event_packet_global(mti, src, data);
                break;
            default:
                /* global message, deliver all, non-subscribe */
                for (RBTree <NodeID, Node*>::Node * id_node = Node::idTree.first();
                     id_node != NULL;
                     id_node = Node::idTree.next(id_node))
                {
                    Node *node = id_node->value;
                    if (node->state != Node::UNINITIALIZED)
                    {
                        /* the node is initialized */
                        switch (mti)
                        {
                            default:
                                break;
                            case MTI_VERIFY_NODE_ID_GLOBAL:
                                if (data != NULL)
                                {
                                    NodeID id = ((NodeID)bytes[5] <<  0) +
                                                ((NodeID)bytes[4] <<  8) +
                                                ((NodeID)bytes[3] << 16) +
                                                ((NodeID)bytes[2] << 24) +
                                                ((NodeID)bytes[1] << 32) +
                                                ((NodeID)bytes[0] << 40);
                                    if (id != node->nodeID)
                                    {
                                        /* not a match, keep looking */
                                        continue;
                                    }
                                }
                                /* we own this id, it is initialized, respond */
                                node->verify_id_number();
                                break;
                        }
                    }
                }
                break;
        }
        /* global messages don't take possession of and free their data */
        if (data)
        {
            data->free();
        }
    }
    Node::mutex.unlock();
}

}; /* namespace NMRAnet */


