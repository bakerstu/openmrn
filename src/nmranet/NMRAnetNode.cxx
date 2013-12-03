/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file NMRAnetNode.cxx
 * This file defines NMRAnet nodes.
 *
 * @author Stuart W. Baker
 * @date 29 September 2013
 */

#include "nmranet/NMRAnetNode.hxx"
#include "utils/BufferQueue.hxx"

namespace NMRAnet
{

// Temporary link to the event handler code
void nmranet_identify_consumers(Node* node, uint64_t event, uint64_t mask);
void nmranet_identify_producers(Node* node, uint64_t event, uint64_t mask);


OSMutex Node::mutex;
RBTree <NodeID, Node*> Node::idTree;

/** Obtain a @ref Node instance from its Node ID.
 * @param node_id Node ID to lookup
 * @return @ref Node instance, else NULL if not found
 */
Node *Node::find(NodeID node_id)
{
    mutex.lock();
    RBTree <NodeID, Node*>::Node *node = idTree.find(node_id);
    mutex.unlock();
    if (node)
    {
        return node->value;
    }
    return NULL;
}

/** Move node into the initialized state.
 */
void Node::initialized()
{
    mutex.lock();
    if (state != INITIALIZED)
    {
        state = INITIALIZED;
        Buffer *buffer = buffer_alloc(6);
        uint8_t *data = (uint8_t*)buffer->start();
        
        data[0] = (nodeID >> 40) & 0xff;
        data[1] = (nodeID >> 32) & 0xff;
        data[2] = (nodeID >> 24) & 0xff;
        data[3] = (nodeID >> 16) & 0xff;
        data[4] = (nodeID >>  8) & 0xff;
        data[5] = (nodeID >>  0) & 0xff;
        
        buffer->advance(6);
        
        write(If::MTI_INITIALIZATION_COMPLETE, {0, 0}, buffer);

        /* identify all of the events this node produces and consumes */
        nmranet_identify_consumers(this, 0, 0);
        nmranet_identify_producers(this, 0, 0);
    }
    mutex.unlock();
}

/** Send a verify node id number message.
 */
void Node::verify_id_number()
{
    Buffer *buffer = buffer_alloc(6);
    uint8_t *data = (uint8_t*)buffer->start();
    
    data[0] = (nodeID >> 40) & 0xff;
    data[1] = (nodeID >> 32) & 0xff;
    data[2] = (nodeID >> 24) & 0xff;
    data[3] = (nodeID >> 16) & 0xff;
    data[4] = (nodeID >>  8) & 0xff;
    data[5] = (nodeID >>  0) & 0xff;
    
    buffer->advance(6);
    
    write(If::MTI_VERIFIED_NODE_ID_NUMBER, {0, 0}, buffer);
}

/** Send an ident info reply message.
 * @param dst destination Node ID to respond to
 */
void Node::ident_info_reply(NodeHandle dst)
{
    /* macro for condensing the size calculation code */
    #define ADD_STRING_SIZE(_str, _max)          \
    {                                            \
        if ((_str))                              \
        {                                        \
            size_t len = strlen((_str));         \
            size += len > (_max) ? (_max) : len; \
        }                                        \
    }

    /* macro for condensing the string insertion  code */
    #define INSERT_STRING(_str, _max)              \
    {                                              \
        if ((_str))                                \
        {                                          \
            size_t len = strlen((_str));           \
            len = len > (_max) ? (_max) : len;     \
            memcpy(pos, (_str), len);              \
            pos[len] = '\0';                       \
            pos += len + 1;                        \
        }                                          \
        else                                       \
        {                                          \
            pos[0] = '\0';                         \
            pos++;                                 \
        }                                          \
    }
    
    /* we make this static so that it does not use up stack */
    /** @todo (Stuart Baker) if this could ever be accessed from more than one
     * thread, we will need a lock
     */
    static char ident[8+40+40+20+20+62+63];
    char       *pos = ident;
    size_t      size = 8;

    ADD_STRING_SIZE(MANUFACTURER, 40);
    ADD_STRING_SIZE(model, 40);
    ADD_STRING_SIZE(HARDWARE_REV, 20);
    ADD_STRING_SIZE(SOFTWARE_REV, 20);
    ADD_STRING_SIZE(userName, 62);
    ADD_STRING_SIZE(userDescription, 63);

    pos[0] = SIMPLE_NODE_IDENT_VERSION_A;
    pos++;
    
    INSERT_STRING(MANUFACTURER, 40);
    INSERT_STRING(model, 40);
    INSERT_STRING(HARDWARE_REV, 20);
    INSERT_STRING(SOFTWARE_REV, 20);

    pos[0] = SIMPLE_NODE_IDENT_VERSION_B;
    pos++;

    INSERT_STRING(userName, 62);
    INSERT_STRING(userDescription, 63);

    for (int index = 0; size; )
    {
        size_t segment_size = size > 6 ? 6 : size;
        Buffer *buffer = buffer_alloc(6);
        memcpy(buffer->start(), ident + index, segment_size);
        buffer->advance(segment_size);
        write(If::MTI_IDENT_INFO_REPLY, dst, buffer);
        size -= segment_size;
        index += segment_size;
    }
}

/** Send an protocols supported reply message.
 * @param dst destination Node ID to respond to
 */
void Node::protocol_support_reply(NodeHandle dst)
{
    /** @todo (Stuart Baker) this needs to be updated as additional protocols
     * are supported
     */
    uint64_t protocols = PROTOCOL_IDENTIFICATION |
                         DATAGRAM |
                         EVENT_EXCHANGE |
                         SIMPLE_NODE_INFORMATION |
                         MEMORY_CONFIGURATION |
                         CDI;

    Buffer *buffer = buffer_alloc(6);
    uint8_t *bytes = (uint8_t*)buffer->start();
    
    bytes[0] = (protocols >> 40) & 0xff;
    bytes[1] = (protocols >> 32) & 0xff;
    bytes[2] = (protocols >> 24) & 0xff;
    bytes[3] = (protocols >> 16) & 0xff;
    bytes[4] = (protocols >>  8) & 0xff;
    bytes[5] = (protocols >>  0) & 0xff;
    
    buffer->advance(6);

    write(If::MTI_PROTOCOL_SUPPORT_REPLY, dst, buffer);
}

/** Write a message from a node.  We should not have a mutex lock at
 * this at this point.
 * @param mti Message Type Indicator
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int Node::write_unlocked(If::MTI mti, NodeHandle dst, Buffer *data)
{
    /* It is important to note that we unlock the mutex before sending
     * data to an interface.  This is required for local nodes such that
     * we don't have mutex recursion.  This is required for remote nodes so
     * that if the interface write would block, other nodes may continue to
     * send messages using the interface.  NMRAnet does not guarantee message
     * sequencing.
     */

    if (dst.id == 0 && dst.alias == 0)
    {
        /* broacast message */
        data->reference();
        nmranetIf->rx_data(mti, {nodeID, 0}, 0, data);
        nmranetIf->if_write(mti, nodeID, dst, data);
    }
    else
    {
        mutex.lock();
        RBTree <NodeID, Node*>::Node *node = idTree.find(dst.id);;
        mutex.unlock();

        if (node)
        {
            /* loop-back */
            nmranetIf->rx_data(mti, {nodeID, 0}, dst.id, data);
        }
        else
        {
            nmranetIf->if_write(mti, nodeID, dst, data);
        }
    }

    return 0;
}

/** Write a message from a node.  We should already have a mutex lock at this
 * at this point.
 * @param mti Message Type Indicator
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int Node::write(If::MTI mti, NodeHandle dst, Buffer *data)
{
    /* It is important to note that we unlock the mutex before sending
     * data to an interface.  This is required for local nodes such that
     * we don't have mutex recursion.  This is required for remote nodes so
     * that if the interface write would block, other nodes may continue to
     * send messages using the interface.  NMRAnet does not guarantee message
     * sequencing.
     */

    if (dst.id == 0 && dst.alias == 0)
    {
        /* broacast message */
        mutex.unlock();
        data->reference();
        nmranetIf->rx_data(mti, {nodeID, 0}, 0, data);
        nmranetIf->if_write(mti, nodeID, dst, data);
    }
    else
    {
        RBTree <NodeID, Node*>::Node *node = idTree.find(dst.id);;
        mutex.unlock();
        
        if (node)
        {
            /* loop-back */
            nmranetIf->rx_data(mti, {nodeID, 0}, dst.id, data);
        }
        else
        {
            nmranetIf->if_write(mti, nodeID, dst, data);
        }
    }
    mutex.lock();

    return 0;
}

};

