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
 * \file NMRAnetNode.hxx
 * This file defines NMRAnet nodes.
 *
 * @author Stuart W. Baker
 * @date 29 September 2013
 */


#ifndef _NMRAnetNode_hxx_
#define _NMRAnetNode_hxx_

#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/NMRAnetDatagram.hxx"
#include "nmranet/NMRAnetStream.hxx"
#include "nmranet/NMRAnetMemoryConfig.hxx"
#include "utils/RBTree.hxx"

namespace NMRAnet
{

/** NMRAnet virtual node.
 */
class Node : public Datagram, public Stream, public MemoryConfig
{
public:
    /** Constructor.
     * @param node_id 48-bit unique Node ID
     * @param nmranet_if interface to bind the node to
     * @param model node decription
     */
    Node(NodeID node_id, If *nmranet_if, const char *model, uint8_t *cdi = NULL)
        : Datagram(),
          Stream(),
          MemoryConfig(cdi),
          nodeID(node_id),
          model(model),
          userName(NULL),
          userDescription(NULL),
          state(UNINITIALIZED),
          nmranetIf(nmranet_if),
          queue()
    {
        mutex.lock();
        HASSERT(idTree.find(nodeID) == NULL);
        idNode.key = nodeID;
        idNode.value = this;
        idTree.insert(&idNode);
        mutex.unlock();
        new OSThread(model, 0, 1024, thread_entry, this);
    }

    /** Destructor.
     */
    ~Node()
    {
        mutex.lock();
        idTree.remove(nodeID);
        mutex.unlock();
    }

    /** Set the user name of the node for simple ident protocol.
     * @param user_name string to use for user name
     */
    void user_name(const char *user_name)
    {
        userName = user_name;
    }

    /** Set the user description of the node for simple ident protocol.
     * @param user_description string to use for user description
     */
    void user_description(const char *user_description)
    {
        userDescription = user_description;
    }

    /** Obtain a @ref Node instance from its Node ID.
     * @param node_id Node ID to lookup
     * @return @ref Node instance, else NULL if not found
     */
    static Node *find(NodeID node_id);
    
    /** Move node into the initialized state.
     */
    void initialized();
    
    /** Get the 48-bit NMRAnet Node ID of a node.
     * @return Node ID
     */
    NodeID id()
    {
        return nodeID;
    }
    
    /** Bitmask for all potentially supported NMRAnet protocols.
     */
    enum Protocols
    {
        PROTOCOL_IDENTIFICATION = 0x800000000000,
        DATAGRAM                = 0x400000000000,
        STREAM                  = 0x200000000000,
        MEMORY_CONFIGURATION    = 0x100000000000,
        RESERVATION             = 0x080000000000,
        EVENT_EXCHANGE          = 0x040000000000,
        IDENTIFICATION          = 0x020000000000,
        LEARN_CONFIGURATION     = 0x010000000000,
        REMOTE_BUTTON           = 0x008000000000,
        ABBREVIATED_DEFAULT_CDI = 0x004000000000,
        DISPLAY                 = 0x002000000000,
        SIMPLE_NODE_INFORMATION = 0x001000000000,
        CDI                     = 0x000800000000,
        RESERVED_MASK           = 0x0007FFFFFFFF
    };

    /** Write a message from a node.  We should already have a mutex lock at this
     * at this point.
     * @param mti Message Type Indicator
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    int write(If::MTI mti, NodeHandle dst, Buffer *data);

    /** Write a message from a node.  We should not have a mutex lock at
     * this at this point.
     * @param mti Message Type Indicator
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    int write_unlocked(If::MTI mti, NodeHandle dst, Buffer *data);

protected:
    /** Process a Buffered message at the application level.
     * @param buffer message buffer to process
     */
    virtual void process(Buffer *buffer) = 0;

private:

    /** Process a Buffered message at the platform level.
     * @param buffer message buffer to process
     */
    static void* thread_entry(void *data)
    {
        Node *node = (Node*)data;
        
        for ( ; /* forever */ ; )
        {
            Buffer *buffer = node->queue.wait();

            switch (buffer->id())
            {
                default:
                    node->process(buffer);
                    break;
            }
        }
        return NULL;
    }
    
    /** Send a verify node id number message.
     */
    void verify_id_number();

    /** Send an ident info reply message.
     * @param dst destination Node ID to respond to
     */
    void ident_info_reply(NodeHandle dst);
    
    /** Send an protocols supported reply message.
     * @param dst destination Node ID to respond to
     */
    void protocol_support_reply(NodeHandle dst);

    /** Get handle to the receive queue for incoming NMRAnet messages.
     * @return handle to queue
     */
    BufferQueueWait *rx_queue()
    {
        return &queue;
    }
    
    /** Operational states of the node */
    enum State
    {
        UNINITIALIZED = 0, /**< uninitialized node state */
        INITIALIZED /**< initialized node state */
    };
    
    /** Mutual exclusion for access to all nodes */
    static OSMutex mutex;
    
    /** 48-bit Node ID for this virtual node */
    NodeID nodeID;
    
    /** Model string */
    const char *model;
    
    /** User name string */
    const char *userName;
    
    /** User description string */
    const char *userDescription;
    
    /** Node's current operational state */
    State state;
    
    /** Interface that node will be bound to */
    If *nmranetIf;

    /** Receive Queue for incoming message */
    BufferQueueWait queue;
    
    /** manufacturer string */
    static const char *MANUFACTURER;
    
    /** hardware rev string */
    static const char *HARDWARE_REV;
    
    /** software rev string */
    static const char *SOFTWARE_REV;

    /* Misc. constant expressions.
     */
    enum
    {
        SIMPLE_NODE_IDENT_VERSION_A = 0x01, /**< simple node identify prefix */
        SIMPLE_NODE_IDENT_VERSION_B = 0x01, /**< simple node identify prefix */
    };

    /** Tree of nodes sorted by node id */
    static RBTree <NodeID, Node*> idTree;
    
    /** Tree entry */
    RBTree <NodeID, Node*>::Node idNode;

    /** Default Constructor */
    Node();
    
    DISALLOW_COPY_AND_ASSIGN(Node);
    
    /** allow If class to access Node members */
    friend class If;
    
    /** allow Datagram class to access Node members */
    friend class Datagram;

    /** allow Stream class to access Node members */
    friend class Stream;
};

};

#endif /* _NMRAnetNode_hxx_ */

