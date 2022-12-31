/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file If.hxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _OPENLCB_IF_HXX_
#define _OPENLCB_IF_HXX_

/// @todo(balazs.racz) remove this dep
#include <string>

#include "executor/Dispatcher.hxx"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"
#include "openlcb/Convert.hxx"
#include "openlcb/Defs.hxx"
#include "openlcb/Node.hxx"
#include "utils/Buffer.hxx"
#include "utils/Map.hxx"
#include "utils/Queue.hxx"

namespace openlcb
{

class Node;
class StreamTransport;

/// Helper function to send an event report to the bus. Performs
/// synchronous (dynamic) memory allocation so use it sparingly and when
/// there is sufficient amount of RAM available.
/// @param event_id is the event to send off.
extern void send_event(Node* src_node, uint64_t event_id);

/** This class is used in the dispatching of incoming or outgoing NMRAnet
 * messages to the message handlers at the protocol-agnostic level (i.e. not
 * CAN or TCP-specific).
 *
 * TODO(balazs.racz) There shall be one instance of this class that will be
 * sent to all handlers that expressed interest in that MTI. When all those
 * handlers are done, the instance should be freed. Currently the instance is
 * copied by the dispatcher separately for each handler. */
struct GenMessage
{
    GenMessage()
        : src({0, 0}), dst({0, 0}), flagsSrc(0), flagsDst(0) {}

    void clear()
    {
        reset((Defs::MTI)0, 0, EMPTY_PAYLOAD);
    }

    void reset(Defs::MTI mti, NodeID src, NodeHandle dst, string payload)
    {
        this->mti = mti;
        this->src = {src, 0};
        this->dst = dst;
        this->payload = std::move(payload);
        this->dstNode = nullptr;
        this->flagsSrc = 0;
        this->flagsDst = 0;
    }

    void reset(Defs::MTI mti, NodeID src, string payload)
    {
        this->mti = mti;
        this->src = {src, 0};
        this->dst = {0, 0};
        this->payload = std::move(payload);
        this->dstNode = nullptr;
        this->flagsSrc = 0;
        this->flagsDst = 0;
    }

    /// Source node.
    NodeHandle src;
    /// Destination node.
    NodeHandle dst;
    /// OpenLCB MTI of the incoming message.
    Defs::MTI mti;
    /// If the destination node is local, this value is non-NULL.
    Node *dstNode;
    /// Data content in the message body. Owned by the dispatcher.
    /// @todo(balazs.racz) figure out a better container.
    string payload;

    unsigned flagsSrc : 4;
    unsigned flagsDst : 4;
    unsigned get_flags_src() {
        return flagsSrc;
    }
    unsigned get_flags_dst() {
        return flagsDst;
    }
    void set_flag_src(unsigned flags) {
        flagsSrc |= flags;
    }
    void clear_flag_src(unsigned flags) {
        flagsSrc &= ~flags;
    }
    /** Returns true if src flags has all the specified flags set. */
    bool has_flag_src(unsigned flags) {
        return ((flagsSrc & flags) == flags);
    }
    void set_flag_dst(unsigned flags) {
        flagsDst |= flags;
    }
    void clear_flag_dst(unsigned flags) {
        flagsDst &= ~flags;
    }
    /** Returns true if src flags has all the specified flags set. */
    bool has_flag_dst(unsigned flags) {
        return ((flagsDst & flags) == flags);
    }

    typedef uint32_t id_type;
    id_type id() const
    {
        return static_cast<uint32_t>(mti);
    }

    /** Returns the NMRAnet-defined priority band, in the range of 0..3. */
    unsigned priority()
    {
        return Defs::mti_priority(mti);
    }

    enum DstFlags {
        /** Specifies that the stack should wait for the local loopback
         * processing before invoking the done notifiable. */
        WAIT_FOR_LOCAL_LOOPBACK = 1,
        /** Signals to the stack that we need to set the continuation bits in
         * the outgoing message to indicate that this is not the first frame of
         * a message. */
        DSTFLAG_NOT_FIRST_MESSAGE = 2,
        /** Signals to the stack that we need to set the continuation bits in
         * the outgoing message to indicate that this is not the last frame of
         * a message. */
        DSTFLAG_NOT_LAST_MESSAGE  = 4,
        // 8: free
    };
    enum SrcFlags {
        // 1, 2, 4, 8: free
    };
};

/// Interface class for all handlers that can be registered in the dispatcher
/// to receive incoming NMRAnet messages.
typedef FlowInterface<Buffer<GenMessage>> MessageHandler;

/// Abstract class representing an OpenLCB Interface. All interaction between
/// the local software stack and the physical bus has to go through this
/// class. The API that's not specific to the wire protocol appears here. The
/// implementations of this class would be specific to the wire protocol
/// (e.g. IfCan for CAN, and a not-yet-implemented class for TCP).
class If : public Service
{
public:
    /** Constructs an NMRAnet interface.
     * @param executor is the thread that will be used for all processing on
     * this interface.
     * @param local_nodes_count is the maximum number of virtual nodes that
     * this interface will support. */
    If(ExecutorBase *executor, int local_nodes_count);

    /** Destructor */
    virtual ~If()
    {
    }

    /** @return Flow to send global messages to the NMRAnet bus. */
    MessageHandler *global_message_write_flow()
    {
        HASSERT(globalWriteFlow_);
        if (txHook_)
        {
            txHook_();
        }
        return globalWriteFlow_;
    }
    /** @return Flow to send addressed messages to the NMRAnet bus. */
    MessageHandler *addressed_message_write_flow()
    {
        HASSERT(addressedWriteFlow_);
        if (txHook_)
        {
            txHook_();
        }
        return addressedWriteFlow_;
    }

    /** Type of the dispatcher of incoming NMRAnet messages. */
    typedef DispatchFlow<Buffer<GenMessage>, 4> MessageDispatchFlow;

    /** @return Dispatcher of incoming NMRAnet messages. */
    MessageDispatchFlow *dispatcher()
    {
        return &dispatcher_;
    }

    /** Transfers ownership of a module to the interface. It will be brought
     * down in the destructor. The destruction order is guaranteed such that
     * all supporting structures are still available when the flow is destryed,
     * but incoming messages can not come in anymore.
     *
     * @todo(balazs.racz) revise whether this needs to be virtual. */
    virtual void add_owned_flow(Executable *e) = 0;

    /** Registers a new local node on this interface. This function must be
     * called from the interface's executor.
     *
     * @param node is the node to register.
     */
    void add_local_node(Node *node)
    {
        NodeID id = node->node_id();
        HASSERT(localNodes_.find(id) == localNodes_.end());
        localNodes_[id] = node;
    }

    /** Removes a local node from this interface. This function must be called
     * from the interface's executor.
     *
     * @param node is the node to delete. The node will not be freed, just
     * removed from the data structures.
     */
    virtual void delete_local_node(Node *node) = 0;

    /** Looks up a node ID in the local nodes' registry. This function must be
     * called from the interface's executor.
     *
     * @param id is the 48-bit NMRAnet node ID to look up.
     * @returns the node pointer or NULL if the node is not registered.
     */
    Node *lookup_local_node(NodeID id)
    {
        auto it = localNodes_.find(id);
        if (it == localNodes_.end())
        {
            return nullptr;
        }
        return it->second;
    }

    /** Looks up a node ID in the local nodes' registry. This function must be
     * called from the interface's executor.
     *
     * @param handle is the NodeHandle representing a target node.
     * @returns the node pointer or NULL if the node is not local registered.
     */
    virtual Node *lookup_local_node_handle(NodeHandle handle)
    {
        return lookup_local_node(handle.id);
    }

    /**
     * @returns the first node (by nodeID order) that is registered in this
     * interface as a local node, or nullptr if this interface has no local
     * nodes.
     */
    Node* first_local_node() {
        auto it = localNodes_.begin();
        if (it == localNodes_.end()) return nullptr;
        return it->second;
    }

    /**
     * Iterator helper on the local nodes map.
     *
     * @param previous is the node ID of a valid local node.
     *
     * @returns the node pointer of the next local node (in node ID order) or
     * null if this was the last node or an invalid argument (not the node ID
     * of a local node).
     */
    Node* next_local_node(NodeID previous) {
        auto it = localNodes_.find(previous);
        if (it == localNodes_.end())
        {
            return nullptr;
        }
        ++it;
        if (it == localNodes_.end())
        {
            return nullptr;
        }
        return it->second;
    }

    /** @returns true if the two node handles match as far as we can tell
     * without doing any network traffic. */
    virtual bool matching_node(NodeHandle expected,
                               NodeHandle actual) = 0;


    /** Canonicalizes the node handle: fills in id and/or alias from the maps
     * the interface holds internally. Noop for TCP interface. Must be called
     * on the interface executor. */
    virtual void canonicalize_handle(NodeHandle *h) {}

    /// @return the node ID of the default node on this interface. For TCP
    /// interfaces this is the gateway node ID, for CAN interfaces this is the
    /// node ID used for alias allocation on the CAN-bus.
    virtual NodeID get_default_node_id() = 0;

    /// Sets a transmit hook. This function will be called once for every
    /// OpenLCB message transmitted. Used for implementing activity LEDs.
    /// @param hook function to call for each transmit message.
    void set_tx_hook(std::function<void()> hook)
    {
        txHook_ = std::move(hook);
    }

    /// @return the object supporting stream transport in OpenLCB. May be null
    /// if stream transport was not initialized for this interface. This is
    /// typical when a small node is low on flash space for code.
    StreamTransport *stream_transport()
    {
        return streamTransport_;
    }

    /// Adds the necessary object for this interface to support stream
    /// transport. May be called only once per interface.
    /// @param s the stream transport object. Ownership is not transferred. (If
    /// needed, see add_owned_flow.)
    void set_stream_transport(StreamTransport *s)
    {
        HASSERT(streamTransport_ == nullptr);
        streamTransport_ = s;
    }

protected:
    void remove_local_node_from_map(Node *node)
    {
        auto it = localNodes_.find(node->node_id());
        HASSERT(it != localNodes_.end());
        localNodes_.erase(it);
    }

    /// Allocator containing the global write flows.
    MessageHandler *globalWriteFlow_;
    /// Allocator containing the addressed write flows.
    MessageHandler *addressedWriteFlow_;

private:
    /// Flow responsible for routing incoming messages to handlers.
    MessageDispatchFlow dispatcher_;

    /// This function is pinged every time a message is transmitted.
    std::function<void()> txHook_;

    typedef Map<NodeID, Node *> VNodeMap;

    /// Local virtual nodes registered on this interface.
    VNodeMap localNodes_;

    /// Accessor for the objects and variables for supporting stream transport.
    StreamTransport *streamTransport_ {nullptr};

    friend class VerifyNodeIdHandler;

    DISALLOW_COPY_AND_ASSIGN(If);
};

/// Message handlers that are implemented as state flows should derive from
/// this class.
typedef StateFlow<Buffer<GenMessage>, QList<4>> MessageStateFlowBase;

/** Base class for incoming message handler flows. */
class IncomingMessageStateFlow
    : public MessageStateFlowBase
{
public:
    IncomingMessageStateFlow(If *iface)
        : MessageStateFlowBase(iface)
    {
    }

    If *iface()
    {
        return static_cast<If *>(service());
    }

    /// Returns the NMRAnet message we received.
    GenMessage *nmsg()
    {
        return message()->data();
    }
};

/// Sends an OpenLCB message to the bus.  Performs synchronous (dynamic) memory
/// allocation so use it sparingly and when there is sufficient amount of RAM
/// available.
/// @param src_node A local virtual node from which to send the message.
/// @param mti message type indicator
/// @param args either a Payload to send a global message, or a NodeHandle dst
/// and a Payload to send an addressed message.
template <typename... Args> void send_message(Node *src_node, Defs::MTI mti, Args &&...args)
{
    Buffer<GenMessage> *msg;
    mainBufferPool->alloc(&msg);
    msg->data()->reset(mti, src_node->node_id(), std::forward<Args>(args)...);
    if (msg->data()->dst == NodeHandle())
    {
        src_node->iface()->global_message_write_flow()->send(msg);
    }
    else
    {
        src_node->iface()->addressed_message_write_flow()->send(msg);
    }
}

} // namespace openlcb

#endif // _OPENLCB_IF_HXX_
