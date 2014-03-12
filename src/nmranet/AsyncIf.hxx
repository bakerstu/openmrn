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
 * \file AsyncIf.hxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _NMRAnetAsyncIf_hxx_
#define _NMRAnetAsyncIf_hxx_

/// @todo(balazs.racz) remove this dep
#include <string>

//#include "nmranet/NMRAnetAsyncNode.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "executor/Dispatcher.hxx"
#include "executor/Service.hxx"
#include "executor/Executor.hxx"
#include "utils/BufferQueue.hxx"
#include "utils/Map.hxx"

namespace NMRAnet
{

class AsyncNode;

/** Convenience function to render a 48-bit NMRAnet node ID into a new buffer.
 *
 * @param id is the 48-bit ID to render.
 * @returns a new buffer (from the main pool) with 6 bytes of used space, a
 * big-endian representation of the node ID.
 */
// extern Buffer* node_id_to_buffer(NodeID id);

/** Converts a 6-byte-long buffer to a node ID.
 *
 * @param buf is a buffer that has to have exactly 6 bytes used, filled with a
 * big-endian node id.
 * @returns the node id (in host endian).
 */
// extern NodeID buffer_to_node_id(Buffer* buf);

/** This class is used in the dispatching of incoming or outgoing NMRAnet
 * messages to the message handlers at the protocol-agnostic level (i.e. not
 * CAN or TCP-specific).
 *
 * TODO(balazs.racz) There shall be one instance of this class that will be
 * sent to all handlers that expressed interest in that MTI. When all those
 * handlers are done, the instance should be freed. */
struct NMRAnetMessage
{
    void reset(If::MTI mti, NodeID src, NodeHandle dst, const string& payload) {
        this->mti = mti;
        this->src = {src, 0};
        this->dst = dst;
        this->payload = payload;
    }

    void reset(If::MTI mti, NodeID src, const string& payload) {
        this->mti = mti;
        this->src = {src, 0};
        this->dst = {0, 0};
        this->payload = payload;
    }

    /// OpenLCB MTI of the incoming message.
    If::MTI mti;
    /// Source node.
    NodeHandle src;
    /// Destination node.
    NodeHandle dst;
    /// If the destination node is local, this value is non-NULL.
    AsyncNode *dstNode;
    /// Data content in the message body. Owned by the dispatcher.
    /// @todo(balazs.racz) figure out a better container.
    string payload;

    typedef If::MTI id_type;
    id_type id() const
    {
        return mti;
    }
};

typedef FlowInterface<Buffer<NMRAnetMessage>> MessageHandler;

#if 0
/** @todo(balazs.racz) delete this class */
class WriteFlow : public ControlFlow
{
public:
    WriteFlow(Executor *e, Notifiable *done) : ControlFlow(e, done)
    {
    }

    /** Initiates sending an addressed message onto the NMRAnet bus.
     *
     * Must only be called if *this is an addressed flow.
     *
     * @param mti of the message to send
     * @param src is the NodeID of the originating node
     * @param dst is the destination node (cannot be 0,0)
     * @param data is the message payload (may be null), takes ownership
     * @param done will be notified when the message is enqueued for sending.
     *  May be set to nullptr.
     */
    virtual void WriteAddressedMessage(If::MTI mti, NodeID src, NodeHandle dst,
                                       Buffer *data, Notifiable *done) = 0;

    /** Initiates sending an unaddressed (global) message onto the NMRAnet bus.
     *
     * Must only be called if *this is a global flow.
     *
     * @param mti of the message to send
     * @param src is the NodeID of the originating node
     * @param data is the message payload (may be null), takes ownership
     * @param done will be notified when the message is enqueued for sending.
     *  May be set to nullptr.
     */
    virtual void WriteGlobalMessage(If::MTI mti, NodeID src, Buffer *data,
                                    Notifiable *done) = 0;
};
#endif

class AsyncIf : public Service
{
public:
    /** Constructs an NMRAnet interface.
     * @param executor is the thread that will be used for all processing on
     * this interface.
     * @param local_nodes_count is the maximum number of virtual nodes that
     * this interface will support. */
    AsyncIf(ExecutorBase *executor, int local_nodes_count);

    /** Destructor */
    virtual ~AsyncIf()
    {
    }

    /** @return Flow to send global messages to the NMRAnet bus. */
    MessageHandler *global_message_write_flow()
    {
        HASSERT(globalWriteFlow_);
        return globalWriteFlow_;
    }
    /** @return Flow to send addressed messages to the NMRAnet bus. */
    MessageHandler *addressed_message_write_flow()
    {
        HASSERT(addressedWriteFlow_);
        return addressedWriteFlow_;
    }

    /** Type of the dispatcher of incoming NMRAnet messages. */
    typedef DispatchFlow<Buffer<NMRAnetMessage>, 4> MessageDispatchFlow;

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
    void add_local_node(AsyncNode *node)
    {
        HASSERT(0);
        /*NodeID id = node->node_id();
        HASSERT(localNodes_.find(id) == localNodes_.end());
        localNodes_[id] = node;*/
    }

    /** Removes a local node from this interface. This function must be called
     * from the interface's executor.
     *
     * @param node is the node to delete.
     */
    void delete_local_node(AsyncNode *node)
    {
        HASSERT(0);
        /*
        auto it = localNodes_.find(node->node_id());
        HASSERT(it != localNodes_.end());
        localNodes_.erase(it);*/
    }

    /** Looks up a node ID in the local nodes' registry. This function must be
     * called from the interface's executor.
     *
     * @param id is the 48-bit NMRAnet node ID to look up.
     * @returns the node pointer or NULL if the node is not registered.
     */
    AsyncNode *lookup_local_node(NodeID id)
    {
        auto it = localNodes_.find(id);
        if (it == localNodes_.end())
        {
            return nullptr;
        }
        return it->second;
    }

protected:
    /// Allocator containing the global write flows.
    MessageHandler *globalWriteFlow_;
    /// Allocator containing the addressed write flows.
    MessageHandler *addressedWriteFlow_;

private:
    /// Flow responsible for routing incoming messages to handlers.
    MessageDispatchFlow dispatcher_;

    typedef Map<NodeID, AsyncNode *> VNodeMap;

    /// Local virtual nodes registered on this interface.
    VNodeMap localNodes_;

    friend class VerifyNodeIdHandler;

    DISALLOW_COPY_AND_ASSIGN(AsyncIf);
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIf_hxx_
