/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file TractionTrain.hxx
 *
 * Defines an NMRAnet Train node.
 *
 * @author Balazs Racz
 * @date 5 May 2014
 */

#ifndef _OPENLCB_TRACTIONTRAIN_HXX_
#define _OPENLCB_TRACTIONTRAIN_HXX_

#include <set>

#include "executor/Service.hxx"
#include "openlcb/Node.hxx"
#include "openlcb/TractionDefs.hxx"
#include "openlcb/TrainInterface.hxx"

namespace openlcb
{


class TrainService;

/// Linked list entry for all registered consist clients for a given train
/// node.
struct ConsistEntry : public QMember {
    ConsistEntry(NodeID s, uint8_t flags) : payload((s << 8) | flags) {}
    NodeID get_slave() const {
        return payload >> 8;
    }
    uint8_t get_flags() const {
        return payload & 0xff;
    }
    void set_flags(uint8_t new_flags) {
        payload ^= (payload & 0xff);
        payload |= new_flags;
    }
private:
    uint64_t payload;
};

/// Virtual node class for an OpenLCB train protocol node.
///
/// Usage:
///   - Create a TrainImpl defining how to send the commands to the hardware.
///   - Create a TrainNode and pass it the pointer to the implementation.
///
/// for train implementations see @ref LoggingTrain, @ref dcc::Dcc28Train, @ref
/// dcc::MMNewTrain etc.
class TrainNode : public Node
{
public:
    ~TrainNode();

    /// @return the train implementation object for issuing control commands to
    /// this train.
    virtual TrainImpl *train() = 0;

    /// @return the last stored controller node.
    virtual NodeHandle get_controller() = 0;

    /// @param id the controller node of this train.
    virtual void set_controller(NodeHandle id) = 0;

    // Thread-safety information
    //
    // The consisting functionality is thread-compatible, which means that it
    // is the responsibility of the caller to ensure that no two threads are
    // calling these methods concurrently.
    //
    // In practice these methods are always called from the TractionService
    // which only operates on a single thread (the service's executor) and will
    // only process one request at a time. All traction protocol requests being
    // forwarded and thus traversing the consist list will be fully processed
    // before any consist change requests would reach the front of the queue
    // for the traction flow.

    /** Adds a node ID to the consist targets. @return false if the node was
     * already in the target list, true if it was newly added. */
    virtual bool add_consist(NodeID tgt, uint8_t flags)
    {
        if (!tgt)
        {
            return false;
        }
        if (tgt == node_id())
        {
            return false;
        }
        auto it = consistSlaves_.begin();
        for (; it != consistSlaves_.end(); ++it)
        {
            if (it->get_slave() == tgt)
            {
                it->set_flags(flags);
                return false;
            }
        }
        consistSlaves_.insert(it, new ConsistEntry(tgt, flags));
        return true;
    }

    /** Removes a node ID from the consist targets. @return true if the target
     * was removed, false if the target was not on the list. */
    virtual bool remove_consist(NodeID tgt)
    {
        for (auto it = consistSlaves_.begin(); it != consistSlaves_.end(); ++it)
        {
            if (it->get_slave() == tgt)
            {
                auto* p = it.operator->();
                consistSlaves_.erase(it);
                delete p;
                return true;
            }
        }
        return false;
    }

    /** Returns the consist target with offset id, or NodeID(0) if there are
     * fewer than id consist targets. id is zero-based. */
    NodeID query_consist(int id, uint8_t* flags)
    {
        int k = 0;
        for (auto it = consistSlaves_.begin();
             it != consistSlaves_.end(); ++it, ++k)
        {
            if (k == id)
            {
                if (flags) *flags = it->get_flags();
                return it->get_slave();
            }
        }
        return 0;
    }

    /** Returns the number of slaves in this consist. */
    int query_consist_length()
    {
        int ret = 0;
        for (auto it = consistSlaves_.begin(); it != consistSlaves_.end();
             ++it, ++ret)
        {
        }
        return ret;
    }

    TypedQueue<ConsistEntry> consistSlaves_;
};

/// Default implementation of a train node.
class DefaultTrainNode : public TrainNode
{
public:
    DefaultTrainNode(TrainService *service, TrainImpl *impl);
    ~DefaultTrainNode();

    NodeHandle get_controller() override
    {
        return controllerNodeId_;
    }

    void set_controller(NodeHandle id) override
    {
        controllerNodeId_ = id;
    }

    If *iface() override;
    bool is_initialized() override
    {
        return isInitialized_;
    }
    void set_initialized() override
    {
        isInitialized_ = 1;
    }
    // Used for restarting the stack.
    void clear_initialized() override
    {
        isInitialized_ = 0;
    }

    TrainImpl *train() override
    {
        return train_;
    }

protected:
    /// Pointer to the traction service.
    TrainService *service_;
    /// Pointer to the train implementation object.
    TrainImpl *train_;

private:
    /// Node is initialized bit for startup transient.
    unsigned isInitialized_ : 1;

    /// Controller node that is assigned to run this train. 0 if none.
    NodeHandle controllerNodeId_;
};

/// Train node class with a an OpenLCB Node ID from the DCC pool. Used for command stations.
class TrainNodeForProxy : public DefaultTrainNode
{
public:
    /// Constructor.
    /// @param service the traction service object that will own this node.
    /// @param train the implementation object that the traction messages
    /// should be forwarded to.
    TrainNodeForProxy(TrainService *service, TrainImpl *train);

    /// Destructor.
    ~TrainNodeForProxy();

    /// @return the OpenLCB node ID, generated from the legacy protocol types
    /// that we get from TrainImpl.
    NodeID node_id() OVERRIDE;
};

/// Train node class with a fixed OpenLCB Node ID. This is useful for native
/// train nodes that are not dynamically generated by a command station.
class TrainNodeWithId : public DefaultTrainNode
{
public:
    /// Constructor.
    /// @param service the traction service object that will own this node.
    /// @param train the implementation object that the traction messages
    /// should be forwarded to.
    /// @param node_id the OpenLCB node ID for this train.
    TrainNodeWithId(TrainService *service, TrainImpl *train, NodeID node_id);

    /// Destructor.
    ~TrainNodeWithId();

    /// @return the openlcb node ID.
    NodeID node_id() OVERRIDE
    {
        return nodeId_;
    }

private:
    /// The OpenLCB node ID.
    NodeID nodeId_;
};

/// Collection of control flows necessary for implementing the Traction
/// Protocol.
///
/// usage: instantiate for the given interface. Pass the pointer to the train
/// nodes upon their construction.
class TrainService : public Service, private Atomic
{
public:
    TrainService(If *iface);
    ~TrainService();

    If *iface()
    {
        return iface_;
    }

    /** Registers a new train with the train service. Will initiate a node
        initialization flow for the train. */
    void register_train(TrainNode *node);

    /// Removes a train node from the local interface.
    /// @param node train to remove from registry.
    void unregister_train(TrainNode *node);

    /// Checks if the a given node is a train node operated by this Traction
    /// Service.
    /// @param node a virtual node
    /// @return true if this is a known train node.
    bool is_known_train_node(Node *node)
    {
        return nodes_.find((TrainNode*)node) != nodes_.end();
    }

private:
    struct Impl;
    /** Implementation flows. */
    Impl *impl_;

    If *iface_;
    /** List of train nodes managed by this Service. */
    std::set<TrainNode *> nodes_;
};

} // namespace openlcb

#endif // _OPENLCB_TRACTIONTRAIN_HXX_
