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
struct ConsistEntry : public QMember
{
    ConsistEntry(NodeID s, uint8_t flags) : payload((s << 8) | flags) {}
    NodeID get_slave() const
    {
        return payload >> 8;
    }
    uint8_t get_flags() const
    {
        return payload & 0xff;
    }
    void set_flags(uint8_t new_flags)
    {
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
    /// Constructor.
    ///
    /// @param service is the @ref TrainService to use for this TrainNode
    /// @param train is the @ref TrainImpl for this TrainNode.
    TrainNode(TrainService *service, TrainImpl *train);

    /// Destructor.
    ~TrainNode();

    /// @return the @ref If for this TrainNode.
    If *iface() OVERRIDE;

    /// @return true if this TrainNode has been initialized.
    bool is_initialized() OVERRIDE
    {
        return isInitialized_;
    }

    /// Sets this TrainNode as initialized.
    void set_initialized() OVERRIDE
    {
        isInitialized_ = 1;
    }

    /// Used for restarting the stack.
    void clear_initialized() OVERRIDE
    {
        isInitialized_ = 0;
    }

    /// @return the @ref TrainImpl that this TrainNode represents.
    TrainImpl *train()
    {
        return train_;
    }

    /// @return the @ref NodeHandle for the controller of this TrainNode.
    NodeHandle get_controller()
    {
        return controllerNodeId_;
    }

    /// Sets the controller for this TrainNode.
    ///
    /// @param id is the @ref NodeHandle for the controller.
    void set_controller(NodeHandle id)
    {
        controllerNodeId_ = id;
    }

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

    /// Adds a node ID to the consist targets. If the node already exists it
    /// will update the target's flags and not add a duplicate.
    ///
    /// @param tgt is the target @ref NodeID to add to the consist.
    /// @param flags are the flags for the consist member.
    ///
    /// @return false if the node was already in the target list, true if it
    /// was newly added.
    bool add_consist(NodeID tgt, uint8_t flags)
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

    /// Removes a node ID from the consist targets.
    ///
    /// @param tgt is the @ref NodeID to be removed from the consist targets.
    ///
    /// @return true if the target was removesd, false if the target was not on
    /// the list.
    bool remove_consist(NodeID tgt)
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

    /// @return the consist target with offset id, or NodeID(0) if there are
    /// fewer than id consist targets. id is zero-based.
    ///
    /// @param id is the offset in the consist list to retrive the @ref NodeID
    /// for.
    /// @param flags is an output parameter to hold the consist member's flags
    /// value.
    NodeID query_consist(int id, uint8_t* flags)
    {
        int k = 0;
        for (auto it = consistSlaves_.begin();
             it != consistSlaves_.end(); ++it, ++k)
        {
            if (k == id)
            {
                if (flags)
                {
                    *flags = it->get_flags();
                }
                return it->get_slave();
            }
        }
        return 0;
    }

    /// @return the number of consist members in this consist.
    int query_consist_length()
    {
        int ret = 0;
        for (auto it = consistSlaves_.begin(); it != consistSlaves_.end();
             ++it, ++ret)
        {
        }
        return ret;
    }

protected:
    /// @ref TrainService for this TrainNode.
    TrainService *service_;
    /// @ref TrainImpl for this TrainNode.
    TrainImpl *train_;

private:
    /// Boolean flag used for tracking if this TrainNode has been initialized.
    unsigned isInitialized_ : 1;

    /// Controller node that is assigned to run this train. 0 if none.
    NodeHandle controllerNodeId_;
    /// Collection of @ref ConsistEntry objects representing the consist
    /// members of this TrainNode.
    TypedQueue<ConsistEntry> consistSlaves_;
};


/// Train node class with a an OpenLCB Node ID from the DCC pool. This is
/// primarily intended for use by command stations.
class TrainNodeForProxy : public TrainNode
{
public:
    /// Constructor
    ///
    /// @param service is the @ref TrainService to use for this @ref TrainNode.
    /// @param train is the @ref TrainImpl for this @ref TrainNode.
    TrainNodeForProxy(TrainService *service, TrainImpl *train);

    /// @return the @ref NodeID for this @ref TrainNode.
    NodeID node_id() OVERRIDE;
};

/// Train node class with a fixed OpenLCB @ref NodeID. This is useful for
/// native train nodes that are not dynamically generated by a command station.
class TrainNodeWithId : public TrainNode
{
public:
    /// Constructor
    ///
    /// @param service is the @ref TrainService to use for this @ref TrainNode.
    /// @param train is the @ref TrainImpl for this @ref TrainNode.
    /// @param node_id is the @ref NodeID for this @ref TrainNode.
    TrainNodeWithId(TrainService *service, TrainImpl *train, NodeID node_id);

    /// @return the @ref NodeID for this @ref TrainNode.
    NodeID node_id() OVERRIDE
    {
        return nodeId_;
    }

private:
    /// @ref NodeID for this @ref TrainNode.
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
    /// Constructor.
    ///
    /// @param iface is the @ref If to use for communication with other nodes.
    TrainService(If *iface);

    /// Destructor.
    ~TrainService();

    /// @return the @ref If for used by this @ref TrainService.
    If *iface()
    {
        return iface_;
    }

    /// Registers a new train with the train service.
    /// This will initiate a node initialization flow for the train.
    ///
    /// @param node is the @ref TrainNode to register.
    void register_train(TrainNode *node);

    /// Unregisters an existing train with the train service.
    ///
    /// @param node is the @ref TrainNode to unregister.
    void unregister_train(TrainNode *node);

private:
    struct Impl;
    /// Internal implementation of this Service.
    Impl *impl_;
    /// Interface used for transmitting/receiving Traction data.
    If *iface_;
    /// List of train nodes managed by this Service.
    std::set<TrainNode *> nodes_;
};

} // namespace openlcb

#endif // _OPENLCB_TRACTIONTRAIN_HXX_
