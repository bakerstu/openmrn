/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file TractionThrottleInterface.hxx
 *
 * Client API for the Traction protocol (declarations only).
 *
 * @author Balazs Racz
 * @date 20 May 2014
 */

#ifndef _OPENLCB_TRACTIONTHROTTLEINTERFACE_HXX_
#define _OPENLCB_TRACTIONTHROTTLEINTERFACE_HXX_

#include "executor/CallableFlow.hxx"
#include "openlcb/Defs.hxx"
#include "openlcb/TrainInterface.hxx"

namespace openlcb
{

class Node;
struct TractionThrottleInput;

/// C++ Namespace for collecting all commands that can be sent to the
/// TractionThrottle flow.
struct TractionThrottleCommands
{
    enum SetDst
    {
        SET_DST,
    };

    enum AssignTrain
    {
        ASSIGN_TRAIN,
    };

    enum ReleaseTrain
    {
        RELEASE_TRAIN,
    };

    enum LoadState
    {
        LOAD_STATE,
    };

    enum ConsistAdd
    {
        CONSIST_ADD,
    };

    enum ConsistDel
    {
        CONSIST_DEL,
    };

    enum ConsistQry
    {
        CONSIST_QRY,
    };
};

/// Request structure used to send requests to the TractionThrottle
/// class. Contains parametrized reset calls for properly supporting
/// @ref StateFlowBase::invoke_subflow_and_wait() syntax.
struct TractionThrottleInput : public CallableFlowRequestBase
{
    enum Command
    {
        CMD_SET_DST,
        CMD_ASSIGN_TRAIN,
        CMD_RELEASE_TRAIN,
        CMD_LOAD_STATE,
        CMD_CONSIST_ADD,
        CMD_CONSIST_DEL,
        CMD_CONSIST_QRY,
    };

    /// Sets the destination node to send messages to without sending assign
    /// commands to that train node.
    void reset(const TractionThrottleCommands::SetDst &, const NodeID &dst)
    {
        cmd = CMD_SET_DST;
        this->dst = dst;
    }

    void reset(const TractionThrottleCommands::AssignTrain &, const NodeID &dst,
        bool listen)
    {
        cmd = CMD_ASSIGN_TRAIN;
        this->dst = dst;
        this->flags = listen ? 1 : 0;
    }

    void reset(const TractionThrottleCommands::ReleaseTrain &)
    {
        cmd = CMD_RELEASE_TRAIN;
    }

    void reset(const TractionThrottleCommands::LoadState &)
    {
        cmd = CMD_LOAD_STATE;
    }

    void reset(const TractionThrottleCommands::ConsistAdd &, NodeID slave,
        uint8_t flags)
    {
        cmd = CMD_CONSIST_ADD;
        dst = slave;
        this->flags = flags;
    }

    void reset(const TractionThrottleCommands::ConsistDel &, NodeID slave)
    {
        cmd = CMD_CONSIST_DEL;
        dst = slave;
    }

    void reset(const TractionThrottleCommands::ConsistQry &)
    {
        cmd = CMD_CONSIST_QRY;
        replyCause = 0xff;
    }

    void reset(const TractionThrottleCommands::ConsistQry &, uint8_t ofs)
    {
        cmd = CMD_CONSIST_QRY;
        consistIndex = ofs;
        replyCause = 0;
    }

    Command cmd;
    /// For assign, this carries the destination node ID. For consisting
    /// requests, this is an in-out argument.
    NodeID dst;
    /// Contains the flags for the consist listener. Specified for Attach
    /// requests, and filled for Query responses.
    uint8_t flags;

    /// For assign controller reply REJECTED, this is 1 for controller refused
    /// connection, 2 fortrain refused connection.
    uint8_t replyCause;
    /// Total number of entries in the consisting list.
    uint8_t consistCount;
    /// Index of the entry in the consisting list that needs to be returned.
    uint8_t consistIndex;
};

class TractionThrottleInterface : public openlcb::TrainImpl
{
public:
    /// Flips a function on<>off.
    virtual void toggle_fn(uint32_t fn) = 0;

    /// Sends a query for a function to the server. The response will be
    /// asynchronously reported by the throttle listener update callback.
    /// @param fn function number.
    virtual void query_fn(uint32_t fn)
    {
    }

    /// Determine if a train is currently assigned to this trottle.
    /// @return true if a train is assigned, else false
    virtual bool is_train_assigned() = 0;

    /// @return the controlling node (virtual node of the throttle, i.e., us.)
    /// @todo this function should not be here
    virtual openlcb::Node *throttle_node() = 0;

    /// Sets up a callback for listening for remote throttle updates. When a
    /// different throttle modifies the train node's state, and the
    /// ASSIGN_TRAIN command was executed with "listen==true" parameter, we
    /// will get notifications about those remote changes. The notifications
    /// update the cached state in TractionThrottle, and call this update
    /// callback. Repeat with nullptr if the callbacks are not desired anymore.
    /// @param update_callback will be executed when a different throttle
    /// changes the train state. fn is the function number changed, or -1 for
    /// speed update.
    virtual void set_throttle_listener(
        std::function<void(int fn)> update_callback) = 0;

    /// @return the controlled node (the train node) ID.
    /// @todo this function should not be here
    virtual openlcb::NodeID target_node() = 0;
};

class TractionThrottleBase : public CallableFlow<TractionThrottleInput>,
                             public TractionThrottleInterface
{
public:
    TractionThrottleBase(Service *s)
        : CallableFlow<TractionThrottleInput>(s)
    {
    }

    enum
    {
        /// Returned from get_fn() when we don't have a cahced value for a
        /// function.
        FN_NOT_KNOWN = 0xffff,
    };
};

} // namespace openlcb

#endif // _OPENLCB_TRACTIONTHROTTLEINTERFACE_HXX_
