/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file TractionThrottle.hxx
 *
 * Client API for the Traction protocol.
 *
 * @author Balazs Racz
 * @date 20 May 2014
 */

#ifndef _NMRANET_TRACTIONTHROTTLE_HXX_
#define _NMRANET_TRACTIONTHROTTLE_HXX_

#include "openlcb/TractionClient.hxx"
#include "openlcb/TractionDefs.hxx"
#include "openlcb/TrainInterface.hxx"

namespace openlcb
{

struct TractionThrottleInput;

/// C++ Namespace for collecting all commands that can be sent to the
/// TractionThrottle flow.
struct TractionThrottleCommands
{
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
struct TractionThrottleInput
{
    enum Command
    {
        CMD_ASSIGN_TRAIN,
        CMD_RELEASE_TRAIN,
        CMD_LOAD_STATE,
        CMD_CONSIST_ADD,
        CMD_CONSIST_DEL,
        CMD_CONSIST_QRY,
    };

    void reset(const TractionThrottleCommands::AssignTrain &, const NodeID &dst)
    {
        cmd = CMD_ASSIGN_TRAIN;
        this->dst = dst;
    }

    void reset(const TractionThrottleCommands::ReleaseTrain &)
    {
        cmd = CMD_RELEASE_TRAIN;
    }

    void reset(const TractionThrottleCommands::LoadState &)
    {
        cmd = CMD_LOAD_STATE;
    }

    void reset(const TractionThrottleCommands::ConsistAdd &, NodeID slave, uint8_t flags)
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


    BarrierNotifiable done;
    /// If high bits are zero, this is a 16-bit OpenLCB result code. Higher
    /// values are OpenMRN errors.
    int resultCode;
    /// For assign controller reply REJECTED, this is 1 for controller refused
    /// connection, 2 fortrain refused connection.
    uint8_t replyCause;
    /// Total number of entries in the consisting list.
    uint8_t consistCount;
    /// Index of the entry in the consisting list that needs to be returned.
    uint8_t consistIndex;
};

/** Interface for a single throttle for running a train node.
 *
 */
class TractionThrottle
    : public StateFlow<Buffer<TractionThrottleInput>, QList<1>>,
      public openlcb::TrainImpl
{
public:
    /// @param node is the openlcb node from which this throttle will be
    /// sending its messages.
    TractionThrottle(Node *node)
        : StateFlow<Buffer<TractionThrottleInput>, QList<1>>(node->iface())
        , node_(node)
    {
    }

    ~TractionThrottle() {
    }

    using Command = TractionThrottleInput::Command;

    enum
    {
        /// Timeout for assign controller request.
        TIMEOUT_NSEC = SEC_TO_NSEC(1),
        /// Returned from get_fn() when we don't have a cahced value for a
        /// function.
        FN_NOT_KNOWN = 0xffff,
        /// Upon a load state request, how far do we go into the function list?
        MAX_FN_QUERY = 8,
        ERROR_UNASSIGNED = 0x4000000,
    };

    void set_speed(SpeedType speed) override
    {
        send_traction_message(TractionDefs::speed_set_payload(speed));
        lastSetSpeed_ = speed;
    }

    SpeedType get_speed() override
    {
        // TODO: if we don't know the current speed, we should probably go and
        // ask.
        return lastSetSpeed_;
    }

    void set_emergencystop() override
    {
        send_traction_message(TractionDefs::estop_set_payload());
        lastSetSpeed_.set_mph(0);
    }

    void set_fn(uint32_t address, uint16_t value) override
    {
        send_traction_message(TractionDefs::fn_set_payload(address, value));
        lastKnownFn_[address] = value;
    }

    uint16_t get_fn(uint32_t address) override
    {
        auto it = lastKnownFn_.find(address);
        if (it != lastKnownFn_.end())
        {
            return it->second;
        }
        return FN_NOT_KNOWN;
    }

    uint32_t legacy_address() override
    {
        return 0;
    }

    dcc::TrainAddressType legacy_address_type() override
    {
        return dcc::TrainAddressType::DCC_SHORT_ADDRESS;
    }

    /// Determine if a train is currently assigned to this trottle.
    /// @return true if a train is assigned, else false
    bool is_train_assigned()
    {
        return assigned_;
    }

private:
    Action entry() override
    {
        switch (message()->data()->cmd)
        {
            case Command::CMD_ASSIGN_TRAIN:
            {
                if (assigned_)
                {
                    return call_immediately(STATE(release_train));
                }
                else
                {
                    return call_immediately(STATE(assign_train));
                }
                break;
            }
            case Command::CMD_RELEASE_TRAIN:
            {
                if (assigned_)
                {
                    return call_immediately(STATE(release_train));
                }
                else
                {
                    return return_ok();
                }
            }
            case Command::CMD_LOAD_STATE:
            {
                if (!dst_)
                {
                    return return_with_error(ERROR_UNASSIGNED);
                }
                return call_immediately(STATE(load_state));
            }
            case Command::CMD_CONSIST_ADD:
            {
                if (!dst_)
                {
                    return return_with_error(ERROR_UNASSIGNED);
                }
                if (!input()->dst)
                {
                    return return_with_error(Defs::ERROR_INVALID_ARGS);
                }
                return call_immediately(STATE(consist_add));
            }
            case Command::CMD_CONSIST_DEL:
            {
                if (!dst_)
                {
                    return return_with_error(ERROR_UNASSIGNED);
                }
                if (!input()->dst)
                {
                    return return_with_error(Defs::ERROR_INVALID_ARGS);
                }
                return call_immediately(STATE(consist_del));
            }
            case Command::CMD_CONSIST_QRY:
            {
                if (!dst_)
                {
                    return return_with_error(ERROR_UNASSIGNED);
                }
                return call_immediately(STATE(consist_qry));
            }
            default:
                LOG_ERROR("Unknown traction throttle command %d received.",
                    input()->cmd);
                return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
    }

    Action release_train()
    {
        send_traction_message(TractionDefs::release_controller_payload(node_));
        clear_assigned();
        clear_cache();
        if (input()->cmd == Command::CMD_ASSIGN_TRAIN)
        {
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(50), STATE(assign_train));
        }
        else
        {
            return return_ok();
        }
    }

    Action assign_train()
    {
        dst_ = input()->dst;
        handler_.wait_for_response(
            NodeHandle(dst_), TractionDefs::RESP_CONTROLLER_CONFIG, &timer_);
        send_traction_message(TractionDefs::assign_controller_payload(node_));
        return sleep_and_call(&timer_, TIMEOUT_NSEC, STATE(assign_response));
    }

    Action assign_response()
    {
        handler_.wait_timeout();
        if (!handler_.response())
        {
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }

        AutoReleaseBuffer<GenMessage> rb(handler_.response());
        const string &payload = handler_.response()->data()->payload;
        if (payload.size() < 3)
        {
            return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
        if (payload[1] != TractionDefs::CTRLRESP_ASSIGN_CONTROLLER)
        {
            // spurious reply message
            return return_with_error(Defs::ERROR_OUT_OF_ORDER);
        }
        input()->replyCause = payload[2];
        if (payload[2] != 0)
        {
            return return_with_error(Defs::ERROR_REJECTED);
        }
        set_assigned();
        return return_ok();
    }

    Action load_state()
    {
        pendingQueries_ = 1;
        send_traction_message(TractionDefs::speed_get_payload());
        for (int i = 0; i < MAX_FN_QUERY; ++i)
        {
            pendingQueries_++;
            send_traction_message(TractionDefs::fn_get_payload(i));
        }
        return sleep_and_call(&timer_, TIMEOUT_NSEC, STATE(load_done));
    }

    Action load_done()
    {
        if (!timer_.is_triggered())
        {
            // timed out
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }
        else
        {
            return return_ok();
        }
    }

    void pending_reply_arrived()
    {
        if (pendingQueries_ > 0)
        {
            if (!--pendingQueries_)
            {
                timer_.trigger();
            }
        }
    }

    void speed_reply(Buffer<GenMessage> *msg)
    {
        AutoReleaseBuffer<GenMessage> rb(msg);
        if (!iface()->matching_node(msg->data()->src, NodeHandle(dst_)))
        {
            return;
        }
        const Payload &p = msg->data()->payload;
        if (p.size() < 1)
            return;
        switch (p[0])
        {
            case TractionDefs::RESP_QUERY_SPEED:
            {
                pending_reply_arrived();
                Velocity v;
                if (TractionDefs::speed_get_parse_last(p, &v))
                {
                    lastSetSpeed_ = v;
                    // TODO(balazs.racz): call a callback for the client.
                }
                return;
            }
            case TractionDefs::RESP_QUERY_FN:
            {
                pending_reply_arrived();
                uint16_t v;
                if (TractionDefs::fn_get_parse(p, &v))
                {
                    uint32_t num = p[1];
                    num <<= 8;
                    num |= p[2];
                    num <<= 8;
                    num |= p[3];
                    lastKnownFn_[num] = v;
                }
            }
        }
    }

    Action consist_add()
    {
        handler_.wait_for_response(
            NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
        send_traction_message(
            TractionDefs::consist_add_payload(input()->dst, input()->flags));
        return sleep_and_call(&timer_, TIMEOUT_NSEC, STATE(consist_add_response));
    }

    Action consist_del()
    {
        handler_.wait_for_response(
            NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
        send_traction_message(TractionDefs::consist_del_payload(input()->dst));
        return sleep_and_call(&timer_, TIMEOUT_NSEC, STATE(consist_add_response));
    }

    Action consist_add_response()
    {
        handler_.wait_timeout();
        if (!handler_.response())
        {
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }

        AutoReleaseBuffer<GenMessage> rb(handler_.response());
        const string &payload = handler_.response()->data()->payload;
        if (payload.size() < 9)
        {
            return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
        if (message()->data()->cmd == Command::CMD_CONSIST_ADD)
        {
            if (payload[1] != TractionDefs::CNSTRESP_ATTACH_NODE)
            {
                // spurious reply message
                return return_with_error(Defs::ERROR_OUT_OF_ORDER);
            }
        }
        else if (message()->data()->cmd == Command::CMD_CONSIST_DEL)
        {
            if (payload[1] != TractionDefs::CNSTRESP_DETACH_NODE)
            {
                // spurious reply message
                return return_with_error(Defs::ERROR_OUT_OF_ORDER);
            }
        } else if (data_to_node_id(&payload[2]) != input()->dst) {
            // spurious reply message
            return return_with_error(Defs::ERROR_OUT_OF_ORDER);
        }
        uint16_t e = payload[8];
        e <<= 8;
        e |= payload[9];
        input()->replyCause = e ? 1 : 0;
        return return_with_error(e);
    }

    Action consist_qry()
    {
        handler_.wait_for_response(
            NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
        if (input()->replyCause == 0xff)
        {
            send_traction_message(TractionDefs::consist_qry_payload());
        }
        else
        {
            send_traction_message(
                TractionDefs::consist_qry_payload(input()->consistIndex));
        }
        return sleep_and_call(
            &timer_, TIMEOUT_NSEC, STATE(consist_qry_response));
    }

    Action consist_qry_response()
    {
        handler_.wait_timeout();
        if (!handler_.response())
        {
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }

        AutoReleaseBuffer<GenMessage> rb(handler_.response());
        const string &payload = handler_.response()->data()->payload;
        if (payload.size() < 3)
        {
            return return_with_error(Defs::ERROR_INVALID_ARGS_MESSAGE_TOO_SHORT);
        }
        if (payload[1] != TractionDefs::CNSTRESP_QUERY_NODES)
        {
            // spurious reply message
            return return_with_error(Defs::ERROR_OUT_OF_ORDER);
        }
        input()->consistCount = payload[2];
        if (payload.size() >= 11) {
            input()->consistIndex = payload[3];
            input()->flags = payload[4];
            input()->dst = data_to_node_id(&payload[5]);
        } else {
            input()->consistIndex = 0xff;
            input()->flags = 0xff;
            input()->dst = 0;
        }
        input()->replyCause = 0;
        return return_ok();
    }

    Action return_ok()
    {
        return return_with_error(0);
    }

    Action return_with_error(int error)
    {
        input()->resultCode = error;
        return_buffer();
        return exit();
    }

    /** Allocates (synchronously) an outgoing openlcb buffer with traction
     * request MTI and the given payload and sends off the message to the bus
     * for dst_. */
    void send_traction_message(const Payload &payload)
    {
        HASSERT(dst_ != 0);
        auto *b = iface()->addressed_message_write_flow()->alloc();
        b->data()->reset(Defs::MTI_TRACTION_CONTROL_COMMAND, node_->node_id(),
            NodeHandle(dst_), payload);
        iface()->addressed_message_write_flow()->send(b);
    }

    void set_assigned()
    {
        iface()->dispatcher()->register_handler(&speedReplyHandler_, Defs::MTI_TRACTION_CONTROL_REPLY, Defs::MTI_EXACT);
        assigned_ = true;
    }

    void clear_assigned()
    {
        if (!assigned_)
        {
            return;
        }
        assigned_ = false;
        iface()->dispatcher()->unregister_handler(&speedReplyHandler_, Defs::MTI_TRACTION_CONTROL_REPLY, Defs::MTI_EXACT);
    }

    void clear_cache()
    {
        lastSetSpeed_ = nan_to_speed();
        lastKnownFn_.clear();
    }

    TractionThrottleInput *input()
    {
        return message()->data();
    }

    If *iface()
    {
        // We know that the service pointer is the node's interface from the
        // constructor.
        return static_cast<If *>(service());
    }

    MessageHandler::GenericHandler speedReplyHandler_{
        this, &TractionThrottle::speed_reply};
    /// How many speed/fn query requests I have sent off to the train node that
    /// have not yet seen a reply.
    unsigned pendingQueries_{0};
    StateFlowTimer timer_{this};
    /// True if the assign controller has returned positive.
    bool assigned_{false};
    NodeID dst_;
    Node *node_;
    /// Helper class for stateful query/return flows.
    TractionResponseHandler handler_{iface(), node_};
    /// Cache: Velocity value that we last commanded to the train.
    SpeedType lastSetSpeed_;
    /// Cache: all known function values.
    std::map<uint32_t, uint16_t> lastKnownFn_;
};

} // namespace openlcb

#endif // _NMRANET_TRACTIONTHROTTLE_HXX_
