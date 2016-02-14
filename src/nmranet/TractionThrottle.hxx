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

#include "nmranet/TractionClient.hxx"
#include "nmranet/TractionDefs.hxx"
#include "nmranet/TrainInterface.hxx"

namespace nmranet
{

struct TractionThrottleInput;

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
};

struct TractionThrottleInput
{
    enum Command
    {
        CMD_ASSIGN_TRAIN,
        CMD_RELEASE_TRAIN,
        CMD_LOAD_STATE,
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

    Command cmd;
    NodeID dst;

    BarrierNotifiable done;
    /// If high bits are zero, this is a 16-bit OpenLCB result code. Higher
    /// values are OpenMRN errors.
    int resultCode;
    /// For assign controller reply REJECTED, this is 1 for controller refused
    /// connection, 2 fortrain refused connection.
    uint8_t replyCause;
};

/** Interface for a single throttle for running a train node.
 *
 */
class TractionThrottle
    : public StateFlow<Buffer<TractionThrottleInput>, QList<1>>,
      public nmranet::TrainImpl
{
public:
    /// @param node is the nmranet node from which this throttle will be
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
                    return return_ok();
                }
                return call_immediately(STATE(load_state));
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

        AutoReleaseBuffer<NMRAnetMessage> rb(handler_.response());
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

    void speed_reply(Buffer<NMRAnetMessage> *msg)
    {
        AutoReleaseBuffer<NMRAnetMessage> rb(msg);
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

    /** Allocates (synchronously) an outgoing nmranet buffer with traction
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

    void set_assigned() {
        assigned_ = true;
        iface()->dispatcher()->register_handler(&speedReplyHandler_, Defs::MTI_TRACTION_CONTROL_REPLY, Defs::MTI_EXACT);
    }

    void clear_assigned() {
        if (!assigned_) return;
        iface()->dispatcher()->unregister_handler(&speedReplyHandler_, Defs::MTI_TRACTION_CONTROL_REPLY, Defs::MTI_EXACT);
        assigned_ = false;
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

} // namespace nmranet

#endif // _NMRANET_TRACTIONTHROTTLE_HXX_
