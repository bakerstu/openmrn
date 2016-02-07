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
};

struct TractionThrottleInput
{
    enum Command
    {
        CMD_ASSIGN_TRAIN,
        CMD_RELEASE_TRAIN,
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

    Command cmd;
    NodeID dst;

    BarrierNotifiable done;
    /// If high bits are zero, this is a 16-bit OpenLCB result code. Higher
    /// values are OpenMRN errors.
    int result_code;
    /// For assign controller reply REJECTED, this is 1 for controller refused
    /// connection, 2 fortrain refused connection.
    uint8_t reply_cause;
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

    using Command = TractionThrottleInput::Command;

    enum
    {
        TIMEOUT_NSEC = SEC_TO_NSEC(1),
    };

    void set_speed(SpeedType speed) override {
        send_traction_message(TractionDefs::speed_set_payload(speed));
        lastSetSpeed_ = speed;
    }

    SpeedType get_speed() override {
        // TODO: if we don't know the current speed, we should probably go and
        // ask.
        return lastSetSpeed_;
    }

    void set_emergencystop() override {
        send_traction_message(TractionDefs::estop_set_payload());
    }

    void set_fn(uint32_t address, uint16_t value) override {
        send_traction_message(TractionDefs::fn_set_payload(address, value));
        lastKnownFn_[address] = value;
    }

    uint16_t get_fn(uint32_t address) override {
        return 0;
    }

    uint32_t legacy_address() override {
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

            default:
                LOG_ERROR("Unknown traction throttle command %d received.",
                    input()->cmd);
                return return_with_error(Defs::ERROR_INVALID_ARGS);
        }
    }

    Action release_train()
    {
        send_traction_message(TractionDefs::release_controller_payload(node_));
        assigned_ = false;
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
            return return_with_error(Defs::TIMEOUT);
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
        input()->reply_cause = payload[2];
        if (payload[2] != 0)
        {
            return return_with_error(Defs::REJECTED);
        }
        assigned_ = true;
        return return_ok();
    }

    Action return_ok()
    {
        return return_with_error(0);
    }

    Action return_with_error(int error)
    {
        input()->result_code = error;
        return_buffer();
        return exit();
    }

    /** Allocates (synchronously) an outgoing nmranet buffer with traction
     * request MTI and the given payload and sends off the message to the bus
     * for dst_. */
    void send_traction_message(const Payload &payload)
    {
        auto *b = iface()->addressed_message_write_flow()->alloc();
        b->data()->reset(Defs::MTI_TRACTION_CONTROL_COMMAND, node_->node_id(),
            NodeHandle(dst_), payload);
        iface()->addressed_message_write_flow()->send(b);
    }

    void clear_cache() {
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

    StateFlowTimer timer_{this};
    bool assigned_{false};
    NodeID dst_;
    Node *node_;
    TractionResponseHandler handler_{iface(), node_};
    SpeedType lastSetSpeed_;
    std::map<uint32_t, uint16_t> lastKnownFn_;
};

} // namespace nmranet

#endif // _NMRANET_TRACTIONTHROTTLE_HXX_
