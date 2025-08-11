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

#ifndef _OPENLCB_TRACTIONTHROTTLE_HXX_
#define _OPENLCB_TRACTIONTHROTTLE_HXX_

#include "executor/CallableFlow.hxx"
#include "openlcb/TractionClient.hxx"
#include "openlcb/TractionDefs.hxx"
#include "openlcb/TractionThrottleInterface.hxx"
#include "openlcb/TrainInterface.hxx"
#include "utils/LinkedObject.hxx"

namespace openlcb
{

/** Interface for a single throttle for running a train node.
 *
 */
class TractionThrottle : public TractionThrottleBase,
                         public LinkedObject<TractionThrottle>
{
public:
    /// @param node is the openlcb node from which this throttle will be
    /// sending its messages.
    TractionThrottle(Node *node)
        : TractionThrottleBase(node->iface())
        , node_(node)
    {
        clear_cache();
    }

    ~TractionThrottle()
    {
        iface()->dispatcher()->unregister_handler_all(&listenReplyHandler_);
        iface()->dispatcher()->unregister_handler_all(&speedReplyHandler_);
    }

    using Command = TractionThrottleInput::Command;

    enum
    {
        /// Timeout for assign controller request.
        TIMEOUT_NSEC = SEC_TO_NSEC(2),
        /// Upon a load state request, how far do we go into the function list?
        MAX_FN_QUERY = 28,
        ERROR_UNASSIGNED = 0x4000000,
        ERROR_ASSIGNED = 0x4010000,
    };

    void set_speed(SpeedType speed) override
    {
        send_traction_message_with_loopback(
            TractionDefs::speed_set_payload(speed));
        lastSetSpeed_ = speed;
        estopActive_ = false;
    }

    SpeedType get_speed() override
    {
        // TODO: if we don't know the current speed, we should probably go and
        // ask.
        return lastSetSpeed_;
    }

    void set_emergencystop() override
    {
        send_traction_message_with_loopback(TractionDefs::estop_set_payload());
        estopActive_ = true;
        lastSetSpeed_.set_mph(0);
    }

    /// Get the current E-Stop state.
    /// @return true if the train is E-Stopped, else false
    bool get_emergencystop() override
    {
        return estopActive_;
    }

    void set_fn(uint32_t address, uint16_t value) override
    {
        send_traction_message_with_loopback(
            TractionDefs::fn_set_payload(address, value));
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

    void toggle_fn(uint32_t fn) override
    {
        auto fnstate = get_fn(fn);
        if (fnstate == FN_NOT_KNOWN)
        {
            fnstate = 1;
        }
        else
        {
            fnstate = !fnstate;
        }
        set_fn(fn, fnstate);        
    }

    /// Sends out a function query command. The throttle listener will be
    /// called when the response is available.
    /// @param address function to query.
    void query_fn(uint32_t address) override
    {
        send_traction_message(TractionDefs::fn_get_payload(address));
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
    bool is_train_assigned() override
    {
        return assigned_;
    }

    /// @return the controlling node (virtual node of the throttle, i.e., us.)
    openlcb::Node* throttle_node() override
    {
        return node_;
    }

    /// @return the controlled node (the train node) ID.
    openlcb::NodeID target_node() override
    {
        return dst_;
    }

    /// Sets up a callback for listening for remote throttle updates. When a
    /// different throttle modifies the train node's state, and the
    /// ASSIGN_TRAIN command was executed with "listen==true" parameter, we
    /// will get notifications about those remote changes. The notifications
    /// update the cached state in TractionThrottle, and call this update
    /// callback. Repeat with nullptr if the callbacks are not desired anymore.
    /// @param update_callback will be executed when a different throttle
    /// changes the train state. fn is the function number changed, or -1 for
    /// speed update.
    void set_throttle_listener(std::function<void(int fn)> update_callback) override
    {
        updateCallback_ = std::move(update_callback);
    }

#ifdef GTEST
    void TEST_assign_listening_node(openlcb::NodeID dst)
    {
        dst_ = dst;
        set_assigned();
        set_listening();
    }
#endif

private:
    Action entry() override
    {
        switch (message()->data()->cmd)
        {
            case Command::CMD_SET_DST:
            {
                if (assigned_)
                {
                    return return_with_error(ERROR_ASSIGNED);
                }
                dst_ = input()->dst;
                return return_ok();
            }
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
        if (listenConsist_)
        {
            // Checks if there is another throttle listening to the same
            // train. If so, we will not unregister ourselves. The last local
            // throttle to release will do the unregistering.
            bool found_other_throttle = false;
            {
                AtomicHolder h(LinkedObject<TractionThrottle>::head_mu());
                for (TractionThrottle *p =
                         LinkedObject<TractionThrottle>::link_head();
                     p && !found_other_throttle;
                     p = p->LinkedObject<TractionThrottle>::link_next())
                {
                    if (p == this)
                    {
                        // self, ignore
                        continue;
                    }
                    if (p->node_ != node_)
                    {
                        // Different virtual node, will get regular
                        // feedback
                        continue;
                    }
                    if (p->dst_ != dst_)
                    {
                        // Target node ID is different.
                        continue;
                    }
                    if (!p->listenConsist_)
                    {
                        // other throttle was not set as listener to begin with
                        continue;
                    }
                    found_other_throttle = true;
                }
            }
            if (!found_other_throttle)
            {
                LOG(VERBOSE, "unregister listener");
                handler_.wait_for_response(NodeHandle(dst_),
                    TractionDefs::RESP_CONSIST_CONFIG, &timer_);
                send_traction_message(
                    TractionDefs::consist_del_payload(node_->node_id()));
                return sleep_and_call(
                    &timer_, TIMEOUT_NSEC, STATE(release_listener_response));
            }
            LOG(VERBOSE,
                "skipping unregister consist because of another throttle.");
            // we do have to remove the handler though.
            clear_listening();
        }
        return call_immediately(STATE(release_step_2));
    }

    Action release_listener_response()
    {
        handler_.wait_timeout();
        if (handler_.response())
        {
            handler_.response()->unref();
        }
        clear_listening();
        return call_immediately(STATE(release_step_2));
    }

    Action release_step_2()
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
        if (input()->flags)
        {
            // need to add consist listener
            handler_.wait_for_response(
                NodeHandle(dst_), TractionDefs::RESP_CONSIST_CONFIG, &timer_);
            send_traction_message(TractionDefs::consist_add_payload(
                node_->node_id(),
                TractionDefs::CNSTFLAGS_HIDE | TractionDefs::CNSTFLAGS_LINKF0 |
                    TractionDefs::CNSTFLAGS_LINKFN));
            return sleep_and_call(
                &timer_, TIMEOUT_NSEC, STATE(assign_consist_response));
        }
        return return_ok();
    }

    Action assign_consist_response()
    {
        // All error responses are actually okay here; we succeeded in the
        // assignment but the listener setup didn't work.
        handler_.wait_timeout();
        if (!handler_.response())
        {
            return return_ok();
        }

        AutoReleaseBuffer<GenMessage> rb(handler_.response());
        // Marks that we are owning the listener.
        set_listening();
        return return_ok();
    }

    Action load_state()
    {
        pendingQueries_ = 1;
        send_traction_message(TractionDefs::speed_get_payload());
        for (int i = 0; i <= MAX_FN_QUERY; ++i)
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
            pendingQueries_ = 0;
            return return_with_error(Defs::OPENMRN_TIMEOUT);
        }
        else
        {
            return return_ok();
        }
    }

    /// Notifies that a pending query during load has gotten a reply.
    /// @return true if we were in the load state.
    bool pending_reply_arrived()
    {
        if (pendingQueries_ > 0)
        {
            if (!--pendingQueries_)
            {
                timer_.trigger();
            }
            return true;
        }
        return false;
    }

    /// Invoked for TRACTION_CONTROL_REPLY messages coming in via the
    /// dispatcher.
    void speed_reply(Buffer<GenMessage> *msg)
    {
        AutoReleaseBuffer<GenMessage> rb(msg);
        if (msg->data()->dstNode != node_)
        {
            // For a different throttle.
            return;
        }
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
                bool expected = pending_reply_arrived();
                Velocity v;
                bool is_estop;
                if (TractionDefs::speed_get_parse_last(p, &v, &is_estop))
                {
                    lastSetSpeed_ = v;
                    estopActive_ = is_estop;
                    if (updateCallback_ && !expected)
                    {
                        updateCallback_(-1);
                    }
                }
                return;
            }
            case TractionDefs::RESP_QUERY_FN:
            {
                bool expected = pending_reply_arrived();
                uint16_t v;
                unsigned num;
                if (TractionDefs::fn_get_parse(p, &v, &num))
                {
                    lastKnownFn_[num] = v;
                    if (updateCallback_ && !expected)
                    {
                        updateCallback_(num);
                    }
                }
            }
            case TractionDefs::RESP_TRACTION_MGMT:
            {
                if (p.size() >= 2 && p[1] == TractionDefs::MGMTRESP_HEARTBEAT)
                {
                    // Automatically responds to heartbeat requests.
                    send_traction_message(TractionDefs::noop_payload());
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

    /// Invoked for TRACTION_CONTROL_COMMAND messages coming in via the
    /// dispatcher. These are generally update commands coming on when another
    /// throttle is controlling the same loco or consist via another member.
    void listen_reply(Buffer<GenMessage> *msg)
    {
        AutoReleaseBuffer<GenMessage> rb(msg);
        if (msg->data()->dstNode != node_)
        {
            // For a different throttle.
            return;
        }
        if (!iface()->matching_node(msg->data()->src, NodeHandle(dst_)))
        {
            return;
        }
        listen_reply_process(msg->data()->payload);
    }

    /// Business logic for interpreting a proxied traction command payload. The
    /// command may be coming back as a consist forward message due to throttle
    /// listener, or may be one that went out from another TractionThrottle
    /// instance to the same train.
    void listen_reply_process(const Payload &p)
    {
        if (p.size() < 1)
            return;
        switch (p[0] & TractionDefs::REQ_MASK)
        {
            case TractionDefs::REQ_SET_SPEED:
            {
                Velocity v;
                // speed get and set have the same signature for what we care
                if (TractionDefs::speed_get_parse_last(p, &v))
                {
                    lastSetSpeed_ = v;
                    estopActive_ = false;
                    if (updateCallback_)
                    {
                        updateCallback_(-1);
                    }
                }
                return;
            }
            case TractionDefs::REQ_EMERGENCY_STOP:
            {
                estopActive_ = true;
                if (updateCallback_)
                {
                    updateCallback_(-1);
                }
                return;
            }
            case TractionDefs::REQ_SET_FN:
            {
                uint16_t v;
                unsigned num;
                // function get and set have the same signature
                if (TractionDefs::fn_get_parse(p, &v, &num))
                {
                    lastKnownFn_[num] = v;
                    if (updateCallback_)
                    {
                        updateCallback_(num);
                    }
                }
                return;
            }
        }
    }

    /// Allocates (synchronously) an outgoing openlcb buffer with traction
    /// request MTI and the given payload and sends off the message to the bus
    /// for dst_.
    ///
    /// Performs loopback to other traction throttles that might be assigned to
    /// the same train.
    ///
    /// @param payload is the data contents of the message
    /// (e.g. TractionDefs::speed_set_payload(...).
    void send_traction_message_with_loopback(Payload payload)
    {
        auto b = send_traction_message_helper(std::move(payload));
        std::function<void()> f = std::bind(
            &TractionThrottle::loopback_traction_message, this, b.release());
        iface()->executor()->add(new CallbackExecutable(std::move(f)));
    }

    /// Performs loopback processing of an outgoing traction message. Run on
    /// the iface()'s executor.
    ///
    /// @param b the message that was sent out. Will be unreffed.
    void loopback_traction_message(Buffer<GenMessage>* b)
    {
        auto rb = get_buffer_deleter(b);
        // Walks all TractionThrottle objects.
        TractionThrottle *p = nullptr;
        do
        {
            {
                // finds next instance that's interesting
                AtomicHolder h(LinkedObject<TractionThrottle>::head_mu());
                while (true)
                {
                    if (!p)
                    {
                        p = LinkedObject<TractionThrottle>::link_head();
                    }
                    else
                    {
                        p = p->LinkedObject<TractionThrottle>::link_next();
                    }
                    if (!p)
                    {
                        break;
                    }
                    if (p == this)
                    {
                        // self, ignore
                        continue;
                    }
                    if (p->node_ != node_)
                    {
                        // Differnet virtual node, will get regular
                        // feedback
                        continue;
                    }
                    if (p->dst_ != dst_)
                    {
                        // Target node ID is different.
                        continue;
                    }
                    if (!p->listenConsist_)
                    {
                        // other throttle was not set as listener to begin with
                        continue;
                    }
                    
                    // Will call p, but we need to get out of the
                    // atomic first.
                    break;
                }
            } // atomic
            if (p)
            {
                p->listen_reply_process(b->data()->payload);
            }
        } while (p != nullptr);
    }

    /// Allocates (synchronously) an outgoing openlcb buffer with traction
    /// request MTI and the given payload and sends off the message to the bus
    /// for dst_.
    ///
    /// @param payload is the data contents of the message
    /// (e.g. TractionDefs::speed_set_payload(...).
    void send_traction_message(Payload payload)
    {
        send_traction_message_helper(std::move(payload));
    }
    
    /// Allocates (synchronously) an outgoing openlcb buffer with traction
    /// request MTI and the given payload and sends off the message to the bus
    /// for dst_.
    ///
    /// Returns a reference to the buffer.
    BufferPtr<GenMessage> send_traction_message_helper(Payload payload)
    {
        HASSERT(dst_ != 0);
        auto *b = iface()->addressed_message_write_flow()->alloc();
        b->data()->reset(Defs::MTI_TRACTION_CONTROL_COMMAND, node_->node_id(),
            NodeHandle(dst_), std::move(payload));
        iface()->addressed_message_write_flow()->send(b->ref());
        return get_buffer_deleter(b);
    }

    void set_listening()
    {
        listenConsist_ = true;
        iface()->dispatcher()->register_handler(&listenReplyHandler_,
            Defs::MTI_TRACTION_CONTROL_COMMAND, Defs::MTI_EXACT);
    }

    void clear_listening()
    {
        listenConsist_ = false;
        iface()->dispatcher()->unregister_handler(&listenReplyHandler_,
            Defs::MTI_TRACTION_CONTROL_COMMAND, Defs::MTI_EXACT);
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
        estopActive_ = false;
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
    MessageHandler::GenericHandler listenReplyHandler_{
        this, &TractionThrottle::listen_reply};
    /// How many speed/fn query requests I have sent off to the train node that
    /// have not yet seen a reply.
    unsigned pendingQueries_{0};
    StateFlowTimer timer_{this};
    /// True if the assign controller has returned positive.
    bool assigned_{false};
    /// True if we also have a consist link with the assigned loco.
    bool listenConsist_{false};
    /// keep track if E-Stop is active
    bool estopActive_{false};
    NodeID dst_{0};
    Node *node_;
    /// Helper class for stateful query/return flows.
    TractionResponseHandler handler_{iface(), node_};
    /// Function to call when a different controller updates the train.
    std::function<void(int fn)> updateCallback_;
    /// Cache: Velocity value that we last commanded to the train.
    SpeedType lastSetSpeed_;
    /// Cache: all known function values.
    std::map<uint32_t, uint16_t> lastKnownFn_;
};

} // namespace openlcb

#endif // _OPENLCB_TRACTIONTHROTTLE_HXX_
