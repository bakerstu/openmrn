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
 * \file TractionTrain.cxx
 *
 * Defines an NMRAnet Train node.
 *
 * @author Balazs Racz
 * @date 5 May 2014
 */

//#define LOGLEVEL VERBOSE

#include "nmranet/TractionTrain.hxx"

#include "utils/logging.h"
#include "nmranet/If.hxx"

namespace nmranet
{

TrainNode::TrainNode(TrainService *service, TrainImpl *train)
    : isInitialized_(0)
    , service_(service)
    , train_(train)
    , controllerNodeId_(0)
{
    service_->register_train(this);
}

NodeID TrainNode::node_id()
{
    /** @TODO(balazs.racz) revise how to specify the nodeid for non-DCC
     * trains. */
    return TractionDefs::NODE_ID_DCC | train_->legacy_address();
}

If *TrainNode::interface()
{
    return service_->interface();
}

struct TrainService::Impl
{
    class TractionRequestFlow;

    Impl(TrainService *parent) : traction_(parent)
    {
    }

    class TractionRequestFlow : public IncomingMessageStateFlow
    {
    public:
        TractionRequestFlow(TrainService *service)
            : IncomingMessageStateFlow(service->interface())
            , reserved_(0)
            , trainService_(service)
        {
            interface()->dispatcher()->register_handler(
                this, Defs::MTI_TRACTION_CONTROL_COMMAND, 0xffff);
        }

        ~TractionRequestFlow()
        {
            interface()->dispatcher()->unregister_handler(
                this, Defs::MTI_TRACTION_CONTROL_COMMAND, 0xffff);
        }

    protected:
        TrainNode *train_node()
        {
            return static_cast<TrainNode *>(nmsg()->dstNode);
        }

        Action entry() OVERRIDE
        {
            // If the message is not for a local node, ignore.
            if (!nmsg()->dstNode)
            {
                LOG(VERBOSE, "Traction message for unknown node.");
                return release_and_exit();
            }
            // Checks if destination is a local traction-enabled node.
            if (trainService_->nodes_.find(train_node()) ==
                trainService_->nodes_.end())
            {
                LOG(VERBOSE, "Traction message for node %p that is not "
                             "traction enabled.",
                    train_node());
                /** @TODO(balazs.racz): This is probably not the good solution
                 * here; since this is an addressed message we should rather
                 * send a reject response. */
                return release_and_exit();
            }
            // No command byte?
            if (size() < 1)
            {
                LOG(VERBOSE, "Traction message with no command byte.");
                return reject_permanent();
            }
            uint8_t cmd = payload()[0];
            switch (cmd)
            {
                /** @TODO(balazs.racz) need to validate caller of mutating
                 * functions. The mutating options should be factored into a
                 * separate flow state. */
                case TractionDefs::REQ_SET_SPEED:
                {
                    SpeedType sp = fp16_to_speed(payload() + 1);
                    train_node()->train()->set_speed(sp);
                    return release_and_exit();
                }
                case TractionDefs::REQ_SET_FN:
                {
                    uint32_t address = payload()[1];
                    address <<= 8;
                    address |= payload()[2];
                    address <<= 8;
                    address |= payload()[3];
                    uint16_t value = payload()[4];
                    value <<= 8;
                    value |= payload()[5];
                    train_node()->train()->set_fn(address, value);
                    return release_and_exit();
                }
                case TractionDefs::REQ_EMERGENCY_STOP:
                {
                    train_node()->train()->set_emergencystop();
                    return release_and_exit();
                }
                case TractionDefs::REQ_QUERY_SPEED: // fall through
                case TractionDefs::REQ_QUERY_FN:
                {
                    // Need a response message first.
                    return allocate_and_call(
                        interface()->addressed_message_write_flow(),
                        STATE(handle_query));
                }
                case TractionDefs::REQ_CONTROLLER_CONFIG:
                {
                    return call_immediately(STATE(handle_controller_config));
                }
                case TractionDefs::REQ_TRACTION_MGMT:
                {
                    return call_immediately(STATE(handle_traction_mgmt));
                }
                default:
                {
                    LOG(VERBOSE, "Rejecting unknown traction message.");
                    return reject_permanent();
                }
            }
        }

        Action handle_query()
        {
            auto *b = initialize_response();
            uint8_t cmd = payload()[0];
            switch (cmd)
            {
                case TractionDefs::REQ_QUERY_SPEED:
                {
                    b->data()->payload.resize(8);
                    uint8_t *d =
                        reinterpret_cast<uint8_t *>(&b->data()->payload[0]);
                    d[0] = TractionDefs::RESP_QUERY_SPEED;
                    speed_to_fp16(train_node()->train()->get_speed(), d + 1);
                    d[3] = 0; // status byte: reserved.
                    speed_to_fp16(train_node()->train()->get_commanded_speed(),
                                  d + 4);
                    speed_to_fp16(train_node()->train()->get_actual_speed(),
                                  d + 6);
                    return send_response(b);
                }
                case TractionDefs::REQ_QUERY_FN:
                {
                    b->data()->payload.resize(6);
                    uint8_t *d =
                        reinterpret_cast<uint8_t *>(&b->data()->payload[0]);
                    d[0] = TractionDefs::RESP_QUERY_FN;
                    d[1] = payload()[1];
                    d[2] = payload()[2];
                    d[3] = payload()[3];
                    uint32_t address = payload()[1];
                    address <<= 8;
                    address |= payload()[2];
                    address <<= 8;
                    address |= payload()[3];
                    uint16_t fn_value = train_node()->train()->get_fn(address);
                    d[4] = fn_value >> 8;
                    d[5] = fn_value & 0xff;
                    return send_response(b);
                }
            }
            DIE("unexpected call to handle_query.");
        }

        Action handle_controller_config()
        {
            uint8_t subcmd = payload()[1];
            switch (subcmd)
            {
                case TractionDefs::CTRLREQ_ASSIGN_CONTROLLER:
                {
                    
                }
            }
            LOG(VERBOSE, "Rejecting unknown traction message.");
            return reject_permanent();
        }

        Action handle_traction_mgmt()
        {
            uint8_t cmd = payload()[1];
            switch (cmd)
            {
                case TractionDefs::MGMTREQ_RESERVE:
                {
                    uint8_t code = 0xff;
                    if (reserved_)
                    {
                        code = 0x1;
                    }
                    else
                    {
                        code = 0;
                        reserved_ = 1;
                    }
                    Payload p;
                    p.push_back(TractionDefs::RESP_TRACTION_MGMT);
                    p.push_back(TractionDefs::MGMTRESP_RESERVE);
                    p.push_back(code);
                    /// @TODO (balazs.racz): make this asynchronous.
                    auto *b =
                        interface()->addressed_message_write_flow()->alloc();
                    b->data()->reset(Defs::MTI_TRACTION_CONTROL_REPLY,
                                     nmsg()->dstNode->node_id(), nmsg()->src,
                                     p);
                    interface()->addressed_message_write_flow()->send(b);
                    return release_and_exit();
                }
                case TractionDefs::MGMTREQ_RELEASE:
                {
                    reserved_ = 0;
                    return release_and_exit();
                }
                default:
                    LOG(VERBOSE, "Unknown Traction proxy manage subcommand %x",
                        cmd);
                    return reject_permanent();
            }
        }

        /** Takes the allocation result of a response buffer (addressed write
         * flow)
         * and fills in src, dest as a response message for traction protocol.
         */
        Buffer<NMRAnetMessage> *initialize_response()
        {
            Buffer<NMRAnetMessage> *b = get_allocation_result(
                interface()->addressed_message_write_flow());
            b->data()->reset(Defs::MTI_TRACTION_CONTROL_REPLY,
                             train_node()->node_id(), nmsg()->src,
                             EMPTY_PAYLOAD);
            return b;
        }

        /** Sends off the response buffer to the client. */
        Action send_response(Buffer<NMRAnetMessage> *b)
        {
            interface()->addressed_message_write_flow()->send(b);
            return release_and_exit();
        }

        /** Returns the size of the incoming message payload. */
        size_t size()
        {
            return nmsg()->payload.size();
        }

        /** Returns the incoming message payload (bytes). */
        const uint8_t *payload()
        {
            return reinterpret_cast<const uint8_t *>(nmsg()->payload.data());
        }

        /** Rejects the incoming message with a permanent error. */
        Action reject_permanent()
        {
            return allocate_and_call(
                trainService_->interface()->addressed_message_write_flow(),
                STATE(send_reject_permanent));
        }

        Action send_reject_permanent()
        {
            auto *b = get_allocation_result(
                trainService_->interface()->addressed_message_write_flow());
            // An alternative would be to send TERMINATE_DUE_TO_ERROR here.
            b->data()->reset(
                Defs::MTI_OPTIONAL_INTERACTION_REJECTED,
                nmsg()->dstNode->node_id(), nmsg()->src,
                error_to_buffer(Defs::ERROR_PERMANENT, nmsg()->mti));
            trainService_->interface()->addressed_message_write_flow()->send(b);
            return release_and_exit();
        }

    private:
        unsigned reserved_ : 1;
        TrainService *trainService_;
    };

    TractionRequestFlow traction_;
};

TrainService::TrainService(If *interface)
    : Service(interface->executor())
    , interface_(interface)
{
    impl_ = new Impl(this);
}

TrainService::~TrainService()
{
    delete impl_;
}

void TrainService::register_train(TrainNode *node)
{
    interface_->add_local_node(node);
    /** @TODO(balazs.racz) We should have a single flow for initializing all
     * trains in sequence instead of creating a new one for each in
     * parallel. */
    extern void StartInitializationFlow(Node * node);
    StartInitializationFlow(node);
    AtomicHolder h(this);
    nodes_.insert(node);
    LOG(VERBOSE, "Registered node %p for traction.", node);
    HASSERT(nodes_.find(node) != nodes_.end());
}

} // namespace nmranet
