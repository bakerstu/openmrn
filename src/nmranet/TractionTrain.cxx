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

#include "nmranet/TractionTrain.hxx"

namespace NMRAnet
{

TrainNode::TrainNode(TrainService *service, TrainImpl *train)
    : isInitialized_(0)
    , service_(service)
    , train_(train)
{
    service_->register_train(this);
}

NodeID TrainNode::node_id()
{
    /** @TODO(balazs.racz) revise how to specify the nodeid for non-DCC
     * trains. */
    return TractionDefs::NODE_ID_DCC | train_->legacy_address();
}

void TrainService::register_train(TrainNode *node)
{
    interface_->add_local_node(node);
    /** @TODO(balazs.racz) We should have a single flow for initializing all
     * trains in sequence instead of creating a new one for each in
     * parallel. */
    new InitializeFlow(node);
    AtomicHolder h(this);
    nodes_.insert(node);
}

TrainService::TrainService(AsyncIf *interface)
    : Service(interface->executor())
    , interface_(interface)
{
    impl_ = new Impl(this);
}

TrainService::~TrainService()
{
    delete impl_;
}

struct TrainService::Impl
{
    class TractionRequestFlow;

    Impl(TrainService *parent)
        : traction(parent)
    {
    }

    TractionRequestFlow traction_;
};

class TrainService::Impl::TractionRequestFlow : public MessageStateFlowBase
{
public:
    TractionRequestFlow(TrainService *service)
        : MessageStateFlowBase(service)
    {
    }

    TrainService *service()
    {
        return static_cast<TrainService *>(MessageStateFlowBase::service());
    }

protected:
    Action entry() OVERRIDE
    {
        // If the message is not for a local node, ignore.
        if (!nmsg()->dstNode)
            return release_and_exit();
        // No command byte?
        if (size() < 1)
            return reject_permanent();
        uint8_t cmd = payload()[0];
        switch (cmd)
        {
            case TractionDefs::REQ_SET_SPEED:
            {

                break;
            }
            default:
            {
                return reject_permanent();
            }
        }
    }

    /** Returns the size of the incoming message payload. */
    size_t size()
    {
        return nmsg()->payload.size();
    }

    /** Returns the incoming message payload (bytes). */
    const uint8_t *payload()
    {
        return static_cast<const uint8_t *>(nmsg()->payload.data());
    }

    /** Rejects the incoming message with a permanent error. */
    Action reject_permanent()
    {
        return allocate_and_call(
            service()->interface()->addressed_message_write_flow(),
            STATE(send_reject_permanent));
    }

    Action send_reject_permanent()
    {
        auto *b = get_allocation_result(
            service()->interface()->addressed_message_write_flow());
        // An alternative would be to send TERMINATE_DUE_TO_ERROR here.
        b->data()->reset(If::MTI_OPTIONAL_INTERACTION_REJECTED,
                         nmsg()->dstNode->node_id(), nmsg()->src,
                         error_to_buffer(If::ERROR_PERMANENT, nmsg()->mti));
        service()->interface()->addressed_message_write_flow()->send(b);
        return release_and_exit();
    }
}

} // namespace NMRAnet
