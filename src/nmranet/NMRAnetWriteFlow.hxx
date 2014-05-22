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
 * \file WriteFlow.hxx
 *
 * Class that allows enqueing an outgoing message.
 *
 * @author Balazs Racz
 * @date 3 Nov 2013
 */

#ifndef _NMRAnetWriteFlow_hxx_
#define _NMRAnetWriteFlow_hxx_

#include <string>

#include "nmranet_config.h"

//#include "nmranet/NMRAnetAsyncNode.hxx"
#include "nmranet/AsyncIf.hxx"

namespace nmranet
{

class WriteHelper : public Executable
{
public:
    typedef AsyncNode *node_type;
    typedef string payload_type;

    static NodeHandle global()
    {
        return {0, 0};
    }

    WriteHelper()
        : waitForLocalLoopback_(0)
    {
    }

    void set_wait_for_local_loopback(bool wait = true)
    {
        waitForLocalLoopback_ = (wait ? 1 : 0);
    }

    /** Originates an NMRAnet message from a particular node.
     *
     * @param node is the originating node.
     * @param mti is the message to send
     * @param dst is the destination node to send to (may be Global())
     * @param buffer is the message payload.
     * @param done will be notified when the packet has been enqueued to the
     * physical layer. If done == nullptr, the sending is invoked synchronously.
     */
    void WriteAsync(AsyncNode *node, Defs::MTI mti, NodeHandle dst,
                    const payload_type &buffer, Notifiable *done)
    {
        if (done)
        {
            done_.reset(done);
        }
        else
        {
            // We don't support synchronous sending anymore.
            HASSERT(0);
        }
        if (!node ||
            (!node->is_initialized() && mti != Defs::MTI_INITIALIZATION_COMPLETE))
        {
            done_.notify();
            return;
        }
        node_ = node;
        mti_ = mti;
        dst_ = dst;
        buffer_ = buffer;
        if (dst == global())
        {
            node->interface()->global_message_write_flow()->alloc_async(this);
        }
        else
        {
            node->interface()->addressed_message_write_flow()->alloc_async(
                this);
        }
    }

private:
    // Callback from the allocator.
    virtual void alloc_result(QMember *entry)
    {

        /*
        x            e->WriteAddressedMessage(mti_, node_->node_id(), dst_,
        buffer_,
        -                                     done_);
        +           e->WriteGlobalMessage(mti_, node_->node_id(), buffer_,
        done_);
        -
        */
        /* NOTE(balazs.racz): We could choose not to pass on the done_
           * callback. That will allow the current write flow to be released
           * earlier for reuse, but breaks the assumption that done means that
           * the current packet is enqueued on the physical layer. */
        if (dst_ == global())
        {
            auto *f = node_->interface()->global_message_write_flow();
            Buffer<NMRAnetMessage> *b = f->cast_alloc(entry);
            b->data()->reset(mti_, node_->node_id(), buffer_);
            if (waitForLocalLoopback_)
            {
                b->data()->set_flag_dst(
                    NMRAnetMessage::WAIT_FOR_LOCAL_LOOPBACK);
            }
            b->set_done(&done_);
            f->send(b);
        }
        else
        {
            auto *f = node_->interface()->addressed_message_write_flow();
            auto *b = f->cast_alloc(entry);
            b->data()->reset(mti_, node_->node_id(), dst_, buffer_);
            if (waitForLocalLoopback_)
            {
                b->data()->set_flag_dst(
                    NMRAnetMessage::WAIT_FOR_LOCAL_LOOPBACK);
            }
            b->set_done(&done_);
            f->send(b);
        }
    }

    virtual void run()
    {
        HASSERT(0);
    }

    unsigned waitForLocalLoopback_ : 1;
    NodeHandle dst_;
    Defs::MTI mti_;
    AsyncNode *node_;
    payload_type buffer_;
    BarrierNotifiable done_;
};
string EventIdToBuffer(uint64_t eventid);

}; /* namespace nmranet */

#endif /* _NMRAnetWriteFlow_hxx_ */
