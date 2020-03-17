/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file DirectHubLegacy.cxx
 *
 * Connection from a DirectHub to a legacy hub.
 *
 * @author Balazs Racz
 * @date 5 Mar 2020
 */

#include "utils/DirectHub.hxx"
#include "utils/Hub.hxx"
#include "utils/gc_format.h"

extern DataBufferPool g_direct_hub_kbyte_pool;

/// Bridge component that converts the outgoing CAN packets into gridconnect
/// format and enqueues them into the DirectHub for sending.
class HubToGcPort : public CanHubPort, public DirectHubPort<uint8_t[]>
{
public:
    HubToGcPort(DirectHubInterface<uint8_t[]> *gc_hub, CanHubFlow *can_hub)
        : CanHubPort(gc_hub->get_service())
        , targetHub_(gc_hub)
        , sourceHub_(can_hub)
    {
        sourceHub_->register_port(this);
        targetHub_->register_port(this);
    }

    ~HubToGcPort()
    {
        sourceHub_->unregister_port(this);
        targetHub_->unregister_port(this);
    }

    /// Handles the next CAN packet that we need to send.
    Action entry() override
    {
        // Allocates output buffer if needed.
        if (buf_.free() < MIN_GC_FREE)
        {
            // Need more output buffer.
            DataBuffer *b;
            g_direct_hub_kbyte_pool.alloc(&b);
            buf_.append_empty_buffer(b);
        }
        // Generates gridconnect message and commits to buffer.
        char *start = (char *)buf_.data_write_pointer();
        char *end = gc_format_generate(message()->data(), start, 0);
        packetSize_ = end - start;
        buf_.data_write_advance(packetSize_);
        pktDone_ = message()->new_child();
        release();
        // Sends off output message.
        wait_and_call(STATE(do_send));
        inlineRun_ = true;
        inlineComplete_ = false;
        targetHub_->enqueue_send(this);
        inlineRun_ = false;
        if (inlineComplete_)
        {
            return exit();
        }
        else
        {
            return wait();
        }
    }

    /// Handles the callback from the direct hub when it is ready for us to
    /// send the message.
    Action do_send()
    {
        auto *m = targetHub_->mutable_message();
        m->buf_ = buf_.transfer_head(packetSize_);
        m->source_ = (DirectHubPort<uint8_t[]> *)this;
        m->done_ = pktDone_;
        targetHub_->do_send();
        if (inlineRun_)
        {
            inlineComplete_ = true;
            return wait();
        }
        else
        {
            return exit();
        }
    }

    /// GC to binary path. Called by the DirectHub with a text packet or a
    /// garbage packet.
    void send(MessageAccessor<uint8_t[]> *msg) override
    {
        auto &buf = msg->buf_;
        uint8_t *p;
        unsigned available;
        buf.head()->get_read_pointer(buf.skip(), &p, &available);
        if (buf.size() == 0 || *p != ':')
        {
            // Not a gridconnect packet. Do not do anything.
            return;
        }
        Buffer<CanHubData> *can_buf = sourceHub_->alloc();
        if (msg->done_)
        {
            can_buf->set_done(msg->done_->new_child());
        }
        can_buf->data()->skipMember_ = (CanHubPort *)this;
        const char *text_packet = nullptr;
        string assembled_packet;
        if (available == buf.size())
        {
            // One block of data. Convert in place.
            text_packet = (const char *)p;
        }
        else
        {
            buf.append_to(&assembled_packet);
            text_packet = assembled_packet.c_str();
        }
        if (gc_format_parse(text_packet, can_buf->data()) < 0)
        {
            string debug(text_packet, buf.size());
            LOG(INFO, "Failed to parse gridconnect packet: '%s'",
                debug.c_str());
            can_buf->unref();
            return;
        }
        /// @todo consider if we need to set the priority here.
        sourceHub_->send(can_buf, 0);
    }

private:
    /// Output buffer of gridconnect bytes that will be sent to the GC
    /// DirectHub.
    LinkedDataBufferPtr buf_;
    /// Where to send the target data.
    DirectHubInterface<uint8_t[]> *targetHub_;
    /// Done notifiable from the source packet.
    BarrierNotifiable *pktDone_ = nullptr;
    /// Hub where we get the input data from (registered).
    CanHubFlow *sourceHub_;
    /// The source pointer we need to use for sending messages to the target
    /// hub.
    HubSource *me_;
    /// True while we are calling the target hub send method.
    bool inlineRun_ : 1;
    /// True if the send completed inline.
    bool inlineComplete_ : 1;
    /// Number of bytes this gridconnect packet is.
    uint16_t packetSize_;
    /// Minimum amount of free bytes in the current send buffer in order to use
    /// it for gridconnect rendering.
    static constexpr unsigned MIN_GC_FREE = 29;
};

Destructable *create_gc_to_legacy_can_bridge(
    DirectHubInterface<uint8_t[]> *gc_hub, CanHubFlow *can_hub)
{

    return new HubToGcPort(gc_hub, can_hub);
}
