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
 * \file NMRAnetAsyncEventHandler.hxx
 *
 * Handler that proxies incoming event messages from Asynchronous IF to the
 * event handler flows.
 *
 * @author Balazs Racz
 * @date 7 Dec 2013
 */

#include "nmranet/NMRAnetAsyncEventHandler.hxx"

#include "nmranet/AsyncIf.hxx"
#include "nmranet/GlobalEventHandler.hxx"

namespace NMRAnet
{

class EventMessageHandler : public IncomingMessageHandler,
                            public AllocationResult
{
public:
    EventMessageHandler(AsyncIf* async_if) : if_(async_if)
    {
        if_->dispatcher()->RegisterHandler(MTI_VALUE_1, MTI_MASK_1, this);
        if_->dispatcher()->RegisterHandler(MTI_VALUE_2, MTI_MASK_2, this);
        lock_.TypedRelease(this);
    }

    ~EventMessageHandler()
    {
        if_->dispatcher()->UnregisterHandler(MTI_VALUE_1, MTI_MASK_1, this);
        if_->dispatcher()->UnregisterHandler(MTI_VALUE_2, MTI_MASK_2, this);
    }

private:
    enum
    {
        // These address/mask should match all the messages carrying an event
        // id.
        MTI_VALUE_1 = If::MTI_EVENT_MASK,
        MTI_MASK_1 = If::MTI_EVENT_MASK,
        // These match the two event messages without event id: Global and
        // addressed identify all events.
        MTI_VALUE_2 =
            If::MTI_EVENTS_IDENTIFY_ADDRESSED & If::MTI_EVENTS_IDENTIFY_GLOBAL,
        MTI_MASK_2 = (~If::MTI_SIMPLE_MASK) & (~If::MTI_ADDRESS_MASK),
    };

    virtual AllocatorBase* get_allocator()
    {
        return &lock_;
    }

    //! Handler callback for incoming messages.
    virtual void handle_message(IncomingMessage* m, Notifiable* done)
    {
        if (m->mti == If::MTI_LEARN_EVENT)
        {
            // The global event flow does not care about learn event messages.
            done->Notify();
            return;
        }
        m_ = m;
        done_ = done;
        GlobalEventFlow::instance->message_allocator()->AllocateEntry(this);
    }

    virtual void AllocationCallback(QueueMember* entry)
    {
        GlobalEventMessage* e =
            GlobalEventFlow::instance->message_allocator()->cast_result(entry);
        e->mti = m_->mti;
        e->src_node = m_->src;
        e->dst_node = m_->dst_node;
        if (m_->payload && m_->payload->used())
        {
            if (m_->payload->used() != 8)
            {
                LOG(INFO, "Invalid input event message, payload length %d",
                    m_->payload->used());
                // We continue with whetever we got.
            }
            memcpy(&e->event, m_->payload->start(), sizeof(uint64_t));
            e->event = be64toh(e->event);
        }
        else
        {
            LOG(INFO, "event message without payload");
            e->event = 0;
        }
        done_->Notify();
        GlobalEventFlow::instance->PostEvent(e);
        lock_.TypedRelease(this);
    }

    virtual void Run()
    {
        HASSERT(0);
    }

private:
    IncomingMessage* m_;
    Notifiable* done_;

    //! Lock for ourselves.
    TypedAllocator<IncomingMessageHandler> lock_;

    //! Parent interface.
    AsyncIf* if_;
};

void AddEventHandlerToIf(AsyncIf* async_if)
{
    HASSERT(GlobalEventFlow::instance);
    async_if->add_owned_flow(new EventMessageHandler(async_if));
}

} // namespace NMRAnet
