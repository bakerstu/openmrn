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
 * \file Dispatcher.hxx
 *
 * Class for dispatching incoming messages to handlers.
 *
 * @author Balazs Racz
 * @date 2 Dec 2013
 */

#ifndef _executor_Dispatcher_hxx_
#define _executor_Dispatcher_hxx_

#include <vector>

#include "executor/notifiable.hxx"
#include "executor/StateFlow.hxx"

/**
   This class takes registrations of StateFlows for incoming messages. When a
   message shows up, all the Flows that match that message will be
   invoked.

   Handlers are called in no particular order.
 */
template <class MessageType, int NUM_PRIO>
class DispatchFlow : public StateFlow<MessageType, QList<NUM_PRIO>>
{
public:
    /** Construct a dispatchflow.
     *
     * @param service the Service this flow belongs to. */
    DispatchFlow(Service *service);

    typedef StateFlowBase::Action Action;

    ~DispatchFlow();

    size_t handler_count()
    {
        return handlers_.size();
    }

    typedef typename MessageType::value_type::id_type ID;
    typedef FlowInterface<MessageType> HandlerType;
    /**
       Adds a new handler to this dispatcher.

       A handler will be called if incoming_id & mask == id & mask.
       If negateMatch_ then the handler will be called if incoming_id & mask !=
       id & mask.

       @param id is the identifier of the message to listen to.
       @param mask is the mask of the ID matcher. 0=match all; 0xff...f=match
       one
       @param handler is the flow to forward message to. It must stay alive so
       long as *this is alive or the handler is removed.
     */
    void register_handler(ID id, ID mask, HandlerType *handler);

    //! Removes a specific instance of a handler from this dispatcher.
    void unregister_handler(ID id, ID mask, HandlerType *handler);

protected:
    typedef typename StateFlow<MessageType, QList<NUM_PRIO>>::Callback Callback;
    using StateFlow<MessageType, QList<NUM_PRIO>>::again;
    using StateFlow<MessageType, QList<NUM_PRIO>>::call_immediately;
    using StateFlow<MessageType, QList<NUM_PRIO>>::message;
    using StateFlow<MessageType, QList<NUM_PRIO>>::release_and_exit;
    using StateFlow<MessageType, QList<NUM_PRIO>>::transfer_message;

    STATE_FLOW_STATE(entry);

/*    Action entry()
    {
        currentIndex_ = 0;
        lastHandlerToCall_ = nullptr;
        return call_immediately(STATE(iterate));
        }*/


    STATE_FLOW_STATE(iterate);
    STATE_FLOW_STATE(iteration_done);
    //STATE_FLOW_STATE(handle_call);

private:
    // true if this flow should negate the match condition.
    bool negateMatch_;

    struct HandlerInfo
    {
        HandlerInfo() : handler(nullptr)
        {
        }
        ID id;
        ID mask;
        // NULL if the handler has been removed.
        HandlerType *handler;

        bool Equals(ID id, ID mask, HandlerType *handler)
        {
            return (this->id == id && this->mask == mask &&
                    this->handler == handler);
        }
    };

    vector<HandlerInfo> handlers_;

    /// Index of the next handler to look at.
    size_t currentIndex_;

    /// If non-NULL we still need to call this handler.
    HandlerType *lastHandlerToCall_;

    OSMutex lock_;
};

// ================== IMPLEMENTATION ==================

template <class MessageType, int NUM_PRIO>
DispatchFlow<MessageType, NUM_PRIO>::DispatchFlow(Service *service)
    : StateFlow<MessageType, QList<NUM_PRIO>>(service), negateMatch_(false)
{
}

template <class MessageType, int NUM_PRIO>
DispatchFlow<MessageType, NUM_PRIO>::~DispatchFlow()
{
    HASSERT(this->is_waiting());
}

template <class MessageType, int NUM_PRIO>
void DispatchFlow<MessageType, NUM_PRIO>::register_handler(ID id, ID mask,
                                                           HandlerType *handler)
{
    OSMutexLock h(&lock_);
    size_t idx = 0;
    while (idx < handlers_.size() && handlers_[idx].handler)
    {
        ++idx;
    }
    if (idx >= handlers_.size())
    {
        handlers_.resize(handlers_.size() + 1);
    }
    handlers_[idx].handler = handler;
    handlers_[idx].id = id;
    handlers_[idx].mask = mask;
}

template <class MessageType, int NUM_PRIO>
void
DispatchFlow<MessageType, NUM_PRIO>::unregister_handler(ID id, ID mask,
                                                        HandlerType *handler)
{
    OSMutexLock h(&lock_);
    /// @todo(balazs.racz) optimize by looking at the current index - 1.
    size_t idx = 0;
    while (idx < handlers_.size() && !handlers_[idx].Equals(id, mask, handler))
    {
        ++idx;
    }
    // Checks that we found the thing to unregister.
    HASSERT(idx < handlers_.size() &&
            "Tried to unregister a handler not previously registered.");
    handlers_[idx].handler = nullptr;
    if (id == handlers_.size() - 1)
    {
        handlers_.resize(handlers_.size() - 1);
    }
}

template <class MessageType, int NUM_PRIO>
StateFlowBase::Action DispatchFlow<MessageType, NUM_PRIO>::entry()
{
    currentIndex_ = 0;
    lastHandlerToCall_ = nullptr;
    return call_immediately(STATE(iterate));
}

template <class MessageType, int NUM_PRIO>
StateFlowBase::Action DispatchFlow<MessageType, NUM_PRIO>::iterate()
{
    HandlerType *handler;
    ID id = message()->data()->id();
    LOG(INFO, "iterate id=%d", id);
    {
        OSMutexLock l(&lock_);
        while (true)
        {
            if (currentIndex_ >= handlers_.size())
            {
                return call_immediately(STATE(iteration_done));
            }
            auto &h = handlers_[currentIndex_];
            ++currentIndex_;
            if (!h.handler)
            {
                continue;
            }
            if (negateMatch_ && (id & h.mask) == (h.id & h.mask))
            {
                continue;
            }
            if ((!negateMatch_) && (id & h.mask) != (h.id & h.mask))
            {
                continue;
            }
            handler = h.handler;
            break;
        }
    }
    // At this point: we have another handler.
    if (!lastHandlerToCall_)
    {
        // This was the first we found.
        lastHandlerToCall_ = handler;
        return again();
    }
    // Now: we have at least two different handler. We need to clone the
    // message.
    HASSERT(0 && "Cloning messages is not implemented yet.");
}

template <class MessageType, int NUM_PRIO>
StateFlowBase::Action DispatchFlow<MessageType, NUM_PRIO>::iteration_done()
{
    if (lastHandlerToCall_)
    {
        lastHandlerToCall_->send(transfer_message());
    }
    return release_and_exit();
}

#endif // _executor_Dispatcher_hxx_
