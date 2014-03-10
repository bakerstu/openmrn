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
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/NMRAnetNode.hxx"

/**
   This class takes registrations of StateFlows for incoming messages. When a
   message shows up, all the Flows that match that message will be
   invoked.

   Handlers are called in no particular order.
 */
template <class MessageType, int NUM_PRIO> class DispatchFlow : public StateFlow<MessageType, QList<NUM_PRIO> >
{
public:
    DispatchFlow(Service* service)
        : StateFlow(service) {}

    size_t handler_count()
    {
        size_t size = handlers_.size();
        if (pending_delete_index_ < 0)
            return size;
        for (int i = pending_delete_index_; i < (int)handlers_.size(); ++i)
        {
            if (handlers_[i].handler == nullptr)
                --size;
        }
        return size;
    }

    typedef typename MessageType::id_type ID;
    typedef FlowInterface<MessageType>* HandlerType;
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
    void register_handler(ID id, ID mask, HandlerType* handler);

    //! Removes a specific instance of a handler from this dispatcher.
    void unregister_handler(ID id, ID mask, HandlerType* handler);

protected:
    /**
     * Makes an implementation-specific call to the actual
     * handler. Implementations will need to take certain handler arguments
     * from member variables. The 'done' notifiable must be called in all
     * cases.
     */
    virtual void CallCurrentHandler(HandlerType* handler, Notifiable* done) = 0;

    /**
       This function will be called when the flow and all children are
       done. Useful for implementations to release memory and buffers
       associated with the current parameters.
     */
    virtual void OnFlowFinished()
    {
    };

    virtual void CheckNotStartedState()
    {
        HASSERT(IsNotStarted());
    }

protected:
    // true if this flow should negate the match condition.
    bool negateMatch_;

private:
    // State handler. Calls the current handler.
    ControlFlowAction HandleCall();
    // State handler. If calling the handler didn't work, this state will be
    // called after the allocation.
    ControlFlowAction HandleAllocateResult();
    // State handler. Waits for the children to finish.
    ControlFlowAction HandleWaitForChildren();

    struct HandlerInfo
    {
        HandlerInfo() : handler(nullptr)
        {
        }
        ID id;
        ID mask;
        // NULL if the handler has been removed.
        HandlerType* handler;

        bool Equals(ID id, ID mask, HandlerType* handler)
        {
            return (this->id == id && this->mask == mask &&
                    this->handler == handler);
        }
    };

    vector<HandlerInfo> handlers_;

    // Index of the current iteration.
    size_t current_index_;

    // These fields contain the message currently in progress.
    ID id_;

    // This notifiable tracks all the pending handlers.
    BarrierNotifiable children_;

    OSMutex lock_;

    TypedAllocator<DispatchFlow<ID>> allocator_;
};

template <typename ID, class Params>
class TypedDispatchFlow : public DispatchFlow<ID>
{
public:
    typedef ParamHandler<Params> Handler;

    TypedDispatchFlow(Executor* e) : DispatchFlow<ID>(e)
    {
    }

    TypedAllocator<TypedDispatchFlow<ID, Params>>* allocator()
    {
        return reinterpret_cast<TypedAllocator<TypedDispatchFlow<ID, Params>>*>(
            DispatchFlow<ID>::allocator());
    }

    /**
       Adds a new handler to this dispatcher.

       A handler will be called if incoming_id & mask == id & mask.

       @param id is the identifier of the message to listen to.
       @param mask is the mask of the ID matcher.
       @param handler is the handler. It must stay alive so long as this
       Dispatcher
       is alive.
     */
    void register_handler(ID id, ID mask, Handler* handler)
    {
        DispatchFlow<ID>::register_handler(id, mask, handler);
    }

    //! Removes a specific instance of a handler.
    void unregister_handler(ID id, ID mask, Handler* handler)
    {
        DispatchFlow<ID>::unregister_handler(id, mask, handler);
    }

    /** @returns the parameters structure to fill in before calling
        IncomingMessage. The caller must be in ownership of *this from an
        allocator before using this pointer.  */
    Params* mutable_params()
    {
        return &params_;
    }

protected:
    virtual void CallCurrentHandler(HandlerType* b_handler, Notifiable* done)
    {
        Handler* handler = static_cast<Handler*>(b_handler);
        handler->handle_message(&params_, done);
    }

    Params params_;
};

// ================== IMPLEMENTATION ==================

template <typename ID>
DispatchFlow<ID>::DispatchFlow(Executor* executor)
    : ControlFlow(executor, nullptr),
      negateMatch_(false),
      pending_delete_index_(-1)
{
    allocator_.Release(this);
}

template <typename ID> DispatchFlow<ID>::~DispatchFlow()
{
    HASSERT(IsNotStarted());
}

template <typename ID>
void DispatchFlow<ID>::register_handler(ID id, ID mask, HandlerType* handler)
{
    OSMutexLock h(&lock_);
    size_t idx = pending_delete_index_;
    while (idx < handlers_.size() && handlers_[idx].handler)
    {
        ++idx;
    }
    if (idx >= handlers_.size())
    {
        idx = handlers_.size();
        handlers_.resize(handlers_.size() + 1);
    }
    handlers_[idx].handler = handler;
    handlers_[idx].id = id;
    handlers_[idx].mask = mask;
}

template <typename ID>
void DispatchFlow<ID>::unregister_handler(ID id, ID mask, HandlerType* handler)
{
    OSMutexLock h(&lock_);
    // First we try the current index - 1.
    size_t idx = current_index_;
    if (idx > 0)
        idx--;
    if (idx >= handlers_.size() || !handlers_[idx].Equals(id, mask, handler))
    {
        // We try all others too.
        idx = 0;
        while (idx < handlers_.size() &&
               !handlers_[idx].Equals(id, mask, handler))
            ++idx;
    }
    // Checks that we found the thing to unregister.
    HASSERT(idx < handlers_.size());
    handlers_[idx].handler = nullptr;
    if (pending_delete_index_ < 0 || idx < (size_t)pending_delete_index_)
    {
        pending_delete_index_ = idx;
    }
}

template <typename ID> void DispatchFlow<ID>::IncomingMessage(ID id)
{
    CheckNotStartedState();
    HASSERT(children_.IsDone());
    current_index_ = 0;
    id_ = id;
    children_.Reset(this);
    StartFlowAt(ST(HandleCall));
}

template <typename ID>
ControlFlow::ControlFlowAction DispatchFlow<ID>::HandleCall()
{
    HandlerType* handler;
    {
        OSMutexLock l(&lock_);
        while (true)
        {
            if (current_index_ >= handlers_.size())
            {
                children_.MaybeDone();
                return WaitAndCall(ST(HandleWaitForChildren));
            }
            auto& h = handlers_[current_index_];
            ++current_index_;
            if (!h.handler)
            {
                continue;
            }
            if (negateMatch_ && (id_ & h.mask) == (h.id & h.mask))
            {
                continue;
            }
            if ((!negateMatch_) && (id_ & h.mask) != (h.id & h.mask))
            {
                continue;
            }
            handler = h.handler;
            break;
        }
    }
    AllocatorBase* a = handler->get_allocator();
    if (a)
    {
        return Allocate(a, ST(HandleAllocateResult));
    }
    else
    {
        CallCurrentHandler(handler, children_.NewChild());
        // Call went OK, proceed to the next handler to call.
        return RetryCurrentState();
    }
}

template <typename ID>
ControlFlow::ControlFlowAction DispatchFlow<ID>::HandleAllocateResult()
{
    HandlerType* h;
    GetAllocationResult(&h);
    CallCurrentHandler(h, children_.NewChild());
    return CallImmediately(ST(HandleCall));
}

template <typename ID>
ControlFlow::ControlFlowAction DispatchFlow<ID>::HandleWaitForChildren()
{
    if (children_.IsDone())
    {
        if (OnFlowFinished())
        {
            // terminate flow.
            return ReleaseAndExit(&allocator_, this);
        }
        else
        {
            // The termination was taken care of by OnFlowFinished.
            return WaitForNotification();
        }
    }
    else
    {
        return WaitForNotification();
    }
}

#endif // _executor_Dispatcher_hxx_
