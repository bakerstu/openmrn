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
#include "executor/allocator.hxx"
#include "executor/control_flow.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/NMRAnetNode.hxx"

/**
   Abstract class for handling an incoming message of a specifc MTI in NMRAnet.
   Handle calls are always made on the IF's read executor.

   There are two allocation mechanism available:

   . in one case the handler is directly able to process messages and performs
   the allocation itself. This is helpful for synchronous handlers, or when
   actual objects (instead of factories) are performing the handling of
   messages, such as when waiting for responses to outbound messages.

   . or the handler call actually returns an Allocator, which the caller has to
   acquire a handler from and re-post the same call to that handler.
 */
class HandlerBase : public QueueMember
{
public:
    /** Retrieves the allocator needed for a handler. The caller must retrieve
     * an instance from the allocator, or if the returned allocator is NULL,
     * then the caller may invoke this->handle_message immediately.
     *
     * @returns an allocator of type HandlerBase; or NULL.
     */
    virtual AllocatorBase* get_allocator()
    {
        return NULL;
    }
};

template <class Param> class ParamHandler : public HandlerBase
{
public:
    /**
     * Initiates handling of an incoming message.
     *
     * @param message is the incoming message. Does not take ownership.
     * @param done notifiable, will be notified when the message handling is
     * completed and message can be deallocated. Required.
     */
    virtual void handle_message(Param* message, Notifiable* done) = 0;
};

/**
   This class takes registered handlers for incoming messages. When a message
   shows up, all the handlers that match that message will be invoked. When all
   the handlers return, the message will be marked as completed, and the
   current flow will be released for further message calls.

   Handlers are called in no particular order.

   Allocation semantics:

   This flow is attached to a TypedAllocator owned by the If. The flow will
   return itself to the allocator automatically.
 */
template <typename ID> class DispatchFlow : public ControlFlow
{
public:
    DispatchFlow(Executor* executor);
    virtual ~DispatchFlow();
    /**
       Handles an incoming message. Prior to this call the parameters needed
       for the call should be injected into the flow using an
       implementation-specific method.

       @param id is the identifier of the incoming message.

       The flow *this will release itself to the allocator when the message
       handling is done.
     */
    void IncomingMessage(ID id);

    TypedAllocator<DispatchFlow<ID>>* allocator()
    {
        return &allocator_;
    }

protected:
    /**
       Adds a new handler to this dispatcher.

       A handler will be called if incoming_mti & mask == mti & mask.

       @param mti is the identifier of the message to listen to.
       @param mask is the mask of the mti matcher.
       @param handler is the MTI handler. It must stay alive so long as this If
       is alive.
     */
    void RegisterHandler(ID id, ID mask, HandlerBase* handler);

    //! Removes a specific instance of a handler from this IF.
    void UnregisterHandler(ID id, ID mask, HandlerBase* handler);

    /**
       Makes an implementation-specific call to the actual
       handler. Implementations will need to take certain handler arguments from
       member variables. The 'done' notifiable must be called in all cases.
     */
    virtual void CallCurrentHandler(HandlerBase* handler, Notifiable* done) = 0;

    /**
       This function will be called when the flow and all children are
       done. Useful for implementations to release memory and buffers
       associated with the current parameters.

       @returns true if the flow instance should be returned to the
       allocator. If returns false, must take care of changing the flow to a
       different state.
     */
    virtual bool OnFlowFinished() { return true; };

    size_t handler_count() {
        return handlers_.size();
    }

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
        HandlerBase* handler;

        bool Equals(ID id, ID mask, HandlerBase* handler)
        {
            return (this->id == id && this->mask == mask &&
                    this->handler == handler);
        }
    };

    vector<HandlerInfo> handlers_;

    // Index of the current iteration.
    size_t current_index_;
    // Points to a deleted entry in the vector, or -1.
    int pending_delete_index_;

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
    void RegisterHandler(ID id, ID mask, Handler* handler)
    {
        DispatchFlow<ID>::RegisterHandler(id, mask, handler);
    }

    //! Removes a specific instance of a handler.
    void UnregisterHandler(ID id, ID mask, Handler* handler)
    {
        DispatchFlow<ID>::UnregisterHandler(id, mask, handler);
    }

    /** @returns the parameters structure to fill in before calling
        IncomingMessage. The caller must be in ownership of *this from an
        allocator before using this pointer.  */
    Params* mutable_params()
    {
        return &params_;
    }

protected:
    virtual void CallCurrentHandler(HandlerBase* b_handler, Notifiable* done)
    {
        Handler* handler = static_cast<Handler*>(b_handler);
        handler->handle_message(&params_, done);
    }

    Params params_;
};

// ================== IMPLEMENTATION ==================

template <typename ID>
DispatchFlow<ID>::DispatchFlow(Executor* executor)
    : ControlFlow(executor, nullptr), pending_delete_index_(-1)
{
    allocator_.Release(this);
}

template <typename ID> DispatchFlow<ID>::~DispatchFlow()
{
    HASSERT(IsNotStarted());
}

template <typename ID>
void DispatchFlow<ID>::RegisterHandler(ID id, ID mask, HandlerBase* handler)
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
void DispatchFlow<ID>::UnregisterHandler(ID id, ID mask, HandlerBase* handler)
{
    OSMutexLock h(&lock_);
    // First we try the current index - 1.
    size_t idx = current_index_;
    if (idx > 0) idx--;
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
    HASSERT(IsNotStarted());
    HASSERT(children_.IsDone());
    current_index_ = 0;
    id_ = id;
    children_.Reset(this);
    StartFlowAt(ST(HandleCall));
}

template <typename ID>
ControlFlow::ControlFlowAction DispatchFlow<ID>::HandleCall()
{
    HandlerBase* handler;
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
            if ((id_ & h.mask) != (h.id & h.mask))
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
    HandlerBase* h;
    GetAllocationResult(&h);
    CallCurrentHandler(h, children_.NewChild());
    return CallImmediately(ST(HandleCall));
}

template <typename ID>
ControlFlow::ControlFlowAction DispatchFlow<ID>::HandleWaitForChildren()
{
    if (children_.IsDone())
    {
        if (OnFlowFinished()) {
            // terminate flow.
            return ReleaseAndExit(&allocator_, this);
        } else {
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
