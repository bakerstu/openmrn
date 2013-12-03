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
 * \file ReadDispatch.hxx
 *
 * Class for dispatching incoming messages to handlers.
 *
 * @author Balazs Racz
 * @date 2 Dec 2013
 */

#ifndef _NMRAnetReadDispatch_hxx_
#define _NMRAnetReadDispatch_hxx_

namespace NMRAnet {

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
class HandlerBase {};

template <class Param>
class ParamHandler : public HandlerBase {
 public:
  /**
     Initiates handling of an incoming message.

     @param message is the incoming message.
     @param done notifiable, see below.

     @returns NULL if the message handling has begun. In this case the done
     notifiable will be invoked when the buffer can be released.

     @returns an allocator if the handler for this message is unavailable. In
     this case `done' will NOT be invoked, the caller has to acquire an
     ParamHandler from that allocator, and re-send the handle request to it.
   */
  virtual TypedAllocator<ParamHandler<Param> >* HandleMessage(
      Param* message, Notifiable* done) = 0;
};

struct MtiParams {
  If::MTI mti;           //< MTI of the incoming message.
  NodeHandle dst;        //< destination node, or {0,0} for global.
  Node* dst;             //< destination node pointer, or NULL for global.
  const Buffer* buffer;  //< message payload.
};

typedef ParamHandler<MtiParams> MtiHandler;

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
template <typename ID>
class DispatchFlow : public ControlFlow, public Lockable {
 public:
  /**
     Handles an incoming message. Prior to this call the parameters needed for
     the call should be injected into the flow using an implementation-specific
     method.

     @param id is the identifier of the incoming message.

     The flow *this will release itself to the allocator when the message
     handling is done.
   */
  void IncomingMessage(ID id);

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
  virtual AllocatorBase* CallCurrentHandler(HandlerBase* handler,
                                            Notifiable* done) = 0;

 private:
  // State handler. Calls the current handler.
  ControlFlowAction HandleCall();
  // State handler. If calling the handler didn't work, this state will be
  // called after the allocation.
  ControlFlowAction HandleAllocateResult();

  struct HandlerInfo {
    HandlerInfo()
        : handler(nullptr) {}
    ID id;
    ID mask;
    // NULL if the handler has been removed.
    HandlerBase* handler;
  };

  vector<HandlerInfo> handlers_;

  // Index of the current iteration.
  int current_index_;
  // Points to a deleted entry in the vector, or -1.
  int pending_delete_index_;

  // These fields contain the message currently in progress.
  ID id_;

  // This notifiable tracks all the pending handlers.
  BarrierNotifiable children_;

  OSMutex lock_;
};

template <typename ID, class Params>
class TypedDispatchFlow : public DispatchFlow<ID> {
 public:
  typedef ParamHandler<Params> Handler;

  /**
     Adds a new handler to this dispatcher.

     A handler will be called if incoming_id & mask == id & mask.

     @param id is the identifier of the message to listen to.
     @param mask is the mask of the ID matcher.
     @param handler is the handler. It must stay alive so long as this Dispatcher
     is alive.
   */
  void RegisterHandler(ID id, ID mask, Handler* handler) {
    DispatchFlow<ID>::RegisterHandler(id, mask, handler);
  }

  //! Removes a specific instance of a handler.
  void UnregisterHandler(ID id, ID mask, Handler* handler) {
    DispatchFlow<ID>::UnregisterHandler(id, mask, handler);
  }

  //! @returns the parameters structure to fill in before calling IncomingMessage.
  Params* mutable_params() {
    return &params_;
  }

 protected:
  virtual AllocatorBase* CallCurrentHandler(HandlerBase* b_handler,
                                            Notifiable* done) {
    Handler* handler = static_cast<Handler>(b_handler);
    TypedAllocator<Handler>* a = handler->HandleMessage(params_, done);
    if (a) {
      done->Notify();
    }
    return a;
  }

 private:
  Params params_;
};

}  // namespace NMRAnet

#endif _NMRAnetReadDispatch_hxx_
