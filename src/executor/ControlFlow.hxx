/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file ControlFlow.hxx
 *
 * Defines a type of state machine flow used within class Service.
 *
 * @author Stuart W Baker
 * @date 25 December 2013
 */

#ifndef _ControlFlow_hxx_
#define _ControlFlow_hxx_

#include <type_traits>

#include "executor/Allocator.hxx"
#include "executor/Message.hxx"
#include "utils/BufferQueue.hxx"

#define STATE(_fn) (Callback)(&std::remove_reference<decltype(*this)>::type::_fn)

#define CONTROL_FLOW_START(_name)    \
    class _name : public ControlFlow \
    {                                \
    public:                          \
        _name(Service *service)      \
            : ControlFlow(service)   \
        {                            \
        }                            \
                                     \
        ~_name()                     \
        {                            \
        }                            \
                                     \
    private:                         \
        Action entry(Message *msg);

#define CONTROL_FLOW_START_WITH_TIMER(_name)                      \
    class _name : public ControlFlow                              \
    {                                                             \
    public:                                                       \
        _name(Service *service)                                   \
            : ControlFlow(service),                               \
              timer(timeout, this, NULL)                          \
        {                                                         \
        }                                                         \
                                                                  \
        ~_name()                                                  \
        {                                                         \
        }                                                         \
                                                                  \
    private:                                                      \
        Action entry(Message *msg);                               \
                                                                  \
        static long long timeout(void* data1, void* data2)        \
        {                                                         \
            ((_name)*) flow = ((_name)*)data1;                    \
            me()->send(flow->timerMsg);                           \
            return OS_TIMER_NONE;                                 \
        }                                                         \
                                                                  \
        Action timeout_and_call(Callback c, Message *msg, period) \
        {                                                         \
            msg->id(msg->id() | Message::IN_PROCESS_MSK);         \
            timerMsg = msg;                                       \
            timer.start(period);                                  \
            return Action(c);                                     \
        }                                                         \
                                                                  \
        OSTimer timer;                                            \
        Message *timerMsg;

#define CONTROL_FLOW_STATE(_state) Action _state(Message *msg);

#define CONTROL_FLOW_USE_TIMEOUT()

#define CONTROL_FLOW_END() };

/** Runs incoming Messages through a State Flow.
 */
class ControlFlow : public Q<Message>
{
protected:
    /** Constructor.
     * @param service Service that this control flow is part of
     */
    ControlFlow(Service *service)
        : Q(),
          service(service),
          state(STATE(terminated))
    {
    }
    
    /** Destructor.
     */
    ~ControlFlow()
    {
    }

    /* forward prototype */
    class Action;

    /** Control Flow callback prototype
     */
    typedef Action (ControlFlow::*Callback)(Message *);

    /** Return type for a control flow callback.
     */
    class Action
    {
    public:
        /** Constructor.
         */
        Action(Callback s)
            : nextState(s)
        {
        }
        
        /** Get the next state for the ControlFlowAction.
         */
        Callback next_state()
        {
            return nextState;
        }

    private:
        /** next state in control flow */
        Callback nextState;
    };

    /** Entry into the ControlFlow activity.  Pure virtual which must be
     * defined by derived class.
     * @param msg Message belonging to the control flow
     * @return function pointer to next state
     */
    virtual Action entry(Message *msg) = 0;

    /** Call the current state again.
     * @return function pointer to current state handler
     */
    Action again()
    {
        return Action(state);
    }

    /** Terminate current ControlFlow activity.
     * @return function pointer to terminated method
     */
    Action exit()
    {
        return STATE(terminated);
    }

    /** Terminate current ControlFlow activity. after releasing the message.
     * @param msg to release
     * @return function pointer to terminated method
     */
    Action release_and_exit(Message *msg)
    {
        msg->free();
        return exit();
    }

    /** Imediately call the next state upon return.
     * @param c Callback "state" to move to
     * @return function pointer to passed in callback
     */
    Action call_immediately(Callback c)
    {
        return Action(c);
    }

    /** Wait for resource to become available before proceeding to next state.
     * @param c Callback "state" to move to
     * @param msg Message instance we are waiting on
     * @return function pointer to passed in callback
     */
    Action wait_and_call(Callback c, Message *msg)
    {
        msg->id(msg->id() | Message::IN_PROCESS_MSK);
        return Action(c);
    }

    /** Wait for resource to become available before proceeding to next state.
     * if an immediate allocation can be made, an immediate call to the next
     * state will be made.
     * @param c Callback "state" to move to
     * @param msg Message instance we are waiting on
     * @return function pointer to passed in callback
     */
    template <class T> Action allocate_and_call(Allocator<T> *a, Callback c, Message *msg)
    {
        return a->allocate_immediate(msg) ? call_immediately(c) : Action(c);
    }

    /** Imediately queue up the next callback for this flow through the executor.
     * Similar to @ref call_immediately, except we place this flow on the back
     * of the Executor queue.
     * @param c Callback "state" to move to
     * @param msg Message instance we are acting upon
     * @return function pointer to passed in callback
     */
    Action yeild_and_call(Callback c, Message *msg);

    /** Return a pointer to the service I am bound to.
     * @return pointer to service
     */
    Service *me()
    {
        return service;
    }

private:
    /** Service this ControlFlow belongs to */
    Service *service;

    /** Terminate current ControlFlow activity.  This method only exists for the
     * purpose of providing a unique address pointer.
     * @param msg unused
     */
    Action terminated(Message *msg);

    /** Process an incoming message.
     * @param msg message to process
     */
    void process(Message *msg);
    
    /** current active state in the flow */
    Callback state;

    /* Allow class Service to access our private members */
    friend class Service;
    
    /* Allow class Message to access our private and protected members */
    friend class Message;
    
    /* Allow clas Allocate to access our private and protected members */

    /** Default constructor.
     */
    ControlFlow();

    DISALLOW_COPY_AND_ASSIGN(ControlFlow);
};

#endif /* _ControlFlow_hxx_ */
