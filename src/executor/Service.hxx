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
 * \file Service.hxx
 *
 * Class to control execution of state machines pulled out of a queue.
 *
 * @author Stuart W Baker
 * @date 20 December 2013
 */

#ifndef _Service_hxx_
#define _Service_hxx_

#include <type_traits>

#include "executor/Executor.hxx"
#include "executor/Message.hxx"

/** Get a &Service::TimerCallback equivalent for a given function.
 * @param _fn class Service derived member function
 * @return &Service::TimerCallback equivalent pointer
 */
#define TIMEOUT(_fn) (TimerCallback)(&std::remove_reference<decltype(*this)>::type::_fn)

/** Get a &Service::TimerCallback equivalent for a given function.
 * @param _service pointer to a class derived from class Service
 * @param _fn class Service derived member function
 * @return &Service::TimerCallback equivalent pointer
 */
#define TIMEOUT_FROM(_service, _fn) (TimerCallback)(&std::remove_reference<decltype(*(_service))>::type::_fn)

class StateFlowBase;

/** Collection of related state machines that pend on incoming messages.
 */
class Service
{
public:
    /** Constructor.
     * @param e Executor to run this service from.
     */
    Service(ExecutorBase *e)
        : executor_(e)
    {
    }
    
    /** Destructor. */
    ~Service()
    {
    }

    /** @returns the executor used by this service. */
    ExecutorBase *executor()
    {
        return executor_;
    }

    /** State Flow callback prototype
     */
    typedef long long (Service::*TimerCallback)(void*);

    
    /** StateFlow timer callback.
     * @param data "this" pointer to a StateFlow instance
     * @return Timer::NONE
     */
    long long state_flow_timeout(void *data);

private:
    /** Process an incoming message.
     * @param msg message to process
     * @param priority priority of message
     */
    void process(Message *msg, unsigned priority);

    /** Process the active timer.
     * @param timer timer to process
     * @return next timeout period in nanoseconds, 0 if we handled a timeout
     */
    long long process_timer(Timer *timer);

    /** Executor to use */
    ExecutorBase *executor_;
};

#endif /* _Service_hxx_ */
