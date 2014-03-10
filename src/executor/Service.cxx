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
 * \file Service.cxx
 *
 * Class to control execution of state machines pulled out of a queue.
 *
 * @author Stuart W Baker
 * @date 20 December 2013
 */

#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"

/** main message pool instance */
DynamicPool *mainMessagePool = new DynamicPool(Bucket::init(4, 8, 16, 32, 0));

/** Process the active timer.
 * @param timer timer to process
 * @return next timeout period in nanoseconds, 0 if we handled a timeout
 */
long long Service::process_timer(Timer *timer)
{
    long long now = OSTime::get_monotonic();
    if ( timer->when <= now)
    {
        /* remove timer from the list */
        executor_->active = timer->next;
        
        long long next_period = (this->*timer->callback)(timer->data);
        switch (next_period)
        {
            case Timer::NONE:
                break;
            default:
                timer->period = next_period;
                /* fall through */
            case Timer::RESTART:
                timer->when += timer->period;
                timer->insert();
                break;
            case Timer::DELETE:
                delete timer;
                break;
        }
        return 0;
    }

    return timer->when - now;
}

/** Insert a timer into the active timer list.
 */
void Service::Timer::insert()
{
    Timer *tp = static_cast<Timer*>(service->executor_->active);
    Timer *last = NULL;
    while (tp)
    {
        if (when <= tp->when)
        {
            break;
        }
        last = tp;
        tp = tp->next;
    }
    if (last)
    {
        next = last->next;
        last->next = this;
    }
    else
    {
        next = static_cast<Timer*>(service->executor_->active);
        service->executor_->active = this;
    }
}

/** Remove a timer from the active timer list.
 * @return true if timer was removed from the list, false if the timer
 *         is not in the list, and therefore not removed.
 */
bool Service::Timer::remove()
{
    /* Search the active list for this timer */
    Timer *tp = static_cast<Timer*>(service->executor_->active);
    Timer *last = NULL;
    while (tp)
    {
        if (tp == this)
        {
            /* Found the timer */
            break;
        }

        last = tp;
        tp = tp->next;
    }

    if (tp)
    {
        /* Remove the timer from the active list */
        if (last) {
            last->next = tp->next;
        } else {
            service->executor_->active = tp->next;
        }
        return true;
    }
    return false;
}

/** ControlFlow timer callback.
 * @param data "this" pointer to a ControlFlow instance
 * @return Timer::NONE
long long Service::state_flow_timeout(void *data)
{
    StateFlowBase *sf = static_cast<StateFlowBase*>(data);
    sf->timeout();
    return Timer::NONE;
}
 */

