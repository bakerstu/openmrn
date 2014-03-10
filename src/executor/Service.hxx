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

    /** A timer that runs in the context of a service using its executor queue.
     */
    class Timer
    {
    public:
        /** Potential return values for the timer callback.
         */
        enum
        {
            NONE    =  0, /**< Do not restart the timer */
            RESTART =  1, /**< Restart the timer with existing period */
            DELETE  = -1, /**< delete the timer, use with extreme caution */
        };
        
        /** Constructor.
         * @param timeout callback method, use @ref TIMEOUT macro for the
         *        proper syntax conversion of a class Service derived method
         * @param service "this" pointer for Service this timer belongs to
         * @param data parameter passed back to callback
         */
        Timer(Service::TimerCallback callback, Service *service, void *data)
            : callback(callback),
              data(data),
              next(NULL),
              service(service),
              when(0),
              period(0),
              early_(false)
        {
        }

        /** Destructor. */
        ~Timer()
        {
        }

        /** Start a timer.
         * @param period period in nanoseconds before expiration
         */
        void start(long long period)
        {
            early_ = false;
            remove();
            when = OSTime::get_monotonic() + period;
            this->period = period;
            insert();
        }

        /** Restart a timer with the existing period.
         */
        void restart()
        {
            early_ = false;
            remove();
            when = OSTime::get_monotonic() + period;
            insert();
        }

        /** Delete a timer.
         */
        void stop()
        {
            remove();
        }

        /** This will wakeup the timer prematurely, immediately.  The timer
         * period remains unchanged in case it is restarted. If the timer is
         * already expired before this call, this does not produce an "early"
         * condition.  If the timer is not running, this method effectively
         * does nothing.
         */
        void trigger()
        {
            long long now = OSTime::get_monotonic();
            if (now > when)
            {
                if (remove())
                {
                    /* timer is running in the active list */
                    when = now;
                    insert();
                    early_ = true;
                }
            }
        }

        /** Test if the timer was triggered before it expired.
         * @return true if timer was triggered, false if timer expired normally
         */
        bool early()
        {
            return early_;
        }
        
    private:
        /** Insert a timer into the active timer list.
         */
        void insert();

        /** Remove a timer from the active timer list.
         * @return true if timer was removed from the list, false if the timer
         *         is not in the list, and therefore not removed.
         */
        bool remove();

        TimerCallback callback; /**< user callback */
        void *data; /**< user data */        
        Timer *next; /**< next timer in the list */
        Service *service; /**< parent service */
        long long when; /**< when in nanoseconds timer should expire */
        long long period; /**< period in nanoseconds for timer */
        bool early_; /**< was the timer triggered early before the full period */

        /** Allows Service ability to access timer callback */
        friend class Service;
        
        /** Allows Executor ability to access timer metadata */
        friend class ExecutorBase;
    };
    
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
