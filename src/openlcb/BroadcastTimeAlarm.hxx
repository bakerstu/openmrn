/** @copyright
 * Copyright (c) 2018, Stuart W. Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * @file BroadcastTimeAlarm.hxx
 *
 * Implementation of a Broadcast Time Protocol client.
 *
 * @author Stuart W. Baker
 * @date 29 October 2018
 */

#ifndef _OPENLCB_BROADCASTTIMEALARM_HXX_
#define _OPENLCB_BROADCASTTIMEALARM_HXX_

#include "openlcb/BroadcastTimeClient.hxx"

namespace openlcb
{

/// Basic alarm type that all other alarms are based off of.
class BroadcastTimeAlarm : public StateFlowBase, private Atomic
{
public:
    /// Return type for the alarm callback
    enum Result
    {
        NONE = 0, ///< do nothing
        RESTART, ///< restart alarm with a new value
    };

    /// Constructor.
    /// @param node the virtual node that our StateFlowBase service will be
    ///             derived from
    /// @param clock clock that our alarm is based off of
    /// @param callback Callback for when alarm expires.  The return value is
    ///                 RESTART to restart the alarm, else the value is NONE.
    ///                 The time_t parameter passed
    ///                 by reference is the time_t value which expired.  The
    ///                 value passed back by the time_t parameter is the next
    ///                 experation time if the alarm is restarted.
    BroadcastTimeAlarm(Node *node, BroadcastTimeClient *clock,
                       std::function<Result(time_t*)> callback)
        : StateFlowBase(node->iface())
        , clock_(clock)
        , callback_(callback)
        , timer_(this)
        , expires_(0)
        , running_(false)
        , set_(false)
        , sleeping_(false)
        , waiting_(false)
    {
        clock_->update_subscribe(std::bind(&BroadcastTimeAlarm::update_notify,
                                           this));
        start_flow(STATE(setup));
    }

    /// Start the alarm to expire at the given period from now.
    /// @time period in seconds from now to expire
    void set_period(time_t period)
    {
        set(clock_->time() + period);
    }

    /// Start the alarm to expire at the given time.
    /// @time time in seconds since epoch to expire
    void set(time_t time)
    {
        {
            AtomicHolder h(this);
            running_ = false;
            set_ = true;
            expires_ = time;
        }
        new Wakeup(this);
    }

    /// Inactivate the alarm
    void clear()
    {
        AtomicHolder h(this);
        running_ = false;
        set_ = false;
        // rather than waking up the state flow, just let it expire naturally.
    }

private:
    // Wakeup helper
    class Wakeup : public Executable
    {
    public:
        /// Constructor.
        /// @param alarm our parent alarm that we will awaken
        Wakeup(BroadcastTimeAlarm *alarm)
        {
            alarm->service()->executor()->add(this);
        }
    private:
        /// Entry point. This funciton will be called when *this gets scheduled
        /// on the CPU.
        void run() override
        {
            {
                alarm_->wakeup();
            }
            delete this;
        }

        BroadcastTimeAlarm *alarm_; ///< our parent alarm we will wakeup
    };

    /// Setup, or wait to setup alarm.
    Action setup()
    {
        AtomicHolder h(this);
        if (!set_)
        {
            waiting_ = true;
            return wait_and_call(STATE(setup));
        }
        else
        {
            running_ = true;
            set_ = false;
            std::pair<time_t, int16_t> time = clock_->time_and_rate();

            if ((time.first >= expires_ && time.second > 0) ||
                (time.first <= expires_ && time.second < 0) ||
                (time.first == expires_ && time.second == 0))
            {
                // have already met the alarm conditions
                return call_immediately(STATE(expired));
            }
            else
            {
                sleeping_ = true;
                long long t = SEC_TO_NSEC(time.first) - OSTime::get_monotonic();
                return sleep_and_call(&timer_, std::abs(t), STATE(timeout));
            }
        }
    }

    /// Wait for timeout or early trigger.
    Action timeout()
    {
        if (timer_.is_triggered())
        {
            // this is a wakeup, not a timeout
            return call_immediately(STATE(setup));
        }
        else
        {
            // timeout
            sleeping_ = false;
            return call_immediately(STATE(expired));
        }
    }

    /// Handle action on timer expiration.
    Action expired()
    {
        bool notify = false;
        time_t alarm_value;
        {
            AtomicHolder h(this);
            if (running_)
            {
                alarm_value = expires_;
                notify = true;
            }
        }
        if (notify)
        {
            Result result = callback_(&alarm_value);
            if (result == RESTART)
            {
                set(alarm_value);
            }
            else
            {
                running_ = false;
            }
        }
        
        return call_immediately(STATE(setup));
    }

    /// Called when the clock time has changed.
    void update_notify()
    {
        AtomicHolder h(this);
        if (running_)
        {
            new Wakeup(this);
        }
    }

    /// wakeup the state machine.
    void wakeup()
    {
        if (waiting_)
        {
            notify();
            waiting_ = false;
        }
        if (sleeping_)
        {
            timer_.trigger();
            sleeping_ = false;
        }
    }
    BroadcastTimeClient *clock_; ///< clock that our alarm is based off of

    /// Callback for when alarm expires.  The return value is RESTART to restart
    /// the alarm, else the value is NONE.  The time_t parameter passed by
    /// reference is the time_t value which expired.  The value passed back by
    /// the time_t parameter is the next experation time if the alarm is
    /// restarted.
    std::function<Result(time_t *)> callback_;

    StateFlowTimer timer_; ///< timer helper
    time_t expires_; ///< time at which the alarm expires
    unsigned running_  : 1; ///< true if running, else false
    unsigned set_      : 1; ///< true if a start request is pending
    unsigned sleeping_ : 1; ///< true if sleeping
    unsigned waiting_  : 1; ///< true if waiting

    /// make our wakeup agent a friend
    friend class BroadcastTimeAlarm::Wakeup;

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeAlarm);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMEALARM_HXX_
