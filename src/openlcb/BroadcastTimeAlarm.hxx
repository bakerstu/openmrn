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
 * Implementation of Broadcast Time Protocol alarms.
 *
 * @author Stuart W. Baker
 * @date 29 October 2018
 */

#ifndef _OPENLCB_BROADCASTTIMEALARM_HXX_
#define _OPENLCB_BROADCASTTIMEALARM_HXX_

#include "openlcb/BroadcastTime.hxx"

namespace openlcb
{

/// Basic alarm type that all other alarms are based off of.
class BroadcastTimeAlarm : public StateFlowBase, protected Atomic
{
public:
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
    BroadcastTimeAlarm(Node *node, BroadcastTime *clock,
        std::function<void()> callback)
        : StateFlowBase(node->iface())
        , clock_(clock)
        , callback_(callback)
        , timer_(this)
        , expires_(0)
        , running_(false)
        , set_(false)
        , waiting_(false)
#if defined(GTEST)
        , shutdown_(false)
#endif
    {
        // By ensuring that the alarm runs in the same thread context as the
        // clock which it uses, we can have much simpler logic for avoiding
        // race conditions in this implementation.
        HASSERT(service() == clock_->service());
        clock_->update_subscribe(std::bind(&BroadcastTimeAlarm::update_notify,
                                           this));
        start_flow(STATE(entry));
    }

    /// Destructor.
    ~BroadcastTimeAlarm()
    {
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
        bool need_wakeup = false;
        {
            AtomicHolder h(this);
            expires_ = time;
            running_ = true;
            if (!set_)
            {
                need_wakeup = true;
                set_ = true;
            }
        }
        if (need_wakeup)
        {
            new Wakeup(this);
        }
    }

    /// Inactivate the alarm.
    void clear()
    {
        {
            AtomicHolder h(this);
            running_ = false;
            set_ = false;
        }
        // rather than waking up the state flow, just let it expire naturally.
    }

#if defined(GTEST)
    void shutdown()
    {
        shutdown_ = true;
        new Wakeup(this);
    }

    bool is_shutdown()
    {
        return is_terminated();
    }
#endif

protected:
    /// Entry point to state flow.
    /// @return next state setup()
    virtual Action entry()
    {
        return call_immediately(STATE(setup));
    }

    /// Called when the clock time has changed.
    virtual void update_notify()
    {
        bool need_wakeup = false;
        {
            AtomicHolder h(this);
            if (running_ && !set_)
            {
                set_ = true;
                need_wakeup = true;
            }
        }
        if (need_wakeup)
        {
            new Wakeup(this);
        }
    }

    BroadcastTime *clock_; ///< clock that our alarm is based off of

private:
    // Wakeup helper
    class Wakeup : public Executable
    {
    public:
        /// Constructor.
        /// @param alarm our parent alarm that we will awaken
        Wakeup(BroadcastTimeAlarm *alarm)
            : alarm_(alarm)
        {
            alarm->service()->executor()->add(this);
        }
    private:
        /// Entry point. This funciton will be called when *this gets scheduled
        /// on the CPU.
        void run() override
        {
            alarm_->wakeup();
            delete this;
        }

        BroadcastTimeAlarm *alarm_; ///< our parent alarm we will wakeup
    };

    /// Setup, or wait to setup alarm.
    /// @return expired() if alarm is set and already expired,
    ///         timeout() if alarm will expire in the future,
    ///         setup() if clock and/or alarm is not currently active
    Action setup()
    {
        waiting_ = false;
#if defined(GTEST)
        if (shutdown_)
        {
            return exit();
        }
#endif

        AtomicHolder h(this);
        if (set_)
        {
            set_ = false;
            time_t now = clock_->time();

            if ((now >= expires_ && clock_->rate() > 0) ||
                (now <= expires_ && clock_->rate() < 0) ||
                (now == expires_ && clock_->rate() == 0))
            {
                // have already met the alarm conditions,
                // typically won't get here
                return call_immediately(STATE(expired));
            }
            else if (clock_->is_running())
            {
                return sleep_and_call(&timer_,
                    clock_->real_nsec_until_rate_time_abs(expires_),
                    STATE(timeout));
            }
        }

        waiting_ = true;
        return wait_and_call(STATE(setup));
    }

    /// Wait for timeout or early trigger.
    /// @return setup() if the timer is triggered prematurely, else
    ///         expired() if the timer has expired
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
            return call_immediately(STATE(expired));
        }
    }

    /// Handle action on timer expiration.
    /// @return setup()
    Action expired()
    {
        if (running_ && clock_->is_running() && callback_)
        {
            running_ = false;
            callback_();
        }

        waiting_ = true;
        return wait_and_call(STATE(setup));
    }        

    /// wakeup the state machine.
    void wakeup()
    {
        timer_.ensure_triggered();
        if (waiting_)
        {
            waiting_ = false;
            notify();
        }
    }

    /// Callback for when alarm expires.  The return value is RESTART to restart
    /// the alarm, else the value is NONE.  The time_t parameter passed by
    /// reference is the time_t value which expired.  The value passed back by
    /// the time_t parameter is the next experation time if the alarm is
    /// restarted.
    std::function<void()> callback_;

    StateFlowTimer timer_; ///< timer helper
    time_t expires_; ///< time at which the alarm expires
    uint8_t running_  : 1; ///< true if running, else false
    uint8_t set_      : 1; ///< true if a start request is pending
    uint8_t waiting_  : 1; ///< true if waiting
#if defined(GTEST)
    uint8_t shutdown_ : 1; ///< true if shutdown
#endif

    /// make our wakeup agent a friend
    friend class BroadcastTimeAlarm::Wakeup;

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeAlarm);
};

/// Specialization of BroadcastTimeAlarm meant to expire at each date rollover
class BroadcastTimeAlarmDate : public BroadcastTimeAlarm
{
public:
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
    BroadcastTimeAlarmDate(Node *node, BroadcastTime *clock,
        std::function<void()> callback)
        : BroadcastTimeAlarm(
              node, clock, std::bind(&BroadcastTimeAlarmDate::expired_callback,
                                     this))
        , callbackUser_(callback)
    {
    }

    /// Destructor.
    ~BroadcastTimeAlarmDate()
    {
    }

private:
    /// Entry point of the state machine.
    /// @return BroadcastTimeAlarm::entry();
    Action entry() override
    {
        update_notify();

        return BroadcastTimeAlarm::entry();
    }

    /// callback for when the alarm expires
    void expired_callback()
    {
        if (callbackUser_)
        {
            callbackUser_();
        }

        if (clock_->rate() > 0)
        {
            set(clock_->time() + (60 * 60 * 24));
        }
        else if (clock_->rate() < 0)
        {
            set(clock_->time() - (60 * 60 * 24));
        }
    }

    /// Called when the clock time has changed.
    void update_notify() override
    {
        const struct tm *tm = clock_->gmtime_recalculate();
        time_t seconds = clock_->time();

        if (clock_->rate() > 0)
        {
            set(seconds + (60 - tm->tm_sec) +
                          (60 * (59 - tm->tm_min)) +
                          (60 * 60 * (23 - tm->tm_hour)));
        }
        else if (clock_->rate() < 0)
        {
            set(seconds - (tm->tm_sec + 60 * (tm->tm_min + 1) + 60 * 60 * tm->tm_hour));
        }
        BroadcastTimeAlarm::update_notify();
    }

    /// Callback for when alarm expires.  The return value is RESTART to restart
    /// the alarm, else the value is NONE.  The time_t parameter passed by
    /// reference is the time_t value which expired.  The value passed back by
    /// the time_t parameter is the next experation time if the alarm is
    /// restarted.
    std::function<void()> callbackUser_;

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeAlarmDate);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMEALARM_HXX_
