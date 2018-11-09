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
 * @file BroadcastTime.hxx
 *
 * Implementation of a Broadcast Time Protocol Interface.
 *
 * @author Stuart W. Baker
 * @date 4 November 2018
 */

#ifndef _OPENLCB_BROADCASTTIME_HXX_
#define _OPENLCB_BROADCASTTIME_HXX_

#include <functional>

#include "openlcb/BroadcastTimeDefs.hxx"
#include "openlcb/EventHandlerTemplates.hxx"

namespace openlcb
{

/// Implementation of a Broadcast Time Protocol Interface.
class BroadcastTime : public SimpleEventHandler
                    , public StateFlowBase
                    , protected Atomic
{
public:
    /// Constructor.
    /// @param node the virtual node that will be listening for events and
    ///             responding to Identify messages.
    /// @param clock_id 48-bit unique identifier for the clock instance
    BroadcastTime(Node *node, NodeID clock_id)
        : StateFlowBase(node->iface())
        , node_(node)
        , clockID_((uint64_t)clock_id << 16)
        , writer_()
        , timer_(this)
        , callbacks_()
        , timestamp_(OSTime::get_monotonic())
        , seconds_(0)
        , rate_(0)
        , rateRequested_(0)
        , started_(false)
    {
        // use a process-local timezone
        clear_timezone();

        time_t time = 0;
        ::gmtime_r(&time, &tm_);
        tm_.tm_isdst = 0;
    }

    virtual ~BroadcastTime()
    {
    }

    /// Get the time as a value of seconds relative to the system epoch.  At the
    /// same time get an atomic matching pair of the rate
    /// @return pair<time in seconds relative to the system epoch, rate>
    std::pair<time_t, int16_t> time_and_rate()
    {
        AtomicHolder h(this);
        return std::make_pair(time(), rate_);
    }

    /// Get the difference in time scaled to real time.
    /// @param t1 time 1 to compare
    /// @param t2 time 2 to compare
    /// @return (t1 - t2) scaled to real time.
    time_t compare_realtime(time_t t1, time_t t2)
    {
        return ((t1 - t2) * 4) / rate_;
    }

    /// Get the time as a value of seconds relative to the system epoch.
    /// @return time in seconds relative to the system epoch
    time_t time()
    {
        AtomicHolder h(this);
        if (started_)
        {
            long long elapsed = OSTime::get_monotonic() - timestamp_;
            elapsed = ((elapsed * rate_) + 2) / 4;

            return seconds_ + (time_t)NSEC_TO_SEC(elapsed);
        }
        else
        {
            // clock is stopped, time is not progressing
            return seconds_;
        }
    }

    /// Get the time as a standard struct tm.
    /// @param result a pointer to the structure that will hold the result
    /// @return pointer to the passed in result on success, nullptr on failure
    struct tm *gmtime_r(struct tm *result)
    {
        time_t now = time();
        return ::gmtime_r(&now, result);
    }

    /// Get the day of the week.
    /// @returns day of the week (0 - 6) upon success, else -1 on failure
    int day_of_week()
    {
        struct tm tm;
        if (gmtime_r(&tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_wday;
    }

    /// Get the day of the year.
    /// @returns day of the year (0 - 365) upon success, else -1 on failure
    int day_of_year()
    {
        struct tm tm;
        if (gmtime_r(&tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_yday;
    }

    /// Report the clock rate as a 12-bit fixed point number
    /// (-512.00 to 511.75).
    /// @return clock rate 
    int16_t rate()
    {
        return rate_;
    }

    /// Test of the clock is running.
    /// @return true if running, else false
    bool is_running()
    {
        AtomicHolder h(this);
        return rate_ != 0 && started_;
    }

    /// Test of the clock is started (rate could still be 0).
    /// @return true if started, else false
    bool is_started()
    {
        return started_;
    }

    /// Register a callback for when the time synchronization is updated.  The
    /// context of the caller will be from a state flow on the Node Interface
    /// executor.
    /// @param callback function callback to be called.  First parameter is the
    ///                 current time in seconds since the Epoch.  Second
    ///                 parameter is the clock rate.  Third parameter is the
    ///                 running state (true == running, false == stopped)
    void update_subscribe(std::function<void(time_t, int16_t, bool)> callback)
    {
        callbacks_.emplace_back(callback);
    }

protected:
    /// Service all of the attached update subscribers
    void service_callbacks(time_t seconds, int16_t rate, bool running)
    {
        for (auto n : callbacks_)
        {
            n(seconds, rate, running);
        }
    }

    struct tm tm_; ///< the time we are last set to as a struct tm

    Node *node_; ///< OpenLCB node to export the consumer on
    uint64_t clockID_; ///< 48-bit unique identifier for the clock instance
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper

    /// update subscribers
    std::vector<std::function<void(time_t, int16_t, bool)>> callbacks_;

    long long timestamp_; ///< monotonic timestamp from last server update
    time_t seconds_; ///< clock time in seconds from last server update
    int16_t rate_; ///< effective clock rate
    int16_t rateRequested_; ///< pending clock rate

    uint16_t started_ : 1; ///< true if clock is started

private:
    /// Reset our process local timezone environment to GMT0.
    void clear_timezone();

    DISALLOW_COPY_AND_ASSIGN(BroadcastTime);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIME_HXX_
