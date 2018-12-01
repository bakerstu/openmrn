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

    /// Set the time in seconds since the system Epoch.
    /// @param hour hour (0 to 23)
    /// @param minutes minutes (0 to 59)
    void set_time(int hours, int minutes)
    {
        new SetFlow(this, SetFlow::Command::SET_TIME, hours, minutes);
    }

    /// Set the time in seconds since the system Epoch.
    /// @param month month (1 to 12)
    /// @param day day of month (1 to 31)
    void set_date(int month, int day)
    {
        new SetFlow(this, SetFlow::Command::SET_DATE, month, day);
    }

    /// Set the time in seconds since the system Epoch.
    /// @param year (0AD to 4095AD)
    void set_year(int year)
    {
        new SetFlow(this, SetFlow::Command::SET_YEAR, year);
    }

    /// Set Rate.
    /// @param rate clock rate ratio as 12 bit sign extended fixed point
    ///             rrrrrrrrrr.rr
    void set_rate(int16_t rate)
    {
        new SetFlow(this, SetFlow::Command::SET_RATE, rate);
    }

    /// Start clock
    void start()
    {
        new SetFlow(this, SetFlow::Command::START);
    }

    /// Stop clock
    void stop()
    {
        new SetFlow(this, SetFlow::Command::STOP);
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
        int rate = rate_;
        if (!rate)
        {
            rate = 4;
        }
        return ((t1 - t2) * 4) / rate;
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

            return seconds_ + (time_t)NSEC_TO_SEC_ROUNDED(elapsed);
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

    /// Convert an absolute rate time to nsec until it will occur.
    /// @param seconds absolute seconds in rate time
    /// @return period in nsec until it will occure
    long long real_nsec_until_rate_time_abs(time_t seconds)
    {
        long long result;
        {
            AtomicHolder h(this);
            result = timestamp_ +
                     rate_sec_to_real_nsec_period(seconds - seconds_);
        }
        return result - OSTime::get_monotonic();
    }

    /// Convert a period at the clock's current rate to a period in real nsec.
    /// @param seconds period in seconds at the current clock rate
    /// @return period in real nanoseconds
    long long rate_sec_to_real_nsec_period(time_t seconds)
    {
        long long abs_seconds = std::abs(seconds);
        int16_t abs_rate = std::abs(rate_);
        return ((SEC_TO_NSEC(abs_seconds) * 4) + (abs_rate >> 1)) / abs_rate;
    }

    /// Convert a period in real nanoseconds to a period at the clock's
    /// current rate in seconds.  Rounded to the nearest whole second
    /// @param nsec period in real nanoseconds
    /// @return period in seconds at the current clock rate
    time_t real_nsec_to_rate_sec_period(long long nsec)
    {
        HASSERT(rate_);
        nsec = std::abs(nsec);
        int16_t abs_rate = std::abs(rate_);
        return (NSEC_TO_SEC_ROUNDED(nsec * 4) + (abs_rate >> 1)) / abs_rate;
    }

    /// Convert a period in real seconds to a period in the clock's
    /// current rate in seconds.  Rounded to the nearest whole second
    /// @param seconds period in real seconds
    /// @return period in seconds at the current clock rate
    time_t real_sec_to_rate_sec_period(time_t seconds)
    {
        HASSERT(rate_);
        seconds = std::abs(seconds);
        int16_t abs_rate = std::abs(rate_);
        return ((seconds * 4) + (abs_rate >> 1)) / abs_rate;
    }

    /// Register a callback for when the time synchronization is updated.  The
    /// context of the caller will be from a state flow on the Node Interface
    /// executor.
    /// @param callback function callback to be called.  First parameter is the
    ///                 current time in seconds since the Epoch.  Second
    ///                 parameter is the clock rate.  Third parameter is the
    ///                 running state (true == running, false == stopped)
    void update_subscribe(std::function<void()> callback)
    {
        callbacks_.emplace_back(callback);
    }

    /// Accessor method to get the Node reference
    /// @return Node reference
    Node *node()
    {
        return node_;
    }

    /// Accessor method to get the clock ID
    /// @return clock ID
    uint64_t clock_id()
    {
        return clockID_;
    }

    /// Recalculate the struct tm reprentation of time.
    /// @return last calculated time in the form of a struct tm
    const struct tm *gmtime_recalculate()
    {
        gmtime_r(&tm_);
        return &tm_;
    }

    /// Get the last calculated reprentation of time.
    /// @return last calculated time in the form of a struct tm
    const struct tm *gmtime_get()
    {
        return &tm_;
    }

protected:
    class SetFlow : public StateFlowBase
    {
    public:
        /// Supported operations.
        enum Command
        {
            SET_TIME, ///< set time request
            SET_DATE, ///< set date request
            SET_YEAR, ///< set year reauest
            SET_RATE, ///< set rate request
            START, ///< stop request
            STOP, ///< start request
        };

        /// Constructor.
        /// @param clock the parent clock instance
        /// @param command operation to perform
        /// @param data1 first data argument
        /// @param data2 second data argument
        SetFlow(BroadcastTime *clock, Command command,
               int data1 = 0, int data2 = 0)
            : StateFlowBase(clock->node()->iface())
            , clock_(clock)
            , command_(command)
            , data1_(data1)
            , data2_(data2)
        {
            start_flow(STATE(send_event));
        }

    private:

        /// Send the necessary event.
        /// @return delete_this()
        Action send_event()
        {
            uint64_t event_id;
            switch (command_)
            {
                case SET_TIME:
                    event_id = 0x8000 +
                        BroadcastTimeDefs::time_to_event(clock_->clock_id(),
                                                         data1_, data2_);
                    break;
                case SET_DATE:
                    event_id = 0x8000 +
                        BroadcastTimeDefs::date_to_event(clock_->clock_id(),
                                                         data1_, data2_);
                    break;
                case SET_YEAR:
                    event_id = 0x8000 +
                        BroadcastTimeDefs::year_to_event(clock_->clock_id(),
                                                         data1_);
                    break;
                case SET_RATE:
                    event_id = 0x8000 +
                        BroadcastTimeDefs::rate_to_event(clock_->clock_id(),
                                                         data1_);
                    break;
                case START:
                    event_id = clock_->clock_id() +
                               BroadcastTimeDefs::START_EVENT_SUFFIX;
                    break;
                case STOP:
                     event_id = clock_->clock_id() +
                               BroadcastTimeDefs::STOP_EVENT_SUFFIX;
                    break;
                default:
                    // should never get here.
                    return delete_this();
            }

            if (!clock_->node()->is_initialized())
            {
                // since the node is not yet initialized, events get thrown on
                // the floor, we will try a shortcut instead
                clock_->set_shortcut(event_id);
                return delete_this();
            }
            else
            {
                writer_.WriteAsync(clock_->node(), Defs::MTI_EVENT_REPORT,
                    WriteHelper::global(), eventid_to_buffer(event_id), this);

                return wait_and_call(STATE(delete_this));
            }
        }

        WriteHelper writer_; ///< helper for sending event messages
        BroadcastTime *clock_; ///< the parent clock instance
        Command command_; ///< operation to perform;
        int data1_; ///< first data argument
        int data2_; ///< second data argument
    };

    /// Try the possible set event shortcut.
    /// @param event event that we would be "setting"
    virtual void set_shortcut(uint64_t event)
    {
    }

    /// Service all of the attached update subscribers
    void service_callbacks()
    {
        for (auto n : callbacks_)
        {
            n();
        }
    }

    struct tm tm_; ///< the time we are last set to as a struct tm

    Node *node_; ///< OpenLCB node to export the consumer on
    uint64_t clockID_; ///< 48-bit unique identifier for the clock instance
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper

    /// update subscribers
    std::vector<std::function<void()>> callbacks_;

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
