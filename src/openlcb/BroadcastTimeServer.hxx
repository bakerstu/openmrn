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
 * @file BroadcastTimeServer.hxx
 *
 * Implementation of a Broadcast Time Protocol Server.
 *
 * @author Stuart W. Baker
 * @date 4 November 2018
 */

#ifndef _OPENLCB_BROADCASTTIMESERVER_HXX_
#define _OPENLCB_BROADCASTTIMESERVER_HXX_

#include "openlcb/BroadcastTime.hxx"
#include "openlcb/BroadcastTimeAlarm.hxx"

namespace openlcb
{

/// Implementation of a Broadcast Time Protocol client.
class BroadcastTimeServer : public BroadcastTime
{
public:
    /// Constructor.
    /// @param node the virtual node that will be listening for events and
    ///             responding to Identify messages.
    /// @param clock_id 48-bit unique identifier for the clock instance
    BroadcastTimeServer(Node *node, NodeID clock_id)
        : BroadcastTime(node, clock_id)
        , nextTimeReport_(0)
        , secondsRequested_(0)
        , updateRequested_(false)
#if defined (GTEST)
        , shutdown_(false)
#endif
    {
        memset(timeSubscriptions_, 0, sizeof(timeSubscriptions_));
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, clockID_), 16);

        start_flow(STATE(initialize));
    }

    /// Destructor.
    ~BroadcastTimeServer()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    /// Set the time in seconds since the system Epoch.
    /// @param time in seconds since the system Epoch
    void set_time(time_t time)
    {
        AtomicHolder h(this);
        secondsRequested_ = time;
        if (rate_ != 0 && started_)
        {
            updateRequested_ = true;
        }
    }

    /// Set Rate.
    /// @param rate clock rate ratio as 12 bit sign extended fixed point
    ///             rrrrrrrrrr.rr
    void set_rate(int16_t rate)
    {
        HASSERT(rate >= -2048 && rate < 2048);
        AtomicHolder h(this);
        if (rate_ != rate)
        {
            rateRequested_ = rate;
            updateRequested_ = true;
        }
    }

    /// Start clock
    void start()
    {
        AtomicHolder h(this);
        if (!started_)
        {
            started_ = true;
            if (rate_ != 0)
            {
                updateRequested_ = true;
            }
        }
    }

    /// Stop clock
    void stop()
    {
        started_ = false;
        // let a running clock expire naturally.
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

private:
    // Wakeup helper
    class Wakeup : public Executable
    {
    public:
        /// Constructor.
        /// @param server our parent time server that we will awaken
        Wakeup(BroadcastTimeServer *server)
            : server_(server)
        {
            server->service()->executor()->add(this);
        }
    private:
        /// Entry point. This funciton will be called when *this gets scheduled
        /// on the CPU.
        void run() override
        {
            server_->timer_.ensure_triggered();
            delete this;
        }

        BroadcastTimeServer *server_; ///< our parent alarm we will wakeup
    };

    /// Handle requested identification message.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_identify_global(const EventRegistryEntry &entry,
                                EventReport *event,
                                BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            // not for us
            return;
        }

        event->event_write_helper<1>()->WriteAsync(
            node_, Defs::MTI_PRODUCER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(entry.event, 0x1 << 16)),
            done->new_child());

        // we can configure ourselves
        event->event_write_helper<2>()->WriteAsync(
            node_, Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(entry.event + 0x8000, 0x1 << 15)),
            done->new_child());
    }

    /// Handle requested identification message.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_identify_consumer(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        // we can configure ourselves
        event->event_write_helper<1>()->WriteAsync(
            node_, Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(entry.event + 0x8000, 0x1 << 15)),
            done->new_child());
    }

    /// Handle requested identification message.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_identify_producer(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        event->event_write_helper<1>()->WriteAsync(
            node_, Defs::MTI_PRODUCER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(entry.event, 0x1 << 16)),
            done->new_child());
    }

    /// Handle an incoming consumer identified.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_consumer_identified(const EventRegistryEntry &entry,
                                    EventReport *event,
                                    BarrierNotifiable *done) override
    {
        done->notify();
        if (get_event_type(event->event) == REPORT_TIME)
        {
            int min = event_to_min(event->event);
            int hour = event_to_hour(event->event);
            if (min != -1 && mour != -1)
            {
                timeSubscriptions_[hour][min / 32] |= 0x1 << (min % 32);
            }
        }
    }

    /// Handle an incoming event report.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_event_report(const EventRegistryEntry &entry,
                             EventReport *event,
                             BarrierNotifiable *done) override
    {
        done->notify();
    }

    /// Initialize the clock.  Starts by sending the Producer Identified message
    /// appropriate for the start/stop event ID as appropriate.
    /// @return wait_and_call(STATE(send_rate_report))
    Action initialize()
    {
        {
            AtomicHolder h(this);
            if (started_ && rate_ != 0)
            {
                if (updateRequested_)
                {
                    if (seconds_ != secondsRequested_)
                    {
                        seconds_ = secondsRequested_;
                        timestamp_ = OSTime::get_monotonic();
                    }
                    if (rate_ != rateRequested_)
                    {
                        rate_ = rateRequested_;
                    }
                    gmtime_r(&tm_);
                }
            }
        }

        uint64_t event_id = clockID_;
        event_id += started_ ? BroadcastTimeDefs::START_EVENT_SUFFIX :
                               BroadcastTimeDefs::STOP_EVENT_SUFFIX;

        writer_.WriteAsync(node_, Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_rate_report));
    }

    /// Send the Producer Identified message appropriate for the rate event ID.
    /// @return wait_and_call(STATE(send_year_report))
    Action send_rate_report()
    {
        uint64_t event_id = BroadcastTimeDefs::rate_to_event(clockID_, rate_);

        writer_.WriteAsync(node_, Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_year_report));
    }

    /// Send the Producer Identified message appropriate for the year event ID.
    /// @return wait_and_call(STATE(send_date_report))
    Action send_year_report()
    {
        int year = tm_.tm_year + 1900;
        if (year < 0)
        {
            year = 0;
        }
        if (year > 4095)
        {
            year = 4095;
        }

        uint64_t event_id = BroadcastTimeDefs::year_to_event(clockID_, year);

        writer_.WriteAsync(node_, Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_date_report));
    }

    /// Send the Producer Identified message appropriate for the date event ID.
    /// @return wait_and_call(STATE(send_time_report))
    Action send_date_report()
    {
        uint64_t event_id = BroadcastTimeDefs::date_to_event(clockID_,
                                                             tm_.tm_mon + 1,
                                                             tm_.tm_mday);

        writer_.WriteAsync(node_, Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_time_report));
    }

    /// Send the Producer Identified message appropriate for the time event ID.
    /// @return report_time_helper(true)
    Action send_time_report()
    {
        return report_time_helper(true);
    }

    /// Send the time event report.
    /// @return if the timer is interrupted,
    ///         call_immediately(STATE(initialize)),
    ///         else report_time_helper(false)
    Action send_update()
    {
        if (timer_.is_triggered())
        {
#if defined(GTEST)
            if (shutdown_)
            {
                return exit();
            }
#endif
            return call_immediately(STATE(initialize));
        }
        else
        {
            return report_time_helper(false);
        }
    }

    /// Helper to send out the report time event or producer identify message.
    /// @param identify true if this is a Producer Identified, else false
    /// @return wait_and_call(STATE(report_time_helper_done))
    Action report_time_helper(bool identify)
    {
        Defs::MTI mti;
        if (identify)
        {
            mti = Defs::MTI_PRODUCER_IDENTIFIED_VALID;
            // We are actually counting on the fact that it takes time to have
            // produced the prior identify events so that we are comfortably
            // past the minute rollower instance for the next time report.
            nextTimeReport_ = OSTime::get_monotonic();

            // If the rate_ is 0, we will just settle in to reporting our
            // current time every 4 seconds.  If the rate is non-zero, we will
            // report the next minute rollover.
            nextTimeReport_ += rate_ ?
                ((SEC_TO_NSEC(60 - tm_.tm_sec) * 4) / rate_) :
                SEC_TO_NSEC(60 * 4);
        }
        else
        {
            time_t least_next_report;
            time_t time = time();
            struct tm least_next_report_tm;

            if (std::abs(rate_) >= 360)
            {
                // We would have cycled through 24 hours befor next time report,
                // therefore, increase the time report rate to once every 2
                // real minutes.
                least_next_report = time + (rate_ * 2);
            }
            else
            {
                // normally report time once every 4 real minutes.
                least_next_report = time + (rate_ * 4);
            }
            ::gmtime_r(&time, &tm)
            ::gmtime_r(&least_next_report, &least_next_report_tm);

            mti = Defs::MTI_EVENT_REPORT;
            nextTimeReport_ += SEC_TO_NSEC(60 * 4);
        }

        /// @todo acount for date rollover event

        uint64_t event_id = BroadcastTimeDefs::time_to_event(clockID_,
                                                             tm_.tm_hour,
                                                             tm_.tm_min);
        writer_.WriteAsync(node_, mti, WriteHelper::global(),
                           eventid_to_buffer(event_id), this);

        if (updateRequested_)
        {
            // If this is an update to the time, we service callbacks twice,
            // once immediately during the sync and again when the minute.
            // rolls over.
            service_callbacks(seconds_, rate_, rate_ && started_);
            if (!identify)
            {
                updateRequested_ = false;
            }
        }

        return wait_and_call(STATE(report_time_helper_done));
    }

    /// Time event report complete, sleep until the next time event report.
    /// @return sleep_and_call(..., ..., STATE(send_update))
    Action report_time_helper_done()
    {
        return sleep_and_call(&timer_,
                              nextTimeReport_ - OSTime::get_monotonic(),
                              STATE(send_update));
    }

    long long nextTimeReport_; ///< timestamp for last time reported
    time_t secondsRequested_; ///< pending clock time in seconds
    /// one bit for each possible time report
    uint32_t timeSubscriptions_[24][2]; ///< 24 hours, 60 minutes (bits)
    uint16_t updateRequested_ : 1; ///< clock settings have change
#if defined(GTEST)
    uint16_t shutdown_ : 1;
#endif

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServer);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMESERVER_HXX_
