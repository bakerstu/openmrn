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
 * @file BroadcastTimeServer.cxx
 *
 * Implementation of a Broadcast Time Protocol Server.
 *
 * @author Stuart W. Baker
 * @date 17 November 2018
 */

#include "openlcb/BroadcastTimeServer.hxx"

#include "executor/CallableFlow.hxx"

namespace openlcb
{

/// Request structure used to send requests to the BroadcastTimeServerSync
/// object.
struct BroadcastTimeServerSyncInput : public CallableFlowRequestBase
{
    /// Setup "null" input.
    void reset()
    {
    }
};

/// State machine for sending the clock sync sequence.
class BroadcastTimeServerSync
    : public CallableFlow<BroadcastTimeServerSyncInput>
{
public:
    /// Constuctor.
    /// @param server reference to our parent
    BroadcastTimeServerSync(BroadcastTimeServer *server)
        : CallableFlow<BroadcastTimeServerSyncInput>(server->node()->iface())
        , server_(server)
        , writer_()
        , timer_(this)
        , syncRequired_(false)
    {
    }

    /// Destructor.
    ~BroadcastTimeServerSync()
    {
    }

    /// Request a sync.
    void request_sync()
    {
        {
            AtomicHolder h(this);
            if (syncRequired_)
            {
                // sync already pending
                return;
            }
            else
            {
                // no sync pending, make it pending
                syncRequired_ = true;
            }
        }

        // abort the current sync if sleeping
        timer_.ensure_triggered();

        invoke_subflow_and_ignore_result(this);
    }

private:
    /// Send the Producer Identified message appropriate for the start/stop
    /// event ID.
    /// @return wait_and_call(STATE(send_rate_report))
    Action entry() override
    {
        server_->gmtime_recalculate();

        uint64_t event_id = server_->clock_id();
        event_id += server_->is_started() ?
            BroadcastTimeDefs::START_EVENT_SUFFIX :
            BroadcastTimeDefs::STOP_EVENT_SUFFIX;

        syncRequired_ = false;
        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_rate_report));
    }

    /// Send the Producer Identified message appropriate for the rate event ID.
    /// @return wait_and_call(STATE(send_year_report))
    Action send_rate_report()
    {
        uint64_t event_id = BroadcastTimeDefs::rate_to_event(
            server_->clock_id(), server_->rate());

        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_year_report));
    }

    /// Send the Producer Identified message appropriate for the year event ID.
    /// @return wait_and_call(STATE(send_date_report))
    Action send_year_report()
    {
        const struct tm *tm = server_->gmtime_get();

        int year = tm->tm_year + 1900;
        if (year < 0)
        {
            year = 0;
        }
        if (year > 4095)
        {
            year = 4095;
        }

        uint64_t event_id =
            BroadcastTimeDefs::year_to_event(server_->clock_id(), year);

        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_date_report));
    }

    /// Send the Producer Identified message appropriate for the date event ID.
    /// @return wait_and_call(STATE(send_time_report))
    Action send_date_report()
    {
        const struct tm *tm = server_->gmtime_get();

        uint64_t event_id = BroadcastTimeDefs::date_to_event(
            server_->clock_id(), tm->tm_mon + 1, tm->tm_mday);

        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_time_report));
    }

    /// Send the Producer Identified message appropriate for the time event ID.
    /// @return send_time_report_done()
    Action send_time_report()
    {
        const struct tm *tm = server_->gmtime_get();

        uint64_t event_id = BroadcastTimeDefs::time_to_event(
            server_->clock_id(), tm->tm_hour, tm->tm_min);
        writer_.WriteAsync(server_->node(), Defs::MTI_PRODUCER_IDENTIFIED_VALID,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_time_report_done));            
    }

    /// If clock is running, sleep until next time report.
    /// @return send_time_report_next() if running, release_and_exit()
    Action send_time_report_done()
    {
        if (server_->is_running())
        {
            // setup to send the next time report
            time_t elapsed = server_->time() - server_->seconds_;
            elapsed += server_->rate_ > 0 ? 1 : -1;
            long long timeout = server_->timestamp_ +
                                server_->rate_sec_to_real_nsec_period(elapsed);

            return sleep_and_call(&timer_, timeout - OSTime::get_monotonic(),
                                  STATE(send_time_report_next));
        }
        return release_and_exit();
    }        

    /// Send the Event Report message appropriate for the time event ID.
    /// @return release_and_exit() if triggered early,
    ///         else wait_and_call(release_and_exit()
    Action send_time_report_next()
    {
        if (timer_.is_triggered())
        {
            // abort early because of a new sync
            return release_and_exit();
        }

        const struct tm *tm = server_->gmtime_recalculate();

        uint64_t event_id = BroadcastTimeDefs::time_to_event(
            server_->clock_id(), tm->tm_hour, tm->tm_min);
        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(release_and_exit));        
    }

    BroadcastTimeServer *server_; ///< reference to our parent
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper
    uint8_t syncRequired_ : 1; ///< flag to keep track of multiple sync requests   

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerSync);
};

/// Request structure used to send requests to the
/// BroadcastTimeServerDateRolloverFinish object.
struct BroadcastTimeServerDateRolloverFinishInput
    : public CallableFlowRequestBase
{
    /// Setup "null" input.
    void reset()
    {
    }
};

/// State machine for sending the date rollover finish (year and date events)
/// sequence.
class BroadcastTimeServerDateRolloverFinish
    : public CallableFlow<BroadcastTimeServerDateRolloverFinishInput>
{
public:
    /// Constuctor.
    /// @param server reference to our parent
    BroadcastTimeServerDateRolloverFinish(BroadcastTimeServer *server)
        : CallableFlow<BroadcastTimeServerDateRolloverFinishInput>(
              server->node()->iface())
        , server_(server)
        , writer_()
        , timer_(this)
        , abortCnt_(0)
    {
    }

    /// Destructor.
    ~BroadcastTimeServerDateRolloverFinish()
    {
    }

    /// Request a date rollover finish.
    void request_finish()
    {
        // Because these requests are very timely, we use the abort count in
        // order to flush out any previously requested date report finishes.
        //
        // Because this can only be called from the same executor as the state
        // flow, we don't need an atomic lock.

        // terminate early so that we can invalidate the active request(s)
        timer_.ensure_triggered();

        if (abortCnt_ == 255)
        {
            // We should never get here as this would mean that we have 127
            // stacked requests.  This is here only as a fail safe.
            return;
        }
        ++abortCnt_;

        invoke_subflow_and_ignore_result(this);
    }
private:
    /// Wait the obligatory 3 seconds before sending the year/date report.
    /// @return normally wait_and_call(STATE(send_year_report)), else
    ///         release_and_exit() on abort
    Action entry() override
    {
        // see if we need to flush any stale requests
        if (--abortCnt_ != 0)
        {
            return release_and_exit();
        }

        return sleep_and_call(&timer_, SEC_TO_NSEC(3), STATE(send_year_report));
    }

    /// Send the Event Report message appropriate for the year event ID.
    /// @return normally wait_and_call(STATE(send_date_report)), else
    ///         release_and_exit() on trigger (abort)
    Action send_year_report()
    {
        if (timer_.is_triggered())
        {
            // abort
            return release_and_exit();
        }

        const struct tm *tm = server_->gmtime_recalculate();

        int year = tm->tm_year + 1900;
        if (year < 0)
        {
            year = 0;
        }
        if (year > 4095)
        {
            year = 4095;
        }

        uint64_t event_id =
            BroadcastTimeDefs::year_to_event(server_->clock_id(), year);

        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_date_report));
    }

    /// Send the Event Report message appropriate for the date event ID.
    /// @return wait_and_call(release_and_exit)
    Action send_date_report()
    {
        const struct tm *tm = server_->gmtime_get();

        uint64_t event_id = BroadcastTimeDefs::date_to_event(
            server_->clock_id(), tm->tm_mon + 1, tm->tm_mday);

        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(release_and_exit));
    }

    BroadcastTimeServer *server_; ///< reference to our parent
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper
    uint8_t abortCnt_; ///< request that the current processing be aborted

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerDateRolloverFinish);
};

/// Request structure used to send requests to the
/// BroadcastTimeServerDateRollover object.
struct BroadcastTimeServerDateRolloverInput : public CallableFlowRequestBase
{
    /// Setup "null" input.
    void reset()
    {
    }
};

/// State machine for sending the clock date rollover sequence.
class BroadcastTimeServerDateRollover
    : public CallableFlow<BroadcastTimeServerDateRolloverInput>
{
public:
    /// Constuctor.
    /// @param server reference to our parent
    BroadcastTimeServerDateRollover(BroadcastTimeServer *server)
        : CallableFlow<BroadcastTimeServerDateRolloverInput>(
              server->node()->iface())
        , finish_(server)
        , server_(server)
        , writer_()
    {
    }

    /// Destructor.
    ~BroadcastTimeServerDateRollover()
    {
    }

    /// Request a date rollover.
    void request_date_rollover()
    {
        invoke_subflow_and_ignore_result(this);
    }

private:
    /// Send the date rollover event report.
    /// @return wait_and_call(STATE(send_time_report))
    Action entry() override
    {
        if (server_->rate() == 0)
        {
            // we shouldn't get here, but it means the clock is not running
            return release_and_exit();
        }

        server_->gmtime_recalculate();

        uint64_t event_id = server_->clock_id();
        event_id += BroadcastTimeDefs::DATE_ROLLOVER_EVENT_SUFFIX;

        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(send_time_report));
    }

    /// Send the time event report, and schedule the year/date report.
    /// return wait_and_call(release_and_exit)
    Action send_time_report()
    {
        const struct tm *tm = server_->gmtime_get();

        uint64_t event_id = BroadcastTimeDefs::time_to_event(
            server_->clock_id(), tm->tm_hour, tm->tm_min);
        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        // schedule the year and date event reports
        finish_.request_finish();

        return wait_and_call(STATE(release_and_exit));
    }

    BroadcastTimeServerDateRolloverFinish finish_; ///< finsh the date rollover
    BroadcastTimeServer *server_; ///< reference to our parent
    WriteHelper writer_; ///< helper for sending event messages

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerDateRollover);
};

/// Request structure used to send requests to the BroadcastTimeServerSet
/// object.
struct BroadcastTimeServerSetInput : public CallableFlowRequestBase
{
    /// Setup "null" input.
    /// @param suffix the clock set event suffix
    void reset(uint16_t suffix)
    {
        suffix_ = suffix;
    }
    
    uint16_t suffix_; ///< clock set event suffix
};

/// State machine for sending the clock set sequence.
class BroadcastTimeServerSet
    : public CallableFlow<BroadcastTimeServerSetInput>
{
public:
    /// Constuctor.
    /// @param server reference to our parent
    BroadcastTimeServerSet(BroadcastTimeServer *server)
        : CallableFlow<BroadcastTimeServerSetInput>(
              server->node()->iface())
        , server_(server)
        , writer_()
        , timer_(this)
    {
    }

    /// Destructor.
    ~BroadcastTimeServerSet()
    {
    }

    /// Request a sync.
    /// @param suffix the clock set event suffix
    void request_set(uint16_t suffix)
    {
        // waiting for a sync sequence to start, abort
        timer_.ensure_triggered();

        invoke_subflow_and_ignore_result(this, suffix);
    }

private:
    /// Set a clock attribute and send the appropriate event report.
    /// @return wait_and_call(STATE(write_done)) if the time has changed, else
    ///         release_and_exit();
    Action entry() override
    {
        bool changed = false;
        struct tm tm;
        server_->gmtime_r(&tm);

        uint16_t suffix = message()->data()->suffix_;

        switch(BroadcastTimeDefs::get_event_type(suffix))
        {
            case BroadcastTimeDefs::START:
                if (!server_->started_)
                {
                    server_->started_ = true;
                    changed = true;
                }
                break;
            case BroadcastTimeDefs::STOP:
                if (!server_->started_)
                {
                    server_->started_ = false;
                    changed = true;
                }
                break;
            case BroadcastTimeDefs::SET_TIME:
            {
                int hour = BroadcastTimeDefs::event_to_hour(suffix);
                int min = BroadcastTimeDefs::event_to_min(suffix);
                if (tm.tm_hour != hour || tm.tm_min != min)
                {
                    tm.tm_hour = hour;
                    tm.tm_min = min;
                    changed = true;
                }
                break;
            }
            case BroadcastTimeDefs::SET_DATE:
            {
                int day = BroadcastTimeDefs::event_to_day(suffix);
                int mon = BroadcastTimeDefs::event_to_month(suffix) - 1;
                if (tm.tm_mday != day || tm.tm_mon != mon)
                {
                    tm.tm_mday = day;
                    tm.tm_mon = mon;
                    changed = true;
                }
                break;
            }
            case BroadcastTimeDefs::SET_YEAR:
            {
                int year = BroadcastTimeDefs::event_to_year(suffix) - 1900;
                if (tm.tm_year != year)
                {
                    tm.tm_year = year;
                    changed = true;
                }
                break;
            }
            case BroadcastTimeDefs::SET_RATE:
            {
                    int16_t rate = BroadcastTimeDefs::event_to_rate(suffix);
                    if (server_->rate_ != rate)
                    {
                        server_->rate_ = rate;
                        changed = true;
                    }
                break;
            }
            default:
                HASSERT(0);
        }

        if (!changed)
        {
            // effectively nothing changes so there is nothing to do
            return release_and_exit();
        }

        {
            AtomicHolder h(server_);
            server_->seconds_ = mktime(&tm);
            server_->timestamp_ = OSTime::get_monotonic();
        }

        server_->service_callbacks();

        uint64_t event_id = server_->clock_id() + (suffix - 0x8000);

        writer_.WriteAsync(server_->node(), Defs::MTI_EVENT_REPORT,
            WriteHelper::global(), eventid_to_buffer(event_id), this);

        return wait_and_call(STATE(write_done));
    }

    /// The previous event write has completed.
    /// @return next state send_sync after 3 second delay
    Action write_done()
    {
        return sleep_and_call(&timer_, SEC_TO_NSEC(3), STATE(send_sync));
    }

    /// Request the sync sequence if timer has not been triggered early.
    /// @return release_and_exit()
    Action send_sync()
    {
        if (!timer_.is_triggered())
        {
            // we have not received any more set events for 3 seconds, sync
            server_->sync_->request_sync();
        }
        return release_and_exit();
    }

    BroadcastTimeServer *server_; ///< reference to our parent
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServerSet);
};

//
// BroadcastTimeServer::BroadcastTimeServer()
//
BroadcastTimeServer::BroadcastTimeServer(Node *node, NodeID clock_id)
    : BroadcastTime(node, clock_id)
    , alarmDate_(node, this,
                 std::bind(&BroadcastTimeServer::alarm_date_callback, this))
    , secondsRequested_(0)
    , updateRequested_(false)
#if defined (GTEST)
    , shutdown_(false)
#endif
    , sync_(new BroadcastTimeServerSync(this))
    , set_(new BroadcastTimeServerSet(this))
    , dateRollover_(new BroadcastTimeServerDateRollover(this))
{
    EventRegistry::instance()->register_handler(
        EventRegistryEntry(this, clockID_), 16);

    start_flow(STATE(entry));
    sync_->request_sync();
}

//
// BroadcastTimeServer::~BroadcastTimeServer()
//
BroadcastTimeServer::~BroadcastTimeServer()
{
    EventRegistry::instance()->unregister_handler(this);
    delete dateRollover_;
    delete set_;
    delete sync_;
}

//
// BroadcastTimeServer::Wakeup::run()
//
void BroadcastTimeServer::Wakeup::run()
{
    switch (command_)
    {
        case SET_TIME:
            server_->set_->request_set(
                BroadcastTimeDefs::time_to_event(0, data1_, data2_));
            break;
        case SET_DATE:
            server_->set_->request_set(
                BroadcastTimeDefs::date_to_event(0, data1_, data2_));
            break;
        case SET_YEAR:
            server_->set_->request_set(
                BroadcastTimeDefs::year_to_event(0, data1_));
            break;
        case SET_RATE:
            server_->set_->request_set(
                BroadcastTimeDefs::rate_to_event(0, data1_));
            break;
        case START:
            server_->set_->request_set(BroadcastTimeDefs::START_EVENT_SUFFIX);
            break;
        case STOP:
            server_->set_->request_set(BroadcastTimeDefs::STOP_EVENT_SUFFIX);
            break;
    }
    delete this;
}

//
// BroadcastTimeServer::alarm_date_callback()
//
void BroadcastTimeServer::alarm_date_callback()
{
    dateRollover_->request_date_rollover();
}

} // namespace openlcb
