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
 * @file BroadcastTimeClient.hxx
 *
 * Implementation of a Broadcast Time Protocol client.
 *
 * @author Stuart W. Baker
 * @date 14 October 2018
 */

#ifndef _OPENLCB_BROADCASTTIMECLIENT_HXX_
#define _OPENLCB_BROADCASTTIMECLIENT_HXX_

#include "openlcb/BroadcastTimeDefs.hxx"
#include "openlcb/EventHandlerTemplates.hxx"

namespace openlcb
{

/// Implementation of a Broadcast Time Protocol client.
class BroadcastTimeClient : public SimpleEventHandler
                          , public StateFlowBase
                          , private Atomic
{
public:
    /// Constructor.
    /// @param node the virtual node that will be listening for events and
    ///             responding to Identify messages.
    /// @param clock_id 48-bit unique identifier for the clock instance
    /// @param configure_agent can configure clock servers.
    BroadcastTimeClient(Node *node, NodeID clock_id,
                        bool configure_agent = false)
        : StateFlowBase(node->iface())
        , node_(node)
        , clockID_((uint64_t)clock_id << 16)
        , writer_()
        , timer_(this)
        , configureAgent_(configure_agent)
        , started_(false)
        , sleeping_(false)
        , rolloverPending_(false)
        , timestamp_(OSTime::get_monotonic())
        , seconds_(0)
        , rate_(0)
    {
        time_t time = 0;
        gmtime_r(&time, &tm_);

        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, clockID_), 16);

        start_flow(STATE(initialize));
    }

    /// Destructor.
    ~BroadcastTimeClient()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    /// Handle requested identification message.
    /// @param entry registry entry for the event range
    /// @param event unused
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
        writer_.WriteAsync(
            node_, Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(entry.event, 0x1 << 16)),
            done->new_child());
        if (configureAgent_)
        {
            // we can configure our complementary time server
            writer_.WriteAsync(
                node_, Defs::MTI_PRODUCER_IDENTIFIED_RANGE,
                WriteHelper::global(),
                eventid_to_buffer(EncodeRange(entry.event + 0x8000, 0x1 << 15)),
                done->new_child());
        }
        else
        {
            // we cannot configure our complementary time server
            writer_.WriteAsync(
                node_, Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN,
                WriteHelper::global(),
                eventid_to_buffer(entry.event +
                                  BroadcastTimeDefs::QUERY_EVENT_SUFFIX),
                done->new_child());
        }
    }

    /// Handle requested identification message.
    /// @param entry registry entry for the event range
    /// @param event unused
    /// @param done used to notify we are finished
    void handle_identify_producer(const EventRegistryEntry &entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (!configureAgent_ &&
            event->event == (entry.event + BroadcastTimeDefs::QUERY_EVENT_SUFFIX))
        {
            // we cannot configure our complementary time server, therefore
            // we only produce one event.
            writer_.WriteAsync(
                node_, Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN,
                WriteHelper::global(),
                eventid_to_buffer(entry.event +
                                  BroadcastTimeDefs::QUERY_EVENT_SUFFIX),
                done->new_child());
        }
    }

    /// Handle an incoming producer identified.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_producer_identified(const EventRegistryEntry &entry,
                                    EventReport *event,
                                    BarrierNotifiable *done) override
    {
        done->notify();

        if (event->state == EventState::VALID)
        {
            // We only care about valid event state.
            // Producer identified only happens when a clock synchronization
            // is taking place.  This voids previous date rollover events.
            rolloverPending_ = false;
            handle_updates(event, false);
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

        handle_updates(event, true);
    }

    /// Get the time as a value of seconds relative to the system epoch.
    /// @return time in seconds relative to the system epoch
    time_t time()
    {
        AtomicHolder h(this);
        if (started_)
        {
            long long now = OSTime::get_monotonic();
            long long elapsed = now - timestamp_;
            elapsed = ((elapsed * rate_) + 2) / 4;

            return seconds_ + (time_t)NSEC_TO_SEC(elapsed + MSEC_TO_NSEC(500));
        }
        else
        {
            // clock is stopped, time is not progressing
            return seconds_;
        }
    }

    /// Get the day of the week.
    /// @returns day of the week (0 - 6) upon success, else -1 on failure
    int day_of_week()
    {
        errno = 0;
        struct tm tm;
        time_t now = time();
        if (::gmtime_r(&now, &tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_wday;
    }

    /// Get the day of the year.
    /// @returns day of the year (0 - 365) upon success, else -1 on failure
    int day_of_year()
    {
        errno = 0;
        struct tm tm;
        time_t now = time();
        if (::gmtime_r(&now, &tm) == nullptr)
        {
            return -1;
        }
        return tm.tm_yday;
    }

private:
    /// Handle an incoming time update.
    /// @param entry registry entry for the event range
    /// @report true of this an event report, false if a Producer Identified
    void handle_updates(EventReport *event, bool report);

    /// Perform a bit of logic that is required whenever the clock's running
    /// state is changed.
    /// @param started true if the clock is started, else false
    void start_stop_logic(bool started)
    {
        // this bit of logic ensures that 
        AtomicHolder h(this);
        if (!started)
        {
            seconds_ = time();
            ::gmtime_r(&seconds_, &tm_);
        }
        timestamp_ = OSTime::get_monotonic();
        started_ = started;
    }

    /// Initialize client by sending a time query.
    /// @return next state is initialize_done
    Action initialize()
    {
        writer_.WriteAsync(node_, Defs::MTI_EVENT_REPORT, WriteHelper::global(),
            eventid_to_buffer(clockID_ + BroadcastTimeDefs::QUERY_EVENT_SUFFIX),
            this);
        return wait_and_call(STATE(initialize_done));
    }

    /// Initialize finished
    /// @return next state client_update.
    Action initialize_done()
    {
        return wait_and_call(STATE(client_update));
    }

    /// Notification arived that we should update our state.
    /// @return next state client_update_commit.
    Action client_update()
    {
        if (immediateUpdate_)
        {
            return call_immediately(STATE(client_update_commit));
        }
        else
        {
            /// because the set time sometimes comes in bursts, we will wait
            /// a little while for them to finish coming in.
            sleeping_ = true;
            return sleep_and_call(&timer_, MSEC_TO_NSEC(200),
                                  STATE(client_update_commit));
        }
    }

    /// commit the client update.
    /// @return next state client_update
    Action client_update_commit()
    {
        // we are about to commit an updated time, reset all the flags
        immediateUpdate_ = false;
        sleeping_ = false;
        rolloverPending_ = false;

        /// @todo As of 14 October 2018, newlib for armgcc uses 32-bit time_t.
        /// It seems that 64-bit time_t may become the default soon.

        time_t last_seconds = time();
        {
            AtomicHolder h(this);
            seconds_ = ::mktime(&tm_);
            timestamp_ = OSTime::get_monotonic();
        }

        if ((last_seconds - 1) > seconds_ || (last_seconds + 1) < seconds_)
        {
            /// @todo a jump in time has occured, invalidated state information
        }

        return wait_and_call(STATE(client_update));
    }

    Node *node_; ///< OpenLCB node to export the consumer on
    uint64_t clockID_; ///< 48-bit unique identifier for the clock instance
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper
    unsigned configureAgent_  : 1; ///< instance can be used to configure clock
    unsigned started_         : 1; ///< true if clock is started
    unsigned immediateUpdate_ : 1; ///< true if the update should be immediate
    unsigned sleeping_        : 1; ///< true if stateflow is waiting on timer
    unsigned rolloverPending_ : 1; ///< a day rollover is about to occur

    struct tm tm_;
    long long timestamp_; ///< monotonic timestamp from last server update
    time_t seconds_; ///< seconds clock time in seconds
    int16_t rate_; ///< clock rate, negative if invalid or unknown

    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeClient);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMECLIENT_HXX_

