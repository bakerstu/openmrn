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

class BroadcastTimeServerSync;
class BroadcastTimeServerSet;
class BroadcastTimeServerDateRollover;

/// Implementation of a Broadcast Time Protocol client.
class BroadcastTimeServer : public BroadcastTime
{
public:
    /// Constructor.
    /// @param node the virtual node that will be listening for events and
    ///             responding to Identify messages.
    /// @param clock_id 48-bit unique identifier for the clock instance
    BroadcastTimeServer(Node *node, NodeID clock_id);

    /// Destructor.
    ~BroadcastTimeServer();

    /// Set the time in seconds since the system Epoch.
    /// @param hour hour (0 to 23)
    /// @param minutes minutes (0 to 59)
    void set_time(int hours, int minutes)
    {
        new Wakeup(this, Wakeup::Command::SET_TIME, hours, minutes);
    }

    /// Set the time in seconds since the system Epoch.
    /// @param month month (1 to 12)
    /// @param day day of month (1 to 31)
    void set_date(int month, int day)
    {
        new Wakeup(this, Wakeup::Command::SET_DATE, month, day);
    }

    /// Set the time in seconds since the system Epoch.
    /// @param year (0AD to 4095AD)
    void set_year(int year)
    {
        new Wakeup(this, Wakeup::Command::SET_YEAR, year);
    }

    /// Set Rate.
    /// @param rate clock rate ratio as 12 bit sign extended fixed point
    ///             rrrrrrrrrr.rr
    void set_rate(int16_t rate)
    {
        new Wakeup(this, Wakeup::Command::SET_RATE, rate);
    }

    /// Start clock
    void start()
    {
        new Wakeup(this, Wakeup::Command::START);
    }

    /// Stop clock
    void stop()
    {
        new Wakeup(this, Wakeup::Command::STOP);
    }

#if defined(GTEST)
    void shutdown()
    {
        shutdown_ = true;
        alarmDate_.clear();
        //new Wakeup(this);
    }

    bool is_shutdown()
    {
        return true;//is_terminated();
    }
#endif

private:
    // Wakeup helper
    class Wakeup : public Executable
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
        /// @param server our parent time server that we will awaken
        /// @param command operation to perform
        /// @param data1 first data argument
        /// @param data2 second data argument
        Wakeup(BroadcastTimeServer *server, Command command, 
               int data1 = 0, int data2 = 0)
            : server_(server)
            , command_(command)
            , data1_(data1)
            , data2_(data2)
        {
            server->service()->executor()->add(this);
        }

    private:
        /// Entry point. This funciton will be called when *this gets scheduled
        /// on the CPU.
        void run() override;

        BroadcastTimeServer *server_; ///< our parent alarm we will wakeup
        Command command_; ///< operation to perform;
        int data1_; ///< first data argument
        int data2_; ///< second data argument
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

        if (BroadcastTimeDefs::get_event_type(event->event) ==
            BroadcastTimeDefs::REPORT_TIME)
        {
            int min = BroadcastTimeDefs::event_to_min(event->event);
            int hour = BroadcastTimeDefs::event_to_hour(event->event);
            if (min != -1 && hour != -1)
            {
                //timeSubscriptions_[hour] |= 0x1 << min;
            }
        }
    }

    /// Handle an incoming event report.
    /// @param entry registry entry for the event range
    /// @param event information about the incoming message
    /// @param done used to notify we are finished
    void handle_event_report(const EventRegistryEntry &entry,
                             EventReport *event,
                             BarrierNotifiable *done) override;

    /// Entry to state machine.
    /// @return query_response after timeout
    Action entry()
    {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(300),
                              STATE(query_response));
    }

    /// Respond to a query by scheduling a sync.
    /// @return exit()
    Action query_response();

    /// The date has rolled over into a new day.
    void alarm_date_callback();

    BroadcastTimeAlarmDate alarmDate_; ///< date rollover alarm
    time_t secondsRequested_; ///< pending clock time in seconds
    uint16_t updateRequested_ : 1; ///< clock settings have change
#if defined(GTEST)
    uint16_t shutdown_ : 1;
#endif

    BroadcastTimeServerSync *sync_;
    BroadcastTimeServerSet *set_;
    BroadcastTimeServerDateRollover *dateRollover_;

    friend class BroadcastTimeServerSync;
    friend class BroadcastTimeServerSet;


    DISALLOW_COPY_AND_ASSIGN(BroadcastTimeServer);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIMESERVER_HXX_
