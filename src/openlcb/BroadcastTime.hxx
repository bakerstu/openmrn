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
 * Implementation of Broadcast Time Protocol.
 *
 * @author Stuart W. Baker
 * @date 4 November 2018
 */

#ifndef _OPENLCB_BROADCASTTIME_HXX_
#define _OPENLCB_BROADCASTTIME_HXX_

#include <functional>

#include "openlcb/BroadcastTimeDefs.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "utils/TimeBase.hxx"

namespace openlcb
{

/// Implementation of Broadcast Time Protocol.
class BroadcastTime : public SimpleEventHandler
                    , public StateFlowBase
                    , public TimeBase
{
public:
    /// An opaque data element that is returned from update subscriber
    /// registration, and allows unregistering a subscriber.
    typedef size_t UpdateSubscribeHandle;

    /// Callback type used for time update subscribers.
    ///
    /// @param old Fast clock's current time according to the pre-update state.
    /// @param current Fast clock's current time according to the post-update
    /// state.
    typedef std::function<void(time_t old, time_t current)> TimeUpdateCallback;

    /// Destructor.
    virtual ~BroadcastTime()
    {
    }

    /// Set the time in seconds since the system Epoch. The new time does not
    /// become valid until the update callbacks are called.
    /// @param hour hour (0 to 23)
    /// @param minutes minutes (0 to 59)
    void set_time(int hours, int minutes)
    {
        EventId event_id = BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK |
            BroadcastTimeDefs::time_to_event(eventBase_, hours, minutes);
        send_event(node_, event_id);
    }

    /// Set the time in seconds since the system Epoch. The new date does not
    /// become valid until the update callbacks are called.
    /// @param month month (1 to 12)
    /// @param day day of month (1 to 31)
    void set_date(int month, int day)
    {
        EventId event_id = BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK |
            BroadcastTimeDefs::date_to_event(eventBase_, month, day);
        send_event(node_, event_id);
    }

    /// Set the time in seconds since the system Epoch. The new year does not
    /// become valid until the update callbacks are called.
    /// @param year (0AD to 4095AD)
    void set_year(int year)
    {
        EventId event_id = BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK |
            BroadcastTimeDefs::year_to_event(eventBase_, year);
        send_event(node_, event_id);
    }

    /// Set the date and year from a C string.
    /// @param date_year date and year format in "Mmm dd, yyyy" format
    void set_date_year_str(const char *date_year);

    /// Set Rate. The new rate does not become valid until the update callbacks
    /// are called.
    /// @param rate clock rate ratio as 12 bit sign extended fixed point
    ///             rrrrrrrrrr.rr
    void set_rate_quarters(int16_t rate)
    {
        EventId event_id = BroadcastTimeDefs::EVENT_SET_SUFFIX_MASK |
            BroadcastTimeDefs::rate_to_event(eventBase_, rate);
        send_event(node_, event_id);
    }

    /// Start clock
    void start()
    {
        send_event(node_, eventBase_ | BroadcastTimeDefs::START_EVENT_SUFFIX);
    }

    /// Stop clock
    void stop()
    {
        send_event(node_, eventBase_ | BroadcastTimeDefs::STOP_EVENT_SUFFIX);
    }

    /// Query the current time.
    void query()
    {
        send_event(node_, eventBase_ | BroadcastTimeDefs::QUERY_EVENT_SUFFIX);
    }

    /// Register a callback for when the time synchronization is updated. The
    /// context of the caller will be from a state flow on the Node Interface
    /// executor.
    /// @param callback function callback to be called.
    /// @return handle to entry that can be used in update_unsubscribe
    UpdateSubscribeHandle update_subscribe_add(TimeUpdateCallback callback)
    {
        AtomicHolder h(this);
        for (size_t i = 0; i < callbacks_.size(); ++i)
        {
            // atempt to garbage collect unused entries
            if (callbacks_[i] == nullptr)
            {
                callbacks_[i] = std::move(callback);
                return i;
            }
        }
        callbacks_.emplace_back(std::move(callback));
        return callbacks_.size() - 1;
    }

    /// Unregister a callback for when the time synchronization is updated.
    /// @param handle returned from corresponding update_subscribe
    void update_subscribe_remove(UpdateSubscribeHandle handle)
    {
        AtomicHolder h(this);
        callbacks_[handle] = nullptr;
    }

    /// Accessor method to get the Node reference
    /// @return Node reference
    Node *node()
    {
        return node_;
    }

    /// Accessor method to get the (48-bit) Clock ID.
    /// @return clock ID
    NodeID clock_id()
    {
        return eventBase_ >> 16;
    }

    /// Access method to get the (64-bit) Event ID base.
    EventId event_base()
    {
        return eventBase_;
    }

    /// Recalculate the struct tm representation of time.
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

    /// Has a time server been detected?
    /// @return true if a time server has been detected, else false
    virtual bool is_server_detected() = 0;

    /// Test if this is a server.
    /// @return true if a BroadcastTimeServer, else false
    virtual bool is_server_self() = 0;

protected:
    /// Constructor.
    /// @param node the virtual node that will be listening for events and
    ///             responding to Identify messages.
    /// @param clock_id 48-bit unique identifier for the clock instance
    BroadcastTime(Node *node, NodeID clock_id)
        : StateFlowBase(node->iface())
        , node_(node)
        , eventBase_((uint64_t)clock_id << 16)
        , writer_()
        , timer_(this)
        , callbacks_()
        , rateRequested_(0)
    {
        // use a process-local timezone
        clear_timezone();

        time_t time = 0;
        ::gmtime_r(&time, &tm_);
        tm_.tm_isdst = 0;
    }

    /// Try the possible set event shortcut. This is typically a bypass of the
    /// OpenLCB loopback.
    /// @param event event that we would be "setting"
    virtual void set_shortcut(uint64_t event)
    {
    }

    /// Service all of the attached update subscribers. These are called when
    /// there are jumps in time or if the clock is stopped or started.
    /// @param old Fast clock's current time according to the pre-update state.
    /// @param current Fast clock's current time according to the post-update
    /// state.
    void service_callbacks(time_t old, time_t current)
    {
        AtomicHolder h(this);
        for (auto n : callbacks_)
        {
            if (n)
            {
                n(old, current);
            }
        }
    }

    struct tm tm_; ///< the time we are last set to as a struct tm

    Node *node_; ///< OpenLCB node to export the consumer on
    EventId eventBase_; ///< 48-bit unique identifier for the clock instance
    WriteHelper writer_; ///< helper for sending event messages
    StateFlowTimer timer_; ///< timer helper

    /// update subscribers
    std::vector<TimeUpdateCallback> callbacks_;

    int16_t rateRequested_; ///< pending clock rate

private:
    /// Reset our process local timezone environment to GMT0.
    void clear_timezone();

    DISALLOW_COPY_AND_ASSIGN(BroadcastTime);
};

} // namespace openlcb

#endif // _OPENLCB_BROADCASTTIME_HXX_
