/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file Logon.hxx
 * Control flows for supporting DCC Automatic Logon.
 *
 * @author Balazs Racz
 * @date 12 Aug 2021
 */

#ifndef _DCC_LOGON_HXX_
#define _DCC_LOGON_HXX_

#include "dcc/LogonFeedback.hxx"
#include "dcc/PacketSource.hxx"
#include "dcc/TrackIf.hxx"
#include "dcc/UpdateLoop.hxx"
#include "executor/StateFlow.hxx"

namespace dcc
{

/// This class needs to be a base class for the template argument of the Logon
/// Handler.
class LogonHandlerModule
{

}; // LogonHandlerModule

/// Handles the automatic logon flow for DCC decoders.
template <class Module>
class LogonHandler : public StateFlowBase, private LogonFeedbackCallbacks
{
public:
    /// Constructor
    ///
    /// @param service points to the executor to use.
    /// @param track pointer to the track interface to send DCC packets to.
    /// @param rcom_hub will register to this railcom hub to get feedback.
    LogonHandler(Service *service, TrackIf *track, RailcomHubFlow *rcom_hub)
        : StateFlowBase(service)
        , trackIf_(track)
        , fbParser_(this, rcom_hub)
        , hasLogonEnableConflict_(0)
        , hasLogonEnableFeedback_(0)
        , needShutdown_(0)
    {
    }

    /// Initiates a logon sequence at startup.
    void startup_logon(uint16_t cid, uint8_t session_id)
    {
        cid_ = cid;
        sessionId_ = session_id;
        start_flow(STATE(allocate_logon_now));
    }

#ifdef GTEST
    void shutdown()
    {
        needShutdown_ = 1;
        timer_.ensure_triggered();
    }
#endif

    // Callbacks from LogonFeedback

    /// Determines based on feedback key what the given DCC packet was.
    /// @param feedback_key from the railcom packet.
    /// @return the packet classification wrt the logon feature.
    PacketType classify_packet(uintptr_t feedback_key) override
    {
        if (feedback_key == LOGON_ENABLE_KEY)
        {
            return LOGON_ENABLE;
        }
        return UNKNOWN;
    }

    /// Handles a Select ShortInfo feedback message.
    /// @param feedback_key refers to the packet it came from.
    /// @param error true if there was a transmission error or the data came in
    /// incorrect format.
    /// @param data 48 bits of payload.
    void process_select_shortinfo(
        uintptr_t feedback_key, bool error, uint64_t data) override
    {
    }

    /// Handles a Logon Assign feedback message.
    /// @param feedback_key refers to the packet it came from.
    /// @param error true if there was a transmission error or the data came in
    /// incorrect format.
    /// @param data 48 bits of payload.
    void process_logon_assign(
        uintptr_t feedback_key, bool error, uint64_t data) override
    {
    }

    /// Handles a Decoder ID feedback message.
    /// @param feedback_key refers to the packet it came from.
    /// @param error true if there was a transmission error or the data came in
    /// incorrect format.
    /// @param data 48 bits of payload. The low 44 bits of this is a decoder ID.
    void process_decoder_id(uintptr_t feedback_key, bool error, uint64_t data)
    {
        hasLogonEnableFeedback_ = 1;
        if (error)
        {
            hasLogonEnableConflict_ = 1;
        }
        /// @TODO: store decoder ID and trigger a select on it.
    }

private:
    Action allocate_logon_now()
    {
        return allocate_and_call(trackIf_, STATE(send_logon_now));
    }

    Action send_logon_now()
    {
        logon_send_helper(Defs::LogonEnableParam::NOW, 0);
        return wait_and_call(STATE(start_logon_wait));
    }

    /// Called when the logon now packet is released. This means the packet is
    /// enqueued in the device driver, but not necessarily that it is on the
    /// track yet.
    ///
    /// Computes the next time that we need to send out a logon packet, and
    /// starts a sleep.
    Action start_logon_wait()
    {
        if (needShutdown_)
        {
            return exit();
        }
        auto next_time = lastLogonTime_ + MSEC_TO_NSEC(LOGON_PERIOD_MSEC);
        timer_.start_absolute(next_time);
        return wait_and_call(STATE(evaluate_logon));
    }

    /// Called when the logon timer expires or is cancelled due to feedback.
    Action evaluate_logon()
    {
        if (needShutdown_)
        {
            return exit();
        }
        if (timer_.is_triggered())
        {
            // found something via logon
            if (hasLogonEnableConflict_)
            {
                hasLogonEnableConflict_ = 0;
                hasLogonEnableFeedback_ = 0;
                return call_immediately(STATE(allocate_logon_many));
            }
            // Not sure why we were woken up, let's start a sleep again.
            return call_immediately(STATE(start_logon_wait));
        }
        else
        {
            // timer expired, send another logon.
            return call_immediately(STATE(allocate_logon_now));
        }
    }

    /// Called when we have seen a conflict on a logon enable packet. Allocates
    /// a buffer to send out many logon packets.
    Action allocate_logon_many()
    {
        return allocate_and_call(trackIf_, STATE(send_logon_many));
    }

    /// Send out the repeated logon request.
    Action send_logon_many()
    {
        logon_send_helper(Defs::LogonEnableParam::ALL, 3);
        return wait_and_call(STATE(many_logon_wait));
    }

    /// Called after the many logon packets' buffer is freed.
    Action many_logon_wait()
    {
        if (countLogonToSend_ >= 4)
        {
            countLogonToSend_ -= 4;
            return call_immediately(STATE(allocate_logon_many));
        }
        else
        {
            countLogonToSend_ = 0;
            /// @TODO we should really evaluate whether we've seen conflicts
            /// coming back.
            return call_immediately(STATE(start_logon_wait));
        }
    }

    /// Helper function to send out logon enable commands. Requirement: a
    /// buffer is allocated before calling.
    /// @param param option of which logon enable command to send out.
    /// @param rept 0-3 for how many repeats to send out (N-1, so it means 1-4
    /// repeats).
    void logon_send_helper(Defs::LogonEnableParam param, unsigned rept)
    {
        HASSERT(rept < 4u);
        auto *b = get_allocation_result(trackIf_);
        b->data()->set_dcc_logon_enable(param, cid_, sessionId_);
        b->data()->feedback_key = LOGON_ENABLE_KEY;
        b->data()->packet_header.rept_count = 3; // 4 repeats.
        b->set_done(bn_.reset(this));

        hasLogonEnableFeedback_ = 0;
        hasLogonEnableConflict_ = 0;
        lastLogonTime_ = os_get_time_monotonic();

        trackIf_->send(b);
    }

    /// We send this as feedback key for logon enable packets.
    static constexpr uintptr_t LOGON_ENABLE_KEY =
        uint32_t((Defs::ADDRESS_LOGON << 24) | (Defs::DCC_LOGON_ENABLE << 16));

    /// How often to send logon enable packets.
    static constexpr unsigned LOGON_PERIOD_MSEC = 295;

    /// If we need to send packets to the track, we can do it here directly.
    TrackIf *trackIf_;

    /// Helper object for timing.
    StateFlowTimer timer_ {this};

    /// Helper object for parsing railcom feedback from the railcom hub.
    LogonFeedbackParser fbParser_;

    BarrierNotifiable bn_;
    
    /// Timestamp of the last logon packet we sent out.
    long long lastLogonTime_ {0};

    /// Command station unique ID.
    uint16_t cid_;
    /// Session ID of the current session.
    uint8_t sessionId_;

    /// 1 if we got an error (presumably a conflict) in the logon enable
    /// feedback.
    uint8_t hasLogonEnableConflict_ : 1;
    /// 1 if we got any feedback packet from logon enable.
    uint8_t hasLogonEnableFeedback_ : 1;
    /// Signals that we need to shut down the flow.
    uint8_t needShutdown_ : 1;

    /// Tracks how many logons to send out.
    uint8_t countLogonToSend_ {0};

    /// Identifier of the storage that provides the next locomotive to look at.
    uint16_t cycleNextId_;

}; // LogonHandler

} // namespace dcc

#endif // _DCC_LOGON_HXX_
