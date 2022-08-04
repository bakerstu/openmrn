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
public:
    /// @return the number of locomotives known. The locomotive IDs are
    /// 0..num_locos() - 1.
    unsigned num_locos();

    /// @param loco_id a locomotive identifier
    /// @return true if this is valid and belongs to a loco we know about.
    bool is_valid_loco_id(unsigned loco_id);

    /// Finds the storage cell for a locomotive and returns the flag byte for
    /// it.
    /// @param loco_id a valid locomotive ID.
    /// @return the flag byte for this loco.
    uint8_t &loco_flags(unsigned loco_id);

    /// Retrieves the decoder unique ID.
    /// @param loco_id the dense locomotive identifier.
    /// @return the decoder unique ID (44 bit, LSb-aligned).
    uint64_t loco_did(unsigned loco_id);

    /// Creates a new locomotive by decoder ID, or looks up an existing
    /// locomotive by decoder ID.
    /// @param decoder_id 44-bit decoder ID (aligned to LSb).
    /// @return locomotive ID for this cell.
    unsigned create_or_lookup_loco(uint64_t decoder_id);

    /// Runs the locomotive address policy. After the address policy is run,
    /// the loco should have the ability to answer the assigned_address
    /// question.
    /// @param loco_id which locomotive this is
    /// @param desired_address the S-9.2.1.1 encoded desired address for this
    /// decoder.
    void run_address_policy(unsigned loco_id, uint16_t desired_address);

    /// @param loco_id
    /// @return the address to be assigned to this locomotive. 14-bit.
    uint16_t assigned_address(unsigned loco_id);

    /// Invoked when the address assignment completes for a decoder.
    /// @param loco_id which decoder.
    void assign_complete(unsigned loco_id);
    
    /// Flags for the logon handler module.
    enum Flags
    {
        /// This decoder needs a get shortinfo command.
        FLAG_NEEDS_GET_SHORTINFO = 0x01,
        /// We sent a get shortinfo command.
        FLAG_PENDING_GET_SHORTINFO = 0x02,

        /// This decoder needs an assign command.
        FLAG_NEEDS_ASSIGN = 0x04,
        /// We sent an assign command
        FLAG_PENDING_ASSIGN = 0x08,

        /// 1 if we completed the address assignment.
        FLAG_COMPLETE = 0x10,
        /// 1 if we ended up in an error state for this loco.
        FLAG_ERROR_STATE = 0x20,
        /// 1 if we have asked for a re-try.
        FLAG_PENDING_RETRY = 0x40,
        /// This is a 1-bit pre-scaler on a shared 50 msec timer that controls
        /// the delay of re-tries. This makes a retry happen in 50 to 100 msec
        /// time from the original failed attempt.
        FLAG_PENDING_TICK = 0x80,
    };
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
    /// @param m the module for the storage and CS interface
    LogonHandler(
        Service *service, TrackIf *track, RailcomHubFlow *rcom_hub, Module *m)
        : StateFlowBase(service)
        , trackIf_(track)
        , module_(m)
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
        logonSelect_.ensure_triggered();
    }
#endif

    // Callbacks from LogonFeedback

    /// Determines based on feedback key what the given DCC packet was.
    /// @param feedback_key from the railcom packet.
    /// @return the packet classification wrt the logon feature.
    PacketType classify_packet(uintptr_t feedback_key) override
    {
        if (feedback_key >= 1 << 14)
        {
            LOG(INFO, "classify %x", (unsigned)feedback_key);
        }
        if (is_logon_enable_key(feedback_key))
        {
            return LOGON_ENABLE;
        }
        else if (is_select_shortinfo_key(feedback_key))
        {
            return SELECT_SHORTINFO;
        }
        else if (is_logon_assign_key(feedback_key))
        {
            return LOGON_ASSIGN;
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
        if (!is_select_shortinfo_key(feedback_key))
        {
            LOG(WARNING, "Unexpected select shortinfo key: %08x",
                (unsigned)feedback_key);
            return;
        }
        unsigned loco_id = feedback_key & LOCO_ID_MASK;
        if (!module_->is_valid_loco_id(loco_id))
        {
            LOG(WARNING,
                "Unexpected select shortinfo key: %08x - invalid loco id",
                (unsigned)feedback_key);
            return;
        }
        uint8_t &flags = module_->loco_flags(loco_id);
        LOG(INFO, "Select shortinfo for loco ID %d, flags %02x error %d",
            loco_id, flags, error);
        flags &= ~LogonHandlerModule::FLAG_PENDING_GET_SHORTINFO;
        if (error)
        {
            if (flags & LogonHandlerModule::FLAG_PENDING_RETRY)
            {
                flags &= ~LogonHandlerModule::FLAG_PENDING_RETRY;
                flags |= LogonHandlerModule::FLAG_ERROR_STATE;
                return;
            }
            else
            {
                flags |= LogonHandlerModule::FLAG_NEEDS_GET_SHORTINFO |
                    LogonHandlerModule::FLAG_PENDING_RETRY;
                logonSelect_.wakeup();
                return;
            }
        }
        if (flags &
            (LogonHandlerModule::FLAG_NEEDS_ASSIGN |
                LogonHandlerModule::FLAG_PENDING_ASSIGN))
        {
            // Got multiple returns.
            return;
        }
        module_->run_address_policy(loco_id, (data >> 32) & 0x3FFF);
        flags |= LogonHandlerModule::FLAG_NEEDS_ASSIGN;
        logonSelect_.wakeup();
    }

    /// Handles a Logon Assign feedback message.
    /// @param feedback_key refers to the packet it came from.
    /// @param error true if there was a transmission error or the data came in
    /// incorrect format.
    /// @param data 48 bits of payload.
    void process_logon_assign(
        uintptr_t feedback_key, bool error, uint64_t data) override
    {
        if (!is_logon_assign_key(feedback_key))
        {
            LOG(WARNING, "Unexpected logon assign key: %08x",
                (unsigned)feedback_key);
            return;
        }
        unsigned loco_id = feedback_key & LOCO_ID_MASK;
        if (!module_->is_valid_loco_id(loco_id))
        {
            LOG(WARNING, "Unexpected logon assign key: %08x - invalid loco id",
                (unsigned)feedback_key);
            return;
        }
        uint8_t &flags = module_->loco_flags(loco_id);
        flags &= ~LogonHandlerModule::FLAG_PENDING_ASSIGN;
        if (flags & LogonHandlerModule::FLAG_COMPLETE)
        {
            // duplicate responses.
            return;
        }
        if (error)
        {
            if (flags & LogonHandlerModule::FLAG_PENDING_RETRY)
            {
                flags &= ~LogonHandlerModule::FLAG_PENDING_RETRY;
                flags |= LogonHandlerModule::FLAG_ERROR_STATE;
                return;
            }
            else
            {
                flags |= LogonHandlerModule::FLAG_NEEDS_ASSIGN |
                    LogonHandlerModule::FLAG_PENDING_RETRY;
                logonSelect_.wakeup();
                return;
            }
        }
        module_->assign_complete(loco_id);
        flags &= ~LogonHandlerModule::FLAG_PENDING_TICK;
        LOG(INFO, "Assign completed for loco %d address %d", loco_id,
            module_->assigned_address(loco_id));
    }

    /// Handles a Decoder ID feedback message.
    /// @param feedback_key refers to the packet it came from.
    /// @param error true if there was a transmission error or the data came in
    /// incorrect format.
    /// @param data 48 bits of payload. The low 44 bits of this is a decoder ID.
    void process_decoder_id(
        uintptr_t feedback_key, bool error, uint64_t data) override
    {
        timer_.ensure_triggered();
        if (data)
        {
            hasLogonEnableFeedback_ = 1;
        } else {
            // No railcom feedback returned.
            return;
        }
        if (LOGLEVEL >= INFO)
        {
            unsigned didh = (data >> 32) & 0xfff;
            unsigned didl = data & 0xffffffffu;
            LOG(INFO, "Decoder id %03x%08x error %d", didh, didl, error);
        }
        if (error)
        {
            hasLogonEnableConflict_ = 1;
            return;
        }
        uint64_t did = data & DECODER_ID_MASK;
        auto lid = module_->create_or_lookup_loco(did);
        if (!module_->is_valid_loco_id(lid))
        {
            return;
        }
        auto &flags = module_->loco_flags(lid);
        flags |= LogonHandlerModule::FLAG_NEEDS_GET_SHORTINFO;
        logonSelect_.wakeup();
    }

private:
    /// Allocates a buffer and sends a Logon Enable(now) packet.
    Action allocate_logon_now()
    {
        return allocate_and_call(trackIf_, STATE(send_logon_now));
    }

    /// Sends a Logon Enable(now) packet.
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
        b->data()->packet_header.rept_count = rept;
        b->set_done(bn_.reset(this));

        hasLogonEnableFeedback_ = 0;
        hasLogonEnableConflict_ = 0;
        lastLogonTime_ = os_get_time_monotonic();

        trackIf_->send(b);
    }

    class LogonSelect;
    friend class LogonSelect;

    /// Flow that sends out addressed packets that are part of the logon
    /// sequences.
    class LogonSelect : public StateFlowBase, public ::Timer
    {
    public:
        LogonSelect(LogonHandler *parent)
            : StateFlowBase(parent->service())
            , ::Timer(parent->service()->executor()->active_timers())
            , parent_(parent)
        {
            start(MSEC_TO_NSEC(50));
        }

        /// Notifies the flow that there is work to do.
        void wakeup()
        {
            if (is_terminated())
            {
                cycleNextId_ = 0;
                start_flow(STATE(search));
            }
        }

        /// Called by a timer every 50 msec.
        void tick()
        {
            bool need_wakeup = false;
            for (unsigned id = 0; id < m()->num_locos() && id < MAX_LOCO_ID;
                 ++id)
            {
                uint8_t &fl = m()->loco_flags(cycleNextId_);
                if (fl & LogonHandlerModule::FLAG_PENDING_TICK)
                {
                    fl &= ~LogonHandlerModule::FLAG_PENDING_TICK;
                }
                else if (fl & LogonHandlerModule::FLAG_PENDING_RETRY)
                {
                    fl &= ~LogonHandlerModule::FLAG_PENDING_RETRY;
                    fl |= LogonHandlerModule::FLAG_ERROR_STATE;
                }
                else if (fl & LogonHandlerModule::FLAG_ERROR_STATE)
                {
                    /// @todo locomotives that are in error state should be
                    // re-tried every now and then. We would probably need an
                    // extra counter for this though somewhere.
                }
                else if (fl & LogonHandlerModule::FLAG_PENDING_GET_SHORTINFO)
                {
                    fl &= ~LogonHandlerModule::FLAG_PENDING_GET_SHORTINFO;
                    fl |= LogonHandlerModule::FLAG_NEEDS_GET_SHORTINFO |
                        LogonHandlerModule::FLAG_PENDING_RETRY;
                    need_wakeup = true;
                }
                else if (fl & LogonHandlerModule::FLAG_PENDING_ASSIGN)
                {
                    fl &= ~LogonHandlerModule::FLAG_PENDING_ASSIGN;
                    fl |= LogonHandlerModule::FLAG_NEEDS_ASSIGN |
                        LogonHandlerModule::FLAG_PENDING_RETRY;
                    need_wakeup = true;
                }
            }
            if (need_wakeup)
            {
                wakeup();
            }
        }

    private:
        /// Timer callback.
        long long timeout() override
        {
            if (parent_->needShutdown_)
            {
                return 0;
            }
            tick();
            return RESTART;
        }

        /// @return the pointer to the storage module.
        Module *m()
        {
            return parent_->module_;
        }

        /// @return the pointer to the track interface.
        TrackIf *track()
        {
            return parent_->trackIf_;
        }

        /// Entry to the flow. Looks through the states to see what we needs to
        /// be done.
        Action search()
        {
            if (parent_->needShutdown_)
            {
                return exit();
            }
            bool mid_cycle = (cycleNextId_ != 0);
            for (;
                 cycleNextId_ < m()->num_locos() && cycleNextId_ <= MAX_LOCO_ID;
                 ++cycleNextId_)
            {
                uint8_t fl = m()->loco_flags(cycleNextId_);
                if (fl & LogonHandlerModule::FLAG_NEEDS_GET_SHORTINFO)
                {
                    return allocate_and_call(
                        parent_->trackIf_, STATE(send_get_shortinfo));
                }
                if (fl & LogonHandlerModule::FLAG_NEEDS_ASSIGN)
                {
                    return allocate_and_call(
                        parent_->trackIf_, STATE(send_assign));
                }
            }
            cycleNextId_ = 0;
            // Check if we need to run the search for the first half of the
            // loco space too.
            if (mid_cycle)
            {
                return again();
            }
            return exit();
        }

        /// Called with a buffer allocated. Sends a get shortinfo command to
        /// the current decoder.
        Action send_get_shortinfo()
        {
            auto *b = get_allocation_result(parent_->trackIf_);
            uint64_t did = m()->loco_did(cycleNextId_);
            b->data()->set_dcc_select_shortinfo(did);
            b->data()->feedback_key =
                SELECT_SHORTINFO_KEY | (cycleNextId_ & LOCO_ID_MASK);
            b->set_done(bn_.reset((StateFlowBase *)this));
            track()->send(b);
            uint8_t &fl = m()->loco_flags(cycleNextId_);
            fl &= ~LogonHandlerModule::FLAG_NEEDS_GET_SHORTINFO;
            fl |= LogonHandlerModule::FLAG_PENDING_GET_SHORTINFO |
                LogonHandlerModule::FLAG_PENDING_TICK;
            return wait_and_call(STATE(search));
        }

        /// Called with a buffer allocated. Sends an assign command to
        /// the current decoder.
        Action send_assign()
        {
            auto *b = get_allocation_result(parent_->trackIf_);
            uint64_t did = m()->loco_did(cycleNextId_);
            b->data()->set_dcc_logon_assign(
                did, m()->assigned_address(cycleNextId_));
            b->data()->feedback_key =
                LOGON_ASSIGN_KEY | (cycleNextId_ & LOCO_ID_MASK);
            b->set_done(bn_.reset((StateFlowBase *)this));
            track()->send(b);
            uint8_t &fl = m()->loco_flags(cycleNextId_);
            fl &= ~LogonHandlerModule::FLAG_NEEDS_ASSIGN;
            fl |= LogonHandlerModule::FLAG_PENDING_ASSIGN |
                LogonHandlerModule::FLAG_PENDING_TICK;
            return wait_and_call(STATE(search));
        }

        /// Owning logon handler.
        LogonHandler *parent_;

        /// Identifier of the storage that provides the next locomotive to look
        /// at.
        unsigned cycleNextId_;

        /// Helper for self notification.
        BarrierNotifiable bn_;
    } logonSelect_ {this};

    /// We send this as feedback key for logon enable packets.
    static constexpr uintptr_t LOGON_ENABLE_KEY =
        uint32_t((Defs::ADDRESS_LOGON << 24) | (Defs::DCC_LOGON_ENABLE << 16));

    /// Checks if a feedback key is for logon enable.
    /// @param feedback_key the key
    /// @return true if this is for a logon enable
    static constexpr bool is_logon_enable_key(uintptr_t feedback_key)
    {
        return feedback_key == LOGON_ENABLE_KEY;
    }

    /// We send this as feedback key for select/get short info packets.
    static constexpr uintptr_t SELECT_SHORTINFO_KEY =
        uint32_t((Defs::ADDRESS_LOGON << 24) | (Defs::DCC_SELECT << 16) |
            (Defs::CMD_READ_SHORT_INFO << 12));

    /// Checks if a feedback key is for select shortinfo.
    /// @param feedback_key the key
    /// @return true if this is for a select short info
    static constexpr bool is_select_shortinfo_key(uintptr_t feedback_key)
    {
        return ((feedback_key & ~LOCO_ID_MASK) == SELECT_SHORTINFO_KEY);
    }

    /// We send this as feedback key for logon assign packets.
    static constexpr uintptr_t LOGON_ASSIGN_KEY =
        uint32_t((Defs::ADDRESS_LOGON << 24) | (Defs::DCC_LOGON_ASSIGN << 16));

    /// Checks if a feedback key is for logon assign.
    /// @param feedback_key the key
    /// @return true if this is for a logon assign
    static constexpr bool is_logon_assign_key(uintptr_t feedback_key)
    {
        return ((feedback_key & ~LOCO_ID_MASK) == LOGON_ASSIGN_KEY);
    }

    /// How often to send logon enable packets.
    static constexpr unsigned LOGON_PERIOD_MSEC = 295;

    /// Maximum allowed locomotive ID.
    static constexpr unsigned MAX_LOCO_ID = 0xfff;
    /// Mask selecting bits that belong to the locomotive ID.
    static constexpr uintptr_t LOCO_ID_MASK = MAX_LOCO_ID;

    /// Mask selecting bits that belong to the decoder ID.
    static constexpr uint64_t DECODER_ID_MASK = (1ull << 44) - 1;

    /// If we need to send packets to the track, we can do it here directly.
    TrackIf *trackIf_;

    /// Storage module.
    Module *module_;

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

}; // LogonHandler

} // namespace dcc

#endif // _DCC_LOGON_HXX_
