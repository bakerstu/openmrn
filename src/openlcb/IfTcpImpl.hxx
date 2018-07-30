/** \copyright
 * Copyright (c) 2017, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file IfTcpImpl.hxx
 *
 * Internal implementation flows for the Tcp interface
 *
 * @author Balazs Racz
 * @date 16 Apr 2017
 */

#ifndef _OPENLCB_IFTCPIMPL_HXX_
#define _OPENLCB_IFTCPIMPL_HXX_

#include "openlcb/If.hxx"
#include "utils/Hub.hxx"

namespace openlcb
{

/** Static class for constants and utilities related to the TCP transport
 * protocol. */
class TcpDefs
{
public:
    /** Renders a TCP message into a single buffer, ready to transmit.
     * @param msg is the OpenLCB message to render.
     * @param gateway_node_id will be populated into the message header as the
     * message source (last sending node ID).
     * @param sequence is a 48-bit millisecond value that's monotonic.
     * @param target is the buffer into which to render the message. */
    static void render_tcp_message(const GenMessage &msg,
        NodeID gateway_node_id, long long sequence, string *tgt)
    {
        bool has_dst = Defs::get_mti_address(msg.mti);
        HASSERT(tgt);
        string& target = *tgt;
        target.assign(HDR_SIZE + msg.payload.size() +
                (has_dst ? MSG_ADR_PAYLOAD_OFS : MSG_GLOBAL_PAYLOAD_OFS),
            '\0');
        uint16_t flags = FLAGS_OPENLCB_MSG;
        error_to_data(flags, &target[HDR_FLAG_OFS]);
        unsigned sz = target.size() - HDR_SIZE_OFS - 3;
        target[HDR_SIZE_OFS] = (sz >> 16) & 0xff;
        target[HDR_SIZE_OFS + 1] = (sz >> 8) & 0xff;
        target[HDR_SIZE_OFS + 2] = sz & 0xff;
        node_id_to_data(gateway_node_id, &target[HDR_GATEWAY_OFS]);
        node_id_to_data(sequence, &target[HDR_TIMESTAMP_OFS]);
        error_to_data(msg.mti, &target[HDR_SIZE + MSG_MTI_OFS]);
        node_id_to_data(msg.src.id, &target[HDR_SIZE + MSG_SRC_OFS]);
        if (has_dst)
        {
            node_id_to_data(msg.dst.id, &target[HDR_SIZE + MSG_DST_OFS]);
            memcpy(&target[HDR_SIZE + MSG_ADR_PAYLOAD_OFS], msg.payload.data(),
                msg.payload.size());
        }
        else
        {
            memcpy(&target[HDR_SIZE + MSG_GLOBAL_PAYLOAD_OFS],
                msg.payload.data(), msg.payload.size());
        }
    }

    enum
    {
        FLAGS_OPENLCB_MSG = 0x8000,
        FLAGS_CHAINING = 0x4000,
        FLAGS_RESVD1_ZERO_CHECK = 0x3000,
        FLAGS_FRAGMENT_NOT_FIRST = 0x0800,
        FLAGS_FRAGMENT_NOT_LAST = 0x0400,
        FLAGS_RESVD2_IGNORED = 0x03FF,

        HDR_FLAG_OFS = 0,
        HDR_SIZE_OFS = 2,
        HDR_GATEWAY_OFS = 2 + 3,
        HDR_TIMESTAMP_OFS = 2 + 3 + 6,
        HDR_SIZE = 2 + 3 + 6 + 6,

        MSG_MTI_OFS = 0,
        MSG_SRC_OFS = 2,
        MSG_DST_OFS = 2 + 6,
        MSG_ADR_PAYLOAD_OFS = 2 + 6 + 6,

        MSG_GLOBAL_PAYLOAD_OFS = MSG_DST_OFS,
    };

private:
    /// No usable constructor; this is a static-only class.
    TcpDefs();
};

/**
   Virtual clock interface. Implementations are not required to be thread-safe.
*/
class SequenceNumberGenerator
{
public:
    /** Returns the next strictly monotonic sequence number. */
    virtual long long get_sequence_number() = 0;
};

/**
   Implementation of sequence number generator that uses the real clock. Not
   thread-safe.
 */
class ClockBaseSequenceNumberGenerator : public SequenceNumberGenerator
{
public:
    long long get_sequence_number() override
    {
        long long current_time_ms = os_get_time_monotonic() / 1000000;
        if (current_time_ms > sequence_)
        {
            sequence_ = current_time_ms;
        }
        else
        {
            ++sequence_;
        }
        return sequence_;
    }

private:
    /// Sequence number of last sent message.
    long long sequence_ = 0;
};

/**
 * This flow renders outgoing OpenLCB messages to their TCP stream
 * representation.
 */
class TcpSendFlow : public MessageStateFlowBase
{
public:
    TcpSendFlow(Service *service, NodeID gateway_node_id,
        HubPortInterface *send_target, HubPortInterface *skip_member,
        SequenceNumberGenerator *sequence)
        : MessageStateFlowBase(service)
        , sendTarget_(send_target)
        , skipMember_(skip_member)
        , gatewayId_(gateway_node_id)
        , sequenceNumberGenerator_(sequence)
    {
    }

private:
    Action entry() override
    {
        return allocate_and_call(sendTarget_, STATE(render_src_message));
    }

    Action render_src_message()
    {
        auto b = get_buffer_deleter(get_allocation_result(sendTarget_));
        TcpDefs::render_tcp_message(*nmsg(), gatewayId_,
            sequenceNumberGenerator_->get_sequence_number(), b->data());
        sendTarget_->send(b.release());
        return release_and_exit();
    }

    /// Returns the NMRAnet message we are trying to send.
    GenMessage *nmsg()
    {
        return message()->data();
    }
    
    /// Where to send the rendered messages to.
    HubPortInterface *sendTarget_;
    /// This value will be populated to the skipMember_ field.
    HubPortInterface *skipMember_;
    /// Populated into the source gateway field of the outgoing messages.
    NodeID gatewayId_;
    /// Responsible for generating thesequence numbers of the outgoing messages.
    std::unique_ptr<SequenceNumberGenerator> sequenceNumberGenerator_;
};
}

#endif // _OPENLCB_IFTCPIMPL_HXX_
