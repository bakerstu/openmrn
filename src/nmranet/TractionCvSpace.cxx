/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file TractionCvSpace.hxx
 *
 * A memory config protocol compatible Memory Space for reading and writing CVs
 * of DCC locomotives.
 *
 * @author Balazs Racz
 * @date 16 May 2015
 */

#include "nmranet/TractionCvSpace.hxx"
#include "nmranet/TractionDefs.hxx"

namespace nmranet
{

TractionCvSpace::TractionCvSpace(MemoryConfigHandler *parent,
                                 dcc::PacketFlowInterface *track,
                                 dcc::RailcomHubFlow *railcom_hub,
                                 uint8_t space_id)
    : StateFlowBase(parent->service())
    , parent_(parent)
    , track_(track)
    , railcomHub_(railcom_hub)
    , errorCode_(ERROR_NOOP)
    , spaceId_(space_id)
    , timer_(this)
{
    parent_->registry()->insert(nullptr, spaceId_, this);
    // We purposefully do not start the state flow until a request comes in.
}

TractionCvSpace::~TractionCvSpace()
{
    parent_->registry()->erase(nullptr, spaceId_, this);
    if (errorCode_ == ERROR_PENDING)
    {
        timer_.cancel();
    }
}

bool TractionCvSpace::set_node(Node *node)
{
    if (!node)
    {
        return false;
    }
    NodeID id = node->node_id();
    /* NOTE(balazs.racz): It is difficult to figure out the DCC address given
     * an abstract NMRAnet node. Here we hardcode the reserved DCC node ID
     * space, which is not a great solution. */
    if ((id & 0xFFFF00000000ULL) == TractionDefs::NODE_ID_DCC)
    {
        uint16_t new_address = id & 0xFFFFU;
        if (dccAddress_ != new_address)
        {
            dccAddress_ = new_address;
            errorCode_ = ERROR_NOOP;
        }
        return true;
    }
    else
    {
        return false;
    }
}

const unsigned TractionCvSpace::MAX_CV;

size_t TractionCvSpace::read(address_t source, uint8_t *dst, size_t len,
                             errorcode_t *error, Notifiable *again)
{
    if (source > MAX_CV)
    {
        *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
        return 0;
    }
    if (errorCode_ == ERROR_OK && source == cvNumber_)
    {
        *dst = cvData_;
        return 1;
    }
    done_ = again;
    cvNumber_ = source;
    errorCode_ = ERROR_NOOP;
    cvData_ = 0;
    start_flow(STATE(try_read1));
    *error = ERROR_AGAIN;
    return 0;
}

StateFlowBase::Action TractionCvSpace::try_read1()
{
    return allocate_and_call(track_, STATE(fill_read1_packet));
}
StateFlowBase::Action TractionCvSpace::try_write1()
{
    return allocate_and_call(track_, STATE(fill_write1_packet));
}

StateFlowBase::Action TractionCvSpace::fill_read1_packet()
{
    auto *b = get_allocation_result(track_);
    b->data()->start_dcc_packet();
    /** @TODO(balazs.racz) here we make bad assumptions about how to decide
     * between long and short addresses */
    if (dccAddress_ >= 0x80)
    {
        b->data()->add_dcc_address(dcc::DccLongAddress(dccAddress_));
    }
    else
    {
        b->data()->add_dcc_address(dcc::DccShortAddress(dccAddress_));
    }
    b->data()->add_dcc_pom_read1(cvNumber_);
    b->data()->feedback_key = reinterpret_cast<size_t>(this);
    railcomHub_->register_port(this);
    errorCode_ = ERROR_PENDING;
    track_->send(b);
    return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(railcom_returned));
}

StateFlowBase::Action TractionCvSpace::railcom_returned()
{
    LOG(WARNING, "railcom returned status %d value %d", errorCode_, cvData_);
    done_->notify();
    return exit();
}

size_t TractionCvSpace::write(address_t destination, const uint8_t *src,
                              size_t len, errorcode_t *error, Notifiable *again)
{
    if (destination > MAX_CV)
    {
        *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
        return 0;
    }
    if (errorCode_ == ERROR_OK && destination == cvNumber_ && *src == cvData_)
    {
        return 1;
    }
    done_ = again;
    cvNumber_ = destination;
    cvData_ = *src;
    errorCode_ = ERROR_NOOP;
    start_flow(STATE(try_write1));
    *error = ERROR_AGAIN;
    return 0;
}

StateFlowBase::Action TractionCvSpace::fill_write1_packet()
{
    auto *b = get_allocation_result(track_);
    b->data()->start_dcc_packet();
    /** @TODO(balazs.racz) here we make bad assumptions about how to decide
     * between long and short addresses */
    if (dccAddress_ >= 0x80)
    {
        b->data()->add_dcc_address(dcc::DccLongAddress(dccAddress_));
    }
    else
    {
        b->data()->add_dcc_address(dcc::DccShortAddress(dccAddress_));
    }
    b->data()->add_dcc_pom_write1(cvNumber_, cvData_);
    b->data()->feedback_key = reinterpret_cast<size_t>(this);
    railcomHub_->register_port(this);
    errorCode_ = ERROR_PENDING;
    track_->send(b);
    return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(railcom_returned));
}

void TractionCvSpace::record_railcom_status(unsigned code)
{
    errorCode_ = code;
    timer_.trigger();
}

void TractionCvSpace::send(Buffer<dcc::RailcomHubData> *b, unsigned priority)
{
    AutoReleaseBuffer<dcc::RailcomHubData> ar(b);
    if (errorCode_ != ERROR_PENDING)
        return;
    const dcc::Feedback &f = *b->data();
    if (f.feedbackKey != reinterpret_cast<size_t>(this))
    {
        return;
    }
    if (!f.ch2Size)
    {
        return record_railcom_status(ERROR_NO_RAILCOM_CH2_DATA);
    }
    uint8_t b0 = dcc::railcom_decode[f.ch2Data[0]];
    LOG(INFO, "railcom byte 0 decoded as 0x%x", b0);
    if (b0 == dcc::RailcomDefs::BUSY || b0 == dcc::RailcomDefs::NACK)
    {
        return record_railcom_status(ERROR_BUSY);
    }
    if (b0 == dcc::RailcomDefs::INV || b0 > 63)
    {
        return record_railcom_status(ERROR_GARBAGE);
    }
    uint8_t cmd = b0 >> 2;
    if (cmd != dcc::RMOB_POM)
    {
        return record_railcom_status(ERROR_UNKNOWN_RESPONSE);
    }
    unsigned value = b0 & 3;
    uint8_t b1 = dcc::railcom_decode[f.ch2Data[1]];
    if (b1 > 63)
    {
        return record_railcom_status(ERROR_GARBAGE);
    }
    value <<= 6;
    value |= b1;
    cvData_ = value;
    return record_railcom_status(ERROR_OK);
}

} // namespace nmranet
