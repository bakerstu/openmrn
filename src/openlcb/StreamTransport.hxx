/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file StreamTransport.hxx
 *
 * Interface for stream functionality attached to an OpenLCB interface.
 *
 * @author Balazs Racz
 * @date 20 Dec 2022
 */

#ifndef _OPENLCB_STREAMTRANSPORT_HXX_
#define _OPENLCB_STREAMTRANSPORT_HXX_

#include "utils/Destructable.hxx"

#include <inttypes.h>

namespace openlcb
{

/// Collects the objects needed to support streams on an OpenLCB interface.
class StreamTransport : public Destructable
{
public:
    StreamTransport()
        : inUseSendStreamIds_(0)
        , nextSendStreamId_(0)
    { }

    /// @return an unused transmit stream source ID. If all transmit stream
    /// source IDs are in use, then returns 0xFF (which is an invalid stream
    /// ID).
    uint8_t get_send_stream_id()
    {
        int ret = -1;
        for (int i = 0; i < MAX_SEND_STREAM_ID && ret < 0; ++i)
        {
            if ((inUseSendStreamIds_ & (1u << nextSendStreamId_)) == 0)
            {
                ret = nextSendStreamId_;
                inUseSendStreamIds_ |= (1u << nextSendStreamId_);
            }
            if (++nextSendStreamId_ > MAX_SEND_STREAM_ID)
            {
                nextSendStreamId_ = 0;
            }
        }
        return ret & 0xFF;
    }

    /// @param stream_id a transmit stream source ID which was previously
    /// allocated using { \link get_send_stream_id } and no longer used.
    void release_send_stream_id(uint8_t stream_id)
    {
        inUseSendStreamIds_ &= ~(1u << stream_id);
    }

private:
    /// Largest stream ID we will be using for transmit stream's local IDs.
    static constexpr uint8_t MAX_SEND_STREAM_ID = 26;
    /// Bits are 1 if the respective stream ID is in use (for transmit
    /// streams).
    unsigned inUseSendStreamIds_ : 27;
    /// Index of the next bit to check in the inUseSendStreamIds_.
    unsigned nextSendStreamId_ : 5;
};

} // namespace openlcb

#endif // _OPENLCB_STREAMTRANSPORT_HXX_
