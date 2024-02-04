/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file RoutableMessage.hxx
 *
 * Structure that can represent a single message in different (both abstract
 * and concrete) formats.
 *
 * @author Balazs Racz
 * @date 18 Sep 2018
 */

#ifndef _OPENLCB_ROUTABLEMESSAGE_HXX_
#define _OPENLCB_ROUTABLEMESSAGE_HXX_

// ============ WARNING =============
// This code is not used currently.
// ============ WARNING =============

#include "utils/Hub.hxx"
#include "utils/SimpleQueue.hxx"

namespace openlcb
{

struct RoutableMessage;

//typedef shared_ptr<HubData>

/// ============ WARNING =============
/// This code is not used currently.
/// ============ WARNING =============
struct RoutableMessage
{
    /// Filled with Node ID when this is an addressed message. 0,0 for a global
    /// message.
    openlcb::NodeHandle dst_;
    /// Parsed message format. One ref is owned by *this unless nullptr.
    Buffer<GenMessage> *genMessage_;
    /// Rendered message format for TCP. One ref is owned by *this unless
    /// nullptr.
    Buffer<HubData> *tcpMessage_;
    /// Sequence of CAN frames in gridconnect (text) format that represent this
    /// message. One ref for each buffer is owned by this.
    TypedQueue<Buffer<HubData>> gcMessages_;
    /// Sequence of CAN frames binary format that represent this message. One
    /// ref for each buffer is owned by this.
    TypedQueue<Buffer<CanHubData>> canMessages_;
    /// Represent the entry port of the message. Used for filtering global
    /// messages.
    void *skipMember_;
    RoutableMessage()
        : genMessage_ {nullptr}
        , tcpMessage_ {nullptr}
    {
    }

    ~RoutableMessage()
    {
        if (genMessage_)
        {
            genMessage_->unref();
        }
        if (tcpMessage_)
        {
            tcpMessage_->unref();
        }
        while (!gcMessages_.empty())
        {
            auto *f = gcMessages_.pop_front();
            f->unref();
        }
        while (!canMessages_.empty())
        {
            auto *f = canMessages_.pop_front();
            f->unref();
        }
    }
};

} // namespace openlcb

#endif // _OPENLCB_ROUTABLEMESSAGE_HXX_
