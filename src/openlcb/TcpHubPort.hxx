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
 * \file TcpHubPort.hxx
 * Establishes the connection for a single remote Tcp client to a Hub.
 *
 * @author Balazs Racz
 * @date 8 Dec 2021
 */

#ifndef _OPENLCB_TCPHUBPORT_HXX_
#define _OPENLCB_TCPHUBPORT_HXX_

#include "openlcb/IfTcpImpl.hxx"
#include "openlcb/TcpHub.hxx"

namespace openlcb {

/// Simple stateless translator for incoming TCP messages from binary format
/// into the structured format. Drops everything to the floor that is not a
/// valid TCP message. Performs synchronous allocation and keeps the done
/// callback passed along.
class TcpHubParseFlow : public HubPortInterface, public Destructable
{
public:
    /// @param target is where to send the parsed messages. Usually the
    /// interface's dispatcher flow.
    /// @param skip_member will be passed in as the skipmember to the hub.
    TcpHubParseFlow(TcpHubFlow *target, TcpHubPortInterface* skip_member)
        : target_(target)
        , skipMember_(skip_member)
    {
    }

    /// Entry point for the incoming (binary) data. These are already segmented
    /// correctly to openlcb-TCP packet boundaries.
    /// @param data buffer
    /// @param prio priority
    void send(Buffer<HubData> *data, unsigned prio) override
    {
        auto src = get_buffer_deleter(data);
        auto dst = get_buffer_deleter(target_->alloc());
        dst->set_done(data->new_child());
        if (TcpDefs::parse_tcp_message(*src->data(), dst->data()))
        {
            dst->data()->skipMember_ = skipMember_;
            target_->send(dst.release(), prio);
        }
    }

private:
    /// Flow where to pass on the parsed GenMessage.
    TcpHubFlow *target_;
    /// What to skip on messages created.
    TcpHubPortInterface* skipMember_;
};


class TcpHubPort : public TcpHubPortInterface, private Notifiable {
public:
    /// Constructor.
    /// @param parent this is where the parsed messages will be sent.
    /// @param gateway_node_id will be used in the gateway node ID field of the
    /// outgoing messages.
    /// @param fd this is the socket connecting to the remote endpoint.
    /// @param on_error will be called if the remote endpoint disconnects.
    TcpHubPort(TcpHubFlow *parent, NodeID gateway_node_id,
        SequenceNumberGenerator *sequence, int fd,
        Notifiable *on_error = nullptr);

    /// Destructor.
    ~TcpHubPort();

    /// @todo what should this do?
    bool shutdown();

    void send(Buffer<TcpHubData> *b, unsigned prio) override;
    
private:
    /// Called by the device when the fd disconnects.
    void notify() override;
    
    /// This is the hub that we are registered on as a port.
    TcpHubFlow* parent_;
    /// This notifiable will be called if the remote endpoint disconnects.
    Notifiable* onError_;
    /// This hub is used to proxy the string-rendered data.
    HubFlow stringHub_;
    /// Maintains the read and write flows for the fd.
    TcpHubDeviceSelect device_;
    /// Flow used for converting GenMessage into the binary
    /// representation.
    TcpRenderFlow sendFlow_;
    /// Flow for parsing incoming binary messages to GenMessage.
    TcpHubParseFlow recvFlow_;
};

} // namespace openlcb



#endif // _OPENLCB_TCPHUBPORT_HXX_
