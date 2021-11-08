/** \copyright
 * Copyright (c) 2021, Robert Heller
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
 * \file TcpHub.hxx
 * A component that starts a OpenLCB HUB listening on a TCP port.
 *
 * @author Robert Heller
 * @date 8 Nov 2021
 */

#ifndef _UTILS_TCPHUB_HXX
#define _UTILS_TCPHUB_HXX

#include "utils/socket_listener.hxx"
#include "utils/Hub.hxx"
#include "openlcb/If.hxx"

class ExecutorBase;
/// Container for (binary) GenMessage messages going through Hubs.
struct GenMessageContainer : public StructContainer<openlcb::GenMessage>
{
    
    /* Constructor. Sets up (outgoing) frames to be empty extended frames by
     * default. */
    GenMessageContainer()
    {
    }

#if 0
    /** @returns a mutable pointer to the embedded GenMessage. */
    struct openlcb::GenMessage *mutable_frame()
    {
        return this;
    }
    /** @returns the embedded GenMessage. */
    const struct openlcb::GenMessage &frame() const
    {
        return *this;
    }
#endif
};



/** This class can be sent via a Buffer to a CAN hub.
 *
 * Access the data content via members \ref GenMessageContainer::mutable_frame
 * and \ref GenMessageContainer::frame.
 *
 * Set skipMember_ to non-NULL to skip a particular entry flow of the output.
 */
typedef HubContainer<GenMessageContainer> TcpHubData;

/// Interface class for a port to an OpenLCB hub.
typedef FlowInterface<Buffer<TcpHubData>> TcpHubPortInterface;
/// Base class for a port to an OpenLCB hub that is implemented as a 
/// stateflow.
typedef StateFlow<Buffer<TcpHubData>, QList<1>> TcpHubPort;


/** A hub that proxies packets of GenMessage messages. */
typedef GenericHubFlow<TcpHubData> TcpHubFlow;


/** This class runs a OpenLCB HUB listening on a TCP socket using the
 * OpenLCB binary Tcp format.  Any new incoming connections wil be
 * wired into the same OpenLCB hub.  All packets will be forwarded
 * to every participant, without loopback.
 */

class TcpHub : private Notifiable, private Atomic
{
public:
    /// Constructor.
    ///
    /// @param tcp_hub Which Tcp hub should we attach the OpenLCB TCP hub
    /// onto.
    /// @param port TCp port number to listen on.
    TcpHub(TcpHubFlow *tcp_hub, int port);
    ~TcpHub();

    /// @return true of the listener is ready to accept incoming connections.
    bool is_started()
    {
        return tcpListener_.is_started();
    }

    /// @return currently connected client count.
    unsigned get_num_clients()
    {
        return numClients_;
    }

private:
    /// Callback when a new connection arrives.
    ///
    /// @param fd filedes of the freshly established incoming connection.
    ///
    void on_new_connection(int fd);

    /// Error callback from the tcp socket. This is invoked when a
    /// client disconnects.
    void notify() override;

    /// @param tcp_hub Which OpenLCB hub should we attach the TCP hub
    /// onto.
    TcpHubFlow *tcpHub_;
    /// How many clients are connected right now.
    unsigned numClients_ {0};
    /// Helper object representing the listening on the socket.
    SocketListener tcpListener_;
};

                        

#endif // _UTILS_TCPHUB_HXX

