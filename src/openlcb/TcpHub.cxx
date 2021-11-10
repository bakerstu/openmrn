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
 * \file TcpHub.cxx
 * A component that starts a OpenLCB HUB listening on a TCP port.
 *
 * @author Robert Heller
 * @date 8 Nov 2021
 */

#include "utils/socket_listener.hxx"
#include "utils/Hub.hxx"
#include "openlcb/If.hxx"
#include "openlcb/IfTcp.hxx"
#include "openlcb/IfTcpImpl.hxx"
#include "openlcb/TcpHub.hxx"
#include <memory>

namespace openlcb {


struct TcpHubPort : public Executable
{
    /// Constructor.
    ///
    /// @param tcp_hub Parent (binary) hub flow.
    /// @param fd device descriptor of open channel (socket)
    /// @param on_exit Notifiable that will be called when the descriptor
    /// experiences an error (typically upon device closed or connection lost).
    /// @param use_select true if fd can be used with select, false if threads
    /// are needed.
    TcpHubPort(TcpHubFlow *tcp_hub, int fd, Notifiable *on_exit)
          : tcpHub_(tcp_hub->service())
          , onExit_(on_exit)
    {
        LOG(VERBOSE, "tcphub port %p", (Executable *)this);
        //tcpWrite_.reset(new TcpHubDeviceSelect(new TcpRecvFlow(&tcpHub_), fd, this));
        // TcpRecvFlow: parse_tcp_message
        // TcpSendFlow: render_tcp_message
    }
    virtual ~TcpHubPort()
    {
    }
    /** This hub sees the character-based representation of the packets. The
     * members of it are: the bridge and the physical device (fd).
     *
     * Destruction requirement: HubFlow should be empty. This means after the
     * disconnection of the bridge (write side) and the FdHubport (read side)
     * we need to wait for the executor until this flow drains. */
    HubFlow tcpHub_;
    /** Reads the characters from the char-hub and sends them to the
     * fd. Similarly, listens to the fd and sends the read charcters to the
     * char-hub. */
    std::unique_ptr<FdHubPortInterface> tcpWrite_;
    /** Writes */
    /** If not null, this notifiable will be called when the device is
     * closed. */
    Notifiable* onExit_;
        /** Callback in case the connection is closed due to error. */
    void notify() OVERRIDE
    {
        /* We would like to delete *this but we cannot do that in this
         * callback, because we don't know what executor we are running
         * on. Deleting on the write executor would cause a deadlock for
         * example. */
        tcpHub_.service()->executor()->add(this);
    }

    void run() OVERRIDE
    {
        if (!tcpHub_.is_waiting())
        {
            // Yield.
            tcpHub_.service()->executor()->add(this);
            return;
        }
        LOG(INFO, "TcpHubPort: Shutting down port %d.",
            tcpWrite_->fd());
        if (onExit_) {
            onExit_->notify();
            onExit_ = nullptr;
        }
        /* We get this call when something is wrong with the FDs and we need to
         * close the connection. It is guaranteed that by the time we got this
         * call the device is unregistered from the char bridge, and the
         * service thread is ready to be stopped. */
        delete this;
    }
};

void TcpHub::on_new_connection(int fd)
{
    {
        AtomicHolder h(this);
        numClients_++;
    }
    new TcpHubPort(tcpHub_, fd, this);
}

void TcpHub::notify()
{
    AtomicHolder h(this);
    if (numClients_)
    {
        numClients_--;
    }
}

TcpHub::TcpHub(TcpHubFlow *tcp_hub, int port) 
      : tcpHub_(tcp_hub)
      , tcpListener_(port,
                     std::bind(&TcpHub::on_new_connection, 
                               this, std::placeholders::_1))
{
}

TcpHub::~TcpHub()
{
    tcpListener_.shutdown();
}
}
