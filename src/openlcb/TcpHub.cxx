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
#include "openlcb/TcpHubPort.hxx"
#include <memory>

namespace openlcb {


void TcpHub::on_new_connection(int fd)
{
    {
        AtomicHolder h(this);
        numClients_++;
    }
    // What are the cases we need to support:
    //
    // - the remote server closing the socket. on_error shall be called and the
    //   port destructed.
    //
    // - the IfTcp being destructed (presumably not on the main executor). We
    //   need to unregister the port, shutdown the socket, flush the data, then
    //   delete this. The destructor of HubDeviceSelect triggers the barrier
    //   callback inline.
    //
    // The contract with HubDeviceSelect is that by the time the on_error
    // notifiable is called, the HubDeviceSelect has been unregistered,
    // flushed, and can be deleted, even if we are on the main executor.
    //
    new TcpHubPort(tcpHub_, 0, nullptr, fd, this);
#if 0    
    struct RemotePort : public Executable
    {
        RemotePort(TcpHub *parent, Notifiable *on_error)
            : parent_(parent)
            , onError_(on_error)
        {
        }
        ~RemotePort()
        {
            // auto* p = port_->write_port();
            // LOG_ERROR("remoteport::delete %p %p", p, this);
            deleting_ = true;
            port_.reset();
            // LOG_ERROR("remoteport::delete done %p", p);
        }
        TcpHub *parent_;
        std::unique_ptr<TcpHubDeviceSelect> port_;
        Notifiable *onError_;
        bool deleting_{false};
        void notify() override
        {
            // auto* p = port_->write_port();
            // LOG(VERBOSE, "remoteport::notify %d %p %p", deleting_, p, this);
            if (onError_)
            {
                onError_->notify();
                onError_ = nullptr;
            }
            if (!deleting_) // avoids duplicate destruction.
            {
                //parent_->executor()->add(this);
            }
        }

        void run() override
        {
            if (deleting_ || !port_)
            {
                return;
            }
            if (!port_->write_done())
            {
                // yield
                //parent_->executor()->add(this);
                return;
            }
            //parent_->delete_owned_flow(this);
        }
    };
    RemotePort *p = new RemotePort(this, on_error);
    p->port_.reset(new TcpHubDeviceSelect(tcpHub_, fd, p));
    //add_owned_flow(p);
#endif    
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
