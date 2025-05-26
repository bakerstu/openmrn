/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file GcTcpHub.cxx
 * A component that starts a gridconnect-protocol HUB listening on a TCP port.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#include "utils/GcTcpHub.hxx"

#include <memory>

#include "nmranet_config.h"
#include "utils/GridConnectHub.hxx"
#include "utils/FdUtils.hxx"

Atomic GcTcpHub::lock_;

void GcTcpHub::on_new_connection(int fd)
{
    const bool use_select =
        (config_gridconnect_tcp_use_select() == CONSTANT_TRUE);
    // Applies kernel parameters like socket options.
    FdUtils::optimize_socket_fd(fd);
    // Create new notification object for tracking the fd.
    OnErrorNotify *n = new OnErrorNotify(this, fd);
    create_gc_port_for_can_hub(canHub_, fd, n, use_select);

    if (onConnectCallback_)
    {
        onConnectCallback_();
    }
}

GcTcpHub::GcTcpHub(CanHubFlow *can_hub, int port,
    std::function<void()> on_connect_callback)
    : onConnectCallback_(on_connect_callback)
    , canHub_(can_hub)
    , tcpListener_(port,
          std::bind(&GcTcpHub::on_new_connection, this, std::placeholders::_1),
          "GcTcpHub")
{
}

GcTcpHub::~GcTcpHub()
{
    // Since shutdown is a blocking call, we cannot get delivered any
    // on_new_connection() callbacks following it.
    tcpListener_.shutdown();

    {
        AtomicHolder h(&GcTcpHub::lock_);
        for (auto it = clients_.begin(); it != clients_.end(); ++it)
        {
            (*it).n_->unregister_port();
        }
    }

    // From this point forward, it should be safe to manipulate the clients
    // vector with exclusive access. This is because any registered
    // notifications will do nothing, having been "unregistered".

    // There is probably a race condition here with regard to the socket being
    // closed elsewhere. Since shutting down a hub is typically proceeding an
    // attempt at a graceful shutdown anyways, we are letting it go.
    for (auto it = clients_.begin(); it != clients_.end(); ++it)
    {
        ::close((*it).fd_);
        LOG(INFO, "GcTcpHub delete, close: %i", (*it).fd_);
    }
}
