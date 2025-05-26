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
 * \file GcTcpHub.hxx
 * A component that starts a gridconnect-protocol HUB listening on a TCP port.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#ifndef _UTILS_GCTCPHUB_HXX_
#define _UTILS_GCTCPHUB_HXX_

#include <vector>
#include <functional>

#include "utils/socket_listener.hxx"
#include "utils/Hub.hxx"

class ExecutorBase;

/// This class runs a CAN-bus HUB listening on TCP socket using the gridconnect
/// format. Any new incoming connection will be wired into the same virtual CAN
/// hub. All packets will be forwarded to every participant, without
/// loopback.
class GcTcpHub
{
public:
    /// Constructor.
    ///
    /// @param can_hub Which CAN-hub should we attach the TCP gridconnect hub
    ///        onto.
    /// @param port TCP port number to listen on.
    /// @param on_connect_callback hook for the application "on connect".
    GcTcpHub(CanHubFlow *can_hub, int port,
        std::function<void()> on_connect_callback = nullptr);

    /// Destructor
    ~GcTcpHub();

    /// @return true of the listener is ready to accept incoming connections.
    bool is_started()
    {
        return tcpListener_.is_started();
    }

    /// @return currently connected client count.
    unsigned get_num_clients()
    {
        return clients_.size();
    }

private:
    /// Helper object for managing connected clients.
    class OnErrorNotify : public Notifiable
    {
    public:
        /// Constructor.
        /// @param parent parent object pointer
        /// @param fd file descriptor to track for notifications
        OnErrorNotify(GcTcpHub *parent, int fd)
            : parent_(parent)
            , fd_(fd)
        {
            AtomicHolder h(&GcTcpHub::lock_);
            parent_->clients_.push_back({this, fd});
        }

        /// Should be called on any active clients when parent is destructed.
        void unregister_port()
        {
            fd_ = -1;
        }

    private:
        /// Provides notification that the client is is being removed.
        void notify() override
        {
            {
                AtomicHolder h(&GcTcpHub::lock_);
                // This check is critical during and after the destruction of
                // the parent. Once the fd_ has been set to < 0, parent_ is
                // is no longer guaranteed to be a valid pointer.
                if (fd_ >= 0)
                {
                    for (auto it = parent_->clients_.begin();
                        it != parent_->clients_.end(); ++it)
                    {
                        if ((*it).fd_ == fd_)
                        {
                            // Found a match, stop tracking it.
                            parent_->clients_.erase(it);
                            break;
                        }
                    }
                }
                else
                {
                    // Likely the parent is undergoing or has completed
                    // destruction.
                }
            }
            LOG(INFO, "GcTcpHub notify, erase: %i", fd_);
            delete this;
        }

        GcTcpHub *parent_; ///< parent object pointer
        int fd_; /// registered file descriptor
    };

    /// Metadata structure for tracking clients.
    struct Client
    {
        OnErrorNotify *n_; ///< reference to the notify object.
        int fd_; ///< file descriptor that is being tracked.
    };

    /// This object allows us to always use and AtomicHolder even if it is used
    /// from a Notify instance that has outlived its parent.
    static Atomic lock_;

    /// Callback when a new connection arrives.
    /// @param fd filedes of the freshly established incoming connection.
    void on_new_connection(int fd);

    /// Callback hook for the application "on connect".
    std::function<void()> onConnectCallback_;

    /// Which CAN-hub should we attach the TCP gridconnect hub onto.
    CanHubFlow *canHub_;

    /// Helper object representing the listening on the socket.
    SocketListener tcpListener_;

    /// List of connected clients
    std::vector<Client> clients_;
};

#endif // _UTILS_GCTCPHUB_HXX_
