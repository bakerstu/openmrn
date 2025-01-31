/** @copyright
 * Copyright (c) 2024, Stuart Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * @file Connection.hxx
 *
 * Information about active connections.
 *
 * @author Stuart Baker
 * @date 18 March 2024
 */

#ifndef _BLE_CONNECTION_HXX_
#define _BLE_CONNECTION_HXX_

#include <functional>
#include <vector>
#include <cstring>

#include "ble/Defs.hxx"
#include "utils/Singleton.hxx"

namespace ble
{

// Forward declaration.
class Connections;

/// Connection instance metadata.
class Connection
{
public:
    /// Get the connection handle.
    /// @return connection handle
    Defs::ConnHandle get_handle()
    {
        return handle_;
    }

    /// Get the connection address.
    /// @return connection address
    void get_addr(Defs::Addr addr, Defs::AddrType *addr_type)
    {
        memcpy(addr, addr_, Defs::ADDR_LEN);
        if (addr_type)
        {
            *addr_type = addrType_;
        }
    }

    /// Destructor. This happens when there is a disconnect.
    ~Connection()
    {
    }

private:
    /// Constructor.
    /// @param handle handle to the connection
    /// @param addr peer address
    /// @param addr_type peer address type
    /// @param central true if central role, else peripheral role
    Connection(Defs::ConnHandle handle, Defs::Addr addr,
               Defs::AddrType addr_type, bool central)
        : handle_(handle)
        , addrType_(addr_type)
        , central_(central)
    {
        memcpy(addr_, addr, Defs::ADDR_LEN);
    }

    Defs::ConnHandle handle_; ///< handle to the connection
    Defs::Addr addr_; ///< peer address
    Defs::AddrType addrType_; ///< peer address type
    bool central_; ///< true if central role, else peripheral role

    /// Allow private access from Connections container object.
    friend class Connections;
};

/// Singleton container of all the active connections.
class Connections : public Singleton<Connections>
{
public:
    /// Constructor.
    Connections()
    {
    }

    /// Register an active connection.
    /// @param handle handle to the connection
    /// @param addr peer address
    /// @param addr_type peer address type
    /// @param central true if central role, else peripheral role
    void register_connection(Defs::ConnHandle handle, Defs::Addr addr,
                             Defs::AddrType addr_type, bool central)
    {
        connections_.emplace_back(Connection(handle, addr, addr_type, central));
    }

    /// Register a callback at disconnection.
    /// @param callback callback to call at disconnection
    void register_disconnect_callback(
        std::function<void(Connection *connection)> callback)
    {
        callbacks_.emplace_back(callback);
    }

    /// Called when a connection is disconnected. Results in the connection
    /// being unregistered.
    /// @param conn connection to disconnect
    void disconnect(Connection *conn)
    {
        // Execute all of the registered disconnect callbacks.
        for (auto &e : callbacks_)
        {
            e(conn);
        }
        // Remove the connection instance from the container.
        for (auto it = connections_.begin(); it != connections_.end(); ++it)
        {
            if (&(*it) == conn)
            {
                connections_.erase(it);
                // Should only be one entry per connection in the container.
                return;
            }
        }      
    }

    /// Called when a connection is disconnected. Results in the connection
    /// being unregistered.
    /// @param handle handle belonging to connection to disconnect
    void disconnect(Defs::ConnHandle handle)
    {
        // Remove the connection instance from the container.
        for (auto it = connections_.begin(); it != connections_.end(); ++it)
        {
            if (it->get_handle() == handle)
            {
                // Execute all of the registered disconnect callbacks.
                for (auto &e : callbacks_)
                {
                    e(&(*it));
                }
                connections_.erase(it);
                // Should only be one entry per connection in the container.
                return;
            }
        }      
    }

    /// Find a connection by its peer address.
    /// @param addr peer address
    /// @param addr_type peer address type
    /// @return connection instance pointer, else nullptr if not found
    Connection *lookup_by_address(Defs::Addr addr, Defs::AddrType addr_type)
    {
        for (auto &e : connections_)
        {
            if (!memcmp(e.addr_, addr, Defs::ADDR_LEN) &&
                e.addrType_ == addr_type)
            {
                // Match.
                return &e;
            }
        }
        return nullptr;
    }

    /// Find a connection by its connection handle.
    /// @param handle connection handle
    /// @return connection instance pointer, else nullptr if not found
    Connection *lookup_by_handle(Defs::ConnHandle handle)
    {
        for (auto &e : connections_)
        {
            if (e.handle_ == handle)
            {
                // Match.
                return &e;
            }
        }
        return nullptr;
    }

    /// Get the number of active connections being tracked by this object.
    /// @return number of active connections
    size_t get_active_count()
    {
        return connections_.size();
    }

private:
    /// All the connections managed by the container.
    std::vector<Connection> connections_;
    /// Callbacks to be called upon disconnection.
    std::vector<std::function<void(Connection *connection)>> callbacks_;
};

} // namespace ble

#endif // _BLE_CONNECTION_HXX_
