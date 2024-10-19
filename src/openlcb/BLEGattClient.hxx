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
 * @file BLEGattClient.hxx
 *
 * Information about OpenLCB BLE Gatt Clients.
 *
 * @author Stuart Baker
 * @date 20 March 2024
 */

#ifndef _OPENLCB_BLEGATTCLIENT_HXX_
#define _OPENLCB_BLEGATTCLIENT_HXX_

#include <memory>

#include "ble/Connection.hxx"
#include "utils/Singleton.hxx"
#include "openlcb/BLEDefs.hxx"

namespace openlcb
{

/// Metadata for a BLE Gatt Client instance.
class BLEGattClient
{
public:
    /// Destructor.
    ~BLEGattClient()
    {
    }

    /// Get the out bound data characteristic handle.
    /// @return out bound data characteristic handle
    ble::Defs::AttHandle get_out_bound()
    {
        return outBound_;
    }

    /// Get the in bound data characteristic handle.
    /// @return in bound data characteristic handle
    ble::Defs::AttHandle get_in_bound()
    {
        return inBound_;
    }

    /// Get the client connection.
    /// @return client connection
    ble::Connection *connection()
    {
        return conn_;
    }

    /// Save a protocol engine into this object's ownership. The new protocol
    /// engine will be deleted when this client is disconnected.  One
    /// connection can set only one protocol engine. Any previous one will be
    /// deleted in this call.
    /// @param protocol the newly created protocol engine. Will take ownership
    /// and delete it via abandon_and_delete.
    void set_protocol_engine(BLEProtocolEnginePtr protocol)
    {
        protocol_ = std::move(protocol);
    }

    /// @return the protocol engine
    BLEProtocolEngine* protocol_engine() {
        return protocol_.get();
    }
    
private:
    /// Constructor.
    /// @param conn BLE device connection
    /// @param out_bound handle to out bound data characteristic
    /// @param in_bound handle to in bound data characteristic
    BLEGattClient(ble::Connection *conn, ble::Defs::AttHandle out_bound,
        ble::Defs::AttHandle in_bound)
        : conn_(conn)
        , outBound_(out_bound)
        , inBound_(in_bound)
    { }

    ble::Connection *conn_;         ///< BLE device connection
    ble::Defs::AttHandle outBound_; ///< handle to out bound data characteristic
    ble::Defs::AttHandle inBound_;  ///< handle to in bound data characteristic
    /// This object implements the protocol over this connection. It is used
    /// only for onwership and notifying it upon disconnect.
    BLEProtocolEnginePtr protocol_;

    /// Allow private access from BLEGattClients container object.
    friend class BLEGattClients;
};

/// Singleton container of all the BLE Gatt Clients.
class BLEGattClients : public Singleton<BLEGattClients>
{
public:
    /// Constructor.
    BLEGattClients()
    {
        ble::Connections::instance()->register_disconnect_callback(std::bind(
            &BLEGattClients::disconnect_callback, this, std::placeholders::_1));
    }

    /// Register an active client.
    /// @param conn BLE device connection
    /// @param out_bound handle to out bound data characteristic
    /// @param in_bound handle to in bound data characteristic
    /// @return newly created client instance. Ownership is retained, and this
    /// object will be deleted at an unspecified time.
    BLEGattClient *register_client(ble::Connection *conn,
        ble::Defs::AttHandle out_bound, ble::Defs::AttHandle in_bound)
    {
        auto *c =
            new BLEGattClient(conn, out_bound, in_bound);
        clients_.emplace_back(c);
        return c;
    }

    /// @return the first client for the given connection ID.
    BLEGattClient *find_client_by_out(
        ble::Connection *conn, ble::Defs::AttHandle out_handle)
    {
        for (auto it = clients_.begin(); it != clients_.end(); ++it)
        {
            if (it->get()->connection() == conn &&
                it->get()->get_out_bound() == out_handle)
            {
                return it->get();
            }
        }
        return nullptr;
    }

    /// @return the first client for the given connection ID.
    BLEGattClient *find_client_by_in(
        ble::Connection *conn, ble::Defs::AttHandle in_handle)
    {
        for (auto it = clients_.begin(); it != clients_.end(); ++it)
        {
            if (it->get()->connection() == conn &&
                it->get()->get_in_bound() == in_handle)
            {
                return it->get();
            }
        }
        return nullptr;
    }
    
    /// Invoke a callback method on each of the registered clients.
    /// @param callback callback to invoke.
    void for_each_call(std::function<void(BLEGattClient *)> callback)
    {
        for (auto it = clients_.begin(); it != clients_.end(); ++it)
        {
            callback(it->get());
        }
    }

private:
    /// Callback for when a connection disconnect occurs.
    /// @param conn BLE device connection
    void disconnect_callback(ble::Connection *conn)
    {
        /// Remove any clients using the given connection from the container.
        for (int i = 0; i < (int)clients_.size(); ++i)
        {
            if (clients_[i]->conn_ == conn)
            {
                clients_.erase(clients_.begin() + i);
                --i;
                continue;
            }
        }
    }

    /// All the BLE Gatt clients managed by the container.
    std::vector<std::unique_ptr<BLEGattClient>> clients_;
};

} // namespace openlcb

#endif // _OPENLCB_BLEGATTCLIENT_HXX_
