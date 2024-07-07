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

#include "ble/Connection.hxx"
#include "utils/Singleton.hxx"

namespace openlcb
{

// Forward declaration.
class BLEGattClients;

/// Metadata for a BLE Gatt Client instance.
class BLEGattClient
{
public:
    /// Destructor.
    ~BLEGattClient()
    { }

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

private:
    /// Constructor.
    /// @param conn BLE device connection
    /// @param out_bound handle to out bound data characteristic
    /// @param in_bound handle to in bound data characteristic
    BLEGattClient(ble::Connection *conn, ble::Defs::AttHandle out_bound,
        ble::Defs::AttHandle in_bound, ByteDirectHubInterface *hub,
        Service *service)
        : conn_(conn)
        , outBound_(out_bound)
        , inBound_(in_bound)
        , port_(new BLEGattCHubPort(hub, service, this))
    { }

    ~BLEGattClient() {
        /// @TODO: destroy hub port properly.
    }
    
    class BLEGattCHubPort : public BLEHubPort
    {
    public:
        BLEGattCHubPort(ByteDirectHubInterface *hub, BLEGattClient *client,
            Service *service)
            : BLEHubPort(hub,
                  std::unique_ptr<MessageSegmenter>(
                      create_gc_message_segmenter()),
                  service)
            , client_(client)
        { }

        void do_send(const uint8_t *data, size_t len) override
        {
            esp_err_t result = esp_ble_gattc_write_char(
                parent_->gattcProfileTable_[PROFILE_OLCB].interface_,
                client->connection()->get_handle(), client->get_out_bound(),
                200, data, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            ESP_LOGI("GATTC_DEMO", "write char status: 0x%x", result);
        }

    private:
        /// Reference to the client object holding the various handles.
        BLEGattClient *client_;
    };

    ble::Connection *conn_;         ///< BLE device connection
    ble::Defs::AttHandle outBound_; ///< handle to out bound data characteristic
    ble::Defs::AttHandle inBound_;  ///< handle to in bound data characteristic
    std::unique_ptr<BLEHubPort> port_; ///< Direct hub I/O object.

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

    void set_stack(ByteDirectHubInterface *hub, Service *send_service)
    {
        hub_ = hub;
        sendService_ = send_service;
    }

    /// Register an active client.
    /// @param conn BLE device connection
    /// @param out_bound handle to out bound data characteristic
    /// @param in_bound handle to in bound data characteristic
    /// @return newly created client instance. Ownership is retained.
    BLEGattClient *register_client(ble::Connection *conn,
        ble::Defs::AttHandle out_bound, ble::Defs::AttHandle in_bound)
    {
        auto *c =
            new BLEGattClient(conn, out_bound, in_bound, hub, sendService_);
        clients_.emplace_back(c);
    }

    /// Invoke a callback method on each of the registered clients.
    /// @param callback callback to invoke.
    void for_each_call(std::function<void(BLEGattClient *)> callback)
    {
        for (auto it = clients_.begin(); it != clients_.end(); ++it)
        {
            callback(&(*it));
        }
    }

private:
    /// Callback for when a connection disconnect occurs.
    /// @param conn BLE device connection
    void disconnect_callback(ble::Connection *conn)
    {
        /// Remove any clients using the given connection from the container.
        for (auto it = clients_.begin(); it != clients_.end(); ++it)
        {
            if (it->conn_ == conn)
            {
                clients_.erase(it);
            }
        }
    }

    /// All the BLE Gatt clients managed by the container.
    std::vector<std::unique_ptr<BLEGattClient>> clients_;
    /// Direct hub to interface with for traffic. Externally owned.
    ByteDirectHubInterface *hub_;
    /// This executor willbe used by the hub ports to perform the send calls on
    /// the BLE stack.
    Service *sendService_;
};

} // namespace openlcb

#endif // _OPENLCB_BLEGATTCLIENT_HXX_
