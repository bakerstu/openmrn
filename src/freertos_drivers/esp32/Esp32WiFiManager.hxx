/** \copyright
 * Copyright (c) 2019, Mike Dunston
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
 * \file Esp32WiFiManager.hxx
 *
 * ESP32 WiFi Manager
 *
 * @author Mike Dunston
 * @date 4 February 2019
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_

#include <Arduino.h>
#include <IPAddress.h>
#include <map>

#include "freertos_drivers/esp32/Esp32WiFiClientAdapter.hxx"
#include "freertos_drivers/esp32/Esp32WiFiConfiguration.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredTcpConnection.hxx"
#include "openlcb/SimpleStack.hxx"
#include "openlcb/TcpDefs.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/macros.h"
#include "utils/SocketClientParams.hxx"
#include "utils/GcTcpHub.hxx"

class Esp32WiFiManager : public DefaultConfigUpdateListener
{
public:
    /// Constructor.
    ///
    /// With this constructor the ESP32 WiFi and MDNS systems will be managed
    /// automatically by the Esp32WiFiManager class in addition to the inbound
    /// and outbound connections. The WiFi and MDNS systems will only be
    /// started after the initial loading of the CDI which occurs only after
    /// the application code calls OpenMRN::begin().
    ///
    /// @param ssid is the WiFi AP to connect to.
    /// @param password is the password for the WiFi AP being connected to.
    /// @param openmrn is the OpenMRN class instance used by the node.
    /// @param cfg is the WiFiConfiguration instance used for this node. This
    /// will be monitored for changes and the WiFi behavior altered
    /// accordingly.
    Esp32WiFiManager(const char *ssid, const char *password,
        OpenMRN *openmrn, const WiFiConfiguration &cfg);

    /// Constructor.
    ///
    /// With this constructor the ESP32 WiFi and MDNS systems will not be
    /// managed by the Esp32WiFiManager class, only the inbound and outbound
    /// connections will be managed. This variation should only be used when
    /// the application code starts the the WiFi and MDNS systems before
    /// calling OpenMRN::begin().
    ///
    /// @param openmrn is the OpenMRN class instance used by the node.
    /// @param cfg is the WiFiConfiguration instance used for this node. This
    /// will be monitored for changes and the WiFi behavior altered
    /// accordingly.
    Esp32WiFiManager(OpenMRN *openmrn, const WiFiConfiguration &cfg);

    /// Updates the WiFiConfiguration settings used by this node.
    ///
    /// @param fd is the file descriptor used for the configuration settings.
    /// @param initial_load is set to true when this node loads the
    /// configuration for the first time, otherwise it is an update to the
    /// configuration and may require a restart.
    /// @param done is the control used by the caller to track when all config
    /// consumers have completed their updates.
    ///
    /// @return UPDATED when the configuration has been successfully updated,
    /// or REBOOT_NEEDED if the node needs to reboot for configuration to take
    /// effect.
    ConfigUpdateListener::UpdateAction apply_configuration(int fd,
        bool initial_load, BarrierNotifiable *done) OVERRIDE;
    void factory_reset(int fd) OVERRIDE;

private:
    /// Default constructor.
    Esp32WiFiManager();

    /// Starts the WiFi system and initiates the SSID connection process.
    void start_wifi_system();

    /// Starts the Esp32WiFiManager, this manages the WiFi subsystem as well as
    /// all interactions with other nodes.
    void start_wifi_task();

    /// Background task used by the Esp32WiFiManager to maintain health of any
    /// connections to other nodes.
    /// @param param is a pointer to the Esp32WiFiManager instance.
    static void *wifi_manager_task(void *param);

    /// @return true if we are already connected to the provided IPAddress.
    /// @param hostname is the remote hostname to be checked.
    /// @param ip is the remote IPAddress to be checked.
    bool is_connected_to(const IPAddress ip, const uint16_t port);

    /// Connects to a remote hub if we are not already connected to it.
    /// @param ip is the remote hub ip address.
    /// @param port is the port on the hub to connect to.
    /// @return true if the connection to the hub was successful or if already
    /// connected to that hub, false otherwise.
    bool connect_to_hub(const IPAddress ip, const uint16_t port);

    /// Attempts to connect to an mDNS discovered hub.
    /// @param preferred_hub_hostname is the hostname of the hub this node
    /// should prefer if found, can be blank.
    /// @param serviceName is the mDNS service name to search for.
    /// @param serviceProtocol is the mDNS service protocol to search for.
    /// @return true if we successfully connected to a hub, false otherwise.
    bool connect_to_mdns_hub(const std::string &preferred_hub_hostname,
        const std::string &serviceName, const std::string &serviceProtocol);

    /// Performs an orderly shutdown of the hub on this node, any connections
    /// will be closed and removed from the stack.
    /// @param serviceName is the previously advertised mDNS service name.
    /// @param serviceProtocol is the previously advertised mDNS service
    /// protocol.
    void shutdown_hub(const std::string &serviceName,
        const std::string &serviceProtocol);

    /// Starts @ref SocketListener for the hub on this node and advertises the
    /// provided mDNS service name details.
    /// @param hub_port is the port to start the GcTcpHub on.
    /// @param serviceName is the mDNS service name to advertise.
    /// @param serviceProtocol is the mDNS service protocol to advertise.
    void start_hub(const uint16_t hub_port, const std::string &serviceName,
        const std::string &serviceProtocol);

    /// Performs orderly shutdown of any outbound connections from this node.
    void disconnect_from_hubs();

    /// Handle for the wifi_manager_task that manages the WiFi stack, including
    /// periodic health checks of the connected hubs or clients.
    TaskHandle_t wifiTaskHandle_;

    /// Map of connections to any hubs that are to be maintained.
    std::map<Esp32WiFiClientAdapter *, Executable *> hubConnections_;

    /// Dynamically generated hostname for this node, esp32_{node-id}.
    std::string hostname_{"esp32_"};

    /// User provided SSID to connect to.
    const char *ssid_;

    /// User provided password for the SSID to connect to.
    const char *password_;

    /// Persistent configuration that will be used for this node's WiFi usage.
    const WiFiConfiguration cfg_;

    /// This is internally used to enable the management of the WiFi stack, in
    /// some environments this may be managed externally.
    const bool manageWiFi_;

    /// OpenMRN stack for the Arduino system
    OpenMRN *openmrn_;

    /// Cached copy of the file descriptor passed into apply_configuration.
    /// This is internally used by the wifi_manager_task to processed deferred
    /// configuration load.
    int configFd_{-1};

    /// Calculated MD5 of cfg_ data. Used to detect changes in configuration
    /// which may require the wifi_manager_task to reload config.
    std::string configMD5_{""};

    /// Internal flag to request the wifi_manager_task reload configuration.
    bool configReloadRequested_{true};

    /// @ref GcTcpHub for this node's hub if enabled.
    std::unique_ptr<GcTcpHub> hub_;

    DISALLOW_COPY_AND_ASSIGN(Esp32WiFiManager);
};
#endif // _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_