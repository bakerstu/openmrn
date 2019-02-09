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
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WString.h>
#include <vector>

#include "freertos_drivers/esp32/Esp32WiFiClientAdapter.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredTcpConnection.hxx"
#include "openlcb/SimpleStack.hxx"
#include "openlcb/TcpDefs.hxx"
#include "utils/macros.h"
#include "utils/SocketClientParams.hxx"
#include "utils/socket_listener.hxx"

using openlcb::Uint8ConfigEntry;
using openlcb::Uint16ConfigEntry;
using openlcb::StringConfigEntry;
using openlcb::TcpClientDefaultParams;
using openlcb::TcpClientConfig;
using openlcb::TcpDefs;

/// <map> of possible keys and descriptive values to show to the user for
/// the search_mode field.
static constexpr const char *BOOLEAN_MAP =
    "<relation><property>0</property><value>No</value></relation>"
    "<relation><property>1</property><value>Yes</value></relation>";

/// Visible name for the WiFi Power Savings mode.
static constexpr const char *WIFI_POWER_SAVE_NAME = "WiFi Power Savings Mode";

/// Visible description for the WiFi Power Savings mode.
static constexpr const char *WIFI_POWER_SAVE_DESC = 
    "When enabled this allows the ESP32 WiFi radio to use power savings mode "
    "which puts the radio to sleep except to receive beacon updates from the "
    "connected SSID. This should generally not need to be enabled unless you "
    "are powering the ESP32 from a battery.";

/// Visible name for the Hub Configuration group.
static constexpr const char *HUB_NAME = "Hub Configuration";

/// Visible description for the Hub Configuration group.
static constexpr const char *HUB_DESC = 
    "Configuration settings for an OpenLCB Hub";

/// Visible name for the hub_mode field.
static constexpr const char *HUB_MODE_NAME = "Enable Hub Mode";

/// Visible description for the hub_mode field.
static constexpr const char *HUB_MODE_DESC = 
    "Defines this node as a hub which can accept connections";

/// Visible name for the hub_listener_port field.
static constexpr const char *HUB_LISTENER_PORT_NAME =
    "Hub Listener Port";

/// Visible name for the hub_listener_port field.
static constexpr const char *HUB_LISTENER_PORT_DESC = 
    "Defines the TCP/IP listener port this node will use when operating as a "
    "hub. Most of the time this does not need to be changed.";

/// Visible name for the link_config group.
static constexpr const char *LINK_NAME = "Node Connect Configuration";

/// Visible name for the link_config group.
static constexpr const char *LINK_DESC =
    "Configures how this node will connect to other nodes.";

CDI_GROUP(HubConfiguration);
CDI_GROUP_ENTRY(enabled, Uint8ConfigEntry,
    Name(HUB_MODE_NAME), Description(HUB_MODE_DESC), Min(0), Max(1),
    Default(0), MapValues(BOOLEAN_MAP));
CDI_GROUP_ENTRY(port, Uint16ConfigEntry,
    Name(HUB_LISTENER_PORT_NAME),
    Description(HUB_LISTENER_PORT_DESC),
    Min(1), Max(65535), Default(TcpClientDefaultParams::DEFAULT_PORT))
CDI_GROUP_ENTRY(service_name, StringConfigEntry<48>,
    Name(TcpClientDefaultParams::SERVICE_NAME),
    Description(TcpClientDefaultParams::SERVICE_DESCR));
CDI_GROUP_END();

CDI_GROUP(WiFiConfiguration);
CDI_GROUP_ENTRY(wifi_sleep, Uint8ConfigEntry,
    Name(WIFI_POWER_SAVE_NAME), Description(WIFI_POWER_SAVE_DESC),
    Min(0), Max(1), Default(0), MapValues(BOOLEAN_MAP));
CDI_GROUP_ENTRY(hub_config, HubConfiguration,
    Name(HUB_NAME),
    Description(HUB_DESC));
CDI_GROUP_ENTRY(link_config, TcpClientConfig<TcpClientDefaultParams>,
    Name(LINK_NAME),
    Description(LINK_DESC));
CDI_GROUP_END();

class Esp32WiFiManager : public ConfigUpdateListener
{
public:
    /// Constructor.
    ///
    /// @param ssid is the WiFi AP to connect to.
    /// @param password is the password for the WiFi AP being connected to.
    /// @param hostname_prefix is the beginning part of the hostname that will
    /// be advertised via mDNS and to the WiFi AP.
    /// @param node_id is this node's unique ID, the last 32 bits are appended
    /// to the hostname_prefix to generate a unique hostname.
    /// @param cfg is the WiFiConfiguration instance used for this node. This
    /// will be monitored for changes and the WiFi behavior altered
    /// accordingly.
    Esp32WiFiManager(const char *ssid, const char *password,
        const char *hostname_prefix, OpenMRN *stack,
        const openlcb::NodeID node_id, const WiFiConfiguration &cfg);

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
    void startWiFiSystem();

    /// Starts the Esp32WiFiManager, this manages the WiFi subsystem as well as
    /// all interactions with other nodes.
    void startWiFiTask();

    /// Stops the Esp32WiFiManager, this will disable the WiFi subsystem and
    /// all related interactions with other nodes.
    void stopWiFiTask();

    /// Background task used by the Esp32WiFiManager to maintain health of any
    /// connections to other nodes.
    static void *wifi_manager_task(void *param);

    /// @return true if we are already connected to the provided IPAddress.
    /// @param hostname is the remote hostname to be checked.
    /// @param ip is the remote IPAddress to be checked.
    bool connectedTo(const String hostname, const IPAddress ip);

    /// @return true if the passed hostname is our preferred hub or if we have
    /// no preference.
    /// @param hostname is the hostname to be checked.
    bool isPreferredHub(const String hostname);

    /// Connects to a remote hub if we are not already connected to it.
    /// @param hubHostname is the hostname for the remote hub.
    /// @param ip is the remote hub ip address.
    /// @param remotePort is the port on the hub to connect to.
    /// @return true if the connection to the hub was successful or if already
    /// connected to that hub, false otherwise.
    bool connectToHub(String hubHostname, IPAddress ip, uint16_t remotePort);

    /// Attempts to connect to a manually configured hub if this node is
    /// configured to do so.
    /// @return true if we successfully connected to the hub or if we need
    /// to try mDNS.
    bool connectToManualHubIfNeeded();

    /// Attempts to connect to any mDNS identified hubs that we are not already
    /// connected to.
    /// @return true if we successfully connected to at least one hub, false
    /// otherwise.
    bool connectToMDNSHubs();

    /// Priority to use for the wifi_manager_task. This is currently set to 1
    /// which matches the arduino-esp32 loop() task priority.
    static constexpr UBaseType_t WIFI_TASK_PRIORITY = 1;

    /// Stack size for the wifi_manager_task.
    static constexpr uint32_t WIFI_TASK_STACK_SIZE = 2048L;

    /// Handle for the wifi_manager_task that manages the WiFi stack, including
    /// periodic health checks of the connected hubs or clients.
    TaskHandle_t wifiTaskHandle_;

    /// Map of connections to any hubs that are to be maintained.
    std::map<Esp32WiFiClientAdapter *, Executable *> hubConnections_;

    /// Map of connections from other nodes to be monitored.
    std::map<Esp32WiFiClientAdapter *, Executable *> clientConnections_;

    /// Internal flag to tell the wifi_manager_task to shutdown.
    bool wifiTaskShutdownReq_{false};

    /// Internal flag used by the wifi_manager_task to indicate it is running.
    bool wifiTaskRunning_{false};

    /// Dynamically generated hostname for this node.
    String hostname_;

    /// User provided SSID to connect to.
    const char *ssid_;

    /// User provided password for the SSID to connect to.
    const char *password_;

    /// Persistent configuration that will be used for this node's WiFi usage.
    const WiFiConfiguration cfg_;

    /// This is internally used to enable the management of the WiFi stack, in
    /// some environments this may be managed externally.
    bool manageWiFi_;

    /// OpenMRN stack for the Arduino system
    OpenMRN *openmrn_;

    /// Cached copy of this nodes wifi_sleep value. If this is true the WiFi
    /// radio can go to into low power mode to conserve power. This is likely
    /// only necessary for battery powered ESP32 devices.
    bool wifiSleepMode_{false};

    /// Cached copy of this nodes hub_config.enabled value, if this is
    /// true this node will advertise itself as a hub via mDNS and accept
    /// incoming TCP/IP connections.
    bool hubEnabled_{false};

    /// Cached copy of this nodes hub_config.port
    uint16_t hubPort_{TcpClientDefaultParams::DEFAULT_PORT};

    /// Cached copy of this nodes hub_config.service_name
    String hubServiceName_{TcpDefs::MDNS_SERVICE_NAME_HUB_TCP};

    /// Cached copy of this nodes hub_config.service_name split to not have
    /// the protocol.
    String hubServiceNameNoProtocol_{TcpDefs::MDNS_SERVICE_NAME_HUB};

    /// Cached copy of this nodes hub_config.service_name split to only be
    /// the protocol.
    String hubServiceProtocol_{TcpDefs::MDNS_PROTOCOL_TCP};

    /// Listener for this node when active as a hub.
    SocketListener *hubSocket_;

    /// Cached copy of this nodes link_config.search_mode
    SocketClientParams::SearchMode tcpClientMode_{
        SocketClientParams::SearchMode::AUTO_MANUAL};

    /// Cached copy of this nodes link_config.auto_address.service_name
    String tcpmDNSServiceName_{TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN};

    /// Cached copy of this nodes link_config.auto_address.service_name
    /// split to not have the protocol.
    String tcpmDNSServiceNameNoProtocol_{TcpDefs::MDNS_SERVICE_NAME_HUB};

    /// Cached copy of this nodes link_config.auto_address.service_name
    /// split to only be the protocol.
    String tcpmDNSServiceProtocol_{TcpDefs::MDNS_PROTOCOL_TCP};

    /// Cached copy of this nodes link_config.auto_address.host_name
    String tcpAutoHostName_{""};

    /// Cached copy of this nodes link_config.manual_address.ip_address
    String tcpManualHostName_{""};

    /// Cached copy of this nodes link_config.manual_address.port
    int tcpManualPort_{TcpClientDefaultParams::DEFAULT_PORT};

    /// cached copy of the IPAddress for this nodes
    /// link_config.manual_address.ip_address.
    IPAddress tcpManualIP_;

    /// Interval at which to perform mDNS scan for remote hubs
    static constexpr TickType_t MDNS_SCAN_INTERVAL = pdMS_TO_TICKS(30000);

    DISALLOW_COPY_AND_ASSIGN(Esp32WiFiManager);
};
#endif // _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_