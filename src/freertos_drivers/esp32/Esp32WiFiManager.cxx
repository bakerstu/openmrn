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
 * \file Esp32WiFiManager.cxx
 *
 * ESP32 WiFi Manager
 *
 * @author Mike Dunston
 * @date 4 February 2019
 */

#include "freertos_drivers/esp32/Esp32WiFiManager.hxx"
#include "OpenMRN.h"
#include "openlcb/ConfiguredTcpConnection.hxx"
#include "openlcb/TcpDefs.hxx"

#include <ESPmDNS.h>
#include <MD5Builder.h>
#include <WiFi.h>

using openlcb::NodeID;
using openlcb::TcpAutoAddress;
using openlcb::TcpClientConfig;
using openlcb::TcpClientDefaultParams;
using openlcb::TcpDefs;
using openlcb::TcpManualAddress;
using std::string;

// Enable select support on the ESP32 since it supports the usage of select for
// tcp/ip sockets.
// TODO: uncomment once OSSelectWakeup is confirmed working
//OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);

/// String values for wifi_status_t values, WL_NO_SHIELD has been intentionally
/// omitted as its value is 255 and is explicitly checked for.
static constexpr char const *WIFI_STATUS[] =
{
    "WiFi Idle",            // WL_IDLE_STATUS
    "SSID not found",       // WL_NO_SSID_AVAIL
    "SSID scan completed",  // WL_SCAN_COMPLETED
    "WiFi connected",       // WL_CONNECTED
    "SSID connect failed",  // WL_CONNECT_FAILED
    "WiFi connection lost", // WL_CONNECTION_LOST
    "WiFi disconnected"     // WL_DISCONNECTED
};

/// String values for openlcb::SocketClientParams::SearchMode, used when
/// logging the configuration used by the Esp32WiFiManager.
static constexpr const char *CLIENT_SEARCH_MODE_STRINGS[] =
{
    "auto, manual",         // AUTO_MANUAL
    "manual, auto",         // MANUAL_AUTO
    "auto only",            // AUTO_ONLY
    "manual only"           // MANUAL_ONLY
};

/// String values used for boolean flags in the CDI, used when logging the
/// configuration used by the Esp32WiFiManager.
static constexpr const char *BOOLEAN_DISPLAY_STRINGS[] =
{
    "No",
    "Yes"
};

/// Priority to use for the wifi_manager_task. This is currently set to
/// one priority level higher than the arduino-esp32 loopTask. The task
/// will primarily be in a sleep state so there will be limited impact on
/// the loopTask.
static constexpr UBaseType_t WIFI_TASK_PRIORITY = 2;

/// Stack size for the wifi_manager_task.
static constexpr uint32_t WIFI_TASK_STACK_SIZE = 2560L;

/// Interval at which to all TCP/IP connections and establish new outbound
/// connections if required.
static constexpr TickType_t CONNECTION_CHECK_TICK_INTERVAL = pdMS_TO_TICKS(30000);

/// Interval at which to check if the GcTcpHub has started or not.
static constexpr TickType_t HUB_STARTUP_DELAY = pdMS_TO_TICKS(50); 

// With this constructor being used the Esp32WiFiManager will manage the
// WiFi connection, mDNS system and the hostname of the ESP32.
Esp32WiFiManager::Esp32WiFiManager(const char *ssid, const char *password,
    OpenMRN *openmrn, const WiFiConfiguration &cfg) :
    DefaultConfigUpdateListener(), ssid_(ssid), password_(password),
    cfg_(cfg), manageWiFi_(true), openmrn_(openmrn)
{
    // Extend the capacity of the hostname to make space for the node-id and
    // underscore.
    hostname_.reserve(TCPIP_HOSTNAME_MAX_SIZE);

    // Generate the hostname for the ESP32 based on the provided node id.
    // node_id : 0x050101011425
    // hostname_ : esp32_050101011425
    NodeID node_id = openmrn_->stack()->node()->node_id();
    hostname_.append(uint64_to_string_hex(node_id, 0));

    // Release any extra capacity allocated for the hostname.
    hostname_.shrink_to_fit();

    // The maximum length hostname for the ESP32 is 32 characters so truncate
    // when necessary.
    // ref https://github.com/espressif/esp-idf/blob/master/components/tcpip_adapter/include/tcpip_adapter.h#L611
    if (hostname_.length() > TCPIP_HOSTNAME_MAX_SIZE)
    {
        LOG(WARNING, "ESP32 hostname is too long, original hostname: %s",
            hostname_.c_str());
        hostname_.resize(TCPIP_HOSTNAME_MAX_SIZE);
        LOG(WARNING, "truncated hostname: %s", hostname_.c_str());
    }
}

// With this constructor being used, it will be the responsibility of the
// application to manage the WiFi and mDNS systems.
Esp32WiFiManager::Esp32WiFiManager(OpenMRN *openmrn,
    const WiFiConfiguration &cfg) : DefaultConfigUpdateListener(), cfg_(cfg),
    manageWiFi_(false), openmrn_(openmrn)
{
    // Nothing to do here.
}


ConfigUpdateListener::UpdateAction Esp32WiFiManager::apply_configuration(
    int fd, bool initial_load, BarrierNotifiable *done)
{
    AutoNotify n(done);
    LOG(VERBOSE, "Esp32WiFiManager::apply_configuration(%d, %d)", fd,
        initial_load);

    // Cache the fd for later use by the wifi background task.
    configFd_ = fd;
    configReloadRequested_ = initial_load;

    // Load the CDI entry into memory to do an MD5 check against our last
    // loaded configuration so we can avoid reloading configuration when there
    // are no interesting changes. MD5Builder from the arduino-esp32 project is
    // used for a relatively lightweight hash of the CDI entry.
    MD5Builder md5;
    std::unique_ptr<uint8_t[]> md5buf(new uint8_t[cfg_.size()]);

    // If we are unable to seek to the right position in the persistent storage
    // give up and request a reboot.
    if(lseek(fd, cfg_.offset(), SEEK_SET) != cfg_.offset())
    {
        LOG(WARNING, "lseek failed to reset fd offset, REBOOT_NEEDED");
        return ConfigUpdateListener::UpdateAction::REBOOT_NEEDED;
    }

    // If we are unable to read the full configuration from persistent storage
    // give up and request a reboot.
    if(read(fd, md5buf.get(), cfg_.size()) != cfg_.size())
    {
        LOG(WARNING, "read failed to fully read the config, REBOOT_NEEDED");
        return ConfigUpdateListener::UpdateAction::REBOOT_NEEDED;
    }

    // Calculate MD5 from the loaded buffer.
    md5.begin();
    md5.add(md5buf.get(), cfg_.size());
    md5.calculate();
    string configMD5 = md5.toString().c_str();
    LOG(VERBOSE, "existing config MD5: \"%s\", new MD5: \"%s\"",
        configMD5_.c_str(), configMD5.c_str());

    // if this is not the initial loading of the CDI entry check the MD5 value.
    if(!initial_load)
    {
        // Check the MD5 against our last known MD5, it doesn't matter which is
        // different, but that they are different as part of setting
        // configReloadRequested_.
        configReloadRequested_ = configMD5_.compare(configMD5);
        LOG(VERBOSE, "Config Change detected: %s",
            BOOLEAN_DISPLAY_STRINGS[configReloadRequested_]);

        // If a configuration change has been detected, wake up the
        // wifi_manager_task so it can consume the change prior to the next
        // wake up interval.
        if(configReloadRequested_)
        {
            xTaskNotifyGive(wifiTaskHandle_);
        }
    }
    else
    {
        // This is the initial loading of the CDI entry, start the background
        // task that will manage the node's WiFi connection(s).
        start_wifi_task();
    }

    // Store the calculated MD5 for future use when the apply_configuration
    // method is called to detect any configuration changes.
    configMD5_ = std::move(configMD5);

    // Inform the caller that the configuration has been updated as the wifi
    // task will reload the configuration as part of it's next wake up cycle.
    return ConfigUpdateListener::UpdateAction::UPDATED;
}

// Factory reset handler for the WiFiConfiguration CDI entry.
void Esp32WiFiManager::factory_reset(int fd)
{
    LOG(VERBOSE, "Esp32WiFiManager::factory_reset(%d)", fd);

    // General WiFi configuration settings.
    CDI_FACTORY_RESET(cfg_.wifi_sleep);

    // Hub specific configuration settings.
    CDI_FACTORY_RESET(cfg_.hub_config().enabled);
    CDI_FACTORY_RESET(cfg_.hub_config().port);
    cfg_.hub_config().service_name().write(fd,
        TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);

    // Node link configuration settings.
    CDI_FACTORY_RESET(cfg_.link_config().search_mode);
    CDI_FACTORY_RESET(cfg_.link_config().reconnect);

    // Node link manual configuration settings.
    cfg_.link_config().manual_address().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.link_config().manual_address().port);

    // Node link automatic configuration settings.
    cfg_.link_config().auto_address().service_name().write(fd,
        TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);
    cfg_.link_config().auto_address().host_name().write(fd, "");

    // Node link automatic last connected node address.
    cfg_.link_config().last_address().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.link_config().last_address().port);

    // Reconnect to last connected node.
    CDI_FACTORY_RESET(cfg_.link_config().reconnect);
}

// If the Esp32WiFiManager is setup to manage the WiFi system, the following
// steps are executed:
// 1) Set the WiFi mode to STATION (WIFI_STA)
// 2) Shutdown the WiFi system, this helps improve reliability in connecting
// to the configured SSID.
// 3) Enable the automatic reconnect to the SSID, this is managed internally
// by the WiFi library provided with arduino-esp32. The ESP-IDF WiFi APIs do
// not support this by default.
// 4) Set the WiFi hostname based on the generated hostname.
// 5) Begin the SSID connection process
// 6) Verify successful SSID connection, if no connection was established
// trigger an abort with FATAL log message.
// 7) Start the mDNS system using the generated hostname.
void Esp32WiFiManager::start_wifi_system()
{
    // If we do not need to manage the WiFi / mDNS systems exit early.
    if(!manageWiFi_)
    {
        // Verify that the WiFi driver is not set to OFF (WIFI_NONE).
        HASSERT(WiFi.getMode() != WIFI_MODE_NULL);

        // Verify that the WiFi driver is already connected.
        HASSERT(WiFi.isConnected());

        // Unforutnately there is no way to verify mDNS has been started by
        // the application so it is not verified, all calls will likely fail
        // at runtime though.
        return;
    }

    // Set the WiFI mode to STATION since we will connect to the SSID. This
    // is also required to allow configuration of the ESP32 prior to calling
    // WiFi.begin().
    WiFi.mode(WIFI_STA);

    // This next line tells the WiFi system to shutdown, this is needed to
    // ensure we have a higher success rate of connecting to the SSID.
    WiFi.disconnect(true);

    // Configure the WiFi system to automatically reconnect to the SSID if we
    // get disconnected for any reason.
    WiFi.setAutoReconnect(true);

    // Set the generated hostname prior to connecting to the SSID so that it
    // shows up with the generated hostname instead of the default "Espressif".
    LOG(INFO, "Setting ESP32 hostname: \"%s\"", hostname_.c_str());
    WiFi.setHostname(hostname_.c_str());

    // Attempt to connect to the SSID, this will block until the ESP32 starts
    // the connection process, note it may not have an IP address immediately
    // thus the need to check the connection result a few times before giving
    // up with a FATAL error.
    LOG(INFO, "Connecting to SSID: \"%s\"", ssid_);
    if (WiFi.begin(ssid_, password_) != WL_CONNECT_FAILED)
    {
        // Allow up to 10 checks to see if we have connected to the SSID.
        static constexpr uint8_t attempts = 10;
        uint8_t attempt = 0;

        // This call waits up to 10sec for a result before timing out so it
        // needs to be called a few times until we get a final result.
        uint8_t wifiStatus = WiFi.waitForConnectResult();
        while (wifiStatus != WL_CONNECTED &&       // successfully connected
                wifiStatus != WL_NO_SSID_AVAIL &&  // SSID not found
                wifiStatus != WL_CONNECT_FAILED && // generic failure
                ++attempt <= attempts)
        {
            esp_task_wdt_reset();
            LOG(WARNING, "[%d/%d] WiFi not connected yet, status: %d (%s)",
                attempt, attempts, wifiStatus, WIFI_STATUS[wifiStatus]);
            wifiStatus = WiFi.waitForConnectResult();
        }
    }

    // Verify if we were successful in connecting to the SSID and obtaining
    // a valid IP address via DHCP. Note the usage of FATAL will trigger the
    // ESP32 to enter a failure state and reboot.
    if (WiFi.status() != WL_CONNECTED)
    {
        LOG(FATAL, "SSID connection failed: %s", WIFI_STATUS[WiFi.status()]);
    }

    // The ESP32 successfully connected to the SSID and has received an IP
    // address via DHCP.
    LOG(INFO, "Connected to SSID: %s, Node IP: %s", ssid_,
        WiFi.localIP().toString().c_str());

    // Start the mDNS system with our generated hostname so it can be resolved
    // by other nodes.
    MDNS.begin(hostname_.c_str());
}

// Starts a background task for the Esp32WiFiManager.
void Esp32WiFiManager::start_wifi_task()
{
    // TBD: should this use xCreateTaskPinnedToCore instead? There appears to
    // be a crash in the WiFi stack when CORE_DEBUG_LEVEL is set higher than
    // ARDUHAL_LOG_LEVEL_INFO. Default for CORE_DEBUG_LEVEL is
    // ARDUHAL_LOG_LEVEL_NONE so it doesn't show up under the typical use case.
    os_thread_create(&wifiTaskHandle_, "OpenMRN-WiFiMgr", WIFI_TASK_PRIORITY,
        WIFI_TASK_STACK_SIZE, wifi_manager_task, this);
}

// Checks if there is an active connection to the ip/port.
bool Esp32WiFiManager::is_connected_to(const IPAddress ip, const uint16_t port)
{
    // Check if the passed ip is this node, port doesn't matter.
    if (WiFi.localIP() == ip)
    {
        // The ip is for this node, we don't need to connect to it.
        return true;
    }

    // Provided ip is not for this node, check if we have an active connection
    // already being tracked using the provided port.
    for (auto const &pair : hubConnections_)
    {
        if (pair.first->remoteIP() == ip && pair.first->remotePort() == port)
        {
            return true;
        }
    }

    // We don't already have a connection so it is safe to connect to it.
    return false;
}

// attempts to connect to a remote hub.
bool Esp32WiFiManager::connect_to_hub(const IPAddress ip, const uint16_t port)
{
    if (is_connected_to(ip, port))
    {
        LOG(WARNING, "[CLIENT] Already connected to %s:%d, skipping.",
            ip.toString().c_str(), port);
        return true;
    }

    // Try and connect the provided ip and port as we are not currently
    // connected to it.
    WiFiClient remoteNode;
    LOG(INFO, "[CLIENT] Connecting to %s:%d...", ip.toString().c_str(), port);
    if (!remoteNode.connect(ip, port))
    {
        LOG(WARNING, "[CLIENT] Failed to connect to %s:%d!",
            ip.toString().c_str(), port);
        return false;
    }
    LOG(INFO, "[CLIENT] Successfully connected to %s:%d!",
        ip.toString().c_str(), port);
    Esp32WiFiClientAdapter *adapter = new Esp32WiFiClientAdapter(remoteNode);

    // Add the new connction to the stack and record the returned excutor so
    // we can clean it up later when needed.
    hubConnections_.insert(std::make_pair(adapter,
        openmrn_->add_gridconnect_port(adapter)));

    return true;
}

// Attempts to connect to an mDNS discovered hub. If a successful connection
// is made it is recorded in the CDI entry under
// WiFiConfiguration.link_config.last_address.
bool Esp32WiFiManager::connect_to_mdns_hub(const string &preferred_hub_hostname,
    const string &serviceName, const string &serviceProtocol)
{
    bool connected = false;
    LOG(INFO, "[CLIENT] Scanning for mDNS service \"%s.%s\"",
        serviceName.c_str(), serviceProtocol.c_str());
    // Note: this call will block for up to 3sec to collect results.
    int entries = MDNS.queryService(serviceName.c_str(),
        serviceProtocol.c_str());
    for (int index = 0; index < entries && !connected; index++)
    {
        // If we have a preferred hub hostname check if this match is for it.
        if (preferred_hub_hostname.length() > 0 &&
            preferred_hub_hostname.compare(MDNS.hostname(index).c_str()) != 0)
        {
            LOG(INFO, "[CLIENT] Discovered \"%s\" but it is not our "
                "preferred hub \"%s\".",
                MDNS.hostname(index).c_str(),
                preferred_hub_hostname.c_str());
            continue;
        }

        // Attempt to connect to the remote hub.
        if (connect_to_hub(MDNS.IP(index), MDNS.port(index)))
        {
            // store our last connected host and port
            cfg_.link_config().last_address().ip_address().write(configFd_,
                MDNS.IP(index).toString().c_str());
            cfg_.link_config().last_address().port().write(configFd_,
                MDNS.port(index));
            connected = true;
        }
    }
    return connected;
}

// Shuts down the GcTcpHub (if started). No attempt is made at this time to
// disconnect any clients from the hub.
void Esp32WiFiManager::shutdown_hub(const string &serviceName,
    const string &serviceProtocol)
{
    // If we have a hub socket we need to clean up any allocated resources.
    if (hub_)
    {
        LOG(INFO, "[HUB] Removing mDNS advertisement %s.%s",
            serviceName.c_str(), serviceProtocol.c_str());

        // Note: not using ESPmDNS as it does not have a method for
        // deregistering an advertised service name.
        mdns_service_remove(serviceName.c_str(), serviceProtocol.c_str());
    }

    // cleanup the configured hub socket listener.
    hub_.reset(nullptr);
}

// Starts a GcTcpHub and advertises the configured mDNS service name.
void Esp32WiFiManager::start_hub(const uint16_t hub_port,
    const string &serviceName, const string &serviceProtocol)
{
    LOG(INFO, "[HUB] Starting TCP/IP listener on port %d", hub_port);
    hub_.reset(new GcTcpHub(openmrn_->stack()->can_hub(), hub_port));

    // wait for the hub to complete it's startup tasks
    while(!hub_->is_started())
    {
        vTaskDelay(HUB_STARTUP_DELAY);
    }

    LOG(INFO, "[HUB] Advertising %s.%s:%d via mDNS",
        serviceName.c_str(), serviceProtocol.c_str(), hub_port);
    // Advertise this node with the mDNS service name so other nodes can find
    // it via queryService.
    MDNS.addService(serviceName.c_str(), serviceProtocol.c_str(), hub_port);
}

// disconnects any outbound connections to other hubs.
void Esp32WiFiManager::disconnect_from_hubs()
{
    for (const auto &pair : hubConnections_)
    {
        LOG(INFO, "[CLIENT] Disconnecting from hub: %s:%d",
            pair.first->remoteIP().toString().c_str(),
            pair.first->remotePort());
        openmrn_->remove_port(pair.second);
        pair.first->stop();
        delete pair.first;
    }
    hubConnections_.clear();
}

// macro for splitting a service name since the ESPmDNS library requires the
// service name and service protocol to be split.
#define SPLIT_MDNS_SERVICE_NAME(service_name, service_protocol) \
    if(service_name.length() && service_name.find('.', 0) != string::npos) \
    { \
        string::size_type split_loc = service_name.find('.', 0); \
        service_protocol = service_name.substr(split_loc + 1); \
        service_name.resize(split_loc); \
    }

// Background task for the Esp32WiFiManager. This handles all outbound
// connection attempts, configuration loading and making this node as a hub.
void *Esp32WiFiManager::wifi_manager_task(void *param)
{
    Esp32WiFiManager *wifi = static_cast<Esp32WiFiManager *>(param);

    // Start the WiFi system before proceeding with remaining tasks.
    wifi->start_wifi_system();

    // Cache of the currently running configuration, these will be updated
    // if/when configReloadRequested_ is set to true by the apply_configuration
    // method.
    bool hub_enabled = false;
    string hub_service_name = TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN;
    string hub_service_protocol = TcpDefs::MDNS_PROTOCOL_TCP;
    uint16_t hub_port = TcpClientDefaultParams::DEFAULT_PORT;
    bool reconnect_to_last_hub = true;
    IPAddress last_hub_ip = INADDR_NONE;
    uint16_t last_hub_port = TcpClientDefaultParams::DEFAULT_PORT;
    SocketClientParams::SearchMode client_mode =
        SocketClientParams::SearchMode::AUTO_MANUAL;
    string tcp_service_name = TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN;
    string tcp_service_protocol = TcpDefs::MDNS_PROTOCOL_TCP;
    string auto_hub_hostname = "";
    IPAddress manual_hub_ip = INADDR_NONE;
    string manual_hub_host = "";
    uint16_t manual_hub_port = TcpClientDefaultParams::DEFAULT_PORT;

    // Helpers for the nested CDI entries.
    const HubConfiguration &hub = wifi->cfg_.hub_config();
    const TcpClientConfig<TcpClientDefaultParams> &link_cfg =
        wifi->cfg_.link_config();
    const TcpManualAddress<TcpClientDefaultParams> &manaddr =
        link_cfg.manual_address();
    const TcpAutoAddress<TcpClientDefaultParams> &autoaddr =
        link_cfg.auto_address();
    const TcpManualAddress<TcpClientDefaultParams> &last_hub =
        link_cfg.last_address();

    while (true)
    {
        // check if there are configuration changes to pick up.
        if(wifi->configReloadRequested_)
        {
            // Since the config has changed since we started, cleanup any
            // existing connections before loading the new config.
            wifi->shutdown_hub(hub_service_name, hub_service_protocol);
            wifi->disconnect_from_hubs();

            // Load the new configuration so we can start using it.
            hub_enabled = CDI_READ_TRIMMED(hub.enabled, wifi->configFd_);
            hub_service_name = hub.service_name().read(wifi->configFd_);
            hub_service_protocol = TcpDefs::MDNS_PROTOCOL_TCP;
            hub_port = CDI_READ_TRIMMED(hub.port, wifi->configFd_);
            SPLIT_MDNS_SERVICE_NAME(hub_service_name, hub_service_protocol)

            auto_hub_hostname = autoaddr.host_name().read(wifi->configFd_);
            tcp_service_name = autoaddr.service_name().read(wifi->configFd_);
            tcp_service_protocol = TcpDefs::MDNS_PROTOCOL_TCP;
            SPLIT_MDNS_SERVICE_NAME(tcp_service_name, tcp_service_protocol)

            client_mode = (SocketClientParams::SearchMode)CDI_READ_TRIMMED(
                link_cfg.search_mode, wifi->configFd_);
            
            manual_hub_host = manaddr.ip_address().read(wifi->configFd_);
            manual_hub_port = CDI_READ_TRIMMED(manaddr.port, wifi->configFd_);

            reconnect_to_last_hub = CDI_READ_TRIMMED(link_cfg.reconnect,
                wifi->configFd_);
            string last_hub_ip_str =
                last_hub.ip_address().read(wifi->configFd_);
            last_hub_port = CDI_READ_TRIMMED(last_hub.port, wifi->configFd_);
            // if we have a last hub address convert it to an IP address, if
            // not leave as default so it doesn't get used.
            last_hub_ip = INADDR_NONE;
            if(last_hub_ip_str.length() > 0)
            {
                last_hub_ip.fromString(last_hub_ip_str.c_str());
            }

            // Verify the updated configuration to make sure it is valid.
            if (client_mode != SocketClientParams::SearchMode::AUTO_ONLY)
            {
                // Reset to default so we can resolve the hostname to an IP
                // if/when we need it.
                manual_hub_ip = INADDR_NONE;

                // If the manual hostname is not provided (or is blank) or we
                // do not have a valid manual port force settings to AUTO_ONLY.
                if (manual_hub_host == "" || manual_hub_port == 0)
                {
                    LOG(WARNING, "Invalid configuration detected, missing the "
                        "manual hub hostname or port, switching to AUTO_ONLY "
                        "mode.");
                    client_mode = SocketClientParams::SearchMode::AUTO_ONLY;
                }
            }

            // This can make the wifi much more responsive. This is disabled by
            // default and should only be enabled if the node is plugged in to
            // a regulated power supply (not a battery).
            WiFi.setSleep(CDI_READ_TRIMMED(wifi->cfg_.wifi_sleep,
                wifi->configFd_));

            LOG(INFO, "Esp32WiFiManager Configuration:");
            LOG(INFO, "Hub (enabled:%s, port: %d, mDNS: \"%s.%s\")",
                BOOLEAN_DISPLAY_STRINGS[hub_enabled], hub_port,
                hub_service_name.c_str(), hub_service_protocol.c_str());
            LOG(INFO, "Client (search-mode: %s, reconnect: %s, last-hub: "
                "%s:%d, auto-addr-mDNS: \"%s.%s\", auto-pref-hub: \"%s\", "
                "manual-hub: \"%s:%d\")",
                CLIENT_SEARCH_MODE_STRINGS[client_mode],
                BOOLEAN_DISPLAY_STRINGS[reconnect_to_last_hub],
                last_hub_ip.toString().c_str(), last_hub_port,
                tcp_service_name.c_str(), tcp_service_protocol.c_str(),
                auto_hub_hostname.c_str(), manual_hub_host.c_str(),
                manual_hub_port);
            LOG(INFO, "WiFi-Sleep: %s",
                BOOLEAN_DISPLAY_STRINGS[WiFi.getSleep()]);

            // If this node is configured as a hub, start the listener.
            if(hub_enabled)
            {
                wifi->start_hub(hub_port, hub_service_name,
                    hub_service_protocol);
            }
            wifi->configReloadRequested_ = false;
        }

        // Validate any hub connections to ensure they are all still active
        // any that are dead we remove from the stack
        std::vector<Esp32WiFiClientAdapter *> deadHubConnections;
        for (const auto &pair : wifi->hubConnections_)
        {
            if (!pair.first->connected())
            {
                LOG(WARNING, "[CLIENT] Lost connection to %s:%d, attempting "
                    "to reconnect.", pair.first->remoteIP().toString().c_str(),
                    pair.first->remotePort());
                // connection has died, try to reconnect
                if (!pair.first->reconnect())
                {
                    LOG_ERROR("[CLIENT] Reconnection attempt failed.");
                    // reconnect failed, consider it a dead connection.
                    wifi->openmrn_->remove_port(pair.second);
                    deadHubConnections.push_back(pair.first);
                }
                else
                {
                    LOG(INFO, "[CLIENT] Successfully reconnected to %s:%d.",
                        pair.first->remoteIP().toString().c_str(),
                        pair.first->remotePort());
                }
            }
        }

        LOG(VERBOSE, "[CLIENT] %d active connections, %d pending cleanup.",
            wifi->hubConnections_.size() - deadHubConnections.size(),
            deadHubConnections.size());
        // Cleanup any dead connections.
        for (auto client : deadHubConnections)
        {
            auto it = wifi->hubConnections_.find(client);
            HASSERT(it != wifi->hubConnections_.end());
            wifi->hubConnections_.erase(it);
            delete client;
        }

        // If we do not have a connection to a hub attempt to connect to one
        // based on the loaded node configuration.
        if (wifi->hubConnections_.empty())
        {
            // Flag to track our connection state.
            bool connected = false;

            // if we are to reconnect to our last hub and we have a valid
            // address from the configuration try and connect.
            if(reconnect_to_last_hub && last_hub_ip != INADDR_NONE)
            {
                connected = wifi->connect_to_hub(last_hub_ip,
                    last_hub_port);
            }

            // Check if we are in manual-auto or manual-only and attempt to
            // connect to the manually configured hub if we successfully
            // resolved the hostname to an ip.
            if(!connected &&
                (client_mode == SocketClientParams::SearchMode::MANUAL_AUTO ||
                client_mode == SocketClientParams::SearchMode::MANUAL_ONLY))
            {
                // If we have a manual hostname try and we have not
                // successfully resolved it to an IP address try to resolve
                // it now.
                if (manual_hub_host.length() > 0 &&
                    manual_hub_ip == INADDR_NONE &&
                    !manual_hub_ip.fromString(manual_hub_host.c_str()))
                {
                    // Note this call can block up to 2sec.
                    manual_hub_ip = MDNS.queryHost(manual_hub_host.c_str());

                    // if mDNS failed to resolve the hostname try normal DNS
                    if(manual_hub_ip == INADDR_NONE)
                    {
                        // no error check is done here as we will check the
                        // resolved IP below.
                        WiFi.hostByName(manual_hub_host.c_str(), manual_hub_ip);
                    }

                    // Check if mDNS resolver was able to resolve it to a valid
                    // ip address.
                    if(manual_hub_ip == INADDR_NONE)
                    {
                        LOG(WARNING,
                            "[CLIENT] Unable to resolve the manually "
                            "configured hostname \"%s\" to an IP.",
                            manual_hub_host.c_str());
                    }
                }

                // if we have resolved the hub's hostname to an IP attempt to
                // connect.
                if(manual_hub_ip != INADDR_NONE)
                {
                    connected = wifi->connect_to_hub(manual_hub_ip,
                        manual_hub_port);
                }
            }

            // If we haven't connected to a hub and we are in one of the auto
            // modes try discovering and connecting to a hub via mDNS.
            if (!connected &&
                client_mode != SocketClientParams::SearchMode::MANUAL_ONLY)
            {
                connected = wifi->connect_to_mdns_hub(auto_hub_hostname,
                    tcp_service_name, tcp_service_protocol);
            }

            // If we still have not connected to a hub, are in AUTO_MANUAL
            // mode and have a manual hub try and connect to it.
            if (!connected &&
                client_mode == SocketClientParams::SearchMode::AUTO_MANUAL &&
                manual_hub_ip != INADDR_NONE)
            {
                // If we didn't connect to any hubs via mDNS, try and
                // connect to a manually configured hub.
                connected = wifi->connect_to_hub(manual_hub_ip,
                    manual_hub_port);
            }

            // No luck connecting to any hubs, log a message and try again
            // on next iteration.
            if(!connected)
            {
                LOG(WARNING, "[CLIENT] Failed to connect to any hubs.");
            }
        }

        // Sleep until the next check interval or if we are woken up early to
        // consume a configuration update.
        ulTaskNotifyTake(pdTRUE, CONNECTION_CHECK_TICK_INTERVAL);
    }

    return nullptr;
}