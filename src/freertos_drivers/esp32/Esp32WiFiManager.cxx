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

using openlcb::TcpDefs;
using openlcb::NodeID;

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

Esp32WiFiManager::Esp32WiFiManager(const char *ssid, const char *password,
    const char *hostname_prefix, OpenMRN *openmrn,
    const openlcb::NodeID node_id, const WiFiConfiguration &cfg) : ssid_(ssid),
    password_(password), cfg_(cfg), openmrn_(openmrn)
{
    // register the Esp32WiFiManager instance to receive config updates.
    ConfigUpdateService::instance()->register_update_listener(this);

    // if these are not passed in we will not manage the WiFi stack with this
    // class, only the hub and outbound connection features.
    if (ssid_ && password_ && hostname_prefix)
    {
        // calculate the hostname for the ESP32 based on the provided prefix and
        // the last 32 bits of the node id.
        // node_id : 0x050101011425
        // hostname_ : hostname_prefix + '-' + 01011425
        hostname_ = String(hostname_prefix) + "-";
        // append the last 32 bits of the provided node id, ie:
        hostname_ += String((uint32_t)(node_id & 0xFFFFFFFF), 16);

        // the maximum length hostname for the ESP32 is 32 bytes so truncate if
        // necessary
        // ref https://github.com/espressif/esp-idf/blob/master/components/tcpip_adapter/include/tcpip_adapter.h#L611
        if (hostname_.length() > TCPIP_HOSTNAME_MAX_SIZE)
        {
            LOG(WARNING, "ESP32 hostname is too long, original hostname: %s", hostname_.c_str());
            hostname_ = hostname_.substring(0, TCPIP_HOSTNAME_MAX_SIZE);
            LOG(WARNING, "truncated hostname: %s", hostname_.c_str());
        }
        manageWiFi_ = true;
    }
    else
    {
        // since we do not have an SSID, Password and hostname prefix we should
        // not manage the WiFi stack within this class.
        manageWiFi_ = false;
    }
}

#define VERIFY_CACHED_VALUE(current_value, new_value) \
    if (current_value != new_value) \
    { \
        current_value = new_value; \
        configUpdated = true; \
    }

ConfigUpdateListener::UpdateAction Esp32WiFiManager::apply_configuration(
    int fd, bool initial_load, BarrierNotifiable *done)
{
    AutoNotify n(done);
    LOG(VERBOSE, "Esp32WiFiManager::apply_configuration(%d, %d)", fd, initial_load);
    bool configUpdated = false;
    bool wifiSleepMode = cfg_.wifi_sleep().read(fd);
    bool hubEnabled = cfg_.hub_config().enabled().read(fd);
    uint16_t hubPort = cfg_.hub_config().port().read(fd);
    String hubServiceName = String(cfg_.hub_config().service_name().read(fd).c_str());
    const TcpClientConfig<TcpClientDefaultParams> &link_config =
        cfg_.link_config();
    SocketClientParams::SearchMode tcpClientMode =
        (SocketClientParams::SearchMode)link_config.search_mode().read(fd);
    String tcpmDNSServiceName =
        String(link_config.auto_address().service_name().read(fd).c_str());
    String tcpAutoHostName =
        String(link_config.auto_address().host_name().read(fd).c_str());
    String tcpManualHostName =
        String(link_config.manual_address().ip_address().read(fd).c_str());
    uint16_t tcpManualPort = link_config.manual_address().port().read(fd);

    // if this is the first time this node is loading its config we can simply
    // accept all config as is and start the WiFi system.
    if (initial_load)
    {
        configUpdated = true;
        wifiSleepMode_ = wifiSleepMode;
        hubEnabled_ = hubEnabled;
        hubPort_ = hubPort;
        hubServiceName_ = hubServiceName;
        tcpClientMode_ = tcpClientMode;
        tcpmDNSServiceName_ = tcpmDNSServiceName;
        tcpAutoHostName_ = tcpAutoHostName;
        tcpManualHostName_ = tcpManualHostName;
        tcpManualPort_ = tcpManualPort;
    }
    else
    {
        // check each parameter and update the cache if it is different
        VERIFY_CACHED_VALUE(wifiSleepMode_, wifiSleepMode)
        VERIFY_CACHED_VALUE(hubEnabled_, hubEnabled)
        VERIFY_CACHED_VALUE(hubPort_, hubPort)
        VERIFY_CACHED_VALUE(hubServiceName_, hubServiceName)
        VERIFY_CACHED_VALUE(tcpClientMode_, tcpClientMode)
        VERIFY_CACHED_VALUE(tcpmDNSServiceName_, tcpmDNSServiceName)
        VERIFY_CACHED_VALUE(tcpAutoHostName_, tcpAutoHostName)
        VERIFY_CACHED_VALUE(tcpManualHostName_, tcpManualHostName)
        VERIFY_CACHED_VALUE(tcpManualPort_, tcpManualPort)
    }

    // verify settings provided to make sure they are valid
    if (tcpClientMode_ != SocketClientParams::SearchMode::AUTO_ONLY)
    {
        // if the manual hostname is not provided (or is blank) or we do not
        // have a valid manual port force settings to AUTO_ONLY.
        if (!tcpManualHostName_.length() || tcpManualPort_ == 0)
        {
            LOG(WARNING, "Missing manual hub hostname or port, "
                "switching to AUTO_ONLY mode");
            tcpClientMode_ = SocketClientParams::SearchMode::AUTO_ONLY;
            // record the updated settings.
            link_config.search_mode().write(fd, tcpClientMode_);
        }
    }

    // if this is the first time we are loaded start the WiFi system.
    if (initial_load)
    {
        startWiFiSystem();
    }

    // if the configuration settings have changed, restart the WiFi manager
    // task to pick up the updated settings.
    if (configUpdated)
    {
        stopWiFiTask();
        startWiFiTask();
    }

    return ConfigUpdateListener::UpdateAction::UPDATED;
}

void Esp32WiFiManager::factory_reset(int fd)
{
    LOG(VERBOSE, "Esp32WiFiManager::factory_reset(%d)", fd);
    // General WiFi configuration settings
    CDI_FACTORY_RESET(cfg_.wifi_sleep);

    // Hub specific configuration settings
    CDI_FACTORY_RESET(cfg_.hub_config().enabled);
    CDI_FACTORY_RESET(cfg_.hub_config().port);
    cfg_.hub_config().service_name().write(fd,
        TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);

    // node link configuration settings
    CDI_FACTORY_RESET(cfg_.link_config().search_mode);
    CDI_FACTORY_RESET(cfg_.link_config().reconnect);

    // node link manual configuration settings
    cfg_.link_config().manual_address().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.link_config().manual_address().port);

    // node link automatic configuration settings
    cfg_.link_config().auto_address().service_name().write(fd,
        TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);
    cfg_.link_config().auto_address().host_name().write(fd, "");

    // node link automatic last connected node address
    cfg_.link_config().last_address().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.link_config().last_address().port);

    // reconnect to last connected node
    CDI_FACTORY_RESET(cfg_.link_config().reconnect);
}

void Esp32WiFiManager::startWiFiSystem()
{
    // if we are to manage the entire WiFi stack start and configure it here
    if (manageWiFi_)
    {
        // set the WiFI mode to STATION since we will connect to an AP
        WiFi.mode(WIFI_STA);

        // this next line tells the WiFi system to shutdown, this is needed to
        // ensure we have a higher success rate of connecting to the AP.
        WiFi.disconnect(true);

        // configure the WiFi system to automatically reconnect to the AP if we
        // get disconnected for any reason.
        WiFi.setAutoReconnect(true);

        LOG(INFO, "ESP32 hostname: %s", hostname_.c_str());
        WiFi.setHostname(hostname_.c_str());
        LOG(INFO, "Connecting to SSID: %s", ssid_);
        if (WiFi.begin(ssid_, password_) != WL_CONNECT_FAILED) {
            // allow up to 10 checks to see if we have connected to the SSID.
            static constexpr uint8_t attempts = 10;
            uint8_t attempt = 0;

            // this call waits up to 10sec for a result before timing out so it
            // needs to be called a few times until we get a final result.
            uint8_t wifiStatus = WiFi.waitForConnectResult();
            while (wifiStatus != WL_CONNECTED &&      // successfully connected
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
        if (WiFi.status() != WL_CONNECTED)
        {
            LOG(FATAL, "SSID connection failed: %s",
                WIFI_STATUS[WiFi.status()]);
        }
        else
        {
            LOG(INFO, "Connected to SSID: %s, IP: %s", ssid_,
                WiFi.localIP().toString().c_str());
        }
    }

    // This can make the wifi much more responsive. Since we are plugged in we
    // don't care about the increased power usage. Disable when on battery.
    // this is managed by this class regardless of manageWiFi_ setting since it
    // is exposed in the CDI.
    WiFi.setSleep(wifiSleepMode_);
}

void Esp32WiFiManager::startWiFiTask()
{
    // if we have a manual hostname defined, resolve the hostname to an IP
    if (!tcpManualHostName_.equals(""))
    {
        // note this call can block for up to 2sec.
        tcpManualIP_ = MDNS.queryHost(tcpManualHostName_);
    }

    if (!hubServiceName_.equals("") && hubEnabled_)
    {
        // split the mDNS service name since the ESPmDNS library wants it to
        // be passed as two parameters.
        hubServiceNameNoProtocol_ = hubServiceName_.substring(
            0, hubServiceName_.indexOf("."));
        hubServiceProtocol_ = hubServiceName_.substring(
            hubServiceName_.indexOf(".") + 1);
    }

    if (!tcpmDNSServiceName_.equals(""))
    {
        // split the mDNS service name since the ESPmDNS library wants it to
        // be passed as two parameters.
        tcpmDNSServiceNameNoProtocol_ = tcpmDNSServiceName_.substring(
            0, tcpmDNSServiceName_.indexOf("."));
        tcpmDNSServiceProtocol_ = tcpmDNSServiceName_.substring(
            tcpmDNSServiceName_.indexOf(".") + 1);
    }

    os_thread_create(&wifiTaskHandle_, "OpenMRN-WiFi-Mgr", WIFI_TASK_PRIORITY,
        WIFI_TASK_STACK_SIZE, wifi_manager_task, this);
}

void Esp32WiFiManager::stopWiFiTask()
{
    // request the wifi_manager_task to shutdown
    wifiTaskShutdownReq_ = true;

    // wait for wifi_manager_task to stop
    while (wifiTaskRunning_)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    wifiTaskHandle_ = nullptr;
}

bool Esp32WiFiManager::connectedTo(const String hostname, const IPAddress ip)
{
    // check if the passed hostname is this node
    if (hostname_ == hostname)
    {
        // it is us, we don't need to connect to it
        return true;
    }

    // passed hostname is not this node, check if we have an active connection
    // already being tracked.
    for (auto const &pair : hubConnections_)
    {
        if (pair.first->remoteIP() == ip)
        {
            return true;
        }
    }

    // it is not this node and we don't already have a connection so it is safe
    // to connect to it.
    return false;
}

bool Esp32WiFiManager::isPreferredHub(const String hostname)
{
    // if we have a preferred hub check if the passed hostname is it
    if (tcpAutoHostName_ != hostname)
    {
        // we have a preferred hub and this is not it
        return false;
    }
    // either we don't have a preferred hub or this was it
    return true;
}

bool Esp32WiFiManager::connectToHub(String hostname, IPAddress ip,
    uint16_t remotePort)
{
    if (connectedTo(hostname, ip))
    {
        LOG(WARNING, "[CLIENT] Already connected to %s:%d, skipping",
            hostname.c_str(), remotePort);
        return true;
    }

    // check if we have been called with an invalid configuration and silently
    // reject it
    if (hostname.equals(""))
    {
        return false;
    }

    // we found a remote hub to try and connect to
    WiFiClient remoteNode;
    LOG(INFO, "[CLIENT] Connecting to %s:%d...", hostname.c_str(), remotePort);
    if (!remoteNode.connect(ip, remotePort))
    {
        LOG(WARNING, "[CLIENT] Failed to connect to %s:%d!",
            hostname.c_str(), remotePort);
        return false;
    }
    LOG(INFO, "[CLIENT] Successfully connected to %s:%d!",
        hostname.c_str(), remotePort);
    Esp32WiFiClientAdapter *adapter = new Esp32WiFiClientAdapter(remoteNode);
    // add the new connction to the stack and record the returned excutor so
    // we can clean it up later if needed.
    hubConnections_.insert(std::make_pair(adapter,
        openmrn_->add_gridconnect_port(adapter)));
    return true;
}

bool Esp32WiFiManager::connectToManualHubIfNeeded()
{
    // if we are in manual-auto or manual-only mode try and connect to the
    // configured hub. NOTE: we are not checking for auto-manual here as
    // this node may successfully connect to an mDNS remote hub instead.
    if(tcpClientMode_ == SocketClientParams::SearchMode::MANUAL_AUTO ||
       tcpClientMode_ == SocketClientParams::SearchMode::MANUAL_ONLY)
    {
        return connectToHub(tcpManualHostName_, tcpManualIP_, tcpManualPort_);
    }
    return false;
}

bool Esp32WiFiManager::connectToMDNSHubs()
{
    bool connected = false;
    LOG(INFO, "[CLIENT] Scanning for mDNS service %s.%s",
        tcpmDNSServiceNameNoProtocol_.c_str(), tcpmDNSServiceProtocol_.c_str());
    // NOTE: this blocks for up to 3sec to collect results
    int entries = MDNS.queryService(hubServiceNameNoProtocol_,
        tcpmDNSServiceProtocol_);
    for (int index = 0; index < entries; index++)
    {
        // check if this is a preferred hub, this will internally check if we
        // have a preferred hub or not.
        if (!isPreferredHub(MDNS.hostname(index)))
        {
            LOG(VERBOSE, "[CLIENT] Found %s but it is not our preferred host.",
                MDNS.hostname(index).c_str());
            continue;
        }
        // attempt to connect to the remote hub, this will internally check if
        // we are already connected or not.
        if (connectToHub(MDNS.hostname(index), MDNS.IP(index),
            MDNS.port(index)))
        {
            connected = true;
        }
    }
    return connected;
}

void *Esp32WiFiManager::wifi_manager_task(void *param)
{
    Esp32WiFiManager *wifi = static_cast<Esp32WiFiManager *>(param);

    // set this flag to indicate we are running.
    wifi->wifiTaskRunning_ = true;

    // set this flag to ensure we loop at least once before shutdown.
    wifi->wifiTaskShutdownReq_ = false;

    // reset this to default
    wifi->hubSocket_ = nullptr;

    // Add this task to the WDT
    esp_task_wdt_add(wifi->wifiTaskHandle_);

    // check if we need to manage the MDNS services
    if (wifi->manageWiFi_)
    {
        // start the MDNS system now that we have an IP address
        MDNS.begin(wifi->hostname_.c_str());
    }

    // if we are a hub we need to setup the mDNS entry to advertise that we can
    // accept connections from other client nodes
    if (wifi->hubEnabled_)
    {
        LOG(INFO, "[HUB] Starting TCP/IP listener on port %d", wifi->hubPort_);
        wifi->hubSocket_ = new SocketListener(wifi->hubPort_,
            [wifi](int socket)
            {
                Esp32WiFiClientAdapter *newClient =
                    new Esp32WiFiClientAdapter(WiFiClient(socket));
                LOG(VERBOSE, "[HUB] New WiFi connection: %s",
                    newClient->remoteIP().toString().c_str());
                wifi->clientConnections_.insert(std::make_pair(newClient,
                    wifi->openmrn_->add_gridconnect_port(newClient)));
            }
        );
        LOG(INFO, "[HUB] Configuring mDNS broadcast %s.%s:%d",
            wifi->hubServiceNameNoProtocol_.c_str(),
            wifi->hubServiceProtocol_.c_str(),
            wifi->hubPort_);
        // Broadcast this node's hostname with the mDNS service name
        // for a TCP GridConnect endpoint.
        MDNS.addService(wifi->hubServiceNameNoProtocol_,
            wifi->hubServiceProtocol_, wifi->hubPort_);
    }

    // cache parameters we are using often
    SocketClientParams::SearchMode client_mode = wifi->tcpClientMode_;

    TickType_t next_mdns_scan_tick = 0;
    while (!wifi->wifiTaskShutdownReq_)
    {
        // tracking list for dead connections, this is common to both the hub
        // mode and client mode.
        std::vector<Esp32WiFiClientAdapter *> deadConnections;

        // Feed the watchdog so it doesn't reset the ESP32
        esp_task_wdt_reset();

        // if this node is a hub verify that all connected clients are still
        // connected, any that have disconnected we need to remove from the
        // stack.
        if (wifi->hubEnabled_)
        {
            for (const auto &pair : wifi->clientConnections_)
            {
                // Feed the watchdog so it doesn't reset the ESP32
                esp_task_wdt_reset();
                if (!pair.first->connected())
                {
                    // connection has died, clean it up
                    wifi->openmrn_->remove_port(pair.second);
                    deadConnections.push_back(pair.first);
                }
            }
        }

        // if sufficient ticks have passed since our last mDNS scan and we
        // do not have any connections to hubs, perform an mDNS scan and
        // connect to any "new" endpoints.
        auto current_tick_count = xTaskGetTickCount();
        if (current_tick_count >= next_mdns_scan_tick)
        {
            next_mdns_scan_tick = current_tick_count + MDNS_SCAN_INTERVAL;
            if (wifi->hubConnections_.empty())
            {
                // TODO: currently this code does not process the reconnect
                // flag declared in the CDI. However, if a connection is
                // established to a hub and that connection goes down it will
                // attempt to reconnect.

                bool connected = wifi->connectToManualHubIfNeeded();
                // if we haven't connected to a hub and we are in an auto mode
                // try and connect to an mDNS discovered hub.
                if (!connected &&
                    client_mode != SocketClientParams::SearchMode::MANUAL_ONLY)
                {
                    connected = wifi->connectToMDNSHubs();
                }

                // if we have not connected to a hub and we are in AUTO_MANUAL
                // mode and have a manual hostname try and connect to it.
                if (!connected &&
                    client_mode == SocketClientParams::SearchMode::AUTO_MANUAL)
                {
                    // if we didn't connect to any hubs via mDNS, try and
                    // connect to a manually configured hub.
                    // NOTE: there is no check that we connected successfully
                    // here as we will loop through and check again for
                    // automatic entries and re-enter this block if we don't
                    // connect to one.
                    wifi->connectToHub(wifi->tcpManualHostName_,
                        wifi->tcpManualIP_, wifi->tcpManualPort_);
                }
            }
        }

        // validate any hub connections to ensure they are all still active
        // any that are dead we remove from the stack
        for (const auto &pair : wifi->hubConnections_)
        {
            // Feed the watchdog so it doesn't reset the ESP32
            esp_task_wdt_reset();

            if (!pair.first->connected())
            {
                // connection has died, try to reconnect
                if (!pair.first->reconnect())
                {
                    // remote hub is gone, remove it from the stack
                    wifi->openmrn_->remove_port(pair.second);
                    deadConnections.push_back(pair.first);
                }
            }
        }

        // if we identified any dead connections above, clean them up.
        for (auto client : deadConnections)
        {
            // Feed the watchdog so it doesn't reset the ESP32
            esp_task_wdt_reset();

            auto it = wifi->clientConnections_.find(client);
            if (it != wifi->clientConnections_.end())
            {
                wifi->clientConnections_.erase(it);
            }
            else
            {
                it = wifi->hubConnections_.find(client);
                if (it != wifi->hubConnections_.end())
                {
                    wifi->hubConnections_.erase(it);
                }
            }
            delete client;
        }
        deadConnections.clear();
    }

    if (wifi->hubSocket_ != nullptr)
    {
        LOG(INFO, "[HUB] Shutting down TCP/IP listener");
        wifi->hubSocket_->shutdown();
        delete wifi->hubSocket_;
        LOG(INFO, "[HUB] Removing mDNS broadcast %s.%s",
            wifi->hubServiceNameNoProtocol_.c_str(),
            wifi->hubServiceProtocol_.c_str());
        // not using MDNS wrapper as it doesn't expose this method
        mdns_service_remove(wifi->hubServiceNameNoProtocol_.c_str(),
            wifi->hubServiceProtocol_.c_str());
    }

    wifi->wifiTaskRunning_ = false;
    return nullptr;
}