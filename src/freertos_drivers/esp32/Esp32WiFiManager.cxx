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

#include "Esp32WiFiManager.hxx"
#include "os/MDNS.hxx"
#include "utils/FdUtils.hxx"

#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>

#include <lwip/dns.h>
#include <mdns.h>
#include <tcpip_adapter.h>

// ESP-IDF v4+ has a slightly different directory structure to previous
// versions.
#ifdef ESP_IDF_VERSION_MAJOR
// ESP-IDF v4+
#include <esp32/rom/crc.h>
#include <esp_private/wifi.h>
#else
// ESP-IDF v3.x
#include <esp_wifi_internal.h>
#include <rom/crc.h>
#endif // ESP_IDF_VERSION_MAJOR

using openlcb::NodeID;
using openlcb::SimpleCanStack;
using openlcb::TcpAutoAddress;
using openlcb::TcpClientConfig;
using openlcb::TcpClientDefaultParams;
using openlcb::TcpDefs;
using openlcb::TcpManualAddress;
using std::string;
using std::unique_ptr;

#ifndef ESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL
/// Allows setting the log level for mDNS related log messages from 
/// @ref DefaultSocketClientParams.
#define ESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL INFO
#endif

#ifndef ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL
/// Allows setting the log level for mDNS results in the @ref mdns_lookup
/// method.
#define ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL INFO
#endif

// Start of global namespace block.

// These must be declared *OUTSIDE* the openmrn_arduino namespace in order to
// be visible in the MDNS.cxx code.

/// Advertises an mDNS service name. This is a hook point for the MDNS class
/// and is used as part of the Esp32 WiFi HUB support.
void mdns_publish(const char *name, const char *service, uint16_t port);

/// Removes advertisement of an mDNS service name. This is not currently
/// exposed in the MDNS class but is supported on the ESP32.
void mdns_unpublish(const char *service);

/// Splits a service name since the ESP32 mDNS library requires the service
/// name and service protocol to be passed in individually.
///
/// @param service_name is the service name to be split.
/// @param protocol_name is the protocol portion of the service name.
///
/// Note: service_name *WILL* be modified by this call.
void split_mdns_service_name(string *service_name, string *protocol_name);

// End of global namespace block.

namespace openmrn_arduino
{

/// Priority to use for the wifi_manager_task. This is currently set to one
/// level higher than the arduino-esp32 loopTask. The task will be in a sleep
/// state until woken up by Esp32WiFiManager::process_wifi_event or
/// Esp32WiFiManager::apply_configuration.
static constexpr UBaseType_t WIFI_TASK_PRIORITY = 2;

/// Stack size for the wifi_manager_task.
static constexpr uint32_t WIFI_TASK_STACK_SIZE = 2560L;

/// Interval at which to check the WiFi connection status.
static constexpr TickType_t WIFI_CONNECT_CHECK_INTERVAL = pdMS_TO_TICKS(5000);

/// Interval at which to check if the GcTcpHub has started or not.
static constexpr uint32_t HUB_STARTUP_DELAY_USEC = MSEC_TO_USEC(50);

/// Bit designator for wifi_status_event_group which indicates we are connected
/// to the SSID.
static constexpr int WIFI_CONNECTED_BIT = BIT0;

/// Bit designator for wifi_status_event_group which indicates we have an IPv4
/// address assigned.
static constexpr int WIFI_GOTIP_BIT = BIT1;

/// Allow up to 36 checks to see if we have connected to the SSID and
/// received an IPv4 address. This allows up to ~3 minutes for the entire
/// process to complete, in most cases this should be complete in under 30
/// seconds.
static constexpr uint8_t MAX_CONNECTION_CHECK_ATTEMPTS = 36;

/// This is the number of consecutive IP addresses which will be available in
/// the SoftAP DHCP server IP pool. These will be allocated immediately
/// following the SoftAP IP address (default is 192.168.4.1). Default number to
/// reserve is 48 IP addresses. Only four stations can be connected to the
/// ESP32 SoftAP at any single time.
static constexpr uint8_t SOFTAP_IP_RESERVATION_BLOCK_SIZE = 48;

/// Event handler for the ESP32 WiFi system. This will receive events from the
/// ESP-IDF event loop processor and pass them on to the Esp32WiFiManager for
/// possible processing. This is only used when Esp32WiFiManager is managing
/// both the WiFi and mDNS systems, if these are managed externally the
/// consumer is responsible for calling Esp32WiFiManager::process_wifi_event
/// when WiFi events occur.
static esp_err_t wifi_event_handler(void *context, system_event_t *event)
{
    auto wifi = static_cast<Esp32WiFiManager *>(context);
    wifi->process_wifi_event(event);
    return ESP_OK;
}

/// Adapter class to load/store configuration via CDI
class Esp32SocketParams : public DefaultSocketClientParams
{
public:
    Esp32SocketParams(
        int fd, const TcpClientConfig<TcpClientDefaultParams> &cfg)
        : configFd_(fd)
        , cfg_(cfg)
    {
        mdnsService_ = cfg_.auto_address().service_name().read(configFd_);
        staticHost_ = cfg_.manual_address().ip_address().read(configFd_);
        staticPort_ = CDI_READ_TRIMMED(cfg_.manual_address().port, configFd_);
    }

    /// @return search mode for how to locate the server.
    SearchMode search_mode() override
    {
        return (SearchMode)CDI_READ_TRIMMED(cfg_.search_mode, configFd_);
    }

    /// @return null or empty string if any mdns server is okay to connect
    /// to. If nonempty, then only an mdns server will be chosen that has the
    /// specific host name.
    string mdns_host_name() override
    {
        return cfg_.auto_address().host_name().read(configFd_);
    }

    /// @return true if first attempt should be to connect to
    /// last_host_name:last_port.
    bool enable_last() override
    {
        return CDI_READ_TRIMMED(cfg_.reconnect, configFd_);
    }

    /// @return the last successfully used IP address, as dotted
    /// decimal. Nullptr or empty if no successful connection has ever been
    /// made.
    string last_host_name() override
    {
        return cfg_.last_address().ip_address().read(configFd_);
    }

    /// @return the last successfully used port number.
    int last_port() override
    {
        return CDI_READ_TRIMMED(cfg_.last_address().port, configFd_);
    }

    /// Stores the last connection details for use when reconnect is enabled.
    ///
    /// @param hostname is the hostname that was connected to.
    /// @param port is the port that was connected to.
    void set_last(const char *hostname, int port) override
    {
        cfg_.last_address().ip_address().write(configFd_, hostname);
        cfg_.last_address().port().write(configFd_, port);
    }

    void log_message(LogMessage id, const string &arg) override
    {
        switch (id)
        {
            case CONNECT_RE:
                LOG(INFO, "[Uplink] Reconnecting to %s.", arg.c_str());
                break;
            case MDNS_SEARCH:
                LOG(ESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL,
                    "[Uplink] Starting mDNS searching for %s.",
                    arg.c_str());
                break;
            case MDNS_NOT_FOUND:
                LOG(ESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL,
                    "[Uplink] mDNS search failed.");
                break;
            case MDNS_FOUND:
                LOG(ESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL,
                    "[Uplink] mDNS search succeeded.");
                break;
            case CONNECT_MDNS:
                LOG(INFO, "[Uplink] mDNS connecting to %s.", arg.c_str());
                break;
            case CONNECT_MANUAL:
                LOG(INFO, "[Uplink] Connecting to %s.", arg.c_str());
                break;
            case CONNECT_FAILED_SELF:
                LOG(ESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL,
                    "[Uplink] Rejecting attempt to connect to localhost.");
                break;
            case CONNECTION_LOST:
                LOG(INFO, "[Uplink] Connection lost.");
                break;
            default:
                // ignore the message
                break;
        }
    }

    /// @return true if we should actively skip connections that happen to
    /// match our own IP address.
    bool disallow_local() override
    {
        return true;
    }

private:
    const int configFd_;
    const TcpClientConfig<TcpClientDefaultParams> cfg_;
};

// With this constructor being used the Esp32WiFiManager will manage the
// WiFi connection, mDNS system and the hostname of the ESP32.
Esp32WiFiManager::Esp32WiFiManager(const char *ssid
                                 , const char *password
                                 , SimpleCanStack *stack
                                 , const WiFiConfiguration &cfg
                                 , const char *hostname_prefix
                                 , wifi_mode_t wifi_mode
                                 , tcpip_adapter_ip_info_t *station_static_ip
                                 , ip_addr_t primary_dns_server
                                 , uint8_t soft_ap_channel
                                 , wifi_auth_mode_t soft_ap_auth
                                 , const char *soft_ap_password
                                 , tcpip_adapter_ip_info_t *softap_static_ip)
    : DefaultConfigUpdateListener()
    , hostname_(hostname_prefix)
    , ssid_(ssid)
    , password_(password)
    , cfg_(cfg)
    , manageWiFi_(true)
    , stack_(stack)
    , wifiMode_(wifi_mode)
    , stationStaticIP_(station_static_ip)
    , primaryDNSAddress_(primary_dns_server)
    , softAPChannel_(soft_ap_channel)
    , softAPAuthMode_(soft_ap_auth)
    , softAPPassword_(soft_ap_password ? soft_ap_password : password)
    , softAPStaticIP_(softap_static_ip)
{
    // Extend the capacity of the hostname to make space for the node-id and
    // underscore.
    hostname_.reserve(TCPIP_HOSTNAME_MAX_SIZE);

    // Generate the hostname for the ESP32 based on the provided node id.
    // node_id : 0x050101011425
    // hostname_ : esp32_050101011425
    NodeID node_id = stack_->node()->node_id();
    hostname_.append(uint64_to_string_hex(node_id, 0));

    // The maximum length hostname for the ESP32 is 32 characters so truncate
    // when necessary. Reference to length limitation:
    // https://github.com/espressif/esp-idf/blob/master/components/tcpip_adapter/include/tcpip_adapter.h#L611
    if (hostname_.length() > TCPIP_HOSTNAME_MAX_SIZE)
    {
        LOG(WARNING, "ESP32 hostname is too long, original hostname: %s",
            hostname_.c_str());
        hostname_.resize(TCPIP_HOSTNAME_MAX_SIZE);
        LOG(WARNING, "truncated hostname: %s", hostname_.c_str());
    }

    // Release any extra capacity allocated for the hostname.
    hostname_.shrink_to_fit();
}

// With this constructor being used, it will be the responsibility of the
// application to manage the WiFi and mDNS systems.
Esp32WiFiManager::Esp32WiFiManager(
    SimpleCanStack *stack, const WiFiConfiguration &cfg)
    : DefaultConfigUpdateListener()
    , cfg_(cfg)
    , manageWiFi_(false)
    , stack_(stack)
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

    // Load the CDI entry into memory to do an CRC-32 check against our last
    // loaded configuration so we can avoid reloading configuration when there
    // are no interesting changes.
    unique_ptr<uint8_t[]> crcbuf(new uint8_t[cfg_.size()]);

    // If we are unable to seek to the right position in the persistent storage
    // give up and request a reboot.
    if (lseek(fd, cfg_.offset(), SEEK_SET) != cfg_.offset())
    {
        LOG_ERROR("lseek failed to reset fd offset, REBOOT_NEEDED");
        return ConfigUpdateListener::UpdateAction::REBOOT_NEEDED;
    }

    // Read the full configuration to the buffer for crc check.
    FdUtils::repeated_read(fd, crcbuf.get(), cfg_.size());

    // Calculate CRC-32 from the loaded buffer.
    uint32_t configCrc32 = crc32_le(0, crcbuf.get(), cfg_.size());
    LOG(VERBOSE, "existing config CRC-32: \"%s\", new CRC-32: \"%s\"",
        integer_to_string(configCrc32_, 0).c_str(),
        integer_to_string(configCrc32, 0).c_str());

    // if this is not the initial loading of the CDI entry check the CRC-32
    // value and trigger a configuration reload if necessary.
    if (!initial_load)
    {
        if (configCrc32 != configCrc32_)
        {
            configReloadRequested_ = true;
            // If a configuration change has been detected, wake up the
            // wifi_manager_task so it can consume the change prior to the next
            // wake up interval.
            xTaskNotifyGive(wifiTaskHandle_);
        }
    }
    else
    {
        // This is the initial loading of the CDI entry, start the background
        // task that will manage the node's WiFi connection(s).
        start_wifi_task();
    }

    // Store the calculated CRC-32 for future use when the apply_configuration
    // method is called to detect any configuration changes.
    configCrc32_ = configCrc32;

    // Inform the caller that the configuration has been updated as the wifi
    // task will reload the configuration as part of it's next wake up cycle.
    return ConfigUpdateListener::UpdateAction::UPDATED;
}

// Factory reset handler for the WiFiConfiguration CDI entry.
void Esp32WiFiManager::factory_reset(int fd)
{
    LOG(VERBOSE, "Esp32WiFiManager::factory_reset(%d)", fd);

    // General WiFi configuration settings.
    CDI_FACTORY_RESET(cfg_.sleep);

    // Hub specific configuration settings.
    CDI_FACTORY_RESET(cfg_.hub().enable);
    CDI_FACTORY_RESET(cfg_.hub().port);
    cfg_.hub().service_name().write(
        fd, TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);

    // Node link configuration settings.
    CDI_FACTORY_RESET(cfg_.uplink().search_mode);
    CDI_FACTORY_RESET(cfg_.uplink().reconnect);

    // Node link manual configuration settings.
    cfg_.uplink().manual_address().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.uplink().manual_address().port);

    // Node link automatic configuration settings.
    cfg_.uplink().auto_address().service_name().write(
        fd, TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP);
    cfg_.uplink().auto_address().host_name().write(fd, "");

    // Node link automatic last connected node address.
    cfg_.uplink().last_address().ip_address().write(fd, "");
    CDI_FACTORY_RESET(cfg_.uplink().last_address().port);

    // Reconnect to last connected node.
    CDI_FACTORY_RESET(cfg_.uplink().reconnect);
}

// Processes a WiFi system event
void Esp32WiFiManager::process_wifi_event(system_event_t *event)
{
    LOG(VERBOSE, "Esp32WiFiManager::process_wifi_event(%d)", event->event_id);

    // We only are interested in this event if we are managing the
    // WiFi and MDNS systems and our mode includes STATION.
    if (event->event_id == SYSTEM_EVENT_STA_START && manageWiFi_ &&
        (wifiMode_ == WIFI_MODE_APSTA || wifiMode_ == WIFI_MODE_STA))
    {
        // Set the generated hostname prior to connecting to the SSID
        // so that it shows up with the generated hostname instead of
        // the default "Espressif".
        LOG(INFO, "[WiFi] Setting ESP32 hostname to \"%s\".",
            hostname_.c_str());
        ESP_ERROR_CHECK(tcpip_adapter_set_hostname(
            TCPIP_ADAPTER_IF_STA, hostname_.c_str()));
        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        LOG(INFO, "[WiFi] MAC Address: %s", mac_to_string(mac).c_str());

        if (stationStaticIP_)
        {
            // Stop the DHCP service before connecting, this allows us to
            // specify a static IP address for the WiFi connection
            LOG(INFO, "[DHCP] Stopping DHCP Client (if running).");
            ESP_ERROR_CHECK(
                tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA));

            LOG(INFO,
                "[WiFi] Configuring Static IP address:\n"
                "IP     : " IPSTR "\n"
                "Gateway: " IPSTR "\n"
                "Netmask: " IPSTR,
                IP2STR(&stationStaticIP_->ip),
                IP2STR(&stationStaticIP_->gw),
                IP2STR(&stationStaticIP_->netmask));
            ESP_ERROR_CHECK(
                tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA,
                                          stationStaticIP_));

            // if we do not have a primary DNS address configure the default
            if (ip_addr_isany(&primaryDNSAddress_))
            {
                IP4_ADDR(&primaryDNSAddress_.u_addr.ip4, 8, 8, 8, 8);
            }
            LOG(INFO, "[WiFi] Configuring primary DNS address to: " IPSTR,
                IP2STR(&primaryDNSAddress_.u_addr.ip4));
            // set the primary server (0)
            dns_setserver(0, &primaryDNSAddress_);
        }
        else
        {
            // Start the DHCP service before connecting so it hooks into
            // the flow early and provisions the IP automatically.
            LOG(INFO, "[DHCP] Starting DHCP Client.");
            ESP_ERROR_CHECK(
                tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA));
        }

        LOG(INFO,
            "[WiFi] Station started, attempting to connect to SSID: %s.",
            ssid_);
        // Start the SSID connection process.
        esp_wifi_connect();
    }
    else if (event->event_id == SYSTEM_EVENT_STA_CONNECTED)
    {
        LOG(INFO, "[WiFi] Connected to SSID: %s", ssid_);
        // Set the flag that indictes we are connected to the SSID.
        xEventGroupSetBits(wifiStatusEventGroup_, WIFI_CONNECTED_BIT);
    }
    else if (event->event_id == SYSTEM_EVENT_STA_GOT_IP)
    {
        // Retrieve the configured IP address from the TCP/IP stack.
        tcpip_adapter_ip_info_t ip_info;
        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
        LOG(INFO,
            "[WiFi] IP address is " IPSTR ", starting hub (if enabled) and "
            "uplink.",
            IP2STR(&ip_info.ip));

        // Start the mDNS system since we have an IP address, the mDNS system
        // on the ESP32 requires that the IP address be assigned otherwise it
        // will not start the UDP listener.
        start_mdns_system();

        // Set the flag that indictes we have an IPv4 address.
        xEventGroupSetBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);

        // Wake up the wifi_manager_task so it can start connections
        // creating connections, this will be a no-op for initial startup.
        xTaskNotifyGive(wifiTaskHandle_);
    }
    else if (event->event_id == SYSTEM_EVENT_STA_LOST_IP)
    {
        // Clear the flag that indicates we are connected and have an
        // IPv4 address.
        xEventGroupClearBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);
        // Wake up the wifi_manager_task so it can clean up connections.
        xTaskNotifyGive(wifiTaskHandle_);
    }
    else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED)
    {
        // flag to indicate that we should print the reconnecting log message.
        bool was_previously_connected = false;

        // Check if we have already connected, this event can be raised
        // even before we have successfully connected during the SSID
        // connect process.
        if (xEventGroupGetBits(wifiStatusEventGroup_) & WIFI_CONNECTED_BIT)
        {
            // track that we were connected previously.
            was_previously_connected = true;

            LOG(INFO, "[WiFi] Lost connection to SSID: %s (reason:%d)", ssid_
              , event->event_info.disconnected.reason);
            // Clear the flag that indicates we are connected to the SSID.
            xEventGroupClearBits(wifiStatusEventGroup_, WIFI_CONNECTED_BIT);
            // Clear the flag that indicates we have an IPv4 address.
            xEventGroupClearBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);

            // Wake up the wifi_manager_task so it can clean up
            // connections.
            xTaskNotifyGive(wifiTaskHandle_);
        }

        // If we are managing the WiFi and MDNS systems we need to
        // trigger the reconnection process at this point.
        if (manageWiFi_)
        {
            if (was_previously_connected)
            {
                LOG(INFO, "[WiFi] Attempting to reconnect to SSID: %s.",
                    ssid_);
            }
            else
            {
                LOG(INFO,
                    "[WiFi] Connection failed, reconnecting to SSID: %s.",
                    ssid_);
            }
            esp_wifi_connect();
        }
    }
    else if (event->event_id == SYSTEM_EVENT_AP_START && manageWiFi_)
    {
        // Set the generated hostname prior to connecting to the SSID
        // so that it shows up with the generated hostname instead of
        // the default "Espressif".
        LOG(INFO, "[SoftAP] Setting ESP32 hostname to \"%s\".",
            hostname_.c_str());
        ESP_ERROR_CHECK(tcpip_adapter_set_hostname(
            TCPIP_ADAPTER_IF_AP, hostname_.c_str()));

        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_AP, mac);
        LOG(INFO, "[SoftAP] MAC Address: %s", mac_to_string(mac).c_str());

        // If the SoftAP is not configured to use a static IP it will default
        // to 192.168.4.1.
        if (softAPStaticIP_ && wifiMode_ != WIFI_MODE_STA)
        {
            // Stop the DHCP server so we can reconfigure it.
            LOG(INFO, "[SoftAP] Stopping DHCP Server (if running).");
            ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));

            LOG(INFO,
                "[SoftAP] Configuring Static IP address:\n"
                "IP     : " IPSTR "\n"
                "Gateway: " IPSTR "\n"
                "Netmask: " IPSTR,
                IP2STR(&softAPStaticIP_->ip),
                IP2STR(&softAPStaticIP_->gw),
                IP2STR(&softAPStaticIP_->netmask));
            ESP_ERROR_CHECK(
                tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP,
                                          softAPStaticIP_));

            // Convert the Soft AP Static IP to a uint32 for manipulation
            uint32_t apIP = ntohl(ip4_addr_get_u32(&softAPStaticIP_->ip));

            // Default configuration is for DHCP addresses to follow
            // immediately after the static ip address of the Soft AP.
            ip4_addr_t first_ip, last_ip;
            ip4_addr_set_u32(&first_ip, htonl(apIP + 1));
            ip4_addr_set_u32(&last_ip,
                             htonl(apIP + SOFTAP_IP_RESERVATION_BLOCK_SIZE));

            dhcps_lease_t dhcp_lease {
                true,                   // enable dhcp lease functionality
                first_ip,               // first ip to assign
                last_ip,                // last ip to assign
            };

            LOG(INFO,
                "[SoftAP] Configuring DHCP Server for IPs: " IPSTR " - " IPSTR,
                IP2STR(&dhcp_lease.start_ip), IP2STR(&dhcp_lease.end_ip));
            ESP_ERROR_CHECK(
                tcpip_adapter_dhcps_option(TCPIP_ADAPTER_OP_SET,
                                           TCPIP_ADAPTER_REQUESTED_IP_ADDRESS,
                                           (void *)&dhcp_lease,
                                           sizeof(dhcps_lease_t)));

            // Start the DHCP server so it can provide IP addresses to stations
            // when they connect.
            LOG(INFO, "[SoftAP] Starting DHCP Server.");
            ESP_ERROR_CHECK(
                tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
        }

        // If we are not operating in SoftAP mode only we can start the mDNS
        // system now, otherwise we need to defer it until the station has
        // received it's IP address to avoid reinitializing the mDNS system.
        if (wifiMode_ == WIFI_MODE_AP)
        {
            start_mdns_system();
        }
    }
    else if (event->event_id == SYSTEM_EVENT_AP_STACONNECTED)
    {
        LOG(INFO, "[SoftAP aid:%d] %s connected.",
            event->event_info.sta_connected.aid,
            mac_to_string(event->event_info.sta_connected.mac).c_str());
    }
    else if (event->event_id == SYSTEM_EVENT_AP_STADISCONNECTED)
    {
        LOG(INFO, "[SoftAP aid:%d] %s disconnected.",
            event->event_info.sta_disconnected.aid,
            mac_to_string(event->event_info.sta_connected.mac).c_str());
    }
    else if (event->event_id == SYSTEM_EVENT_SCAN_DONE)
    {
        {
            OSMutexLock l(&ssidScanResultsLock_);
            uint16_t num_found{0};
            esp_wifi_scan_get_ap_num(&num_found);
            LOG(VERBOSE, "[WiFi] %d SSIDs found via scan", num_found);
            ssidScanResults_.resize(num_found);
            esp_wifi_scan_get_ap_records(&num_found, ssidScanResults_.data());
#if LOGLEVEL >= VERBOSE
            for (int i = 0; i < num_found; i++)
            {
                LOG(VERBOSE, "SSID: %s, RSSI: %d, channel: %d"
                  , ssidScanResults_[i].ssid
                  , ssidScanResults_[i].rssi, ssidScanResults_[i].primary);
            }
#endif
        }
        if (ssidCompleteNotifiable_)
        {
            ssidCompleteNotifiable_->notify();
            ssidCompleteNotifiable_ = nullptr;
        }
    }

    {
        OSMutexLock l(&eventCallbacksLock_);
        // Pass the event received from ESP-IDF to any registered callbacks.
        for(auto callback : eventCallbacks_)
        {
            callback(event);
        }
    }
}

void Esp32WiFiManager::enable_verbose_logging()
{
    esp32VerboseLogging_ = true;
    enable_esp_wifi_logging();
}

// If the Esp32WiFiManager is setup to manage the WiFi system, the following
// steps are executed:
// 1) Start the TCP/IP adapter.
// 2) Hook into the ESP event loop so we receive WiFi events.
// 3) Initialize the WiFi system.
// 4) Set the WiFi mode to STATION (WIFI_STA)
// 5) Configure the WiFi system to store parameters only in memory to avoid
// potential corruption of entries in NVS.
// 6) Configure the WiFi system for SSID/PW.
// 7) Set the hostname based on the generated hostname.
// 8) Connect to WiFi and wait for IP assignment.
// 9) Verify that we connected and received a IP address, if not log a FATAL
// message and give up.
void Esp32WiFiManager::start_wifi_system()
{
    // Create the event group used for tracking connected/disconnected status.
    // This is used internally regardless of if we manage the rest of the WiFi
    // or mDNS systems.
    wifiStatusEventGroup_ = xEventGroupCreate();

    // If we do not need to manage the WiFi and mDNS systems exit early.
    if (!manageWiFi_)
    {
        return;
    }

    // Initialize the TCP/IP adapter stack.
    LOG(INFO, "[WiFi] Starting TCP/IP stack");
    tcpip_adapter_init();

    // Install event loop handler.
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, this));

    // Start the WiFi adapter.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    LOG(INFO, "[WiFi] Initializing WiFi stack");

    // Disable NVS storage for the WiFi driver
    cfg.nvs_enable = false;

    // override the defaults coming from arduino-esp32, the ones below improve
    // throughput and stability of TCP/IP, for more info on these values, see:
    // https://github.com/espressif/arduino-esp32/issues/2899 and
    // https://github.com/espressif/arduino-esp32/pull/2912
    //
    // Note: these numbers are slightly higher to allow compatibility with the
    // WROVER chip and WROOM-32 chip. The increase results in ~2kb less heap
    // at runtime.
    //
    // These do not require recompilation of arduino-esp32 code as these are
    // used in the WIFI_INIT_CONFIG_DEFAULT macro, they simply need to be redefined.
    cfg.static_rx_buf_num = 16;
    cfg.dynamic_rx_buf_num = 32;
    cfg.rx_ba_win = 16;

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (esp32VerboseLogging_)
    {
        enable_esp_wifi_logging();
    }

    wifi_mode_t requested_wifi_mode = wifiMode_;
    if (wifiMode_ == WIFI_MODE_AP)
    {
      // override the wifi mode from AP only to AP+STA so we can perform wifi
      // scans on demand.
      requested_wifi_mode = WIFI_MODE_APSTA;
    }
    // Set the requested WiFi mode.
    ESP_ERROR_CHECK(esp_wifi_set_mode(requested_wifi_mode));

    // This disables storage of SSID details in NVS which has been shown to be
    // problematic at times for the ESP32, it is safer to always pass fresh
    // config and have the ESP32 resolve the details at runtime rather than
    // use a cached set from NVS.
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    // If we want to host a SoftAP configure it now.
    if (wifiMode_ == WIFI_MODE_APSTA || wifiMode_ == WIFI_MODE_AP)
    {
        wifi_config_t conf;
        bzero(&conf, sizeof(wifi_config_t));
        conf.ap.authmode = softAPAuthMode_;
        conf.ap.beacon_interval = 100;
        conf.ap.channel = softAPChannel_;
        conf.ap.max_connection = 4;
        if (wifiMode_ == WIFI_MODE_AP)
        {
            // Configure the SSID for the Soft AP based on the SSID passed to
            // the Esp32WiFiManager constructor.
            strcpy(reinterpret_cast<char *>(conf.ap.ssid), ssid_);
        }
        else
        {
            // Configure the SSID for the Soft AP based on the generated
            // hostname when operating in WIFI_MODE_APSTA mode.
            strcpy(reinterpret_cast<char *>(conf.ap.ssid), hostname_.c_str());
        }
        
        if (password_ && softAPAuthMode_ != WIFI_AUTH_OPEN)
        {
            strcpy(reinterpret_cast<char *>(conf.ap.password), password_);
        }

        LOG(INFO, "[WiFi] Configuring SoftAP (SSID: %s)", conf.ap.ssid);
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &conf));
    }

    // If we need to connect to an SSID, configure it now.
    if (wifiMode_ == WIFI_MODE_APSTA || wifiMode_ == WIFI_MODE_STA)
    {
        // Configure the SSID details for the station based on the SSID and
        // password provided to the Esp32WiFiManager constructor.
        wifi_config_t conf;
        bzero(&conf, sizeof(wifi_config_t));
        strcpy(reinterpret_cast<char *>(conf.sta.ssid), ssid_);
        if (password_)
        {
            strcpy(reinterpret_cast<char *>(conf.sta.password), password_);
        }

        LOG(INFO, "[WiFi] Configuring Station (SSID: %s)", conf.sta.ssid);
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &conf));
    }

    // Start the WiFi stack. This will start the SoftAP and/or connect to the
    // SSID based on the configuration set above.
    LOG(INFO, "[WiFi] Starting WiFi stack");
    ESP_ERROR_CHECK(esp_wifi_start());

    // If we are using the STATION interface, this will block until the ESP32
    // starts the connection process, note it may not have an IP address
    // immediately thus the need to check the connection result a few times
    // before giving up with a FATAL error.
    if (wifiMode_ == WIFI_MODE_APSTA || wifiMode_ == WIFI_MODE_STA)
    {
        uint8_t attempt = 0;
        EventBits_t bits = 0;
        uint32_t bit_mask = WIFI_CONNECTED_BIT;
        while (++attempt <= MAX_CONNECTION_CHECK_ATTEMPTS)
        {
            // If we have connected to the SSID we then are waiting for IP
            // address.
            if (bits & WIFI_CONNECTED_BIT)
            {
                LOG(INFO, "[IPv4] [%d/%d] Waiting for IP address assignment.",
                    attempt, MAX_CONNECTION_CHECK_ATTEMPTS);
            }
            else
            {
                // Waiting for SSID connection
                LOG(INFO, "[WiFi] [%d/%d] Waiting for SSID connection.",
                    attempt, MAX_CONNECTION_CHECK_ATTEMPTS);
            }
            bits = xEventGroupWaitBits(wifiStatusEventGroup_,
                bit_mask, // bits we are interested in
                pdFALSE,  // clear on exit
                pdTRUE,   // wait for all bits
                WIFI_CONNECT_CHECK_INTERVAL);
            // Check if have connected to the SSID
            if (bits & WIFI_CONNECTED_BIT)
            {
                // Since we have connected to the SSID we now need to track
                // that we get an IP.
                bit_mask |= WIFI_GOTIP_BIT;
            }
            // Check if we have received an IP.
            if (bits & WIFI_GOTIP_BIT)
            {
                break;
            }
        }

        // Check if we successfully connected or not. If not, force a reboot.
        if ((bits & WIFI_CONNECTED_BIT) != WIFI_CONNECTED_BIT)
        {
            LOG(FATAL, "[WiFi] Failed to connect to SSID: %s.", ssid_);
        }

        // Check if we successfully connected or not. If not, force a reboot.
        if ((bits & WIFI_GOTIP_BIT) != WIFI_GOTIP_BIT)
        {
            LOG(FATAL, "[IPv4] Timeout waiting for an IP.");
        }
    }
}

// Starts a background task for the Esp32WiFiManager.
void Esp32WiFiManager::start_wifi_task()
{
    LOG(INFO, "[WiFi] Starting WiFi Manager task");
    os_thread_create(&wifiTaskHandle_, "Esp32WiFiMgr", WIFI_TASK_PRIORITY,
        WIFI_TASK_STACK_SIZE, wifi_manager_task, this);
}

// Background task for the Esp32WiFiManager. This handles all outbound
// connection attempts, configuration loading and making this node as a hub.
void *Esp32WiFiManager::wifi_manager_task(void *param)
{
    Esp32WiFiManager *wifi = static_cast<Esp32WiFiManager *>(param);

    // Start the WiFi system before proceeding with remaining tasks.
    wifi->start_wifi_system();

    while (true)
    {
        EventBits_t bits = xEventGroupGetBits(wifi->wifiStatusEventGroup_);
        if (bits & WIFI_GOTIP_BIT)
        {
            // If we do not have not an uplink connection force a config reload
            // to start the connection process.
            if (!wifi->uplink_)
            {
                wifi->configReloadRequested_ = true;
            }
        }
        else
        {
            // Since we do not have an IP address we need to shutdown any
            // active connections since they will be invalid until a new IP
            // has been provisioned.
            wifi->stop_hub();
            wifi->stop_uplink();

            // Make sure we don't try and reload configuration since we can't
            // create outbound connections at this time.
            wifi->configReloadRequested_ = false;
        }

        // Check if there are configuration changes to pick up.
        if (wifi->configReloadRequested_)
        {
            // Since we are loading configuration data, shutdown the hub and
            // uplink if created previously.
            wifi->stop_hub();
            wifi->stop_uplink();

            if (CDI_READ_TRIMMED(wifi->cfg_.sleep, wifi->configFd_))
            {
                // When sleep is enabled this will trigger the WiFi system to
                // only wake up every DTIM period to receive beacon updates.
                // no data loss is expected for this setting but it does delay
                // receiption until the DTIM period.
                ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
            }
            else
            {
                // When sleep is disabled the WiFi radio will always be active.
                // This will increase power consumption of the ESP32 but it
                // will result in a more reliable behavior when the ESP32 is
                // connected to an always-on power supply (ie: not a battery).
                ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
            }

            if (CDI_READ_TRIMMED(wifi->cfg_.hub().enable, wifi->configFd_))
            {
                // Since hub mode is enabled start the HUB creation process.
                wifi->start_hub();
            }
            // Start the uplink connection process in the background.
            wifi->start_uplink();
            wifi->configReloadRequested_ = false;
        }

        // Sleep until we are woken up again for configuration update or WiFi
        // event.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    return nullptr;
}

// Shuts down the hub listener (if enabled and running) for this node.
void Esp32WiFiManager::stop_hub()
{
    if (hub_)
    {
        mdns_unpublish(hubServiceName_);
        LOG(INFO, "[HUB] Shutting down TCP/IP listener");
        hub_.reset(nullptr);
    }
}

// Creates a hub listener for this node after loading configuration details.
void Esp32WiFiManager::start_hub()
{
    hubServiceName_ = cfg_.hub().service_name().read(configFd_);
    uint16_t hub_port = CDI_READ_TRIMMED(cfg_.hub().port, configFd_);

    LOG(INFO, "[HUB] Starting TCP/IP listener on port %d", hub_port);
    hub_.reset(new GcTcpHub(stack_->can_hub(), hub_port));

    // wait for the hub to complete it's startup tasks
    while (!hub_->is_started())
    {
        usleep(HUB_STARTUP_DELAY_USEC);
    }
    mdns_publish(hubServiceName_, hub_port);
}

// Disconnects and shuts down the uplink connector socket if running.
void Esp32WiFiManager::stop_uplink()
{
    if (uplink_)
    {
        LOG(INFO, "[UPLINK] Disconnecting from uplink.");
        uplink_->shutdown();
        uplink_.reset(nullptr);
    }
}

// Creates an uplink connector socket that will automatically add the uplink to
// the node's hub.
void Esp32WiFiManager::start_uplink()
{
    unique_ptr<SocketClientParams> params(
        new Esp32SocketParams(configFd_, cfg_.uplink()));
    uplink_.reset(new SocketClient(stack_->service(), stack_->executor(),
        stack_->executor(), std::move(params),
        std::bind(&Esp32WiFiManager::on_uplink_created, this,
            std::placeholders::_1, std::placeholders::_2)));
}

// Converts the passed fd into a GridConnect port and adds it to the stack.
void Esp32WiFiManager::on_uplink_created(int fd, Notifiable *on_exit)
{
    LOG(INFO, "[UPLINK] Connected to hub, configuring GridConnect port.");

    const bool use_select =
        (config_gridconnect_tcp_use_select() == CONSTANT_TRUE);

    // create the GridConnect port from the provided socket fd.
    create_gc_port_for_can_hub(stack_->can_hub(), fd, on_exit, use_select);

    // restart the stack to kick off alias allocation and send node init
    // packets.
    stack_->restart_stack();
}

// Enables the ESP-IDF wifi module logging at verbose level, will also set the
// sub-modules to verbose if they are available.
void Esp32WiFiManager::enable_esp_wifi_logging()
{
    esp_log_level_set("wifi", ESP_LOG_VERBOSE);

// arduino-esp32 1.0.2 uses ESP-IDF 3.2 which does not have these two methods
// in the headers, they are only available in ESP-IDF 3.3.
#if defined(WIFI_LOG_SUBMODULE_ALL)
    esp_wifi_internal_set_log_level(WIFI_LOG_VERBOSE);
    esp_wifi_internal_set_log_mod(
        WIFI_LOG_MODULE_ALL, WIFI_LOG_SUBMODULE_ALL, true);
#endif // WIFI_LOG_SUBMODULE_ALL
}

// Starts a background scan of SSIDs that can be seen by the ESP32.
void Esp32WiFiManager::start_ssid_scan(Notifiable *n)
{
    clear_ssid_scan_results();
    std::swap(ssidCompleteNotifiable_, n);
    // If there was a previous notifiable notify it now, there will be no
    // results but that should be fine since a new scan will be started.
    if (n)
    {
        n->notify();
    }
    // Start an active scan all channels, 120ms per channel (defaults)
    wifi_scan_config_t cfg;
    bzero(&cfg, sizeof(wifi_scan_config_t));
    // The boolean flag when set to false triggers an async scan.
    ESP_ERROR_CHECK(esp_wifi_scan_start(&cfg, false));
}

// Returns the number of SSIDs found in the last scan.
size_t Esp32WiFiManager::get_ssid_scan_result_count()
{
    OSMutexLock l(&ssidScanResultsLock_);
    return ssidScanResults_.size();
}

// Returns one SSID record from the last scan.
const wifi_ap_record_t& Esp32WiFiManager::get_ssid_scan_result(size_t index)
{
    OSMutexLock l(&ssidScanResultsLock_);
    if (index < ssidScanResults_.size())
    {
        return ssidScanResults_[index];
    }
    return defaultApRecord_;
}

// Clears all cached SSID scan results.
void Esp32WiFiManager::clear_ssid_scan_results()
{
    OSMutexLock l(&ssidScanResultsLock_);
    ssidScanResults_.clear();
}

// Advertises a service via mDNS.
//
// If mDNS has not yet been initialized the data will be cached and replayed
// after mDNS has been initialized.
void Esp32WiFiManager::mdns_publish(string service, const uint16_t port)
{
    {
        OSMutexLock l(&mdnsInitLock_);
        if (!mdnsInitialized_)
        {
            // since mDNS has not been initialized, store this publish until
            // it has been initialized.
            mdnsDeferredPublish_[service] = port;
            return;
        }
    }

    // Schedule the publish to be done through the Executor since we may need
    // to retry it.
    stack_->executor()->add(new CallbackExecutable([service, port]()
    {
        string service_name = service;
        string protocol_name;
        split_mdns_service_name(&service_name, &protocol_name);
        esp_err_t res = mdns_service_add(
            NULL, service_name.c_str(), protocol_name.c_str(), port, NULL, 0);
        LOG(VERBOSE, "[mDNS] mdns_service_add(%s.%s:%d): %s."
          , service_name.c_str(), protocol_name.c_str(), port
          , esp_err_to_name(res));
        // ESP_FAIL will be triggered if there is a timeout during publish of
        // the new mDNS entry. The mDNS task runs at a very low priority on the
        // PRO_CPU which is also where the OpenMRN Executor runs from which can
        // cause a race condition.
        if (res == ESP_FAIL)
        {
            // Send it back onto the scheduler to be retried
            Singleton<Esp32WiFiManager>::instance()->mdns_publish(service
                                                                , port);
        }
        else
        {
            LOG(INFO, "[mDNS] Advertising %s.%s:%d.", service_name.c_str()
              , protocol_name.c_str(), port);
        }
    }));
}

// Removes advertisement of a service from mDNS.
void Esp32WiFiManager::mdns_unpublish(string service)
{
    {
        OSMutexLock l(&mdnsInitLock_);
        if (!mdnsInitialized_)
        {
            // Since mDNS is not in an initialized state we can discard the
            // unpublish event.
            return;
        }
    }
    string service_name = service;
    string protocol_name;
    split_mdns_service_name(&service_name, &protocol_name);
    LOG(INFO, "[mDNS] Removing advertisement of %s.%s."
      , service_name.c_str(), protocol_name.c_str());
    esp_err_t res =
        mdns_service_remove(service_name.c_str(), protocol_name.c_str());
    LOG(VERBOSE, "[mDNS] mdns_service_remove: %s.", esp_err_to_name(res));
}

// Initializes the mDNS system on the ESP32.
//
// After initialization, if any services are pending publish they will be
// published at this time.
void Esp32WiFiManager::start_mdns_system()
{
    OSMutexLock l(&mdnsInitLock_);
    // If we have already initialized mDNS we can exit early.
    if (mdnsInitialized_)
    {
        return;
    }

    // Initialize the mDNS system.
    LOG(INFO, "[mDNS] Initializing mDNS system");
    ESP_ERROR_CHECK(mdns_init());

    // Set the mDNS hostname based on our generated hostname so it can be
    // found by other nodes.
    LOG(INFO, "[mDNS] Setting mDNS hostname to \"%s\"", hostname_.c_str());
    ESP_ERROR_CHECK(mdns_hostname_set(hostname_.c_str()));

    // Set the default mDNS instance name to the generated hostname.
    ESP_ERROR_CHECK(mdns_instance_name_set(hostname_.c_str()));

    // Publish any deferred mDNS entries
    for (auto & entry : mdnsDeferredPublish_)
    {
        mdns_publish(entry.first, entry.second);
    }
    mdnsDeferredPublish_.clear();

    // Set flag to indicate we have initialized mDNS.
    mdnsInitialized_ = true;
}

} // namespace openmrn_arduino

/// Maximum number of milliseconds to wait for mDNS query responses.
static constexpr uint32_t MDNS_QUERY_TIMEOUT = 2000;

/// Maximum number of results to capture for mDNS query requests.
static constexpr size_t MDNS_MAX_RESULTS = 10;

// Advertises an mDNS service name.
void mdns_publish(const char *name, const char *service, uint16_t port)
{
    // The name parameter is unused today.
    Singleton<Esp32WiFiManager>::instance()->mdns_publish(service, port);
}

// Removes advertisement of an mDNS service name.
void mdns_unpublish(const char *service)
{
    Singleton<Esp32WiFiManager>::instance()->mdns_unpublish(service);
}

// Splits an mDNS service name.
void split_mdns_service_name(string *service_name, string *protocol_name)
{
    HASSERT(service_name != nullptr);
    HASSERT(protocol_name != nullptr);

    // if the string is not blank and contains a period split it on the period.
    if (service_name->length() && service_name->find('.', 0) != string::npos)
    {
        string::size_type split_loc = service_name->find('.', 0);
        protocol_name->assign(service_name->substr(split_loc + 1));
        service_name->resize(split_loc);
    }
}


// EAI_AGAIN may not be defined on the ESP32
#ifndef EAI_AGAIN
#ifdef TRY_AGAIN
#define EAI_AGAIN TRY_AGAIN
#else
#define EAI_AGAIN -3
#endif
#endif // EAI_AGAIN

// Looks for an mDNS service name and converts the results of the query to an
// addrinfo struct.
int mdns_lookup(
    const char *service, struct addrinfo *hints, struct addrinfo **addr)
{
    unique_ptr<struct addrinfo> ai(new struct addrinfo);
    if (ai.get() == nullptr)
    {
        LOG_ERROR("[mDNS] Allocation failed for addrinfo.");
        return EAI_MEMORY;
    }
    bzero(ai.get(), sizeof(struct addrinfo));

    unique_ptr<struct sockaddr> sa(new struct sockaddr);
    if (sa.get() == nullptr)
    {
        LOG_ERROR("[mDNS] Allocation failed for sockaddr.");
        return EAI_MEMORY;
    }
    bzero(sa.get(), sizeof(struct sockaddr));

    struct sockaddr_in *sa_in = (struct sockaddr_in *)sa.get();
    ai->ai_flags = 0;
    ai->ai_family = hints->ai_family;
    ai->ai_socktype = hints->ai_socktype;
    ai->ai_protocol = hints->ai_protocol;
    ai->ai_addrlen = sizeof(struct sockaddr_in);
    sa_in->sin_len = sizeof(struct sockaddr_in);
    sa_in->sin_family = hints->ai_family;

    string service_name = service;
    string protocol_name;
    split_mdns_service_name(&service_name, &protocol_name);

    mdns_result_t *results = NULL;
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(
            mdns_query_ptr(service_name.c_str(),
                           protocol_name.c_str(),
                           MDNS_QUERY_TIMEOUT,
                           MDNS_MAX_RESULTS,
                           &results)))
    {
        // failed to find any matches
        return EAI_FAIL;
    }

    if (!results)
    {
        // failed to find any matches
        LOG(ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL,
            "[mDNS] No matches found for service: %s.",
            service);
        return EAI_AGAIN;
    }

    // make a copy of the results to preserve the original list for cleanup.
    mdns_result_t *res = results;
    // scan the mdns query results linked list, the first match with an IPv4
    // address will be returned.
    bool match_found = false;
    while (res && !match_found)
    {
        mdns_ip_addr_t *ipaddr = res->addr;
        while (ipaddr && !match_found)
        {
            // if this result has an IPv4 address process it
            if (ipaddr->addr.type == IPADDR_TYPE_V4)
            {
                LOG(ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL,
                    "[mDNS] Found %s as providing service: %s on port %d.",
                    res->hostname, service, res->port);
                inet_addr_from_ip4addr(
                    &sa_in->sin_addr, &ipaddr->addr.u_addr.ip4);
                sa_in->sin_port = htons(res->port);
                match_found = true;
            }
            ipaddr = ipaddr->next;
        }
        res = res->next;
    }

    // free up the query results linked list.
    mdns_query_results_free(results);

    if (!match_found)
    {
        LOG(ESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL,
            "[mDNS] No matches found for service: %s.",
            service);
        return EAI_AGAIN;
    }

    // return the resolved data to the caller
    *addr = ai.release();
    (*addr)->ai_addr = sa.release();

    // successfully resolved an address, inform the caller
    return 0;
}

// The functions below are not available via the standard ESP-IDF provided
// API.

/// Retrieves the IPv4 address from the ESP32 station interface.
///
/// @param ifap will hold the IPv4 address for the ESP32 station interface when
/// successfully retrieved.
/// @return zero for success, -1 for failure.
int getifaddrs(struct ifaddrs **ifap)
{
    tcpip_adapter_ip_info_t ip_info;

    /* start with something "safe" in case we bail out early */
    *ifap = nullptr;

    if (!tcpip_adapter_is_netif_up(TCPIP_ADAPTER_IF_STA))
    {
        // Station TCP/IP interface is not up
        errno = ENODEV;
        return -1;
    }

    // allocate memory for various pieces of ifaddrs
    std::unique_ptr<struct ifaddrs> ia(new struct ifaddrs);
    if (ia.get() == nullptr)
    {
        errno = ENOMEM;
        return -1;
    }
    bzero(ia.get(), sizeof(struct ifaddrs));
    std::unique_ptr<char[]> ifa_name(new char[6]);
    if (ifa_name.get() == nullptr)
    {
        errno = ENOMEM;
        return -1;
    }
    strcpy(ifa_name.get(), "wlan0");
    std::unique_ptr<struct sockaddr> ifa_addr(new struct sockaddr);
    if (ifa_addr == nullptr)
    {
        errno = ENOMEM;
        return -1;
    }
    bzero(ifa_addr.get(), sizeof(struct sockaddr));

    // retrieve TCP/IP address from the interface
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);

    // copy address into ifaddrs structure
    struct sockaddr_in *addr_in = (struct sockaddr_in *)ifa_addr.get();
    addr_in->sin_family = AF_INET;
    addr_in->sin_addr.s_addr = ip_info.ip.addr;
    ia.get()->ifa_next = nullptr;
    ia.get()->ifa_name = ifa_name.release();
    ia.get()->ifa_flags = 0;
    ia.get()->ifa_addr = ifa_addr.release();
    ia.get()->ifa_netmask = nullptr;
    ia.get()->ifa_ifu.ifu_broadaddr = nullptr;
    ia.get()->ifa_data = nullptr;

    // report results
    *ifap = ia.release();
    return 0;
}

/// Frees memory allocated as part of the call to @ref getifaddrs.
///
/// @param ifa is the ifaddrs struct to be freed.
void freeifaddrs(struct ifaddrs *ifa)
{
    while (ifa)
    {
        struct ifaddrs *next = ifa->ifa_next;

        HASSERT(ifa->ifa_data == nullptr);
        HASSERT(ifa->ifa_ifu.ifu_broadaddr == nullptr);
        HASSERT(ifa->ifa_netmask == nullptr);

        delete ifa->ifa_addr;
        delete[] ifa->ifa_name;
        delete ifa;

        ifa = next;
    }
}

/// @return the string equivalant of the passed error code.
const char *gai_strerror(int __ecode)
{
    switch (__ecode)
    {
        default:
            return "gai_strerror unknown";
        case EAI_AGAIN:
            return "temporary failure";
        case EAI_FAIL:
            return "non-recoverable failure";
        case EAI_MEMORY:
            return "memory allocation failure";
    }
}
