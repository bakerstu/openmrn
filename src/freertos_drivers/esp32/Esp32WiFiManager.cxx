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

// Ensure we only compile this code for ESP32 MCUs
#ifdef ESP_PLATFORM

#include "Esp32WiFiManager.hxx"
#include "openlcb/SimpleStack.hxx"
#include "openlcb/TcpDefs.hxx"
#include "os/MDNS.hxx"
#include "utils/FdUtils.hxx"
#include "utils/format_utils.hxx"
#include "utils/SocketClient.hxx"
#include "utils/SocketClientParams.hxx"

#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <lwip/dns.h>
#include <mdns.h>

// ESP-IDF v4+ has a slightly different directory structure to previous
// versions.
#include <dhcpserver/dhcpserver.h>
#include <esp_event.h>
#include <esp_private/wifi.h>

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#include <esp32s2/rom/crc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#include <esp32c3/rom/crc.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include <esp32s3/rom/crc.h>
#else // default to ESP32
#include <esp32/rom/crc.h>
#endif // CONFIG_IDF_TARGET
#include <stdint.h>

using openlcb::NodeID;
using openlcb::SimpleCanStackBase;
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

/// Removes advertisement of an mDNS service name.
void mdns_unpublish(const char *name, const char *service);

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
// When running on a unicore (ESP32-S2 or ESP32-C3) use a lower priority so the
// task will run in co-op mode with loop. When running on a multi-core SoC we
// can use a higher priority for the background task.
#if CONFIG_FREERTOS_UNICORE
/// Priority for the task performing the mdns lookups, connections for the
/// wifi uplink and any other background tasks needed by the wifi manager.
static constexpr UBaseType_t EXECUTOR_TASK_PRIORITY = 1;
#else // multi-core
/// Priority for the task performing the mdns lookups, connections for the
/// wifi uplink and any other background tasks needed by the wifi manager.
static constexpr UBaseType_t EXECUTOR_TASK_PRIORITY = 3;
#endif // CONFIG_FREERTOS_UNICORE

/// Stack size for the background task executor.
static constexpr uint32_t EXECUTOR_TASK_STACK_SIZE = 5120L;

/// Interval at which to check the WiFi connection status.
static constexpr uint32_t WIFI_CONNECT_CHECK_INTERVAL_SEC = 5;

/// Interval at which to check if the GcTcpHub has started or not.
static constexpr uint32_t HUB_STARTUP_DELAY_USEC = MSEC_TO_USEC(50);

/// Interval at which to check if the WiFi task has shutdown or not.
static constexpr uint32_t TASK_SHUTDOWN_DELAY_USEC = MSEC_TO_USEC(1);

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

/// Adapter class to load/store configuration via CDI
class Esp32SocketParams : public DefaultSocketClientParams
{
public:
    Esp32SocketParams(
        int fd, const TcpClientConfig<TcpClientDefaultParams> &cfg)
        : configFd_(fd)
        , cfg_(cfg)
    {
        // set the parameters on the parent class, all others are loaded
        // on-demand.
        mdnsService_ = cfg_.auto_address().service_name().read(configFd_);
        staticHost_ = cfg_.manual_address().ip_address().read(configFd_);
        staticPort_ = CDI_READ_TRIMMED(cfg_.manual_address().port, configFd_);
    }

    /// @return search mode for how to locate the server.
    SocketClientParams::SearchMode search_mode() override
    {
        return (SocketClientParams::SearchMode)CDI_READ_TRIMMED(cfg_.search_mode, configFd_);
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
Esp32WiFiManager::Esp32WiFiManager(const char *station_ssid
    , const char *station_password, openlcb::SimpleStackBase *stack
    , const WiFiConfiguration &cfg, wifi_mode_t wifi_mode
    , ConnectionMode connection_mode, const char *hostname_prefix
    , const char *sntp_server, const char *timezone, bool sntp_enabled
    , uint8_t softap_channel, wifi_auth_mode_t softap_auth_mode
    , const char *softap_ssid, const char *softap_password)
    : DefaultConfigUpdateListener(), Service(&executor_)
    , hostname_(hostname_prefix)
    , ssid_(station_ssid), password_(station_password), cfg_(cfg)
    , stack_(stack), wifiMode_(wifi_mode), softAPChannel_(softap_channel)
    , softAPAuthMode_(softap_auth_mode), softAPName_(softap_ssid)
    , softAPPassword_(softap_password), sntpEnabled_(sntp_enabled)
    , sntpServer_(sntp_server), timeZone_(timezone)
    , connectionMode_(connection_mode)
{
    // Extend the capacity of the hostname to make space for the node-id and
    // underscore.
    hostname_.reserve(MAX_HOSTNAME_LENGTH);

    // Generate the hostname for the ESP32 based on the provided node id.
    // node_id : 0x050101011425
    // hostname_ : esp32_050101011425
    NodeID node_id = stack_->node()->node_id();
    hostname_.append(uint64_to_string_hex(node_id, 0));

    // The maximum length hostname for the ESP32 is 32 characters so truncate
    // when necessary. Reference to length limitation:
    // https://github.com/espressif/esp-idf/blob/master/components/tcpip_adapter/include/tcpip_adapter.h#L611
    if (hostname_.length() > MAX_HOSTNAME_LENGTH)
    {
        LOG(WARNING, "ESP32 hostname is too long, original hostname:%s",
            hostname_.c_str());
        hostname_.resize(MAX_HOSTNAME_LENGTH);
        LOG(WARNING, "truncated hostname:%s", hostname_.c_str());
    }

    // Release any extra capacity allocated for the hostname.
    hostname_.shrink_to_fit();
}

Esp32WiFiManager::~Esp32WiFiManager()
{
    // Remove our event listeners from the event loop, note that we do not stop
    // the event loop.
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID
                               , &Esp32WiFiManager::process_idf_event);
    esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID
                               , &Esp32WiFiManager::process_idf_event);

    // shutdown the background task executor.
    executor_.shutdown();

    // cleanup event group
    vEventGroupDelete(wifiStatusEventGroup_);

    // cleanup internal vectors/maps
    ssidScanResults_.clear();
    mdnsDeferredPublish_.clear();
}

void Esp32WiFiManager::display_configuration()
{
    static const char * const wifiModes[] =
    {
        "Off",
        "Station Only",
        "SoftAP Only",
        "Station and SoftAP",
        "Unknown"
    };

    LOG(INFO, "[WiFi] Configuration Settings:");
    LOG(INFO, "[WiFi] Mode:%s (%d)", wifiModes[wifiMode_], wifiMode_);
    LOG(INFO, "[WiFi] Station SSID:'%s'", ssid_.c_str());
    LOG(INFO, "[WiFi] SoftAP SSID:'%s'", softAPName_.c_str());
    LOG(INFO, "[WiFi] Hostname:%s", hostname_.c_str());
    LOG(INFO, "[WiFi] SNTP Enabled:%s, Server:%s, TimeZone:%s",
        sntpEnabled_ ? "true" : "false", sntpServer_.c_str(),
        timeZone_.c_str());
}

#ifndef CONFIG_FREERTOS_UNICORE
/// Entrypoint for Esp32WiFiManager Executor thread when running on an ESP32
/// with more than one core.
///
/// @param param @ref Executor to be started.
static void thread_entry(void *param)
{
    // donate our task to the executor.
    static_cast<Executor<1> *>(param)->thread_body();
    vTaskDelete(nullptr);
}
#endif // !CONFIG_FREERTOS_UNICORE

ConfigUpdateListener::UpdateAction Esp32WiFiManager::apply_configuration(
    int fd, bool initial_load, BarrierNotifiable *done)
{
    AutoNotify n(done);
    LOG(VERBOSE, "Esp32WiFiManager::apply_configuration(%d, %d)", fd,
        initial_load);

    // Cache the fd for later use by the wifi background task.
    configFd_ = fd;

    // always load the connection mode.
    connectionMode_ = CDI_READ_TRIMMED(cfg_.connection_mode, fd);

    // Load the CDI entry into memory to do an CRC32 check against our last
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

    // Calculate CRC32 from the loaded buffer.
    uint32_t configCrc32 = crc32_le(0, crcbuf.get(), cfg_.size());
    LOG(VERBOSE, "existing config CRC32:%" PRIu32 ", new CRC32:%" PRIu32,
        configCrc32_, configCrc32);

    if (initial_load)
    {
        // If we have more than one core start the Executor on the APP CPU (1)
        // since the main OpenMRN Executor normally will run on the PRO CPU (0)
#ifndef CONFIG_FREERTOS_UNICORE
        xTaskCreatePinnedToCore(&thread_entry,              // entry point
                                "Esp32WiFiConn",            // task name
                                EXECUTOR_TASK_STACK_SIZE,   // stack size
                                &executor_,                 // entry point arg
                                EXECUTOR_TASK_PRIORITY,     // priority
                                nullptr,                    // task handle
                                APP_CPU_NUM);               // cpu core
#else
        // start the background task executor since it will be used for any
        // callback notifications that arise from starting the network stack.
        executor_.start_thread(
            "Esp32WiFiConn", EXECUTOR_TASK_PRIORITY, EXECUTOR_TASK_STACK_SIZE);
#endif // !CONFIG_FREERTOS_UNICORE
    }
    else if (configCrc32 != configCrc32_)
    {
        // If a configuration change has been detected, wake up the wifi stack
        // so it can consume the change.
        wifiStackFlow_.notify();
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
    // NOTE: this is not using CDI_FACTORY_RESET so consumers can provide a
    // default value as part of constructing the Esp32WiFiManager instance.
    cfg_.connection_mode().write(fd, connectionMode_);

    // Hub specific configuration settings.
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

void Esp32WiFiManager::process_idf_event(void *arg, esp_event_base_t event_base
                                       , int32_t event_id, void *event_data)
{
    LOG(VERBOSE, "Esp32WiFiManager::process_idf_event(%s, %" PRIi32 ", %p)",
        event_base, event_id, event_data);
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        Singleton<Esp32WiFiManager>::instance()->on_station_started();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        Singleton<Esp32WiFiManager>::instance()->on_station_connected();
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        Singleton<Esp32WiFiManager>::instance()->on_station_disconnected(
            static_cast<wifi_event_sta_disconnected_t *>(event_data)->reason);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START)
    {
        Singleton<Esp32WiFiManager>::instance()->on_softap_start();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP)
    {
        Singleton<Esp32WiFiManager>::instance()->on_softap_stop();
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        auto sta_data = static_cast<wifi_event_ap_staconnected_t *>(event_data);
        Singleton<Esp32WiFiManager>::instance()->on_softap_station_connected(
            sta_data->mac, sta_data->aid);
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        auto sta_data = static_cast<wifi_event_ap_staconnected_t *>(event_data);
        Singleton<Esp32WiFiManager>::instance()->on_softap_station_disconnected(
            sta_data->mac, sta_data->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        auto scan_data = static_cast<wifi_event_sta_scan_done_t *>(event_data);
        Singleton<Esp32WiFiManager>::instance()->on_wifi_scan_completed(
            scan_data->status, scan_data->number);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *data = static_cast<ip_event_got_ip_t *>(event_data);
        Singleton<Esp32WiFiManager>::instance()->on_station_ip_assigned(
            htonl(data->ip_info.ip.addr));
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP)
    {
        Singleton<Esp32WiFiManager>::instance()->on_station_ip_lost();
    }
}

// Adds a callback which will be called when the network is up.
void Esp32WiFiManager::register_network_up_callback(
    esp_network_up_callback_t callback)
{
    OSMutexLock l(&networkCallbacksLock_);
    networkUpCallbacks_.push_back(callback);
}

// Adds a callback which will be called when the network is down.
void Esp32WiFiManager::register_network_down_callback(
    esp_network_down_callback_t callback)
{
    OSMutexLock l(&networkCallbacksLock_);
    networkDownCallbacks_.push_back(callback);
}

// Adds a callback which will be called when the network is initializing.
void Esp32WiFiManager::register_network_init_callback(
    esp_network_init_callback_t callback)
{
    OSMutexLock l(&networkCallbacksLock_);
    networkInitCallbacks_.push_back(callback);
}

// Adds a callback which will be called when SNTP packets are processed.
void Esp32WiFiManager::register_network_time_callback(
    esp_network_time_callback_t callback)
{
    OSMutexLock l(&networkCallbacksLock_);
    networkTimeCallbacks_.push_back(callback);
}

// Shuts down the hub listener (if enabled and running) for this node.
void Esp32WiFiManager::stop_hub()
{
    auto stack = static_cast<openlcb::SimpleCanStackBase *>(stack_);
    stack->shutdown_tcp_hub_server();
}

// Creates a hub listener for this node after loading configuration details.
void Esp32WiFiManager::start_hub()
{
    auto stack = static_cast<openlcb::SimpleCanStackBase *>(stack_);

    // Check if we already have a GC Hub running or not, if we do then we can
    // skip creating one. It would be best to validate the port in use but it
    // is not exposed by GcHub at this time.
    if (!stack->get_tcp_hub_server())
    {
        hubServiceName_ = cfg_.hub().service_name().read(configFd_);
        uint16_t hub_port = CDI_READ_TRIMMED(cfg_.hub().port, configFd_);
        LOG(INFO, "[HUB] Starting TCP/IP listener on port %" PRIu16, hub_port);
        stack->start_tcp_hub_server(hub_port);
        auto hub = stack->get_tcp_hub_server();

        // wait for the hub to complete it's startup tasks
        while (!hub->is_started())
        {
            usleep(HUB_STARTUP_DELAY_USEC);
        }

        mdns_publish(hubServiceName_, hub_port);
    }
}

// Disconnects and shuts down the uplink connector socket if running.
void Esp32WiFiManager::stop_uplink()
{
    if (uplink_)
    {
        LOG(INFO, "[Uplink] Disconnecting from uplink.");
        uplink_->shutdown();
        uplink_.reset(nullptr);

        // Mark our cached notifiable as invalid
        uplinkNotifiable_ = nullptr;

        // force close the socket to ensure it is disconnected and cleaned up.
        ::close(uplinkFd_);
    }
}

// Creates an uplink connector socket that will automatically add the uplink to
// the node's hub.
void Esp32WiFiManager::start_uplink()
{
    unique_ptr<SocketClientParams> params(
        new Esp32SocketParams(configFd_, cfg_.uplink()));

    if (uplink_)
    {
        // If we already have an uplink, update the parameters it is using.
        uplink_->reset_params(std::move(params));
    }
    else
    {
        // create a new uplink and pass in the parameters.
        uplink_.reset(new SocketClient(stack_->service(), &executor_,
            &executor_, std::move(params),
            std::bind(&Esp32WiFiManager::on_uplink_created, this,
                std::placeholders::_1, std::placeholders::_2)));
    }
}

// Converts the passed fd into a GridConnect port and adds it to the stack.
void Esp32WiFiManager::on_uplink_created(int fd, Notifiable *on_exit)
{
    LOG(INFO, "[Uplink] Connected to hub, configuring GridConnect HubPort.");

    // stash the socket handle and notifiable for future use if we need to
    // clean up or re-establish the connection.
    uplinkFd_ = fd;
    uplinkNotifiable_ = on_exit;

    const bool use_select =
        (config_gridconnect_tcp_use_select() == CONSTANT_TRUE);

    // create the GridConnect port from the provided socket fd.
    // NOTE: this uses a local notifiable object instead of the provided
    // on_exit since it will be invalidated upon calling stop_uplink() which
    // may result in a crash or other undefined behavior.
    create_gc_port_for_can_hub(
        static_cast<openlcb::SimpleCanStackBase *>(stack_)->can_hub(),
        uplinkFd_, &uplinkNotifiableProxy_, use_select);

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
wifi_ap_record_t Esp32WiFiManager::get_ssid_scan_result(size_t index)
{
    OSMutexLock l(&ssidScanResultsLock_);
    wifi_ap_record_t record = wifi_ap_record_t();
    if (index < ssidScanResults_.size())
    {
        record = ssidScanResults_[index];
    }
    return record;
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
    executor_.add(new CallbackExecutable([service, port]()
    {
        string service_name = service;
        string protocol_name;
        split_mdns_service_name(&service_name, &protocol_name);
        esp_err_t res = mdns_service_add(
            NULL, service_name.c_str(), protocol_name.c_str(), port, NULL, 0);
        LOG(VERBOSE, "[mDNS] mdns_service_add(%s.%s:%" PRIu16 "):%s.",
            service_name.c_str(), protocol_name.c_str(), port,
            esp_err_to_name(res));
        // ESP_FAIL will be triggered if there is a timeout during publish of
        // the new mDNS entry. The mDNS task runs at a very low priority on the
        // PRO_CPU which is also where the OpenMRN Executor runs from which can
        // cause a race condition.
        if (res == ESP_FAIL)
        {
            // Send it back onto the scheduler to be retried
            Singleton<Esp32WiFiManager>::instance()->mdns_publish(
                service, port);
        }
        else if (res == ESP_ERR_INVALID_ARG)
        {
            // ESP_ERR_INVALID_ARG can be returned if the mDNS server is not UP
            // which we have previously checked via mdnsInitialized_, this can
            // also be returned if the service name has already been published
            // since the server came up. If we see this error code returned we
            // should try unpublish and then publish again.
            Singleton<Esp32WiFiManager>::instance()->mdns_unpublish(service);
            Singleton<Esp32WiFiManager>::instance()->mdns_publish(
                service, port);
        }
        else if (res == ESP_OK)
        {
            LOG(INFO, "[mDNS] Advertising %s.%s:%" PRIu16 ".",
                service_name.c_str(), protocol_name.c_str(), port);
        }
        else
        {
            LOG_ERROR("[mDNS] Failed to publish:%s.%s:%" PRIu16,
                      service_name.c_str(), protocol_name.c_str(), port);
            Singleton<Esp32WiFiManager>::instance()->mdns_publish(
                service, port);
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
    LOG(VERBOSE, "[mDNS] mdns_service_remove:%s.", esp_err_to_name(res));
    // TODO: should we queue up unpublish requests for future retries?
}

// Initializes the mDNS system on the ESP32.
//
// After initialization, if any services are pending publish they will be
// published at this time.
void Esp32WiFiManager::start_mdns_system()
{
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
        LOG(INFO, "[mDNS] Setting hostname to \"%s\"", hostname_.c_str());
        ESP_ERROR_CHECK(mdns_hostname_set(hostname_.c_str()));

        // Set the default mDNS instance name to the generated hostname.
        ESP_ERROR_CHECK(mdns_instance_name_set(hostname_.c_str()));

        // Set flag to indicate we have initialized mDNS.
        mdnsInitialized_ = true;
    }

    // Publish any deferred mDNS entries
    for (auto & entry : mdnsDeferredPublish_)
    {
        mdns_publish(entry.first, entry.second);
    }
    mdnsDeferredPublish_.clear();
}

void Esp32WiFiManager::on_station_started()
{
    // Set the generated hostname prior to connecting to the SSID
    // so that it shows up with the generated hostname instead of
    // the default "Espressif".
    LOG(INFO, "[Station] Setting hostname to \"%s\".", hostname_.c_str());
    esp_netif_set_hostname(espNetIfaces_[STATION_INTERFACE], hostname_.c_str());
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    LOG(INFO, "[Station] MAC Address:%s", mac_to_string(mac).c_str());

    // Start the DHCP service before connecting so it hooks into
    // the flow early and provisions the IP automatically.
    LOG(INFO, "[Station] Starting DHCP Client.");
    ESP_ERROR_CHECK(esp_netif_dhcpc_start(espNetIfaces_[STATION_INTERFACE]));

    LOG(INFO, "[Station] Connecting to SSID:%s.", ssid_.c_str());
    // Start the SSID connection process.
    esp_wifi_connect();

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp_network_init_callback_t cb : networkInitCallbacks_)
        {
            executor_.add(new CallbackExecutable([cb]
            {
                cb(STATION_INTERFACE);
            }));
        }
    }
}

void Esp32WiFiManager::on_station_connected()
{
    LOG(INFO, "[Station] Connected to SSID:%s", ssid_.c_str());
    // Set the flag that indictes we are connected to the SSID.
    xEventGroupSetBits(wifiStatusEventGroup_, WIFI_CONNECTED_BIT);
}

void Esp32WiFiManager::on_station_disconnected(uint8_t reason)
{
    // flag to indicate that we should print the reconnecting log message.
    bool was_previously_connected = false;

    // capture the current state so we can check if we were already connected
    // with an IP address or still in the connecting phase.
    EventBits_t event_bits = xEventGroupGetBits(wifiStatusEventGroup_);

    // Check if we have already connected, this event can be raised
    // even before we have successfully connected during the SSID
    // connect process.
    if (event_bits & WIFI_CONNECTED_BIT)
    {
        // If we were previously connected and had an IP address we should
        // count that as previously connected, otherwise we will just reconnect
        // and not wake up the state flow since it may be waiting for an event
        // and will wakeup on it's own.
        was_previously_connected = event_bits & WIFI_GOTIP_BIT;

        LOG(INFO, "[Station] Lost connection to SSID:%s (reason:%d)",
            ssid_.c_str(), reason);
        // Clear the flag that indicates we are connected to the SSID.
        xEventGroupClearBits(wifiStatusEventGroup_, WIFI_CONNECTED_BIT);
        // Clear the flag that indicates we have an IPv4 address.
        xEventGroupClearBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);
    }

    // If we are managing the WiFi and MDNS systems we need to
    // trigger the reconnection process at this point.
    if (was_previously_connected)
    {
        LOG(INFO, "[Station] Reconnecting to SSID:%s.",
            ssid_.c_str());
        wifiStackFlow_.notify();
    }
    else
    {
        LOG(INFO,
            "[Station] Connection to SSID:%s (reason:%d) failed, retrying.",
            ssid_.c_str(), reason);
    }
    esp_wifi_connect();

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp_network_init_callback_t cb : networkInitCallbacks_)
        {
            executor_.add(new CallbackExecutable([cb]
            {
                cb(STATION_INTERFACE);
            }));
        }
    }
}

void Esp32WiFiManager::on_station_ip_assigned(uint32_t ip_address)
{
    LOG(INFO, "[Station] IP address:%s", ipv4_to_string(ip_address).c_str());

    // Start the mDNS system since we have an IP address, the mDNS system
    // on the ESP32 requires that the IP address be assigned otherwise it
    // will not start the UDP listener.
    start_mdns_system();

    // Set the flag that indictes we have an IPv4 address.
    xEventGroupSetBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);

    // wake up the stack to process the event
    wifiStackFlow_.notify();

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp_network_up_callback_t cb : networkUpCallbacks_)
        {
            executor_.add(new CallbackExecutable([cb, ip_address]
            {
                cb(STATION_INTERFACE, ip_address);
            }));
        }
    }

    configure_sntp();
    reconfigure_wifi_tx_power();
    if (statusLed_)
    {
        statusLed_->write(true);
    }
}

void Esp32WiFiManager::on_station_ip_lost()
{
    // Clear the flag that indicates we are connected and have an
    // IPv4 address.
    xEventGroupClearBits(wifiStatusEventGroup_, WIFI_GOTIP_BIT);

    // wake up the stack to process the event
    wifiStackFlow_.notify();

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp_network_down_callback_t cb : networkDownCallbacks_)
        {
            executor_.add(new CallbackExecutable([cb]
            {
                cb(STATION_INTERFACE);
            }));
        }
    }
}

void Esp32WiFiManager::on_softap_start()
{
    uint32_t ip_address = 0;
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    LOG(INFO, "[SoftAP] MAC Address:%s", mac_to_string(mac).c_str());

    // Set the generated hostname prior to connecting to the SSID
    // so that it shows up with the generated hostname instead of
    // the default "Espressif".
    LOG(INFO, "[SoftAP] Setting hostname to \"%s\".", hostname_.c_str());
    ESP_ERROR_CHECK(esp_netif_set_hostname(
        espNetIfaces_[SOFTAP_INTERFACE], hostname_.c_str()));

    // fetch the IP address from the adapter since it defaults to
    // 192.168.4.1 but can be altered via sdkconfig.
    esp_netif_ip_info_t ip_info;
    ESP_ERROR_CHECK(
        esp_netif_get_ip_info(espNetIfaces_[SOFTAP_INTERFACE], &ip_info));
    ip_address = ntohl(ip4_addr_get_u32(&ip_info.ip));

    LOG(INFO, "[SoftAP] IP address:%s", ipv4_to_string(ip_address).c_str());

    // If we are operating only in SoftAP mode we can start any background
    // services that would also be started when the station interface is ready
    // and has an IP address.
    if (wifiMode_ == WIFI_MODE_AP)
    {
        start_mdns_system();
        reconfigure_wifi_tx_power();
        if (connectionMode_ & CONN_MODE_HUB_BIT)
        {
            start_hub();
        }
    }

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp_network_up_callback_t cb : networkUpCallbacks_)
        {
            executor_.add(new CallbackExecutable([cb, ip_address]
            {
                cb(SOFTAP_INTERFACE, htonl(ip_address));
            }));
        }
    }
}

void Esp32WiFiManager::on_softap_stop()
{
    if (wifiMode_ == WIFI_MODE_AP)
    {
        stop_uplink();
        stop_hub();
    }

    // Schedule callbacks via the executor rather than call directly here.
    {
        OSMutexLock l(&networkCallbacksLock_);
        for (esp_network_down_callback_t cb : networkDownCallbacks_)
        {
            executor_.add(new CallbackExecutable([cb]
            {
                cb(SOFTAP_INTERFACE);
            }));
        }
    }
}

void Esp32WiFiManager::on_softap_station_connected(uint8_t mac[6], uint8_t aid)
{
    LOG(INFO, "[SoftAP aid:%d] %s connected.", aid,
        mac_to_string(mac).c_str());
}

void Esp32WiFiManager::on_softap_station_disconnected(uint8_t mac[6],
    uint8_t aid)
{
    LOG(INFO, "[SoftAP aid:%d] %s disconnected.", aid,
        mac_to_string(mac).c_str());
}

void Esp32WiFiManager::on_wifi_scan_completed(uint32_t status, uint8_t count)
{
    OSMutexLock l(&ssidScanResultsLock_);
    if (status)
    {
        LOG_ERROR("[WiFi] SSID scan failed!");
    }
    else
    {
        uint16_t num_found = count;
        esp_wifi_scan_get_ap_num(&num_found);
        LOG(VERBOSE, "[WiFi] %" PRIu16 " SSIDs found via scan", num_found);
        ssidScanResults_.resize(num_found);
        esp_wifi_scan_get_ap_records(&num_found, ssidScanResults_.data());
#if LOGLEVEL >= VERBOSE
        for (int i = 0; i < num_found; i++)
        {
            LOG(VERBOSE, "SSID:%s, RSSI:%d, channel:%d"
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

// SNTP callback hook to schedule callbacks.
void Esp32WiFiManager::sync_time(time_t now)
{
    OSMutexLock l(&networkCallbacksLock_);
    for (esp_network_time_callback_t cb : networkTimeCallbacks_)
    {
        executor_.add(new CallbackExecutable([cb,now]
        {
            cb(now);
        }));
    }
}

static void sntp_update_received(struct timeval *tv)
{
    time_t new_time = tv->tv_sec;
    LOG(INFO, "[SNTP] Received time update, new localtime:%s"
      , ctime(&new_time));
    Singleton<Esp32WiFiManager>::instance()->sync_time(new_time);
}

void Esp32WiFiManager::configure_sntp()
{
    if (sntpEnabled_ && !sntpConfigured_)
    {
        sntpConfigured_ = true;
        LOG(INFO, "[SNTP] Polling %s for time updates", sntpServer_.c_str());
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,1,0)
        esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, sntpServer_.c_str());
        sntp_set_time_sync_notification_cb(sntp_update_received);
        esp_sntp_init();
#else
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, sntpServer_.c_str());
        sntp_set_time_sync_notification_cb(sntp_update_received);
        sntp_init();
#endif // IDF v5.1+

        if (!timeZone_.empty())
        {
            LOG(INFO, "[TimeZone] %s", timeZone_.c_str());
            setenv("TZ", timeZone_.c_str(), 1);
            tzset();
        }
    }
}

void Esp32WiFiManager::reconfigure_wifi_radio_sleep()
{
    wifi_ps_type_t current_mode = WIFI_PS_NONE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_get_ps(&current_mode));
    uint8_t sleepEnabled = CDI_READ_TRIMMED(cfg_.sleep, configFd_);

    if (sleepEnabled && current_mode != WIFI_PS_MIN_MODEM)
    {
        LOG(INFO, "[WiFi] Enabling radio power saving mode");
        // When sleep is enabled this will trigger the WiFi system to
        // only wake up every DTIM period to receive beacon updates.
        // no data loss is expected for this setting but it does delay
        // receiption until the DTIM period.
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    }
    else if (!sleepEnabled && current_mode != WIFI_PS_NONE)
    {
        LOG(INFO, "[WiFi] Disabling radio power saving mode");
        // When sleep is disabled the WiFi radio will always be active.
        // This will increase power consumption of the ESP32 but it
        // will result in a more reliable behavior when the ESP32 is
        // connected to an always-on power supply (ie: not a battery).
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    }
}

void Esp32WiFiManager::reconfigure_wifi_tx_power()
{
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(wifiTXPower_));
}

Esp32WiFiManager::WiFiStackFlow::WiFiStackFlow(Esp32WiFiManager * parent)
    : StateFlowBase(parent), parent_(parent),
    wifiConnectBitMask_(WIFI_CONNECTED_BIT)
{
    start_flow(STATE(startup));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::startup()
{
    if (parent_->wifiMode_ != WIFI_MODE_NULL)
    {
        return yield_and_call(STATE(init_interface));
    }
    return yield_and_call(STATE(noop));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::noop()
{
    return wait_and_call(STATE(noop));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::init_interface()
{
    // create default interfaces for station and SoftAP, ethernet is not used
    // today.
    ESP_ERROR_CHECK(esp_netif_init());

    // create the event loop.
    esp_err_t err = esp_event_loop_create_default();

    // The esp_event_loop_create_default() method will return either ESP_OK if
    // the event loop was created or ESP_ERR_INVALID_STATE if one already
    // exists.
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        LOG(FATAL, "[WiFi] Failed to initialize the default event loop:%s",
            esp_err_to_name(err));
    }

    parent_->espNetIfaces_[STATION_INTERFACE] =
        esp_netif_create_default_wifi_sta();
    parent_->espNetIfaces_[SOFTAP_INTERFACE] =
        esp_netif_create_default_wifi_ap();

    // Connect our event listeners.
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               &Esp32WiFiManager::process_idf_event, nullptr);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                               &Esp32WiFiManager::process_idf_event, nullptr);

    return yield_and_call(STATE(init_wifi));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::init_wifi()
{
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
    if (cfg.static_rx_buf_num < 16)
    {
        cfg.static_rx_buf_num = 16;
    }
    if (cfg.dynamic_rx_buf_num < 32)
    {
        cfg.dynamic_rx_buf_num = 32;
    }
    if (cfg.rx_ba_win < 16)
    {
        cfg.rx_ba_win = 16;
    }

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (parent_->verboseLogging_)
    {
        parent_->enable_esp_wifi_logging();
    }

    // Create the event group used for tracking connected/disconnected status.
    // This is used internally regardless of if we manage the rest of the WiFi
    // or mDNS systems.
    parent_->wifiStatusEventGroup_ = xEventGroupCreate();

    wifi_mode_t requested_wifi_mode = parent_->wifiMode_;
    if (parent_->wifiMode_ == WIFI_MODE_AP)
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

    if (parent_->wifiMode_ == WIFI_MODE_APSTA ||
        parent_->wifiMode_ == WIFI_MODE_AP)
    {
        return yield_and_call(STATE(configure_softap));
    }
    return yield_and_call(STATE(configure_station));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::configure_station()
{
    // Configure the SSID details for the station based on the SSID and
    // password provided to the Esp32WiFiManager constructor.
    wifi_config_t conf;
    memset(&conf, 0, sizeof(wifi_config_t));
    strcpy(reinterpret_cast<char *>(conf.sta.ssid), parent_->ssid_.c_str());
    if (!parent_->password_.empty())
    {
        strcpy(reinterpret_cast<char *>(conf.sta.password),
               parent_->password_.c_str());
    }

    LOG(INFO, "[WiFi] Configuring Station (SSID:%s)", conf.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &conf));

    return yield_and_call(STATE(start_wifi));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::configure_softap()
{
    wifi_config_t conf;
    memset(&conf, 0, sizeof(wifi_config_t));
    conf.ap.authmode = parent_->softAPAuthMode_;
    conf.ap.beacon_interval = 100;
    conf.ap.channel = parent_->softAPChannel_;
    conf.ap.max_connection = 4;

    if (!parent_->softAPName_.empty())
    {
        // Configure the SSID for the Soft AP based on the SSID passed
        // to the Esp32WiFiManager constructor.
        strcpy(reinterpret_cast<char *>(conf.ap.ssid),
                parent_->softAPName_.c_str());
    }
    else if (parent_->wifiMode_ == WIFI_MODE_AP && !parent_->ssid_.empty())
    {
        strcpy(reinterpret_cast<char *>(conf.ap.ssid),
                parent_->ssid_.c_str());
    }
    else
    {
        // Configure the SSID for the Soft AP based on the generated
        // hostname when operating in WIFI_MODE_APSTA mode.
        strcpy(reinterpret_cast<char *>(conf.ap.ssid),
                parent_->hostname_.c_str());
    }

    if (parent_->softAPAuthMode_ != WIFI_AUTH_OPEN)
    {
        if (!parent_->softAPPassword_.empty())
        {
            strcpy(reinterpret_cast<char *>(conf.ap.password),
                    parent_->softAPPassword_.c_str());
        }
        else if (!parent_->password_.empty())
        {
            strcpy(reinterpret_cast<char *>(conf.ap.password),
                    parent_->password_.c_str());
        }
        else
        {
            LOG(WARNING,
                "[WiFi] SoftAP password is blank, using OPEN auth mode.");
            parent_->softAPAuthMode_ = WIFI_AUTH_OPEN;
        }
    }

    LOG(INFO, "[WiFi] Configuring SoftAP (SSID:%s)", conf.ap.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &conf));

    // If we are only enabling the SoftAP we can transition to starting the
    // WiFi stack.
    if (parent_->wifiMode_ == WIFI_MODE_AP)
    {
        return yield_and_call(STATE(start_wifi));
    }
    // Configure the station interface
    return yield_and_call(STATE(configure_station));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::start_wifi()
{
    // Start the WiFi stack. This will start the SoftAP and/or connect to the
    // SSID based on the configuration set above.
    LOG(INFO, "[WiFi] Starting WiFi stack");
    ESP_ERROR_CHECK(esp_wifi_start());

    // force WiFi power to maximum during startup.
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    if (parent_->wifiMode_ != WIFI_MODE_AP && parent_->waitForStationConnect_)
    {
        return sleep_and_call(&timer_,
                              SEC_TO_NSEC(WIFI_CONNECT_CHECK_INTERVAL_SEC),
                              STATE(wait_for_connect));
    }
    return wait_and_call(STATE(reload));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::wait_for_connect()
{
    EventBits_t bits =
        xEventGroupWaitBits(parent_->wifiStatusEventGroup_,
                            wifiConnectBitMask_, pdFALSE, pdTRUE, 0);
    // If we need the STATION interface *AND* configured to wait until 
    // successfully connected to the SSID this code block will wait for up to
    // approximately three minutes for an IP address to be assigned. In most
    // cases this completes in under thirty seconds. If there is a connection
    // failure the esp32 will be restarted via a FATAL error being logged.
    if (++wifiConnectAttempts_ <= MAX_CONNECTION_CHECK_ATTEMPTS)
    {
        // If we have connected to the SSID we then are waiting for IP
        // address.
        if (bits & WIFI_CONNECTED_BIT)
        {
            LOG(INFO, "[IPv4] [%d/%d] Waiting for IP address assignment.",
                wifiConnectAttempts_, MAX_CONNECTION_CHECK_ATTEMPTS);
        }
        else
        {
            // Waiting for SSID connection
            LOG(INFO, "[WiFi] [%d/%d] Waiting for SSID connection.",
                wifiConnectAttempts_, MAX_CONNECTION_CHECK_ATTEMPTS);
        }
        // Check if have connected to the SSID
        if (bits & WIFI_CONNECTED_BIT)
        {
            // Since we have connected to the SSID we now need to track
            // that we get an IP.
            wifiConnectBitMask_ |= WIFI_GOTIP_BIT;
        }
        // Check if we have received an IP.
        if (bits & WIFI_GOTIP_BIT)
        {
            return yield_and_call(STATE(reload));
        }
        return sleep_and_call(&timer_,
                              SEC_TO_NSEC(WIFI_CONNECT_CHECK_INTERVAL_SEC),
                              STATE(wait_for_connect));
    }
    // Check if we successfully connected or not. If not, force a reboot.
    if ((bits & WIFI_CONNECTED_BIT) != WIFI_CONNECTED_BIT)
    {
        LOG(FATAL, "[WiFi] Failed to connect to SSID:%s.",
            parent_->ssid_.c_str());
    }

    // Check if we successfully connected or not. If not, force a reboot.
    if ((bits & WIFI_GOTIP_BIT) != WIFI_GOTIP_BIT)
    {
        LOG(FATAL, "[IPv4] Timeout waiting for an IP.");
    }

    return yield_and_call(STATE(reload));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::reload()
{
    if (parent_->wifiMode_ == WIFI_MODE_STA ||
        parent_->wifiMode_ == WIFI_MODE_APSTA)
    {
        EventBits_t bits = xEventGroupGetBits(parent_->wifiStatusEventGroup_);
        if (!(bits & WIFI_GOTIP_BIT))
        {
            // Since we do not have an IP address we need to shutdown any
            // active connections since they will be invalid until a new IP
            // has been provisioned.
            parent_->stop_hub();
            parent_->stop_uplink();
            return wait_and_call(STATE(reload));
        }
    }

    parent_->reconfigure_wifi_radio_sleep();
    if (parent_->connectionMode_ & CONN_MODE_HUB_BIT)
    {
        parent_->start_hub();
    }
    else
    {
        parent_->stop_hub();
    }

    if (parent_->connectionMode_ & CONN_MODE_UPLINK_BIT)
    {
        parent_->start_uplink();
    }
    else
    {
        parent_->stop_uplink();
    }

    if (parent_->connectionMode_ & CONN_MODE_SHUTDOWN_BIT)
    {
        HASSERT((parent_->connectionMode_ & CONN_MODE_HUB_BIT) == 0);
        HASSERT((parent_->connectionMode_ & CONN_MODE_UPLINK_BIT) == 0);
        // If we have gotten here, then the CONN_MODE_UPLINK_BIT and
        // CONN_MODE_SHUTDOWN_BIT should both be clear. This means that both
        // stop_hub() and stop_uplink() should have been called above. Alow a
        // bit of time for lingering sockets to terminate before completeing the
        // shutdown.
        return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(shutdown));
    }
    return wait_and_call(STATE(reload));
}

StateFlowBase::Action Esp32WiFiManager::WiFiStackFlow::shutdown()
{
    // Some time has passed since calling stop_hub() and stop_uplink(). Now the
    // WiFi can be shutdown.
    LOG(INFO, "[WiFi] Shutting down.");
    esp_wifi_stop();
    return exit();
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
void mdns_unpublish(const char *name, const char *service)
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
            "[mDNS] No matches found for service:%s.",
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
                    "[mDNS] Found %s providing service:%s on port %" PRIu16,
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
            "[mDNS] No matches found for service:%s.", service);
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
    esp_netif_ip_info_t ip_info;

    /* start with something "safe" in case we bail out early */
    *ifap = nullptr;

    // Lookup the interface by it's internal name assigned by ESP-IDF.
    esp_netif_t *iface = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (iface == nullptr)
    {
        // Station interface was not found.
        errno = ENODEV;
        return -1;
    }

    // retrieve TCP/IP address from the interface
    if (esp_netif_get_ip_info(iface, &ip_info) != ESP_OK)
    {
        // Failed to retrieve IP address.
        errno = EADDRNOTAVAIL;
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

#endif // ESP_PLATFORM
