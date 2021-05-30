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

#include "freertos_drivers/esp32/Esp32WiFiConfiguration.hxx"
#include "executor/Executor.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredTcpConnection.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/GcTcpHub.hxx"
#include "utils/macros.h"

#include <freertos/event_groups.h>
#include <esp_idf_version.h>
#include <esp_event.h>
#include <esp_wifi_types.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
#include <esp_netif.h>
#else
#include <tcpip_adapter.h>
#endif // IDF v4.1+

namespace openlcb
{
    class SimpleStackBase;
}

class Gpio;
class SocketClient;

namespace openmrn_arduino
{

/// ESP32 network interfaces.
///
/// At this time only the Station and SoftAP interfaces are supported. There
/// are no plans to support the Ethernet interface at this time.
typedef enum : uint8_t
{
    /// This is used for the Station WiFi interface.
    STATION_INTERFACE = 0,

    /// This is used for the SoftAP WiFi interface.
    SOFTAP_INTERFACE,

    /// This is the maximum supported WiFi interfaces of the ESP32 and is not
    /// a valid network interface for user code.
    MAX_NETWORK_INTERFACES
} esp_network_interface_t;

/// Callback function definition for the network up events.
///
/// The first parameter is the interface that is up and read to use.
/// The second is the IP address for the interface in network byte order.
///
/// NOTE: The callback will be invoked for SOFTAP_INTERFACE (SoftAP) upon start
/// and STATION_INTERFACE (station) only after the IP address has been received.
/// The callback will be called multiple times if both SoftAP and Station are
/// enabled.
typedef std::function<void(esp_network_interface_t
                         , uint32_t)> esp_network_up_callback_t;

/// Callback function definition for the network down events.
///
/// The first parameter is the interface that is down.
///
/// NOTE: The callback will be invoked for SOFTAP_INTERFACE (SoftAP) when the
/// interface is stopped, for STATION_INTERFACE (station) it will be called when
/// the IP address has been lost or connection to the AP has been lost.
typedef std::function<void(esp_network_interface_t)> esp_network_down_callback_t;

/// Callback function definition for the network is initializing.
///
/// The first parameter is the interface that is initializing.
///
/// NOTE: This will be called for STATION_INTERFACE only. It will be called for
/// initial startup and reconnect events.
typedef std::function<void(esp_network_interface_t)> esp_network_init_callback_t;

/// Callback function definition for network time synchronization.
///
/// The parameter is the standard time_t structure containing the new time.
typedef std::function<void(time_t)> esp_network_time_callback_t;

/// This class provides a simple way for ESP32 nodes to manage the WiFi and
/// mDNS systems of the ESP32, the node being a hub and connecting to an
/// uplink node to participate in the CAN bus.
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
    /// @param station_ssid is the WiFi AP to connect to.
    /// @param station_password is the password for the WiFi AP being connected
    /// to.
    /// @param stack is the SimpleCanStack for this node. Must stay alive
    /// forever.
    /// @param cfg is the WiFiConfiguration instance used for this node. This
    /// will be monitored for changes and the WiFi behavior altered
    /// accordingly.
    /// @param wifi_mode is the WiFi operating mode, defaults to WIFI_MODE_STA.
    /// When set to WIFI_MODE_STA the Esp32WiFiManager will attempt to connect
    /// to the configured WiFi station_ssid. When wifi_mode is set to
    /// WIFI_MODE_AP the Esp32WiFiManager will create a SoftAP with the
    /// configured softap_ssid, softap_password, softap_channel, and
    /// softap_auth_mode. When set to WIFI_MODE_APSTA both the SoftAP and
    /// STATION interfaces will be enabled.
    /// @param sntp_server is the SNTP server to poll for time updates, this
    /// defaults to pool.ntp.org.
    /// @param timezone is the POSIX formatted TimeZone of the node, this
    /// defaults to UTC0.
    /// @param sntp_enabled Enables SNTP synchronization, defaults to false.
    /// @param hostname_prefix is the hostname prefix to use for this node.
    /// The @ref NodeID will be appended to this value. The maximum length for
    /// final hostname is 32 bytes.
    /// @param softap_channel is the WiFi channel to use for the SoftAP.
    /// @param softap_auth_mode is the authentication mode for the AP when
    /// wifi_mode is set to WIFI_MODE_AP or WIFI_MODE_APSTA.
    /// @param softap_ssid is the name for the SoftAP, if null the node
    /// hostname will be used.
    /// @param softap_password will be used as the password for the SoftAP,
    /// if null and softap_auth_mode is not WIFI_AUTH_OPEN station_password
    /// will be used.
    Esp32WiFiManager(const char *station_ssid
                   , const char *station_password
                   , openlcb::SimpleStackBase *stack
                   , const WiFiConfiguration &cfg
                   , wifi_mode_t wifi_mode = WIFI_MODE_STA
                   , const char *hostname_prefix = "esp32_"
                   , const char *sntp_server = "pool.ntp.org"
                   , const char *timezone = "UTC0"
                   , bool sntp_enabled = false
                   , uint8_t softap_channel = 1
                   , wifi_auth_mode_t softap_auth_mode = WIFI_AUTH_OPEN
                   , const char *softap_ssid = nullptr
                   , const char *softap_password = nullptr
    );

    /// Destructor.
    ~Esp32WiFiManager();

    /// Configures a @ref Gpio to be used as a visual indication of the current
    /// WiFi status.
    ///
    /// @param led is the @ref Gpio instance connected to the LED.
    void set_status_led(const Gpio *led = nullptr)
    {
        statusLed_ = led;
    }

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
    ConfigUpdateListener::UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) override;

    /// Resets the WiFiConfiguration settings used by this node.
    ///
    /// @param fd is the file descriptor used for the configuration settings.
    void factory_reset(int fd) override;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    /// Processes an event coming from the ESP-IDF default event loop.
    ///
    /// @param ctx context parameter (unused).
    /// @param event_base Determines the category of event being sent.
    /// @param event_id Specific event from the event_base being sent.
    /// @param event_data Data related to the event being sent, may be null.
    ///
    /// NOTE: This is not intended to be called by the user.
    static void process_idf_event(void *ctx, esp_event_base_t event_base
                                , int32_t event_id, void *event_data);
#else
    /// Processes an ESP-IDF WiFi event based on the event raised by the
    /// ESP-IDF event loop processor. This should be used when the
    /// Esp32WiFiManager is not managing the WiFi or MDNS systems so that
    /// it can react to WiFi events to cleanup or recreate the hub or uplink
    /// connections as required. When Esp32WiFiManager is managing the WiFi
    /// connection this method will be called automatically from the
    /// esp_event_loop. Note that ESP-IDF only supports one callback being
    /// registered. 
    ///
    /// @param ctx context parameter (unused).
    /// @param event is the system_event_t raised by ESP-IDF.
    ///
    /// NOTE: This is not intended to be called by the user.
    static esp_err_t process_wifi_event(void *ctx, system_event_t *event);
#endif

    /// If called, sets the ESP32 wifi stack to log verbose information to the
    /// console.
    void enable_verbose_logging()
    {
        verboseLogging_ = true;
        enable_esp_wifi_logging();
    }

    /// Starts a scan for available SSIDs.
    ///
    /// @param n is the @ref Notifiable to notify when the SSID scan completes.
    void start_ssid_scan(Notifiable *n);

    /// @return the number of SSIDs that were found via the scan.
    size_t get_ssid_scan_result_count();

    /// Returns one entry from the SSID scan.
    ///
    /// @param index is the index of the SSID to retrieve. If the index is
    /// invalid or no records exist a blank wifi_ap_record_t will be returned.
    wifi_ap_record_t get_ssid_scan_result(size_t index);

    /// Clears the SSID scan results.
    void clear_ssid_scan_results();

    /// Advertises a service via mDNS.
    ///
    /// @param service is the service name to publish.
    /// @param port is the port for the service to be published.
    ///
    /// Note: This will schedule a @ref CallbackExecutable on the @ref Executor
    /// used by the @ref SimpleCanStack.
    void mdns_publish(std::string service, uint16_t port);

    /// Removes the advertisement of a service via mDNS.
    ///
    /// @param service is the service name to remove from advertising.
    void mdns_unpublish(std::string service);

    /// Forces the Esp32WiFiManager to wait until SSID connection completes.
    ///
    /// @param enable when true will force the Esp32WiFiManager to wait for
    /// successful SSID connection (including IP assignemnt), when false and
    /// the Esp32WiFiManager will not check the SSID connection process.
    ///
    /// The default behavior is to wait for SSID connection to complete when
    /// the WiFi mode is WIFI_MODE_STA or WIFI_MODE_APSTA. When operating in
    /// WIFI_MODE_APSTA mode the application may opt to present a configuration
    /// portal to allow reconfiguration of the SSID.
    void wait_for_ssid_connect(bool enable)
    {
        waitForStationConnect_ = enable;
    }

    /// Configures the WiFi maximum transmit power setting.
    ///
    /// @param power is the maximum transmit power in 0.25dBm units, range is
    /// 8-84 (2-20dBm).
    ///
    /// NOTE: This should be called as early as possible, once the Station or
    /// SoftAP has been started this setting will not be used.
    void set_tx_power(uint8_t power)
    {
        HASSERT(power >= 8 && power <= 84);
        wifiTXPower_ = power;
    }

    /// Registers a callback for when the WiFi connection is up.
    ///
    /// @param callback The callback to invoke when the WiFi connection is
    /// up.
    void register_network_up_callback(esp_network_up_callback_t callback);

    /// Registers a callback for when the WiFi connection is down.
    ///
    /// @param callback The callback to invoke when the WiFi connection is
    /// down.
    void register_network_down_callback(esp_network_down_callback_t callback);

    /// Registers a callback for when WiFi interfaces are being initialized.
    ///
    /// @param callback The callback to invoke when the WiFi interface is
    /// initializing.
    /// 
    /// NOTE: this will not be invoked for ESP_IF_WIFI_AP since there are no
    /// events raised between enabling the interface and when it is ready.
    void register_network_init_callback(esp_network_init_callback_t callback);

    /// Time synchronization callback for SNTP.
    ///
    /// @param now is the current time.
    ///
    /// NOTE: This is not intended to be called by the user.
    void sync_time(time_t now);

private:
    /// Default constructor.
    Esp32WiFiManager();

    /// Starts the WiFi system and initiates the SSID connection process.
    ///
    /// Note: This is a blocking call and will reboot the node if the WiFi
    /// connection is not successful after ~3min.
    void start_wifi_system();

    /// Starts the Esp32WiFiManager, this manages the WiFi subsystem as well as
    /// all interactions with other nodes.
    void start_wifi_task();

    /// Background task used by the Esp32WiFiManager to maintain health of any
    /// connections to other nodes.
    /// @param param is a pointer to the Esp32WiFiManager instance.
    static void *wifi_manager_task(void *param);

    /// Shuts down the hub listener (if running) for this node.
    void stop_hub();

    /// Creates a hub listener for this node after loading configuration
    /// details.
    ///
    /// Note: This method will block until the hub is active.
    void start_hub();

    /// Disconnects and shuts down the uplink connector socket (if running).
    void stop_uplink();

    /// Creates an uplink connector socket that will automatically add the
    /// uplink to the node's hub.
    void start_uplink();

    /// Callback for the @ref SocketClient to handle a newly connected outbound
    /// socket connection.
    ///
    /// @param fd is the connected socket descriptor.
    /// @param on_exit is the Notifiable for when this socket has closed.
    void on_uplink_created(int fd, Notifiable *on_exit);

    /// Enables the esp_wifi logging, including the esp_wifi_internal APIs when
    /// available.
    void enable_esp_wifi_logging();

    /// Initializes the mDNS system if it hasn't already been initialized.
    void start_mdns_system();

    /// Event handler called when the ESP32 Station interface has started.
    ///
    /// This will handle configuration of any static IP address, hostname, DNS
    /// and initiating the SSID connection process.
    void on_station_started();

    /// Event handler called when the ESP32 Station interface has connected to
    /// an SSID.
    void on_station_connected();

    /// Event handler called when the ESP32 Station interface has lost it's
    /// connection to the SSID or failed to connect.
    ///
    /// @param reason The reason for the disconnected event.
    void on_station_disconnected(uint8_t reason);

    /// Event handler called when the ESP32 Station interface has received an
    /// IP address (DHCP or static).
    void on_station_ip_assigned(uint32_t ip_address);

    /// Event handler called when the ESP32 Station interface has lost it's
    /// assigned IP address.
    void on_station_ip_lost();

    /// Event handler called when the ESP32 SoftAP interface has started.
    ///
    /// This will handle the configuration of the SoftAP Static IP (if used).
    void on_softap_start();

    /// Event handler called when the ESP32 SoftAP interface has shutdown.
    void on_softap_stop();

    /// Event handler called when a station connects to the ESP32 SoftAP.
    ///
    /// @param mac Station MAC address.
    /// @param aid Station access point identifier.
    void on_softap_station_connected(uint8_t mac[6], uint8_t aid);

    /// Event handler called when a station disconnects from the ESP32 SoftAP.
    ///
    /// @param mac Station MAC address.
    /// @param aid Station access point identifier.
    void on_softap_station_disconnected(uint8_t mac[6], uint8_t aid);

    /// Event handler called when a WiFi scan operation completes.
    ///
    /// @param status is the status of the WiFi scan request.
    /// @param count is the number of access points found.
    void on_wifi_scan_completed(uint32_t status, uint8_t count);

    /// Configures SNTP and TimeZone (if enabled).
    void configure_sntp();

    /// Reconfigures the WiFi radio sleep mode.
    void reconfigure_wifi_radio_sleep();
    
    /// Reconfigures the WiFi radio transmit power.
    ///
    /// NOTE: This will only be called after the station connection has been
    /// established or after the SoftAP has been started. Before these events
    /// the transmit power will be configured to the maximum value.
    void reconfigure_wifi_tx_power();

    /// Handle for the wifi_manager_task that manages the WiFi stack, including
    /// periodic health checks of the connected hubs or clients.
    os_thread_t wifiTaskHandle_;

    /// Dynamically generated hostname for this node, esp32_{node-id}. This is
    /// also used for the SoftAP SSID name (if enabled).
    std::string hostname_;

    /// User provided SSID to connect to.
    std::string ssid_;

    /// User provided password for the SSID to connect to.
    std::string password_;

    /// Persistent configuration that will be used for this node's WiFi usage.
    const WiFiConfiguration cfg_;

    /// OpenMRN stack for the Arduino system.
    openlcb::SimpleStackBase *stack_;

    /// WiFi connection status indicator @ref Gpio instance.
    const Gpio *statusLed_{nullptr};

    /// WiFi operating mode.
    const wifi_mode_t wifiMode_;

    /// Channel to use for the SoftAP interface.
    const uint8_t softAPChannel_;

    /// Authentication mode to use for the SoftAP. If not set to WIFI_AUTH_OPEN
    /// @ref softAPPassword_ will be used.
    wifi_auth_mode_t softAPAuthMode_;

    /// User provided name for the SoftAP when active, defaults to
    /// @ref hostname_ when null.
    /// NOTE: Only used when @ref wifiMode_ is set to WIFI_MODE_AP or
    /// WIFI_MODE_APSTA.
    std::string softAPName_;

    /// User provided password for the SoftAP when active, defaults to
    /// @ref password when null and softAPAuthMode_ is not WIFI_AUTH_OPEN.
    /// NOTE: Only used when @ref wifiMode_ is set to WIFI_MODE_AP or
    /// WIFI_MODE_APSTA.
    std::string softAPPassword_;

    /// Enables SNTP polling.
    const bool sntpEnabled_;

    /// SNTP server address.
    std::string sntpServer_;

    /// TimeZone of the node.
    std::string timeZone_;

    /// Tracks if SNTP has been configured.
    bool sntpConfigured_{false};

    /// Cached copy of the file descriptor passed into apply_configuration.
    /// This is internally used by the wifi_manager_task to processed deferred
    /// configuration load.
    int configFd_{-1};

    /// Calculated CRC-32 of cfg_ data. Used to detect changes in configuration
    /// which may require the wifi_manager_task to reload config.
    uint32_t configCrc32_{0};

    /// Internal flag to request the wifi_manager_task reload configuration.
    bool configReloadRequested_{true};

    /// Internal flag to request the wifi_manager_task to shutdown.
    bool shutdownRequested_{false};

    /// If true, the esp32 will block startup until the SSID connection has
    /// successfully completed and upon failure (or timeout) the esp32 will be
    /// restarted.
    bool waitForStationConnect_{true};

    /// If true, request esp32 wifi to do verbose logging.
    bool verboseLogging_{false};

    /// If true, the managed uplink will be started and stopped when necessary.
    bool uplinkEnabled_{false};

    /// If true, the managed hub will be started and stopped when necessary.
    bool hubEnabled_{false};

    /// Maximum WiFi transmit power setting.
    uint8_t wifiTXPower_{84};

    /// mDNS service name being advertised by the hub, if enabled.
    std::string hubServiceName_;

    /// @ref SocketClient for this node's uplink.
    std::unique_ptr<SocketClient> uplink_;

    /// Internal event group used to track the IP assignment events.
    EventGroupHandle_t wifiStatusEventGroup_;

    /// WiFi SSID scan results holder.
    std::vector<wifi_ap_record_t> ssidScanResults_;

    /// Protects ssidScanResults_ vector.
    OSMutex ssidScanResultsLock_;

    /// Notifiable to be called when SSID scan completes.
    Notifiable *ssidCompleteNotifiable_{nullptr};

    /// Protects the mdnsInitialized_ flag and mdnsDeferredPublish_ map.
    OSMutex mdnsInitLock_;

    /// Internal flag for tracking that the mDNS system has been initialized.
    bool mdnsInitialized_{false};

    /// Executor to use for the uplink connections and callbacks.
    Executor<1> executor_{NO_THREAD()};

    /// Internal holder for mDNS entries which could not be published due to
    /// mDNS not being initialized yet.
    std::map<std::string, uint16_t> mdnsDeferredPublish_;

    /// Protects the networkUpCallbacks_, networkDownCallbacks_,
    /// networkInitCallbacks_ and networkTimeCallbacks_ vectors.
    OSMutex networkCallbacksLock_;

    /// Holder for callbacks to invoke when the WiFi connection is up.
    std::vector<esp_network_up_callback_t> networkUpCallbacks_;

    /// Holder for callbacks to invoke when the WiFi connection is down.
    std::vector<esp_network_down_callback_t> networkDownCallbacks_;

    /// Holder for callbacks to invoke when the WiFi subsystem has started.
    std::vector<esp_network_init_callback_t> networkInitCallbacks_;

    /// Holder for callbacks to invoke when network time synchronizes.
    std::vector<esp_network_time_callback_t> networkTimeCallbacks_;

    /// Maximum length of the hostname for the ESP32.
    static constexpr uint8_t MAX_HOSTNAME_LENGTH = 32;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,1,0)
    /// Network interfaces that are managed by Esp32WiFiManager.
    esp_netif_t *espNetIfaces_[MAX_NETWORK_INTERFACES]
    {
        nullptr, nullptr
    };
#endif // IDF v4.1+

    DISALLOW_COPY_AND_ASSIGN(Esp32WiFiManager);
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32WiFiManager;
using openmrn_arduino::esp_network_interface_t;
using openmrn_arduino::esp_network_up_callback_t;
using openmrn_arduino::esp_network_down_callback_t;
using openmrn_arduino::esp_network_init_callback_t;
using openmrn_arduino::esp_network_time_callback_t;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32WIFIMGR_HXX_
