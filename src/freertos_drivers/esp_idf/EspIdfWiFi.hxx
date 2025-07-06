/** @copyright
 * Copyright (c) 2024, Balazs Racz, 2025, Stuart Baker
 * All rights reserved
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
 * @file EspIdfWiFi.hxx
 *
 * WiFi manager for ESP-IDF platform. Implements WiFiInterface.
 *
 * @author Balazs Racz, extended by Stuart Baker
 * @date 27 Aug 2024, extended starting Jun 17 2025
 */

#ifndef _FREERTOS_DRIVERS_ESP_IDF_ESPIDFWIFI_HXX_
#define _FREERTOS_DRIVERS_ESP_IDF_ESPIDFWIFI_HXX_

#include <esp_netif.h>
#include <nvs_flash.h>

#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include "freertos_drivers/common/WiFiInterface.hxx"
#include "openmrn_features.h"

#include <sys/socket.h>

#include <list>

#if OPENMRN_HAVE_BSD_SOCKETS_IPV6
#if CONFIG_LWIP_IPV6
#define ESP_IDF_WIFI_IPV6
#endif
#endif

/// Object for managing WiFi and network interfaces. The following sequence
/// must be called within or after app_main() in the following order:
///   -# init()
///       - performs initialize before starting WiFi
///   -# setup_ap()
///       - option, can be used to override the stored AP profile in bootloader
///   -# start()
///       - starts the WiFi based on config (AP, STA, or AP + STA mode)
///
/// By default, all user configuration is volatile, and will be forgotten upon
/// reboot. This is useful for a bootloader that just needs to connect to the
/// "last" known STA and broadcast the "default" AP. Bootloader example:
///
/// @code
/// EspIdfWifi wifi;
///
/// void app_main(void)
/// {
///     ESP_ERROR_CHECK(nvs_flash_init());
///     wifiMgr.init();
///     wifiMgr.setup_ap(wifiMgr.get_hostname().c_str(),
///         wifiMgr.DEFAULT_PASSWORD, wifiMgr.SEC_WPA2);
///     wifiMgr.start(WlanRole::AP_STA);
/// }
/// @endcode
///
/// In order to support non-volatile configuration, use the specialized version
/// of this object that includes a DefaultConfigUpdateListener instead.
/// Whenever there is an update to the WiFi profiles managed by this object,
/// the virtual method config_sync() is called. config_sync() can be overriden
/// in a derived version of this object in order to trigger synchronization with
/// more complex user configuration, such as that of memory spaces and CDI.
///
/// Note: The protected status variables in the WiFiInterface object are only
///       modified from the ESP event handler thread. Therefore, they do not
///       need an additional atomic or mutex lock.
class EspIdfWiFi : public WiFiInterface, public StateFlowBase
{
public:
    /// Default wifi password.
    static constexpr char DEFAULT_PASSWORD[] = "123456789";

    /// Constructor.
    /// @param service Service to bind this object to. This object does not
    ///        block the executor, except in the case of a mutex used for
    ///        mutual exclusion. Within this object, no blocking calls are made
    ///        while the mutex is held. In the case of a derived version of
    ///        this object, it is possible that a derived object will have
    ///        different blocking behavior in either the use of this service or
    ///        it internal mutex.
    /// @param hostname hostname to publish over the network, it is be copied
    ///        over to an std::string
    EspIdfWiFi(Service *service, const char *hostname)
        : StateFlowBase(service)
        , lock_(true) // Recursive.
        , initialized_(false)
        , timer_(this)
        , apIface_(nullptr)
        , staIface_(nullptr)
        , hostname_(hostname)
        , mdnsStaLockCount_(1) // Start disabled, enable when IP received.
        , apClientCount_(0)
        , mdnsClientStarted_(false)
        , mdnsClientTrigRefresh_(false)
        , mdnsAdvInhibit_(false)
        , mdnsAdvInhibitSta_(false)
        , fastConnectOnlySta_(false)
    {
        wait_and_call(STATE(mdns_start));
    }

    /// Initialize the WiFi.
    void init() override
    {
        init_config();
    }

    /// Start the WiFi.
    /// @param role device role
    void start(WlanRole role = WlanRole::DEFAULT_ROLE) override
    {
        init_wifi(role);
        initialized_ = true;
    }

    /// Stop the WiFi.
    void stop() override;

    /// Connect to access point. This is a non-blocking call. The results will
    /// be delivered by callback registered with set_wlan_connect_callback().
    /// @param ssid access point SSID
    /// @param pass access point password
    /// @param sec_type access point security type
    void connect(
        const char *ssid, const char *pass, SecurityType sec_type) override;

    /// Disconnect from the current AP. This is a non-blocking call. The
    /// results will be delivered by callback registered with
    /// set_wlan_connect_callback().
    void disconnect() override;

    /// Setup access point role credentials. May require reboot to take effect.
    /// @param ssid access point SSID
    /// @param pass access point password
    /// @param sec_type access point security type
    void setup_ap(
        const char *ssid, const char *pass, SecurityType sec_type) override;

    /// Retrieve current access point config.
    /// @param ssid access point SSID
    /// @param sec_type access point security type
    void get_ap_config(std::string *ssid, SecurityType *sec_type) override
    {
        OSMutexLock locker(&lock_);
        *ssid = userCfg_.ap_.ssid_;
        *sec_type = sec_type_encode(userCfg_.ap_.sec_);
    }

    /// Retrieves the number of clients connected to the WiFi in access point
    /// mode.
    /// @return number of connected stations, else -1 if not in AP mode.
    int get_ap_sta_count() override
    {
        return apIface_ ? apClientCount_ : -1;
    }

    /// Get the WiFi role.
    /// @return current WiFi role
    WlanRole role() override
    {
        if (apIface_)
        {
            return staIface_ ? WlanRole::AP_STA : WlanRole::AP;
        }
        else if (staIface_)
        {
            return WlanRole::STA;
        }
        return WlanRole::UNKNOWN;
    }

    /// Change the default role. This will be used in the next start() if the
    /// DEFAULT_ROLE is specified. The new setting takes effect when the
    /// device is restarted (either via reboot or stop + start)
    /// @param new_role new role, must not be UNKOWN or DEFAULT_ROLE
    void set_role(WlanRole new_role) override;

    /// Add a saved WiFi access point profile.
    /// @param ssid access point SSID
    /// @param pass access point password
    /// @param sec_type access point security type
    /// @param priority priority when more than one profile is saved, 0 =
    ///        lowest priority, may be unused
    /// @return resulting index in the list of profiles, else -1 on error
    int profile_add(const char *ssid, const char *pass,
        SecurityType sec_type, uint8_t priority) override;

    /// Delete a saved WiFi access point profile.
    /// @param ssid access point SSID
    /// @return profile index deleted, else -1 if profile not found
    int profile_del(const char *ssid) override
    {
        int index = find_sta_profile(std::string(ssid));
        if (index >= 0)
        {
            profile_del(index);
        }
        return index;
    }

    /// Delete a saved WiFi access point profile.
    /// @param index index of profile to delete, 0xFF removes all
    /// @return 0 if successful, else -1 if profile not found
    int profile_del(uint8_t index) override
    {
        if (index >= ARRAYSIZE(userCfg_.sta_))
        {
            return -1;
        }
        {
            OSMutexLock locker(&lock_);
            memset(userCfg_.sta_[index].ssid_, 0,
                sizeof(userCfg_.sta_[index].ssid_));
            memset(userCfg_.sta_[index].pass_, 0,
                sizeof(userCfg_.sta_[index].pass_));
        }
        config_sync();
        return 0;
    }

    /// Get a saved WiFi access point profile by index.
    /// @param index index within saved profilelist to get
    /// @param ssid 33 byte array that will return the SSID of the index
    /// @param sec_type will return the security type of the index
    /// @param priority will return the priority of the index, may be unused
    /// @return 0 upon success, -1 on error (index does not exist)
    int profile_get(int index, char ssid[], SecurityType *sec_type,
        uint8_t *priority) override;

    /// Get a list of available networks. This is based on a prior scan. Use
    /// rescan() and wait for the scan to complete to refresh the list.
    /// @param entries returns a list of available network entries
    /// @param count max size of the entry list to return.
    /// @return number of valid network entries in the list, same as
    ///         entries->size()
    int network_list_get(
        std::vector<NetworkEntry> *entries, size_t count) override
    {
        OSMutexLock locker(&lock_);
        *entries = scanResults_;
        return entries->size();
    }

    /// Get the indexed network entry from the list of scan results. Use
    /// rescan() and wait for the scan to complete to refresh the results.
    /// @param entry location to fill in the network entry
    /// @param index index in the network entry list to get
    /// @return 0 on success, else -1 if index is beyond the list of entries.
    int network_get(NetworkEntry *entry, unsigned index) override
    {
        OSMutexLock locker(&lock_);
        if (index >= scanResults_.size())
        {
            return -1;
        }
        *entry = scanResults_[index];
        return 0;
    }

    /// Initiate scanning of available networks.
    void scan() override;

    /// Get the RSSI of the AP in STA mode.
    /// @return signal strength, 0 when not connected to an access point
    int rssi() override;

    /// Get the network hostname for the device.
    /// @return hostname
    std::string get_hostname() override
    {
        // We don't need to get this from the interface because it is cached.
        return hostname_;
    }

    /// In some cases, we want to disable mDNS publishing in station mode.
    void disable_mdns_publish_on_sta()
    {
        OSMutexLock locker(&lock_);
        mdns_disable_sta();
        mdnsAdvInhibitSta_ = true;
    }

    /// In some cases, we want to only use the last known connection
    /// credentials in STA mode.
    void enable_fast_connect_only_on_sta()
    {
        OSMutexLock locker(&lock_);
        fastConnectOnlySta_ = true;
    }

    /// Add service to mDNS server.
    /// @param service service, e.g. _openlcb-can._tcp
    /// @param port service port
    void mdns_service_add(const char *service, uint16_t port);

    /// Remove service from mDNS server.
    /// @param service service, e.g. _openlcb-can._tcp
    void mdns_service_remove(const char *service);

    /// Lookup an mDNS name.
    /// @param service servicename to lookup
    /// @param hints hints about limiting the types of services that will
    ///        respond
    /// @param addrinfo structure containing one or more service addressess that
    ///        match the enquery, else nullptr if no matching service found
    /// @return 0 upon success, or appropriate EAI_* error on failure, use
    ///         ::freeaddrinfo() to free up memory allocated to the non nullptr
    ///         *addr returned
    int mdns_lookup(
        const char *service, struct addrinfo *hints, struct addrinfo **addr);

    /// Start continuous scan for mDNS service name.
    /// @param service servicename to scan
    void mdns_scan(const char *service);

protected:
    /// WiFi STA or AP credentials.
    struct WiFiConfigCredentialsNVS
    {
        char ssid_[33]; ///< STA SSID
        uint8_t sec_; ///< STA security mode
        char pass_[64]; ///< STA password
    };

    /// C structure version of the WiFi configuration settings. These are
    /// essentially the connection profiles (SSID, password, security type).
    struct WiFiConfigNVS
    {
        uint32_t magic_; ///< magic number to detect initialization
        WlanRole mode_; ///< role that the device will operate (AP, STA, etc.)
        WiFiConfigCredentialsNVS ap_; ///< access point configuration
        WiFiConfigCredentialsNVS sta_[7]; ///< station profiles
    };
    static_assert(sizeof(WiFiConfigNVS) == 792,
        "The size of WiFiConfigNVS has changed. This could lead to "
        "compatiblity issues in deployed devices.");

    /// Private configuration metadata. This is non-volatile information that
    /// informs the WiFi "state", primarily at startup, and is not exposed to
    /// the user.
    struct ConfigPrivate
    {
        uint32_t magic_;
        WiFiConfigCredentialsNVS last_; ///< last STA successfully connected to
        uint8_t channelLast_; ///< last channel successfully connected with
    };

    /// Magic number to detect initialization.
    static constexpr uint32_t WIFI_CONFIG_INIT_MAGIC = 0x6160CBC6;

    /// Magic number to detect initialization.
    static constexpr uint32_t PRIV_CONFIG_INIT_MAGIC = 0xBD959C63;

    /// Default STA SSID.
    static constexpr char DEFAULT_STA_SSID[] = "LAYOUTWIFI";

    /// NVS key for the WiFi user config.
    static constexpr char NVS_KEY_USER_NAME[] = "wifi_user.v1";

    /// NVS key for the WiFi private config.
    static constexpr char NVS_KEY_LAST_NAME[] = "wifi_last.v1";

    /// Maximum length of a stored SSID not including '\0' termination.
    static constexpr size_t MAX_SSID_SIZE =
        sizeof(WiFiConfigCredentialsNVS::ssid_) - 1;
    static_assert(MAX_SSID_SIZE == 32, "Invalid maximum SSID length.");

    /// Maximum length of a stored password not including '\0' termination.
    static constexpr size_t MAX_PASS_SIZE =
        sizeof(WiFiConfigCredentialsNVS::pass_) - 1;
    static_assert(MAX_PASS_SIZE == 63, "Invalid maximum password length.");

    /// Reset configuration to factory defaults.
    void factory_default();

    /// Trigger synchronize configuration between NVS and MemorySpace.
    virtual void config_sync()
    {
    }

    /// It is impossible to predict from what thread the public API will be
    /// called from. Additionally, the WiFi "stack" invokes callbacks from its
    /// own thread. Therefore, some of the resources managed by this object
    /// need mutual exclusion protection. There are at least two different
    /// threads in use:
    /// 1. Service passed in.
    /// 2. WiFi "stack".
    ///
    /// ...and the potential for one or more additional threads that invoke the
    /// public API. Protected resources include:
    /// - AP and STA profiles configuration
    /// - AP scan results
    /// - mDNS scanning state machine
    ///   - mdnsClientStarted_ (are we looking for any services yet)
    ///   - mdnsAdvInhibit_ (is advertising currently blocked, so we can scan)
    ///   - mdnsAdvInhibitSta_ (should we inhibit advertising on STA interface)
    ///   - mdnsAdvInhibitStaActive_ (advertising inhibit on STA is active)
    ///   - mdnsClientTrigRefresh_ (is there a scan refresh pending)
    ///   - mdnsStaLockCount_ (locking count STA interface, 0 = STA disabled)
    ///   - mdnsServices_ (services being advertised)
    ///   - mdnsClientCache_ (services being looked for)
    OSMutex lock_;

    nvs_handle_t cfg_; ///< handle to the nvs config for wifi configuration
    WiFiConfigNVS userCfg_; ///< cached copy of the wifi user config
    ConfigPrivate privCfg_; ///< private WiFi configuration
    bool initialized_; ///< initialization complete

private:
    /// Metadata for a registered mDNS service.
    struct MDNSService
    {
        /// Add service to mDNS server.
        /// @param service service name
        /// @param proto service protocol (_tcp, _udp, ect.)
        /// @param port service port
        MDNSService(std::string name, std::string proto, uint16_t port)
            : name_(name)
            , proto_(proto)
            , port_(port)
        {
        }
        std::string name_; ///< service name
        std::string proto_; ///< service protocol
        uint16_t port_; ///< service port
    };

    /// Metadata for an subscribed mDNS cashed address.
    struct MDNSCacheAddr
    {
        sa_family_t family_; ///< protocol family
        uint16_t port_; ///< port number, network endianness
        union
        {
            struct sockaddr addr_; ///< address
            struct sockaddr_in addrIn_; ///< IPv4 address
#if defined(ESP_IDF_WIFI_IPV6)
            struct sockaddr_in6 addrIn6_; ///< IPv6 address
#endif
        };
    };

    /// Metadata for a subscribed mDNS client.
    struct MDNSCacheItem
    {
        /// Constructor.
        MDNSCacheItem(std::string service)
            : service_(service)
            , searchHandle_(nullptr)
        {
        }

        /// Reset the search.
        /// @param handle the new search handle value, delete previous handle
        ///        if still valid, should be called with mutex lock
        void reset(void *handle = nullptr);

        /// Get the current search handle
        /// @return current search handle
        void *get()
        {
            return searchHandle_;
        }

        std::string service_; ///< sevice name
        std::list<MDNSCacheAddr> addr_; ///< list of addresses
        void *searchHandle_; ///< mDNS search once key
    };

    /// NVS namespace for the wifi configuration.
    static constexpr char NVS_NAMESPACE_NAME[] = "wifi_config";

    /// Maximum number of MDNS results we will cache for a given service.
    static constexpr size_t MDNS_RESULT_COUNT_MAX = 5;

    /// Timeout value passed for an mDNS query in asynchronous mode.
    static constexpr uint32_t MDNS_QUERY_ASYNC_NEW_TIMEOUT_MSEC = 700;

    /// State flow timeout for checking on mDNS query results.
    static constexpr long long MDNS_QUERY_CHECK_TIMEOUT_NSEC =
        MSEC_TO_NSEC(100);

    /// State flow timeout for inactive time between mDNS queries.
    static constexpr long long MDNS_QUERY_INACTIVE_TIMEOUT_NSEC =
        SEC_TO_NSEC(10);

    /// Minimum RSSI threshold for an AP signal before a connection attempt
    /// will be made in STA mode.
    static constexpr int8_t STA_CONNECT_RSSI_THRESHOLD = -100;

    /// Entry point. Wait for mDNS client to be invoked.
    /// @return next state mdns_start()
    Action mdns_wait()
    {
        return wait_and_call(STATE(mdns_start));
    }

    /// Start an mDNS query sequence. If mDNS advertising is inhibited in STA
    /// mode, then inhibit advertisements during the query.
    /// @return next state mdns_query()
    Action mdns_start();

    /// Register the mDNS queries with the mDNS client.
    /// @return next state mdns_check() after timeout
    Action mdns_query();

    /// Check the results of the mDNS queries.
    /// @return next state mdns_check() after timeout if there are still active
    ///         queries, mdns_query() if another mdns_scan() was triggered,
    ///         else mdns_start() after timeout
    Action mdns_check();

    /// Split a service into its name and protocol parts
    /// @param service service, e.g. _openlcb-can._tcp
    /// @param name name component, e.g. _openlcb-can
    /// @param proto protocol component, e.g. _tcp
    void mdns_split(const char *service, std::string *name, std::string *proto)
    {
        *name = service;
        size_t split = name->find('.');
        if (split != std::string::npos)
        {
            proto->assign(name->substr(split + 1));
            name->resize(split);
        }
    }

    /// Put an advertising inhibit in place.
    void mdns_adv_inhibit();

    /// Remove the advertising inhibit.
    void mdns_adv_inhibit_remove();

    /// Unconditionally disable mDNS on the station interface.
    void mdns_disable_sta();

    /// Restore mDNS status on the STA interface.
    void mdns_restore_sta();

    /// Static callback for the ESP event handler.
    /// @param arg passed in context (this pointer)
    /// @param base event base
    /// @param id event id
    /// @param data event data
    static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id,
        void *data)
    {
        static_cast<EspIdfWiFi*>(arg)->wifi_event_handler(base, id, data);
    }

        /// Static callback for the ESP event handler.
    /// @param arg passed in context (this pointer)
    /// @param base event base
    /// @param id event id
    /// @param data event data
    static void ip_event_handler(void *arg, esp_event_base_t base, int32_t id,
        void *data)
    {
        static_cast<EspIdfWiFi*>(arg)->ip_event_handler(base, id, data);
    }

    /// In context ESP event handler.
    /// @param base event base
    /// @param id event id
    /// @param data event data
    void wifi_event_handler(esp_event_base_t base, int32_t id, void *data);

    /// In context ESP event handler.
    /// @param base event base
    /// @param id event id
    /// @param data event data
    void ip_event_handler(esp_event_base_t base, int32_t id, void *data);

    /// Initialize wifi configuration, including program defaults if necessary.
    void init_config();

    /// Initialize the WiFi.
    /// @param role device role
    void init_wifi(WlanRole role);

    /// Initialize the WiFi soft access point.
    /// @param ssid SSID
    /// @param pass password
    void init_softap(std::string ssid, std::string pass);

    /// Initialize the Wifi station.
    void init_sta();

    /// Initiate a WiFi station connect.
    /// @param ssid SSID
    /// @param pass password
    /// @param authmode authentication mode, default 1 = WIFI_AUTH_WEP
    /// @param channel WiFi channel
    void sta_connect(std::string ssid, std::string pass, uint8_t authmode = 1,
        uint8_t channel = 0);

    /// Set/Update the last WiFi STA mode connection parameters.
    /// @param ssid SSID
    /// @param pass password
    /// @param authmode authentication mode
    /// @param channel WiFi channel
    void last_sta_update(
        std::string ssid, std::string pass, uint8_t authmode, uint8_t channel);

    /// Get the last WiFi STA mode connection parameters.
    /// @param ssid SSID, empty string if invalid
    /// @param pass password
    /// @param authmode authentication mode
    /// @param channel WiFi channel
    void last_sta_get(
        std::string *ssid, std::string *pass, uint8_t *authmode,
        uint8_t *channel);

    /// Find a WiFi STA profile that matches the given SSID
    /// @param ssid SSID to match
    /// @return index within the wifi profiles config, else -1 if not found
    int find_sta_profile(std::string ssid);

    /// Translate from generic security type to ESP security type.
    /// @param sec_type generic security type
    /// @return ESP security type
    uint8_t sec_type_translate(SecurityType sec_type);

    /// Translate from ESP security type to generic security type.
    /// @param sec_type ESP security type
    /// @return generic security type
    SecurityType sec_type_encode(uint8_t sec_type);

    /// Translate from ESP connection reason to generic connection result.
    /// @param reason ESP connection reason
    /// @return generic connection result
    ConnectionResult connection_result_encode(uint8_t reason);

    StateFlowTimer timer_; ///< sleep timer helper object
    esp_netif_t *apIface_; ///< access point network interface
    esp_netif_t *staIface_; ///< station network interface
    std::vector<MDNSService> mdnsServices_; ///< registered mDNS services
    /// Cache of all the subscribed mDNS services.
    std::vector<MDNSCacheItem> mdnsClientCache_;
    std::vector<NetworkEntry> scanResults_; ///< AP scan results
    std::string staConnectPass_; ///< last station connect attempt password
    const std::string hostname_; ///< published hostname
    unsigned mdnsStaLockCount_; ///< counter for recursive mDNS STA lock
    uint8_t apClientCount_; ///< number of connected wifi clients

    //
    // The following objects are write protected for mutual exclusion by
    // the lock_ mutex.
    //

    /// mDNS client state machine started.
    bool mdnsClientStarted_       : 1;
    /// an mDNS client refresh has been triggered
    bool mdnsClientTrigRefresh_   : 1;
    /// true if services are blocked from advertising
    bool mdnsAdvInhibit_          : 1;
    /// true if mDNS advertising is blocked on STA
    bool mdnsAdvInhibitSta_       : 1;
    /// true if mdnsAdvInhibitSta_ is active
    bool mdnsAdvInhibitStaActive_ : 1;
    /// mDNS scanning is active
    bool mdnsScanActive_          : 1;
    /// true if only to use fast connect credentials
    bool fastConnectOnlySta_      : 1;
};

#endif // _FREERTOS_DRIVERS_ESP_IDF_ESPIDFWIFI_HXX_