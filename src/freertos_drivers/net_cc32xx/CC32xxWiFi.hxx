/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file CC32xxWiFi.hxx
 * This file instantiates and initializes the CC32xx Wi-Fi.
 *
 * @author Stuart W. Baker
 * @date 18 March 2016
 */

#ifndef _FREERTOS_DRIVERS_NET_C32XX_CC32XXWIFI_HXX_
#define _FREERTOS_DRIVERS_NET_C32XX_CC32XXWIFI_HXX_

#include <functional>
#include <vector>
#include <string>

#include "os/OS.hxx"
#include "utils/Singleton.hxx"
#include "utils/format_utils.hxx"
#include "freertos_drivers/common/WifiDefs.hxx"

class CC32xxSocket;
class NetworkSpace;

/** Interface that aids in unit testing.
 */
class CC32xxWiFiInterface
{
public:
    /** Security types.
     */
    enum SecurityType
    {
        SEC_OPEN, /**< open (no security) */
        SEC_WEP,  /**< WEP security mode */
        SEC_WPA2, /**< WPA2 security mode */
    };

protected:
    /** Destructor.
     */
    virtual ~CC32xxWiFiInterface()
    {
    }

    /** Setup access point role credentials. It is OK to leave ssid as nullptr
     * or password as nullptr, in which case those properties will not be
     * changed.
     * @param ssid access point ssid (name)
     * @param security_key access point security key (password)
     * @param security_type specifies security type. Required.
     */
    virtual void wlan_setup_ap(const char *ssid, const char *security_key,
                               SecurityType security_type) = 0;

    /** Delete a saved WLAN profile.
     * @param index index within saved profile list to remove, 0xFF removes all
     * @return 0 upon success, else -1 on error
     */
    virtual int wlan_profile_del(int index) = 0;
};

/** Provides the startup and mantainance methods for configuring and using the
 * CC32xx Wi-Fi stack.  This is designed to be a singleton.  It should only
 * be instantiated once.
 */
class CC32xxWiFi : public CC32xxWiFiInterface, public Singleton<CC32xxWiFi>
{
public:

    /** the value passed to wlan_profile_del() to remove all profiles */
    static constexpr int PROFILE_DELETE_ALL = 0xFF;

    /** What is the maximum number of profiles in the CC32xx. Profile index is
     * 0 to NUM_PROFILES-1.*/
    static constexpr int NUM_PROFILES = 7;

    /** Pass this option as protocol to ::socket to create a secure socket. */
    static constexpr unsigned IPPROTO_TCP_TLS = 254;

    /** Retrieves the socket descriptor for setting TLS parameters. */
    static constexpr unsigned SO_SIMPLELINK_SD = 65537;
    
    /** CC32xx SimpleLink forward declaration */
    struct WlanEvent;

    /** CC32xx SimpleLink forward declaration */
    struct NetAppEvent;

    /** CC32xx SimpleLink forward declaration */
    struct SockEvent;

    /** CC32xx SimpleLink forward declaration */
    struct SockTriggerEvent;

    /** CC32xx SimpleLink forward declaration */
    struct HttpServerEvent;

    /** CC32xx SimpleLink forward declaration */
    struct HttpServerResponse;

    /** CC32xx SimpleLink forward declaration */
    struct NetAppRequest;

    /** CC32xx SimpleLink forward declaration */
    struct NetAppResponse;

    /** CC32xx SimpleLink forward declaration */
    struct FatalErrorEvent;
    
    /** The Wlan reconnect policy */
    enum WlanConnectionPolicy {
        WLAN_CONNECTION_NO_CHANGE,
        /// Scan for wifi networks and connect to the nearest that is stored in
        /// the profiles (by profile priority and security settings).
        WLAN_CONNECTION_SCAN,
        /// Reconnect to the last connected AP.
        WLAN_CONNECTION_FAST_RECONNECT
    };
    
    /** The WLAN power policy.
     */
    enum WlanPowerPolicy
    {
        WLAN_NORMAL_POLICY,      /**< WLAN power policy normal */
        WLAN_LOW_LATENCY_POLICY, /**< WLAN power policy low latency */
        WLAN_LOW_POWER_POLICY,   /**< WLAN power policy low power */
        WLAN_ALWAYS_ON_POLICY,   /**< WLAN power policy always on */
        WLAN_NO_CHANGE_POLICY,   /**< WLAN power policy to be left alone */
    };

    /** metadata for a WLAN netowrk entry.
     */
    struct WlanNetworkEntry
    {
        string ssid; /**< SSID of AP */
        SecurityType sec_type; /**< security type of the AP */
        int rssi; /**< receive signal strength indicator of the AP */
    };

    /** This function type is used for POST callback operations to the
     * application.
     * @param handle the operation handle, needs to be provided to the future
     * operations to fetch followup data and send response.
     * @param content_length value of the Content-Length header, or -1 if such
     * a header is not found.
     * @param md encoded metadata. See the CC32xx documentation on how metadata
     * is encoded. The lifetime is restricted to this call inline.
     * @param md_len number of bytes in the metadata array
     * @param payload the content (or beginning of the content). The lifetime is
     * restricted to this call inline.
     * @param payload_len how many bytes are in this chunk of the content
     * @param has_more true if there is a continuation of the payload, which
     * needs to be fetched with get_post_data. */
    using PostFunction = std::function<void(uint16_t handle,
        uint32_t content_length, const uint8_t *md, size_t md_len,
        const uint8_t *payload, size_t payload_len, bool has_more)>;

    /** Constructor.
     */
    CC32xxWiFi();

    /** Destructor.
     */
    ~CC32xxWiFi()
    {
    }

    /** Startup the Wi-Fi in test mode. This mode of operation does not support
     * connection to an AP. This mode is only for RF testing. Test mode can be
     * exited by @ref stop().
     */
    void test_mode_start();

    /** Startup the Wi-Fi.
     * @param device role
     * @param power_policy desired power policy
     */
    void start(WlanRole role = WlanRole::STA,
        WlanPowerPolicy power_policy = WLAN_NO_CHANGE_POLICY,
        WlanConnectionPolicy connection_policy = WLAN_CONNECTION_NO_CHANGE);

    /** Stops the Wi-Fi in preparation for a reboot. TODO: does this need to be
     * called from a critical section?
     */
    void stop();

    /** Get the started state of the network processor.
     * @return true if started, else false
     */
    bool is_started()
    {
        return started;
    }

    /** Connect to access point.
     * @param ssid access point ssid
     * @param security_key access point security key
     * @param security_type specifies security type
     * @return WlanConnectResult::OK upon success, else error on failure
     */
    WlanConnectResult wlan_connect(const char *ssid, const char *security_key,
                                   SecurityType security_type);

    /** Disconnects from the current AP. */
    void wlan_disconnect();

    /** Initiate a WPS Push Button Control connection.
     */
    void wlan_wps_pbc_initiate();

    /** Setup access point role credentials.
     * @param ssid access point ssid
     * @param security_key access point security key
     * @param security_type specifies security type 
     */
    void wlan_setup_ap(const char *ssid, const char *security_key,
                       SecurityType security_type) override;

    /** Retrieve current AP config.
     * @param ssid will be filled with the SSID of the AP
     * @param security_type will be filled with the security type
     */
    void wlan_get_ap_config(string *ssid, SecurityType *security_type);

    /** Retrieves how many stations are connected to the wifi in AP mode.
     * @return number of connected stations (0 to 4). If not in AP mode,
     * returns 0.
     */
    int wlan_get_ap_station_count();

    /** @return true if the wlan interface is ready to establish outgoing
     * connections. */
    bool wlan_ready()
    {
        return (connected || wlanRole == WlanRole::AP) && ipAcquired;
    }

    /** Get the current Wi-Fi role.
     * @return the current Wi-Fi role.
     */
    WlanRole wlan_role()
    {
        return wlanRole;
    }

    /** Change the default Wlan Role. This will be used in the next start(...)
     * if the UNKNOWN role is specified. The new setting takes effect when the
     * device is restarted (either via reboot or stop + start).
     * @param role new role. Must not be UNKNOWN
     */
    void wlan_set_role(WlanRole new_role);

    /** @return 0 if !wlan_ready, else a debugging status code. */
    WlanState wlan_startup_state()
    {
        if (!connected && wlanRole != WlanRole::AP)
        {
            if (securityFailure)
            {
                return WlanState::WRONG_PASSWORD;
            }
            return WlanState::NOT_ASSOCIATED;
        }
        if (!ipAcquired)
        {
            return WlanState::NO_IP;
        }
        return WlanState::OK;
    }

#ifdef GTEST
    /** Used by unit tests to simulate wifi connection states.
     * @param conn if true, we are associated to an AP
     * @param has_ip if true, we have an IP address
     * @param ssid will be returned when caller wants the AP name
     * @param wrong_password if true, simulates security failure
     */
    void TEST_set_state(
        bool conn, bool has_ip, bool wrong_password, const string &ssid)
    {
        connected = conn ? 1 : 0;
        ipAcquired = has_ip ? 1 : 0;
        securityFailure = wrong_password ? 1 : 0;
        strcpy(this->ssid, ssid.c_str());
    }

    /** Used by unit tests to simulate wifi state.
     * @param ip sets the IP address given to the device by DHCP.
     */
    void TEST_set_ip(uint32_t ip)
    {
        ipAddress = ip;
    }
#endif

    /** Updates the blinker based on connection state. Noop if wlan_ready()
     * returns true.*/
    void connecting_update_blinker();

    /** Get the current country code.
     * @return current country code, CountryCode::UNKNOWN on error
     */
    CountryCode wlan_country_code_get();

    /** Set the current country code.
     * @param cc country code to set
     * @param restart true to restart NWP, else false, use extreme caution when
     *        restart = true;
     * @return 0 upon success, else -1 on error
     */
    int wlan_country_code_set(CountryCode cc, bool restart = false);

    /** Sets the scan parameters.
     * @param mask the channel mask (bit 0 = channel1, bit1=channel2). If -1
     * then the channel mask is not changed.
     * @param min_rssi the minimal RSSI to return a wifi in the scan. If >= 0
     * then the min_rssi is not changed. (Default min_rssi is -95.) */
    void wlan_set_scan_params(int mask, int min_rssi);
    
    /** Add a saved WLAN profile.
     * @param ssid WLAN SSID of the profile to save
     * @param sec_type @ref SecurityType of the profile to be saved
     * @param key password of the SSID, nullptr allowed if sec_type is
     *            @ref SEC_OPEN
     * @param priority connection priority when more than one of the saved
     *        networks is available, 0 == lowest priority
     * @return resulting index in the list of profiles, else -1 on error
     */
    int wlan_profile_add(const char *ssid, SecurityType sec_type,
                         const char *key, unsigned priority);

    /** Delete a saved WLAN profile.
     * @param ssid WLAN SSID of the profile to delete
     * @return 0 upon success, else -1 on error
     */
    int wlan_profile_del(const char *ssid);

    /** Delete a saved WLAN profile.
     * @param index index within saved profile list to remove, 0xFF removes all
     * @return 0 upon success, else -1 on error
     */
    int wlan_profile_del(int index) override;

    /** Get a saved WLAN profile by index.
     * @param index index within saved profile list to get
     * @param ssid 33 byte array that will return the ssid of the index
     * @param sec_type will return the security type of the index
     * @param priority will return the priority of the index
     * @return 0 upon success, else -1 on error
     */
    int wlan_profile_get(int index, char ssid[], SecurityType *sec_type,
                         uint32_t *priority);

    /** Test if there are any saved profiles.
     * @return true if there are no profiles saved, else false
     */
    bool wlan_profile_test_none();

    /** Set the power policy.
     * @param wpp power policy to set
     * @return 0 upon success, else -1 on error
     */
    int wlan_power_policy_set(WlanPowerPolicy wpp);

    /** Get the power policy.
     * @param wpp power policy to returned
     * @return 0 upon success, else -1 on error
     */
    int wlan_power_policy_get(WlanPowerPolicy *wpp);

    /** Sets connection policy to auto connect. Updates the Wifi fast-reconnect
     * policy if desired.
     * @param policy the desired policy
     */
    void wlan_connection_policy_set(WlanConnectionPolicy policy);
    
    /** Get a list of available networks.
     * @param entries returns a list of available network entries
     * @param count size of entry list in number of elements, max 20
     * @return number of valid network entries in the list
     */
    int wlan_network_list_get(WlanNetworkEntry *entries, size_t count);

    /** Initiate rescanning available networks.
     */
    void wlan_rescan();
    
    /** Get the device MAC address.
     * @param mac 6 byte array which will hold the resulting MAC address.
     */
    void wlan_mac(uint8_t mac[6]);

    /** Sets the device MAC address. WARNING. The MAC address will be
     * persistently set to the value indicated. Only a factory reset of the
     * device can undo this operation. After calling this API there is no way
     * to recover the factory MAC address. Make sure not to call this API too
     * many times in the lifetime of the product, as flash wear is a concern.
     * @param mac 6 byte array which holds the desired MAC address.
     */
    void wlan_set_mac(uint8_t mac[6]);

    /** Get the assigned IP address.
     * @return assigned IP address, else 0 if not assigned
     */
    uint32_t wlan_ip()
    {
        return ipAcquired ? ipAddress : 0;
    }

    /** Get the SSID of the access point we are connected to. In AP mode gives
     * the current advertised AP.
     * @return SSID of the access point we are connected to or in AP mode the
     * access point name of the current device.
     */
    const char *wlan_ssid()
    {
        ssid[32] = '\0';
        return ssid;
    }

    /** Get the receive signal strength indicator.
     * @return receive signal strength
     */
    int wlan_rssi()
    {
        // the RSSI value is only reliable when associatedd
        return wlan_startup_state() == WlanState::NOT_ASSOCIATED ? 0 : rssi;
    }

    void set_ip_acquired_callback(std::function<void(bool)> callback)
    {
        ipAcquiredCallback_ = callback;
    }

    /** Executes the given function on the network thread. @param callback
     * isthe function to execute.*/
    void run_on_network_thread(std::function<void()> callback);

    /** Add an HTTP get token callback.  A get token is a simple macro
     * substitution that is applied to all files (e.g. HTML, JS) served by the
     * builtin webserver of the CC32xx. The token has a fixed form "__SL_G_*".
     * The form "__SL_G_U*" is the form that is reserved for user defined
     * tokens.  The * can be any two characters that uniquely identify the
     * token.  When the token is found in an HTML file, the network processor
     * will call the supplied callback in order for the user to return the
     * substitution string.  The result returned will be clipped at
     * (MAX_TOKEN_VALUE_LEN - 1), which is (64 - 1) bytes.  All tokens must be
     * an exact match.
     *
     * Use Case Example:
     * @code
     * class SomeClass
     * {
     * public:
     *     SomeClass()
     *     {
     *         add_http_get_token_callback("__SL_G_U.A", std::bind(&SomeClass::http_get, this));
     *     }
     *
     * private:
     *     string http_get()
     *     {
     *         return "some_string";
     *     }
     * }
     * @endcode
     * In the example above, the string "__SL_G_U.A" when used in an HTML file
     * will be recognized by the HTTP server replaced with result of the server
     * calling SomeClass::http_get().
     *
     * Additional documentation on the CC32xx HTTP Web Server can be found in
     * the
     * <a href="http://www.ti.com/lit/ug/swru368a/swru368a.pdf">
     * CC3100/CC3200 SimpleLink Wi-Fi Internet-on-a-Chip User's Guide</a>
     *
     * @param token_name The token name to match. Must live for the entire
     * lifetime of the binary. Must be of the form __SL_G_U??
     * @param callback the function to execute to give the replacement.
     */
    void add_http_get_token_callback(
        const char *token_name, std::function<std::string()> callback)
    {
        OSMutexLock l(&lock_);
        httpGetTokenCallbacks_.emplace_back(token_name, std::move(callback));
    }

    /** Registers a handler for an HTTP POST operation.
     * @param uri the target of the form submit, of the format "/foo/bar"
     * @param callback this function will be called from the network processor
     * context when a POST happens to the given URI.
     */
    void add_http_post_callback(const char *uri, PostFunction callback)
    {
        OSMutexLock l(&lock_);
        httpPostCallbacks_.emplace_back(uri, std::move(callback));
    }

    /** Retrieves additional payload for http POST operations. This function
     * blocks the calling thread. After the lat chunk is retrieved, the caller
     * must invoke the post response function.
     * @param handle the POST operation handle, given by the POST callback.
     * @param buf where to deposit additional data.
     * @param len at input, set to the max number of bytes to store. Will be
     * overwritten by the number of actual bytes that arrived.
     * @return true if there is additional data that needs to be fetched, false
     * if this was the last chunk. */
    bool get_post_data(uint16_t handle, void *buf, size_t *len);

    /** Sends a POST response.
     * @param handle the POST operation handle, given by the POST callback.
     * @param code HTTP error code (e.g. 204 for success).
     * @param redirect optional, if present, will send back a 302 redirect
     * status with this URL (http_status will be ignored).
     */
    void send_post_respose(uint16_t handle, uint16_t http_status = 204,
        const string &redirect = "");

    /** This function handles WLAN events.  This is public only so that an
     * extern "C" method can call it.  DO NOT use directly.
     * @param event pointer to WLAN Event Info
     */
    void wlan_event_handler(WlanEvent *event);

    /** This function handles network events such as IP acquisition, IP leased,
     * IP released etc.  This is public only so that an extern "C" method can
     * call it.  DO NOT use directly.
     * @param event Pointer indicating device acquired IP
     */
    void net_app_event_handler(NetAppEvent *event);

    /** This function handles socket events indication.  This is public only so
     * that an extern "C" method can call it.  DO NOT use directly.
     * @param event pointer to Socket Event Info
     */
    void sock_event_handler(SockEvent *event);

    /** Notifies the service about a wifi asynchronous socket event
     * callback. This means that sl_Select needs to be re-run and certain
     * sockets might need wakeup. DO NOT use directly.
     * @param event parameters from the socket. */
    void trigger_event_handler(SockTriggerEvent *event);

    /** This function handles http server callback indication.  This is public
     * only so that an extern "C" method can call it.  DO NOT use directly.
     * @param event pointer to HTTP Server Event info
     * @param response pointer to HTTP Server Response info
     */
    void http_server_callback(HttpServerEvent *event,
                              HttpServerResponse *response);

    /** This function handles netapp request callbacks.  This is public
     * only so that an extern "C" method can call it.  DO NOT use directly.
     * @param request pointer to NetApp Request info
     * @param response pointer to NetApp Response info
     */
    void netapp_request_callback(
        NetAppRequest *request, NetAppResponse *response);

    /** This Function Handles the Fatal errors
     *  @param  event - Contains the fatal error data
     *  @return     None
     */
    void fatal_error_callback(FatalErrorEvent* event);

    /** Returns a string contianing the version numbers of the network
     * interface. */
    static std::string get_version();

private:
    friend class ::NetworkSpace;

    /** Translates the SecurityType enum to the internal SimpleLink code.
     * @param sec_type security type
     * @return simplelink security type
     */
    static uint8_t security_type_to_simplelink(SecurityType sec_type);

    /** Translates the SimpleLink code to SecurityType enum.
     * @param sec_type simplelink security type
     * @return security type
     */
    static SecurityType security_type_from_simplelink(uint8_t sec_type);

    /** Translates the SimpleLink code from the network scan to SecurityType
     * enum.
     * @param sec_type simplelink network scan security result
     * @return security type
     */
    static SecurityType security_type_from_scan(unsigned sec_type);
    
    /** Set the CC32xx to its default state, including station mode.
     */
    void set_default_state();

    /** Thread that will manage the WLAN connection.
     * @param context context passed into the stack.
     */
    static void* wlan_task_entry(void *context)
    {
        instance()->wlan_task();
        return nullptr;
    }

    /** Thread that will manage the WLAN connection inside object context.
     */
    void wlan_task();

    /** Asynchronously wakeup the select call.
     */
    void select_wakeup();

    /** Remove a socket from the known sockets that might be part of the
     * sl_Select fdset.
     * @param sd socket descriptor to remove
     */
    void fd_remove(int16_t sd);

    /** Add socket to the read fd set.
     * @param sd socket descriptor to add
     */
    void fd_set_read(int16_t sd);

    /** Add socket to the write fd set.
     * @param sd socket descriptor to add
     */
    void fd_set_write(int16_t sd);

    /** Get the IP address for a http request.
     * @return string representation of the IP address
     */
    string http_get_ip_address()
    {
        return ipv4_to_string(ipAddress);
    }

    uint32_t ipAddress; /**< assigned IP adress */
    char ssid[33]; /**< SSID of AP, or AP we are connected to */

    /// Callback for when IP is acquired
    std::function<void(bool)> ipAcquiredCallback_;

    /// List of callbacks to execute on the network thread.
    std::vector<std::function<void()> > callbacks_;

    /// List of callbacks for http get tokens
    std::vector<std::pair<const char *, std::function<std::string()>>>
        httpGetTokenCallbacks_;

    /// List of callbacks for http post handlers
    std::vector<std::pair<const char *, PostFunction>> httpPostCallbacks_;

    /// Protects callbacks_ vector.
    OSMutex lock_;

    int wakeup; /**< loopback socket to wakeup select() */

    int16_t rssi; /**< receive signal strength indicator */

    WlanRole wlanRole; /**< the Wi-Fi role we are in */
    WlanPowerPolicy wlanPowerPolicy; /**< the desired power policy */
    WlanConnectionPolicy connectionPolicy; /**< scan or reconnect to last AP */

    unsigned started          : 1; /**< network processor started */
    unsigned connected        : 1; /**< AP connected state */
    unsigned connectionFailed : 1; /**< Connection attempt failed status */
    unsigned ipAcquired       : 1; /**< IP address aquired state */
    unsigned ipLeased         : 1; /**< IP address leased to a client(AP mode)*/
    unsigned smartConfigStart : 1; /**< Smart config in progress */
    unsigned securityFailure  : 1; /**< Disconnected due to wrong password */

    /** allow access to private members from CC32xxSocket */
    friend class CC32xxSocket;

    DISALLOW_COPY_AND_ASSIGN(CC32xxWiFi);
};

#endif /* _FREERTOS_DRIVERS_NET_C32XX_CC32XXWIFI_HXX_ */
