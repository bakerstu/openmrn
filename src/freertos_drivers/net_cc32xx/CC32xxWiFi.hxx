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

/** Provides the startup and mantainance methods for configuring and using the
 * CC32xx Wi-Fi stack.  This is designed to be a singleton.  It should only
 * be instantiated once.
 */
class CC32xxWiFi : public Singleton<CC32xxWiFi>
{
public:
    /** the value passed to wlan_profile_del() to remove all profiles */
    static constexpr int PROFILE_DELETE_ALL = 0xFF;

    /** CC32xx SimpleLink forward declaration */
    struct WlanEvent;

    /** CC32xx SimpleLink forward declaration */
    struct NetAppEvent;

    /** CC32xx SimpleLink forward declaration */
    struct SockEvent;

    /** CC32xx SimpleLink forward declaration */
    struct HttpServerEvent;

    /** CC32xx SimpleLink forward declaration */
    struct HttpServerResponse;

    /** CC32xx SimpleLink forward declaration */
    struct FatalErrorEvent;
    
    /** Security types.
     */
    enum SecurityType
    {
        SEC_OPEN, /**< open (no security) */
        SEC_WEP,  /**< WEP security mode */
        SEC_WPA2, /**< WPA2 security mode */
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

    /** Constructor.
     */
    CC32xxWiFi();

    /** Destructor.
     */
    ~CC32xxWiFi()
    {
    }

    /** Startup the Wi-Fi.
     * @param device role
     * @param power_policy desired power policy
     */
    void start(WlanRole role = WlanRole::STA,
               WlanPowerPolicy power_policy = WLAN_NO_CHANGE_POLICY);

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
     */
    void wlan_connect(const char *ssid, const char *security_key,
                      SecurityType security_type);


    /** Initiate a WPS Push Button Control connection.
     * @return 0 upon success, else -1 on error
     */
    int wlan_wps_pbc_initiate();

    /** Setup access point role credentials.
     * @param ssid access point ssid
     * @param security_key access point security key
     * @param security_type specifies security type 
     */
    void wlan_setup_ap(const char *ssid, const char *security_key,
                       SecurityType security_type);

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

    /** @return 0 if !wlan_ready, else a debugging status code. */
    WlanState wlan_startup_state()
    {
        if (!connected && wlanRole != WlanRole::AP)
        {
            return WlanState::NOT_ASSOCIATED;
        }
        if (!ipAcquired)
        {
            return WlanState::NO_IP;
        }
        return WlanState::OK;
    }

    /** Updates the blinker based on connection state. Noop if wlan_ready()
     * returns true.*/
    void connecting_update_blinker();

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
    int wlan_profile_del(int index);

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

    /** Get the assigned IP address.
     * @return assigned IP address, else 0 if not assigned
     */
    uint32_t wlan_ip()
    {
        return ipAcquired ? ipAddress : 0;
    }

    /** Get the SSID of the access point we are connected to.
     * @return SSID of the access point we are connected to
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
        return rssi;
    }

    /** Executes the given function on the network thread. @param callback
     * isthe function to execute.*/
    void run_on_network_thread(std::function<void()> callback);

    /** Add an HTTP get token callback.  A get token is a string that takes the
     * form "__SL_G_*".  The form "__SL_G_U*" is the form that is reserved for
     * user defined tokens.  The * can be any two characters that uniquely
     * identify the token.  When the token is found in an HTML file, the
     * network processor will call the supplied callback in order for the user
     * to return the resulting string.  The result returned will be clipped at
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
     *         add_http_get_callback(std::make_pair(std::bind(&SomeClass::http_get, this), "__SL_G_U.A");
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
     * @param callback the std::pair<> of the function to execute and the
     *        matching token to execute the callback on.  The second (const
     *        char *) argument of the std::pair must live for as long as the
     *        callback is valid.
     */
    void add_http_get_callback(std::pair<std::function<std::string()>,
                                         const char *> callback)
    {
        OSMutexLock l(&lock_);
        httpGetCallbacks_.emplace_back(std::move(callback));
    }

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

    /** This function handles http server callback indication.  This is public
     * only so that an extern "C" method can call it.  DO NOT use directly.
     * @param event pointer to HTTP Server Event info
     * @param response pointer to HTTP Server Response info
     */
    void http_server_callback(HttpServerEvent *event,
                              HttpServerResponse *response);

    /** This Function Handles the Fatal errors
     *  @param  event - Contains the fatal error data
     *  @return     None
     */
    void fatal_error_callback(FatalErrorEvent* event);

    /** Returns a string contianing the version numbers of the network
     * interface. */
    static std::string get_version();
    
private:
    /** Translates the SecurityType enum to the internal SimpleLink code.
     * @param sec_type security type
     * @return simplelink security type
     */
    uint8_t security_type_to_simplelink(SecurityType sec_type);

    /** Translates the SimpleLink code to SecurityType enum.
     * @param sec_type simplelink security type
     * @return security type
     */
    SecurityType security_type_from_simplelink(uint8_t sec_type);

    /** Translates the SimpleLink code from the network scan to SecurityType
     * enum.
     * @param sec_type simplelink network scan security result
     * @return security type
     */
    SecurityType security_type_from_scan(unsigned sec_type);
    
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

    static CC32xxWiFi *instance_; /**< singleton instance pointer. */
    uint32_t ipAddress; /**< assigned IP adress */
    char ssid[33]; /**< SSID of AP we are connected to */

    /// List of callbacks to execute on the network thread.
    std::vector<std::function<void()> > callbacks_;

    /// List of callbacks for http get tokens
    std::vector<std::pair<std::function<std::string()>, const char *>>
        httpGetCallbacks_;

    /// Protects callbacks_ vector.
    OSMutex lock_;

    int wakeup; /**< loopback socket to wakeup select() */

    int16_t rssi; /**< receive signal strength indicator */

    WlanRole wlanRole; /**< the Wi-Fi role we are in */
    WlanPowerPolicy wlanPowerPolicy; /**< the desired power policy */

    unsigned started          : 1; /**< network processor started */
    unsigned connected        : 1; /**< AP connected state */
    unsigned connectionFailed : 1; /**< Connection attempt failed status */
    unsigned ipAcquired       : 1; /**< IP address aquired state */
    unsigned ipLeased         : 1; /**< IP address leased to a client(AP mode)*/
    unsigned smartConfigStart : 1; /**< Smart config in progress */

    /** allow access to private members from CC32xxSocket */
    friend class CC32xxSocket;

    DISALLOW_COPY_AND_ASSIGN(CC32xxWiFi);
};

#endif /* _FREERTOS_DRIVERS_NET_C32XX_CC32XXWIFI_HXX_ */
