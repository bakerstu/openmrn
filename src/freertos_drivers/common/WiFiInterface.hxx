/** @copyright
 * Copyright (c) 2025 Stuart Baker
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
 * @file WiFiInterface.hxx
 *
 * Common interface for WiFi operations.
 *
 * @author Stuart Baker
 * @date 23 Jun 2025
 */

#ifndef _FREERTOS_DRIVERS_COMMON_WIFIINTERFACE_HXX_
#define _FREERTOS_DRIVERS_COMMON_WIFIINTERFACE_HXX_

#include <functional>
#include <string>
#include <vector>

#include "utils/Singleton.hxx"
#include "freertos_drivers/common/WifiDefs.hxx"

/// Abstract interface for WiFi operations
class WiFiInterface : public WiFiDefs, public Singleton<WiFiInterface>
{
public:
    /// Initialize the WiFi.
    virtual void init()
    {
    }

    /// Start the WiFi.
    /// @param role device role
    virtual void start(WlanRole role = WlanRole::DEFAULT_ROLE) = 0;

    /// Stop the WiFi. Has no expectation that start() can be called again
    /// without first rebooting.
    virtual void stop() = 0;

    /// Get the started state of the WiFi.
    /// @return true if started, else false
    virtual bool is_started()
    {
        return started_;
    }

    /// Get the default AP password.
    /// @return default AP password, should point to persistent memory
    virtual const char *default_ap_password() = 0;

    /// Get the default STA password.
    /// @return default STA password, should point to persistent memory
    virtual const char *default_sta_password() = 0;

    /// Get the default AP SSID.
    /// @return default AP SSID, should point to persistent memory
    virtual const char *default_ap_ssid() = 0;

    /// Get the default STA SSID.
    /// @return default STA SSID, should point to persistent memory
    virtual const char *default_sta_ssid() = 0;

    /// Get the maximum number of STA client connections in AP mode. Be careful,
    // the max supported by hardware is device dependent.
    /// @return maximum number of STA client connections in AP mode
    virtual uint8_t max_ap_client_connections() = 0;

    /// Get the maximum number of stored STA profiles.
    /// @return maximum number of stored STA profiles
    virtual uint8_t max_sta_profiles() = 0;

    /// Get the maximum number of AP scan results collected, per scan.
    /// @return maximum number of scan results
    virtual uint8_t max_ap_scan_results() = 0;

    /// Get the configuration for pruning duplicate AP scan results by RSSI.
    /// @return true to prune out duplicate AP scan results, taking the highest
    ///         RSSI, else false to keep duplicates
    virtual bool prune_duplicate_ap_scan_results_by_rssi() = 0;

    /// Connect to access point. This is a non-blocking call. The results will
    /// be delivered by callback registered with set_wlan_connect_callback().
    /// The AP credentials are not saved as a connection profile, but they may
    /// be saved in non-volatile storage for use on the next connection attempt.
    /// @param ssid access point SSID
    /// @param pass access point password
    /// @param sec_type access point security type
    virtual void connect(
        const char *ssid, const char *pass, SecurityType sec_type) = 0;

    /// Disconnect from the current AP. This is a non-blocking call. The
    /// result will be delivered by callback registered with
    /// set_wlan_connect_callback().
    virtual void disconnect() = 0;

    /// Gets the connected state of the STA interface
    /// @return true if STA connected/associated to an AP, else false
    virtual bool is_connected()
    {
        return connected_;
    }

    /// Setup access point role credentials. May require reboot to take effect.
    /// @param ssid access point SSID
    /// @param pass access point password
    /// @param sec_type access point security type
    virtual void setup_ap(
        const char *ssid, const char *pass, SecurityType sec_type) = 0;

    /// Retrieve current access point config.
    /// @param ssid access point SSID
    /// @param sec_type access point security type
    virtual void get_ap_config(std::string *ssid, SecurityType *sec_type) = 0;

    /// Retrieves the number of clients connected to the WiFi in access point
    /// mode.
    /// @return number of connected stations, else -1 if not in AP mode.
    virtual int get_ap_sta_count() = 0;

    /// Retrieves if ready to start establishing outgoing connections.
    /// @return true if ready, else false
    virtual bool ready()
    {
        // It is important to call the API and not access the member variables
        // directly. This is in case any derived object overrides sta_ready() or
        // ap_ready() with additional implementation specific logic.
        return (sta_ready() || ap_ready());
    }

    /// Get the STA mode ready state.
    /// @return true if STA is connected to an AP and has an IP address, else
    ///         false if not connected to an AP or no IP assigned
    virtual bool sta_ready()
    {
        // Typically, an IP cannot be acquired without being connected, so
        // AND'ing with the connected state flag is simply defensive. An
        // alternative might be to use an HASSERT(connected_) here instead.
        return connected_ && ipAcquiredSta_;
    }

    /// Get the AP mode ready state.
    /// @return true if AP has an IP address, else false if no IP assigned
    virtual bool ap_ready()
    {
        return ipAcquiredAp_;
    }

    /// Get the WiFi role.
    /// @return current WiFi role
    virtual WlanRole role() = 0;

    /// Change the default role. This will be used in the next start() if the
    /// DEFAULT_ROLE is specified. The new setting takes effect when the
    /// device is restarted (either via reboot or stop + start)
    /// @param new_role new role, must not be UNKOWN or DEFAULT_ROLE
    virtual void set_role(WlanRole new_role) = 0;

    /// Add a saved WiFi access point profile.
    /// @param ssid access point SSID
    /// @param pass access point password
    /// @param sec_type access point security type
    /// @param priority priority when more than one profile is saved, 0 =
    ///        lowest priority, may be unused
    /// @return resulting index in the list of profiles, else -1 on error
    virtual int profile_add(const char *ssid, const char *pass,
        SecurityType sec_type, uint8_t priority) = 0;

    /// Delete a saved WiFi access point profile.
    /// @param ssid access point SSID
    /// @return profile index deleted, else -1 if profile not found
    virtual int profile_del(const char *ssid) = 0;

    /// Delete a saved WiFi access point profile.
    /// @param index index of profile to delete, 0xFF removes all
    /// @return 0 if successful, else -1 if profile not found, may be unused
    virtual int profile_del(uint8_t index) = 0;

    /// Get a saved WiFi access point profile by index.
    /// @param index index within saved profilelist to get
    /// @param ssid 33 byte array that will return the SSID of the index
    /// @param sec_type will return the security type of the index
    /// @param priority will return the priority of the index
    /// @return 0 upon success, -1 on error (index does not exist)
    virtual int profile_get(
        int index, char ssid[], SecurityType *sec_type, uint8_t *priority) = 0;

    /// Get a list of available networks. This is based on a prior scan. Use
    /// scan() and wait for the scan to complete to refresh the list.
    /// @param entries returns a list of available network entries
    virtual void network_list_get(std::vector<NetworkEntry> *entries) = 0;

    /// Get the indexed network entry from the last of scan results. Use
    /// scan() and wait for the scan to complete to refresh the results.
    /// @param entry location to fill in the network entry
    /// @param index index in the network entry list to get
    /// @return 0 on success, else -1 if index is beyond the list of entries.
    virtual int network_get(NetworkEntry *entry, unsigned index) = 0;

    /// Initiate scanning of available networks. Use 
    /// set_scan_finished_callback() to register a callback upon completion of
    /// the scan.
    virtual void scan() = 0;

    /// Get the RSSI of the AP in STA mode.
    /// @return signal strength, 0 when not connected to an access point, else
    ///         should be a negative number.
    virtual int rssi() = 0;

    /// Get the network hostname for the device. This is implemented as a const
    /// std::string and does not mutate.
    /// @return hostname
    virtual const std::string &get_hostname() = 0;

    /// Set the callback for when an IP address is acquired.
    /// @param iface interface index
    /// @param acquired true if acquired, false if lost
    virtual void set_ip_acquired_callback(
        std::function<void(Interface iface, bool acquired)> callback)
    {
        ipAcquiredCallback_ = std::move(callback);
    }

    /// Set the callback for when a wlan connect or disconnect occurs.
    /// Back-to-back deliveries of "disconnected" are possible without first
    /// transitioning to a connected state. This can occur when a connection
    /// "attempt" has failed to succeed.
    /// @param connected true if connected, else false if disconnected
    /// @param result result code for the connection event
    /// @param ssid SSID of the connection event
    virtual void set_wlan_connect_callback(std::function<void(bool connected,
        ConnectionResult result, const std::string &ssid)> callback)
    {
        wlanConnectedCallback_ = std::move(callback);
    }

    /// Set the callback for when a WiFi access point scan is finished.
    virtual void set_scan_finished_callback(std::function<void()> callback)
    {
        scanFinishedCallback_ = std::move(callback);
    }

    /// Reset any configuration and/or non-volatile storage to factory defaults.
    virtual void factory_reset() = 0;

protected:
    /// Constructor.
    WiFiInterface()
        : started_(false)
        , connected_(false)
        , ipAcquiredSta_(false)
        , ipAcquiredAp_(false)
        , ipLeased_(false)
    {
    }

    /// Deliver IP acquired callback, if registered.
    /// @param iface interface index
    /// @param acquired true if acquired, false if lost
    void ip_acquired(Interface iface, bool acquired)
    {
        if (ipAcquiredCallback_)
        {
            ipAcquiredCallback_(iface, acquired);
        }
    }

    /// Deliver WLAN connected callback, if registered.
    /// @param connected true if connected, else false if disconnected
    /// @param result result code for the connection event
    /// @param ssid SSID of the connection event
    void wlan_connected(
        bool connected, ConnectionResult result, const std::string &ssid)
    {
        if (wlanConnectedCallback_)
        {
            wlanConnectedCallback_(connected, result, ssid);
        }
    }

    /// Deliver scan finished callback, if registered.
    void scan_finished()
    {
        if (scanFinishedCallback_)
        {
            scanFinishedCallback_();
        }
    }

    /// Callback for when IP is acquired.
    /// @param iface interface index
    /// @param acquired true if acquired, false if lost
    std::function<void(Interface iface, bool acquired)> ipAcquiredCallback_;

    /// Callback for when WLAN connect or disconnect on STA occurs.
    /// @param connected true if connected, else false if disconnected
    /// @param result result code for the connection event
    /// @param ssid SSID of the connection event
    std::function<void(
        bool connected, ConnectionResult result, const std::string &ssid)>
            wlanConnectedCallback_;

    /// Callback for when a scan is finished.
    std::function<void()> scanFinishedCallback_;

    // Note: Because these are bitmasks, care must be taken in the derived
    //       class that there are no mutual exclusion issues on write.
    //       Implementations are expected to continuously update these bits as
    //       state transitions occur.
    bool started_       : 1; ///< WiFi started
    bool connected_     : 1; ///< STA mode connection to AP connected state
    bool ipAcquiredSta_ : 1; ///< IP address acquired STA mode
    bool ipAcquiredAp_  : 1; ///< IP address acquired AP mode
    bool ipLeased_      : 1; ///< IP address leased to a client (AP mode)
};

#endif // _FREERTOS_DRIVERS_COMMON_WIFIINTERFACE_HXX_