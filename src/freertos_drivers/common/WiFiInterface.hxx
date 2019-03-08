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
 * \file WiFiInterface.hxx
 * Generic interface for a WiFi Interface implementation.
 *
 * @author Mike Dunston
 * @date 7 March 2019
 */

#ifndef _FREERTOS_DRIVERS_COMMON_WIFI_IFACE_HXX_
#define _FREERTOS_DRIVERS_COMMON_WIFI_IFACE_HXX_

#include <stdint.h>
#include <string>

/// Common interface for WiFi interfaces.
class WiFiInterface
{
public:
    /// Security types for access points.
    enum WiFiSecurity
    {
        /// Access point has no security.
        OPEN,
        /// Access point uses WEP security.
        WEP,
        /// Access point uses WPA2 security.
        WPA2,
    };

    /// Status of the WiFi interface.
    enum class WiFiState : uint8_t
    {
        /// The interface has not been initialized or is otherwise not in a
        /// usable state. A call to @ref start would be required. This is the
        /// default state of the interface when the WiFiInterface instance is
        /// created.
        OFF = 0,
        /// The interface is in an IDLE state (not STATION or ACCESS POINT) but
        /// has been initialized. This is the expected state after calling
        /// @ref stop. A call to @ref start would be required to prepare the
        /// interface for use.
        IDLE,
        /// The interface is currently in an initializing state. It is not
        /// expected to be usable at this point.
        INITIALIZING,
        /// The interface is currently connecting to an access point.
        CONNECTING,
        /// Interface is not associated with an access point. This state can be
        /// used in the event the interface has lost it's connection to the
        /// access point.
        NOT_ASSOCIATED,
        /// Interface has not yet received an IP address from the connected
        /// access point.
        NO_IP,
        /// The interface is in normal operating status and is ACTIVE as either
        /// a STATION, ACCESS POINT or both (if supported).
        ACTIVE
    };

    /// WiFi Interface Operating mode.
    enum class InterfaceMode : uint8_t
    {
        /// Interface mode is not known.
        UNKNOWN = 0,
        /// Interface operates as a STATION (connects to access point).
        STATION,
        /// Interface operates as an ACCESS POINT (other STATIONS can connect
        /// to it).
        ACCESS_POINT,
        /// Interface will operate as both a STATION and ACCESS POINT. If the
        /// interface does not support this mode an error will be raised by the
        /// @ref start method.
        STATION_AND_ACCESS_POINT
    };

    /// Contains the metadata for an access point.
    struct NetworkListEntry
    {
        /// SSID of the access point.
        std::string ssid;
        /// Security type of the access point.
        WiFiSecurity security;
        /// Receive Signal Strength Indicator of the access point.
        int rssi;
    };

    /// Startup the WiFi interface.
    ///
    /// @param mode is the requested mode of the WiFi interface.
    /// @return zero upon successfully starting in the provided mode, non-zero
    /// otherwise.
    int start(InterfaceMode mode = InterfaceMode::STATION);

    /// Stops the WiFi interface in preparation for a reboot.
    void stop();

    /// @return true if WiFi interface has been started, false otherwise.
    bool is_started();

    /// Connect to access point.
    ///
    /// @param ssid access point ssid.
    /// @param security_key access point security key.
    /// @param security_type specifies security type.
    void connect(const char *ssid, const char *security_key,
                 WiFiSecurity security_type);

    /// Connects to an access point based on stored SSID profiles.
    ///
    /// The SSID profiles will be ordered first by priority and second by
    /// the RSSI value for the access point. Note that a successful connection
    /// is not guaranteed by this method.
    void connect();

    /// Creates an access point on this WiFi interface.
    /// 
    /// @param ssid access point ssid.
    /// @param security_key access point security key.
    /// @param security_type specifies security type.
    /// @param ap_ip is the IP address for the access point.
    /// @param dns_ip is the DNS server IP address for the access point to
    /// provide to stations.
    void setup_access_point(const char *ssid, const char *security_key,
                            WiFiSecurity security_type, uint32_t ap_ip=0,
                            uint32_t dns_ip=0);

    /// Initiate the WPS Push Button Control connection process.
    ///
    /// @param timeout is the number of milliseconds to wait for the WPS
    /// process to complete. A value of zero indicates that the interface
    /// default timeout should be used.
    void initiate_wps_connect(uint32_t timeout=0);

    /// @return the current WiFi interface operating mode.
    InterfaceMode get_mode();

    /// @return the current WiFi interface status.
    WiFiState get_state();

    /// Adds a saved SSID profile.
    ///
    /// @param ssid SSID of the profile to save.
    /// @param sec_type @ref WiFiSecurity of the profile to be saved.
    /// @param key password of the SSID, nullptr allowed if sec_type is OPEN.
    /// @param priority connection priority when more than one of the saved
    /// networks is available, 0 == lowest priority
    /// @return resulting index in the list of profiles, else -1 on error
    int add_ssid_profile(const char *ssid, WiFiSecurity sec_type,
                         const char *key, unsigned priority);

    /// Delete a saved SSID profile by it's SSID.
    ///
    /// @param ssid SSID of the profile to delete.
    /// @return 0 upon success, else -1 on error.
    int del_ssid_profile(const char *ssid);

    /// Delete a saved SSID profile.
    ///
    /// @param index index within saved profile list to remove, a value of 0xFF
    /// will remove all stored profiles.
    /// @return 0 upon success, else -1 on error.
    int del_ssid_profile(int index);

    /// Retrieves a saved SSID profile by index.
    ///
    /// @param index of the stored profile to retrieve.
    /// @param ssid is a 33 byte array containing the ssid of the profile.
    /// @param sec_type is the security type of the profile.
    /// @param priority is the priority of the profile.
    /// @return 0 upon success, else -1 on error.
    int get_ssid_profile(int index, char ssid[], WiFiSecurity *sec_type,
                         uint32_t *priority);

    /// @return true if there are no saved SSID profiles, false otherwise.
    bool are_ssid_profiles_empty();

    /// Retrieves a list of available access points.
    ///
    /// @param entries will contain a list of available network entries.
    /// @param count is the number of network entries to retrieve, max of 20.
    /// @return number of network entries retrieved.
    int get_network_list(NetworkListEntry *entries, size_t count);

    /// Initiate scanning available networks.
    ///
    /// @param blocking will cause this method to block until the network scan
    /// completes.
    /// @return the number of networks found via scan, this value should be
    /// ignored when blocking is false (default).
    int network_list_scan(bool blocking=false);

    /// Retrieves the WiFi interface MAC address.
    ///
    /// @param mac 6 byte array which will hold the resulting MAC address.
    void get_mac(uint8_t mac[6]);

    /// @return the assigned IP address for the WiFi interface, if not
    /// connected or unavailable this will return zero.
    uint32_t get_ip();

    /// @return SSID of the access point we are connected to, can return
    /// nullptr if not currently connected to an access point. When the
    /// interface is operating as an ACCESS POINT only this can return the 
    /// SSID of the ACCESS POINT operating on the WiFi interface.
    const char *get_ssid();

    /// @return RSSI value of the currently connected access point, can return
    /// zero if not currently connected or if it is unknown.
    int get_rssi();
};

#endif // _FREERTOS_DRIVERS_COMMON_WIFI_IFACE_HXX_