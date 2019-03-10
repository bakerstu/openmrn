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
#include <vector>

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
        /// The interface is currently in an initializing state. It is not
        /// expected to be usable at this point.
        INITIALIZING,
        /// The interface is in an IDLE state (not STATION or ACCESS POINT) but
        /// has been initialized. This is the expected state after calling
        /// @ref stop. The WiFi interface may also use OFF instead of this
        /// state. A call to @ref start would be required to prepare the
        /// interface for use.
        IDLE,
        /// The interface is currently connecting to an access point.
        CONNECTING,
        /// Interface is not associated with an access point. This state can be
        /// used in the event the interface has lost it's connection to the
        /// access point.
        NOT_ASSOCIATED,
        /// The WiFi interface has connected to an access point but has not yet
        /// received an IP address from the access point.
        NO_IP,
        /// The WiFi interface is in normal operating status and is ACTIVE as a
        /// STATION, ACCESS POINT or both (if supported).
        ACTIVE
    };

    /// WiFi Interface Operating mode.
    enum class WiFiMode : uint8_t
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

    /// Error codes used by the WiFiInterface APIs.
    enum class WiFiErrorCode : uint8_t
    {
        /// API completed without error.
        NO_ERROR = 0,
        /// The requested API is not supported by the implementation.
        UNSUPPORTED,
        /// An invalid argument was passed to the API.
        INVALID_ARGUMENT,

        /// Requested SSID was not found via network scan or connection attempts.
        SSID_NOT_FOUND,
        /// Invalid password (or preshared key) was provided when connecting to
        /// the requested access point.
        INVALID_PASSWORD,
        /// A timeout occurred for the SSID connection process.
        CONNECT_TIMEOUT,

        /// No networks were found via the network scan API.
        NO_NETWORKS_FOUND,
        /// The network scan operation timed out.
        NETWORK_SCAN_TIMEOUT,
        /// The network scan failed due to out of memory.
        NETWORK_SCAN_NOMEM,
        /// The network scan encountered a generic failure. This is WiFi
        /// interface specific and should have corresponding log entries to
        /// indicate the nature of the failure.
        NETWORK_SCAN_GENERIC_FAILURE,
        /// The network list is not available, this could indicate that no scan
        /// was performed or the scan failed.
        NETWORK_LIST_UNAVAILABLE,

        /// The SSID profile was successfully stored.
        PROFILE_CREATED,
        /// The SSID profile was successfully removed.
        PROFILE_DELETED,
        /// A stored SSID profile was not found with the provided SSID or
        /// the provided index is not valid.
        PROFILE_NOT_FOUND,
        /// The SSID profile could not be successfully stored as there was no
        /// space.
        PROFILE_OUT_OF_BOUNDS,
        /// The SSID profile could not be successfully stored due to generic
        /// failure. This is WiFi interface specific and should have
        /// corresponding log entries to indicate the nature of the failure.
        PROFILE_GENERIC_FAILURE,

        /// The WiFi interface does not have an IPv4 address available.
        IP_UNAVAILABLE,

        /// RSSI value is not known or is otherwise unavailable.
        RSSI_UNKNOWN
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
        /// BSSID (MAC address) of the access point.
        uint8_t bssid[6];
        /// Primary channel used by the access point.
        uint8_t channel;
    };

    /// Start/Initialize the WiFi interface with the requested @ref mode.
    ///
    /// @param mode is the requested mode of the WiFi interface.
    ///
    /// Note: If the WiFi interface has been started previously this API
    /// is treated as a NOOP by the WiFi interface.
    ///
    /// @return @ref WiFiErrorCode as the result starting the WiFi interface.
    virtual WiFiErrorCode start(WiFiMode mode = WiFiMode::STATION) = 0;

    /// Stops the WiFi interface in preparation for a reboot.
    ///
    /// Note: If the WiFi interface has not been started previously this API
    /// is treated as a NOOP by the WiFi interface.
    ///
    /// @return @ref WiFiErrorCode as the result starting the WiFi interface.
    virtual WiFiErrorCode stop() = 0;

    /// Connect to access point.
    ///
    /// Note: This API is synchronous and will block until the timeout value or
    /// the SSID connection completes.
    ///
    /// @param ssid access point ssid.
    /// @param password access point password (preshared key)
    /// @param security_type specifies security type.
    /// @param timeout is the number of milliseconds to allow for a connection
    /// to the access point. A value of zero indicates block until a final
    /// status has been reached (successful or otherwise).
    ///
    /// @return @ref WiFiErrorCode as the result of the connection attempt.
    virtual WiFiErrorCode connect(const std::string &ssid,
                                  const std::string &password,
                                  WiFiSecurity security_type,
                                  const uint32_t timeout = 0) = 0;

    /// Connects to an access point based on stored SSID profiles.
    ///
    /// The SSID profiles will be ordered first by priority and second by the
    /// RSSI value for the access point. Note that a successful connection is
    /// not guaranteed by this method.
    ///
    /// Note: This API is synchronous and will block until the timeout value or
    /// the SSID connection completes.
    ///
    /// @param timeout is the number of milliseconds to allow for a connection
    /// to the access point. A value of zero indicates block until a final
    /// status has been reached (successful or otherwise).
    ///
    /// @return @ref WiFiErrorCode as the result of the connection attempt.
    virtual WiFiErrorCode connect(const uint32_t timeout = 0) = 0;

    /// Connects to an access point via WPS Push Button Control.
    ///
    /// @param blocking will cause this method to block until the WPS process
    /// completes. Default is non-blocking.
    /// @param timeout is the number of milliseconds to wait for the WPS
    /// process to complete. A value of zero indicates that the interface
    /// default timeout should be used.
    ///
    /// @return @ref WiFiErrorCode as the result of the connection attempt.
    virtual WiFiErrorCode wps_pbc_connect(bool blocking = false,
                                          const uint32_t timeout = 0) = 0;

    /// Initiate the WPS PIN connection process.
    ///
    /// @param pin is the PIN value to send as part of the WPS connect process.
    /// @param blocking will cause this method to block until the WPS process
    /// completes. Default is non-blocking.
    /// @param timeout is the number of milliseconds to wait for the WPS
    /// process to complete. A value of zero indicates that the interface
    /// default timeout should be used.
    ///
    /// @return @ref WiFiErrorCode as the result of the connection attempt.
    virtual WiFiErrorCode wps_pin_connect(const std::string pin,
                                          bool blocking = false,
                                          const uint32_t timeout = 0) = 0;

    /// Creates an access point on this WiFi interface.
    ///
    /// Note: The ap_ip and dns_ip are in NETWORK byte order (big endian).
    ///
    /// @param ssid access point ssid.
    /// @param password password of the access point (preshared key), a blank
    /// string is only valid for @ref WiFiSecurity::OPEN.
    /// @param security specifies security type.
    /// @param ap_ip is the IP address for the access point.
    /// @param dns_ip is the DNS server IP address for the access point to
    /// provide to stations.
    ///
    /// @return @ref WiFiErrorCode as the result of setting up the access point.
    virtual WiFiErrorCode access_point_setup(const std::string &ssid,
                                             const std::string &password = "",
                                             WiFiSecurity security = WiFiSecurity::OPEN,
                                             uint32_t ap_ip = 0,
                                             uint32_t dns_ip = 0) = 0;

    /// @return the current WiFi interface operating mode.
    virtual WiFiMode get_mode() = 0;

    /// @return the current WiFi interface status.
    virtual WiFiState get_state() = 0;

    /// Adds a saved SSID profile.
    ///
    /// @param ssid SSID of the profile to save.
    /// @param password password of the SSID (preshared key), a blank string
    /// is only valid for @ref WiFiSecurity::OPEN.
    /// @param security @ref WiFiSecurity of the profile to be saved.
    /// @param priority connection priority when more than one of the saved
    /// networks is available, 0 == lowest priority
    /// @param profile_index will contain the index of the newly stored
    /// profile, if successfully stored.
    ///
    /// @return @ref WiFiErrorCode as the result of storing the profile.
    virtual WiFiErrorCode ssid_profile_add(const std::string &ssid,
                                           const std::string &password,
                                           const WiFiSecurity security,
                                           const uint8_t priority,
                                           uint8_t *profile_index) = 0;

    /// Delete a saved SSID profile by it's SSID.
    ///
    /// @param ssid SSID of the profile to delete.
    ///
    /// @return @ref WiFiErrorCode as the result of the removal of the profile.
    virtual WiFiErrorCode ssid_profile_del(const std::string &ssid) = 0;

    /// Delete a saved SSID profile by it's index.
    ///
    /// @param index index within saved profile list to remove, a value of 0xFF
    /// will remove all stored profiles.
    ///
    /// @return @ref WiFiErrorCode as the result of the removal of the profile.
    virtual WiFiErrorCode ssid_profile_del(const int index) = 0;

    /// Retrieves a saved SSID profile by index.
    ///
    /// @param index of the stored profile to retrieve.
    /// @param ssid will contain the ssid of the profile.
    /// @param security will contain the security type of the profile.
    /// @param priority will contain the priority of the profile.
    ///
    /// @return @ref WiFiErrorCode as the result of the retrieval of the
    /// profile.
    virtual WiFiErrorCode ssid_profile_get(const int index, std::string *ssid,
                                           WiFiSecurity *security,
                                           uint8_t *priority) = 0;

    /// @return number of stored SSID profiles.
    virtual uint8_t ssid_profile_count() = 0;

    /// Starts an access point scan.
    ///
    /// Note: This API will only execute *ONE* network scan per call.
    ///
    /// @param blocking will cause this method to block until the network scan
    /// completes. Default is non-blocking.
    /// @param timeout is the number of milliseconds to allow for the network
    /// scan to complete. This is only valid when @ref blocking is true.
    /// @param max_results is the maximum number of results to scan for. If the
    /// WiFi interface supports a lower number than requested and in such case
    /// the WiFi interface supported limit will be used instead.
    ///
    /// @return the number of networks found via scan, this value will be zero
    /// when @ref blocking is false.
    virtual int network_scan_single(bool blocking = false,
                                    uint32_t timeout = 0,
                                    uint8_t max_results = 20) = 0;

    /// Starts a continuous access point scan.
    ///
    /// Note: When there is already a network scan running, this will API will
    /// be treated as a NOOP.
    ///
    /// @return @ref WiFiErrorCode as the result of starting the network scan.
    virtual WiFiErrorCode network_scan_start() = 0;

    /// Stops a continuous access point scan.
    ///
    /// Note: When there is not a network scan running, this will API be
    /// treated as a NOOP.
    ///
    /// @return @ref WiFiErrorCode as the result of stopping the network scan.
    virtual WiFiErrorCode network_scan_stop() = 0;

    /// Retrieves a list of available access points.
    ///
    /// @param list will contain a list of available network entries. Note this
    /// will be cleared prior to appending the retrieved entries.
    ///
    /// @return @ref WiFiErrorCode as the result of retrieving the network
    /// list. When the network scan has not been completed this API should
    /// return @ref WiFiErrorCode::NETWORK_LIST_UNAVAILABLE.
    virtual WiFiErrorCode get_network_list(std::vector<NetworkListEntry> *list) = 0;

    /// Retrieves the WiFi interface MAC address.
    ///
    /// @param mac is a 6 byte array which will hold the resulting MAC address.
    /// @param mode is the WiFi interface to retrieve the MAC address for, if
    /// an interface does not support the passed mode it will set the mac to
    /// all zeros.
    ///
    /// @return @ref WiFiErrorCode as the result of retrieving the MAC address.
    virtual WiFiErrorCode get_mac(uint8_t *mac,
                                  WiFiMode mode = WiFiMode::STATION) = 0;

    /// Retrieves the IPv4 address assigned to the WiFi iterface.
    ///
    /// @param ip will contain the IP address for the requested WiFi interface.
    /// @param mode is the WiFi interface to return the IPv4 address for.
    /// 
    /// Note: The ip is in NETWORK byte order (big endian).
    ///
    /// @return @ref WiFiErrorCode as the result of retrieving the IPv4
    /// address of the WiFi interface. If the requested @ref mode is not
    /// supported by the WiFi interface it will return
    /// @ref WiFiErrorCode::UNSUPPORTED. If there is no IPv4 address assigned
    /// to the requested WiFi interface this will return
    /// @ref WiFiErrorCode::IP_UNAVAILABLE.
    virtual WiFiErrorCode get_ip(uint32_t *ip,
                                 WiFiMode mode = WiFiMode::STATION) = 0;

    /// @return SSID of the access point we are connected to, this can return
    /// a blank string when not connected to an access point. When the WiFi
    /// interface is operating as an ACCESS POINT only this can return the 
    /// SSID of the ACCESS POINT operating on the WiFi interface.
    virtual const std::string get_ssid() = 0;

    /// Retrieves the RSSI value for the WiFi interface.
    ///
    /// @param rssi will contain the RSSI value (if known).
    ///
    /// @return @ref WiFiErrorCode as the result of retrieval of the RSSI. When
    /// the WiFi interface is operating in ACCESS POINT only mode this API
    /// should return @ref WiFiErrorCode::INVALID_ARGUMENT. When the WiFi
    /// interface is operating in STATION and ACCESS POINT mode only the
    /// STATION RSSI will be retrieved.
    virtual WiFiErrorCode get_rssi(int *rssi) = 0;
};

#endif // _FREERTOS_DRIVERS_COMMON_WIFI_IFACE_HXX_