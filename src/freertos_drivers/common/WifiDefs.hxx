#ifndef _FREERTOS_DRIVERS_COMMON_WIFIDEFS_HXX_
#define _FREERTOS_DRIVERS_COMMON_WIFIDEFS_HXX_

#include <stdint.h>

#include <string>

/// Wifi not associated to access point: continuous short blinks.
#define WIFI_BLINK_NOTASSOCIATED  0b1010
/// Waiting for IP address: double short blink, pause, double short blink, ...
#define WIFI_BLINK_ASSOC_NOIP  0b101000
/// Connecting to hub: long blinks
#define WIFI_BLINK_CONNECTING  0b1100
/// Connecting to hub: long blinks
#define WIFI_BLINK_FAILED  0b10101100

enum class WlanState : uint8_t
{
    OK = 0,
    NOT_ASSOCIATED = 1,
    NO_IP,
    NO_CONNECTION,
    CONNECTING,
    MDNS_LOOKUP = 5,
    CONNECT_MDNS,
    CONNECT_STATIC,
    CONNECT_FAILED,
    CONNECTION_LOST,
    WRONG_PASSWORD,
    UPDATE_DISPLAY = 20,
};

/** Operating Role.
 */
enum class WlanRole : uint8_t
{
    UNKNOWN = 0,       /**< Default mode (from stored configuration) */
    DEFAULT_ROLE = UNKNOWN, /**< Default mode (from stored configuration) */
    STA,               /**< Wi-Fi station mode */
    AP,                /**< Wi-Fi access point mode */
    AP_STA,            /**< Wi-Fi access point + station mode */
};

enum class CountryCode : uint8_t
{
    US, ///< United States
    EU, ///< European Union
    JP, ///< Japan
    UNKNOWN, ///< unknown country code
};

enum class WlanConnectResult
{
    CONNECT_OK = 0, ///< success
    PASSWORD_INVALID, /// password privided is invalid
};

extern "C" {
/// Name of wifi accesspoint to connect to.
extern char WIFI_SSID[];
/// Password of wifi connection. If empty, use no encryption.
extern char WIFI_PASS[];
/// Hostname at which the OpenLCB hub is at.
extern char WIFI_HUB_HOSTNAME[];
/// Port number of the OpenLCB hub.
extern int WIFI_HUB_PORT;
}

/// Useful WiFi Definitions. Eventually, the enums above should be encorporated
/// into this structure, but for now they are left separate for legacy code.
struct WiFiDefs
{
    /// Interface index by type.
    enum Interface : uint8_t
    {
        IFACE_STA, ///< STA mode interface
        IFACE_AP, ///< AP mode interface
    };

    /// Security types.
    enum SecurityType : uint8_t
    {
        SEC_OPEN = 0, ///< open (no security)
        SEC_WEP, ///< WEP security mode
        SEC_WPA2, ///< WPA2 security mode
        SEC_WPA3, ///< WPA3 security mode
    };

    /// Turns a security type into a user-readable string.
    /// @param sec the security type enum.
    /// @return a user-readable string. The pointer is valid for the entire
    /// lifetime of the program.
    static const char *security_type_to_string(SecurityType sec);

    /// Result code for connections and disconnections.
    enum ConnectionResult : uint8_t
    {
        CONNECT_OK = 0, ///< connection succeeded
        AUTHENTICATION_FAILED, ///< authentication failure
        ASSOCIATION_FAILED, ///< association failure
        CONNECT_UNKNOWN, ///< unknown result
    };

    /// Network info, typically used in an access point scan.
    struct NetworkEntry
    {
        /// Constructor.
        /// @param ssid SSID of the access point
        /// @param sec_type security type of the access point
        /// @param rssi receive signal strength of the access point
        /// @param channel channel of the AP, 0 means any/unknown
        NetworkEntry(const char *ssid, int rssi, SecurityType sec_type,
            uint8_t channel = 0)
            : ssid(ssid)
            , rssi(rssi)
            , secType(sec_type)
            , channel(channel)
        {
        }

        std::string ssid; ///< SSID of the access point
        int16_t rssi; ///< receive signal strength of the access point
        SecurityType secType; ///< security type of the access point
        uint8_t channel; ///< channel of the AP, 0 means any/unknown
    };
};

#endif // _FREERTOS_DRIVERS_COMMON_WIFIDEFS_HXX_
