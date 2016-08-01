

/// Wifi not associated to access point: continuous short blinks.
#define WIFI_BLINK_NOTASSOCIATED  0b1010
/// Waiting for IP address: double short blink, pause, double short blink, ...
#define WIFI_BLINK_ASSOC_NOIP  0b101000
/// Connecting to hub: long blinks
#define WIFI_BLINK_CONNECTING  0b1100


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
