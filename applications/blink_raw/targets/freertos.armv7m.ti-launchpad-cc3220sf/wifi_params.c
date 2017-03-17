/// Name of wifi accesspoint to connect to.
char WIFI_SSID[] __attribute__((weak)) = "YourWifiNameHere";
/// Password of wifi connection. If empty, use no encryption.
char WIFI_PASS[] __attribute__((weak)) = "PasswordHere";
/// Hostname at which the OpenLCB hub is at.
char WIFI_HUB_HOSTNAME[] __attribute__((weak)) = "192.168.0.2";
/// Port number of the OpenLCB hub.
int WIFI_HUB_PORT __attribute__((weak)) = 12021;
