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
 * \file Esp32WiFiConfiguration.hxx
 *
 * ESP32 WiFiConfiguration CDI declarations
 *
 * @author Mike Dunston
 * @date 11 February 2019
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32WIFICONFIG_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32WIFICONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredTcpConnection.hxx"

namespace openmrn_arduino
{

/// Names and Descriptions for all ESP32 exposed WiFi configuration options.
class Esp32WiFiConfigurationParams
{
public:
    /// <map> of possible keys and descriptive values to show to the user for
    /// the wifi_sleep and hub_mode fields.
    static constexpr const char *BOOLEAN_MAP =
        "<relation><property>0</property><value>No</value></relation>"
        "<relation><property>1</property><value>Yes</value></relation>";

    /// Visible name for the WiFi Power Savings mode.
    static constexpr const char *WIFI_POWER_SAVE_NAME =
        "WiFi Power Savings Mode";

    /// Visible description for the WiFi Power Savings mode.
    static constexpr const char *WIFI_POWER_SAVE_DESC =
        "When enabled this allows the ESP32 WiFi radio to use power savings "
        "mode which puts the radio to sleep except to receive beacon updates "
        "from the connected SSID. This should generally not need to be "
        "enabled unless you are powering the ESP32 from a battery.";

    /// Visible name for the WiFi TX Power.
    static constexpr const char *WIFI_TX_POWER_NAME =
        "WiFi Transmit Power";

    /// Visible description for the WiFi TX Power.
    static constexpr const char *WIFI_TX_POWER_DESC =
        "WiFi Radio transmit power in dBm. This can be used to limit the WiFi "
        "range. This option generally does not need to be changed.\n"
        "NOTE: Setting this option to a very low value can cause communication "
        "failures.";

    /// <map> of possible key and descriptive values to show to the user for
    /// the power field.
    static constexpr const char *WIFI_TX_POWER_MAP =
        "<relation><property>8</property><value>2 dBm</value></relation>"
        "<relation><property>20</property><value>5 dBm</value></relation>"
        "<relation><property>28</property><value>7 dBm</value></relation>"
        "<relation><property>34</property><value>8 dBm</value></relation>"
        "<relation><property>44</property><value>11 dBm</value></relation>"
        "<relation><property>52</property><value>13 dBm</value></relation>"
        "<relation><property>56</property><value>14 dBm</value></relation>"
        "<relation><property>60</property><value>15 dBm</value></relation>"
        "<relation><property>66</property><value>16 dBm</value></relation>"
        "<relation><property>72</property><value>18 dBm</value></relation>"
        "<relation><property>78</property><value>20 dBm</value></relation>";

    /// Visible name for the Hub Configuration group.
    static constexpr const char *HUB_NAME = "Hub Configuration";

    /// Visible description for the Hub Configuration group.
    static constexpr const char *HUB_DESC =
        "Configuration settings for an OpenLCB Hub";

    /// Visible name for the hub enable field.
    static constexpr const char *HUB_ENABLE_NAME = "Enable Hub Mode";

    /// Visible description for the hub enable field.
    static constexpr const char *HUB_ENABLE_DESC =
        "Defines this node as a hub which can accept connections";

    /// Visible name for the hub_listener_port field.
    static constexpr const char *HUB_LISTENER_PORT_NAME = "Hub Listener Port";

    /// Visible name for the hub_listener_port field.
    static constexpr const char *HUB_LISTENER_PORT_DESC =
        "Defines the TCP/IP listener port this node will use when operating "
        "as a hub. Most of the time this does not need to be changed.";

    /// Visible name for the link_config group.
    static constexpr const char *UPLINK_NAME = "Node Uplink Configuration";

    /// Visible name for the link_config group.
    static constexpr const char *UPLINK_DESC =
        "Configures how this node will connect to other nodes.";
};

/// CDI Configuration for an @ref Esp32WiFiManager managed hub.
CDI_GROUP(HubConfiguration);
/// Allows the node to become a Grid Connect Hub.
CDI_GROUP_ENTRY(enable, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::HUB_ENABLE_NAME),
    Description(Esp32WiFiConfigurationParams::HUB_ENABLE_DESC), Min(0), Max(1),
    Default(0), MapValues(Esp32WiFiConfigurationParams::BOOLEAN_MAP));
/// Specifies the port which should be used by the hub.
CDI_GROUP_ENTRY(port, openlcb::Uint16ConfigEntry,
    Name(Esp32WiFiConfigurationParams::HUB_LISTENER_PORT_NAME),
    Description(Esp32WiFiConfigurationParams::HUB_LISTENER_PORT_DESC), Min(1),
    Max(65535), Default(openlcb::TcpClientDefaultParams::DEFAULT_PORT))
/// Specifies the mDNS service name to advertise for the hub.
CDI_GROUP_ENTRY(service_name, openlcb::StringConfigEntry<48>,
    Name(openlcb::TcpClientDefaultParams::SERVICE_NAME),
    Description(openlcb::TcpClientDefaultParams::SERVICE_DESCR));
/// Reserved space for future expansion.
CDI_GROUP_ENTRY(reserved, openlcb::BytesConfigEntry<6>, Hidden(true));
CDI_GROUP_END();

/// CDI Configuration for an @ref Esp32WiFiManager managed node.
CDI_GROUP(WiFiConfiguration);
/// Allows the WiFi system to use power-saving techniques to conserve power
/// when the node is powered via battery.
CDI_GROUP_ENTRY(sleep, openlcb::Uint8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::WIFI_POWER_SAVE_NAME),
    Description(Esp32WiFiConfigurationParams::WIFI_POWER_SAVE_DESC), Min(0),
    Max(1), Default(0), MapValues(Esp32WiFiConfigurationParams::BOOLEAN_MAP));
/// Allows adjustment of the WiFi TX power. This can be beneficial for reducing
/// the available range of the SoftAP. However, it can cause communication
/// failures when connecting nodes via TCP/IP.
CDI_GROUP_ENTRY(tx_power, openlcb::Int8ConfigEntry,
    Name(Esp32WiFiConfigurationParams::WIFI_TX_POWER_NAME),
    Description(Esp32WiFiConfigurationParams::WIFI_TX_POWER_DESC), Min(8),
    Max(79), Default(78),
    MapValues(Esp32WiFiConfigurationParams::WIFI_TX_POWER_MAP));
#if defined(CONFIG_IDF_TARGET_ESP32)
/// CDI Configuration to enable this node to be a hub.
CDI_GROUP_ENTRY(hub, HubConfiguration,
    Name(Esp32WiFiConfigurationParams::HUB_NAME),
    Description(Esp32WiFiConfigurationParams::HUB_DESC));
#endif
/// CDI Configuration for this node's connection to an uplink hub.
CDI_GROUP_ENTRY(uplink,
    openlcb::TcpClientConfig<openlcb::TcpClientDefaultParams>,
    Name(Esp32WiFiConfigurationParams::UPLINK_NAME),
    Description(Esp32WiFiConfigurationParams::UPLINK_DESC));
// TODO: update uplink to only expose auto (mDNS only) and manual host/port.
CDI_GROUP_END();

} // namespace openmrn_arduino

using openmrn_arduino::WiFiConfiguration;

#endif // _FREERTOS_DRIVERS_ESP32_ESP32WIFICONFIG_HXX_