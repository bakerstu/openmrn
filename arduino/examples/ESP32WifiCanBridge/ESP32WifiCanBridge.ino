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
 * \file ESP32WiFiCanBridge.ino
 *
 * Example application for the ESP32 showing how to configure a WiFi to CAN
 * bridge.
 *
 * @author Mike Dunston
 * @date 21 January 2019
 */

#include <Arduino.h>
#include <SPIFFS.h>

#include <OpenMRNLite.h>

#include "config.h"

// uncomment the line below to have all packets printed to the Serial
// output. This is not recommended for production deployment.
//#define PRINT_PACKETS

/// This is the speed used for UART0 on the ESP32, this is primarily for
/// debug/log messages.
constexpr uint32_t SERIAL_BAUD = 115200L;

/// This is the ESP32 pin connected to the SN65HVD23x/MCP2551 R (RX) pin.
/// Recommended pins: 4, 16, 21.
/// Note: Any pin can be used for this other than 6-11 which are connected to
/// the onboard flash.
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_4;

/// This is the ESP32 pin connected to the SN65HVD23x/MCP2551 D (TX) pin.
/// Recommended pins: 5, 17, 22.
/// Note: Any pin can be used for this other than 6-11 which are connected to
/// the onboard flash and 34-39 which are input only.
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_5;

/// This is the node id to assign to this device, this must be unique
/// on the CAN bus.
static constexpr uint64_t NODE_ID = UINT64_C(0x050101011828);

// Configuring WiFi accesspoint name and password
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// There are two options:
// 1) edit the sketch to set this information just below. Use quotes:
//     const char* ssid     = "linksys";
//     const char* password = "superSecret";
// 2) add a new file to the sketch folder called something.cpp with the
// following contents:
//     #include <OpenMRNLite.h>
//
//     char WIFI_SSID[] = "linksys";
//     char WIFI_PASS[] = "theTRUEsupers3cr3t";

/// This is the name of the WiFi network (access point) to connect to.
const char *ssid = WIFI_SSID;

/// Password of the wifi network.
const char *password = WIFI_PASS;

/// This is the hostname which the ESP32 will advertise via mDNS, it should be
/// unique.
const char *hostname = "esp32mrn";

// Uncomment this line to enable usage of ::select() within the Grid Connect
// code.
//OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);

/// This is the primary entrypoint for the OpenMRN/LCC stack.
OpenMRN openmrn(NODE_ID);

// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

// ConfigDef comes from config.h and is specific to this particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

Esp32WiFiManager wifi_mgr(ssid, password, openmrn.stack(), cfg.seg().wifi());

// @ref Esp32HardwareTwai instance to use for this node.
static Esp32HardwareTwai twai(CAN_RX_PIN, CAN_TX_PIN);

class FactoryResetHelper : public DefaultConfigUpdateListener
{
public:
    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);
        return UPDATED;
    }

    void factory_reset(int fd) override
    {
        cfg.userinfo().name().write(fd, openlcb::SNIP_STATIC_DATA.model_name);
        cfg.userinfo().description().write(
            fd, "OpenLCB + Arduino-ESP32 on an ESP32.");
    }
} factory_reset_helper;

namespace openlcb
{
    // Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = "/spiffs/cdi.xml";

    // This will stop openlcb from exporting the CDI memory space upon start.
    const char CDI_DATA[] = "";

    // Path to where OpenMRN should persist general configuration data.
    const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";

    // The size of the memory space to export over the above device.
    const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

    // Default to store the dynamic SNIP data is stored in the same persistant
    // data file as general configuration data.
    const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);

    // Initialize the SPIFFS filesystem as our persistence layer
    if (!SPIFFS.begin())
    {
        printf("SPIFFS failed to mount, attempting to format and remount\n");
        if (!SPIFFS.begin(true))
        {
            printf("SPIFFS mount failed even with format, giving up!\n");
            while (1)
            {
                // Unable to start SPIFFS successfully, give up and wait
                // for WDT to kick in
            }
        }
    }

    // Create the CDI.xml dynamically
    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

    // Create the default internal configuration file
    openmrn.stack()->create_config_file_if_needed(cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // Initialize the TWAI peripheral
    twai.hw_init();
    // Add TWAI port to the stack
    openmrn.add_can_port_select("/dev/twai/twai0");

    // Start the OpenMRN stack
    openmrn.begin();
    openmrn.start_executor_thread();

#if defined(PRINT_PACKETS)
    // Dump all packets as they are sent/received.
    // Note: This should not be enabled in deployed nodes as it will
    // have performance impact.
    openmrn.stack()->print_all_packets();
#endif // PRINT_PACKETS
}

void loop()
{
    // Call the OpenMRN executor, this needs to be done as often
    // as possible from the loop() method.
    openmrn.loop();
}
