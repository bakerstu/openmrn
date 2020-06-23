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
 * \file ESP32S2IOBoard.ino
 *
 * Main file for the io board application on an ESP32-S2.
 *
 * @author Mike Dunston
 * @date 13 January 2019
 */

#include <Arduino.h>
#include <SPIFFS.h>

#include <OpenMRNLite.h>
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/MultiConfiguredConsumer.hxx"
#include "utils/GpioInitializer.hxx"

// uncomment the line below to have all packets printed to the Serial output.
// This is not recommended for production deployment.
//#define PRINT_PACKETS

// uncomment the line below to use GPIO 18 as an IO pin. This pin is typically
// connected to a WS2812 RGB LED. With GPIO 18 reserved for the LED, there are
// 31 IO pins exposed in the CDI, by enabling this option it will increase that
// to 32 IO pins.
// #define USE_GPIO_18_FOR_IO

#include "config.h"

/// This is the node id to assign to this device, this must be unique
/// on the CAN bus.
static constexpr uint64_t NODE_ID = UINT64_C(0x050101011825);

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

/// ConfigDef comes from config.h and is specific to this particular device and
/// target. It defines the layout of the configuration memory space and is also
/// used to generate the cdi.xml file. Here we instantiate the configuration
/// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

Esp32WiFiManager wifi_mgr(ssid, password, openmrn.stack(), cfg.seg().wifi());

// Declare all GPIO pins to be used on the ESP32-S2.
GPIO_PIN(GPIO0, GpioOutputSafeLow, 0);
GPIO_PIN(GPIO1, GpioOutputSafeLow, 1);
GPIO_PIN(GPIO2, GpioOutputSafeLow, 2);
GPIO_PIN(GPIO3, GpioOutputSafeLow, 3);
GPIO_PIN(GPIO4, GpioOutputSafeLow, 4);
GPIO_PIN(GPIO5, GpioOutputSafeLow, 5);
GPIO_PIN(GPIO6, GpioOutputSafeLow, 6);
GPIO_PIN(GPIO7, GpioOutputSafeLow, 7);
GPIO_PIN(GPIO8, GpioOutputSafeLow, 8);
GPIO_PIN(GPIO9, GpioOutputSafeLow, 9);
GPIO_PIN(GPIO10, GpioOutputSafeLow, 10);
GPIO_PIN(GPIO11, GpioOutputSafeLow, 11);
GPIO_PIN(GPIO12, GpioOutputSafeLow, 12);
GPIO_PIN(GPIO13, GpioOutputSafeLow, 13);
GPIO_PIN(GPIO14, GpioOutputSafeLow, 14);
GPIO_PIN(GPIO15, GpioOutputSafeLow, 15);
GPIO_PIN(GPIO16, GpioOutputSafeLow, 16);
GPIO_PIN(GPIO17, GpioOutputSafeLow, 17);
#ifdef USE_GPIO_18_FOR_IO
GPIO_PIN(GPIO18, GpioOutputSafeLow, 18);
#endif // USE_GPIO_18_FOR_IO
GPIO_PIN(GPIO19, GpioOutputSafeLow, 19);
GPIO_PIN(GPIO20, GpioOutputSafeLow, 20);
GPIO_PIN(GPIO21, GpioOutputSafeLow, 21);
GPIO_PIN(GPIO33, GpioOutputSafeLow, 33);
GPIO_PIN(GPIO34, GpioOutputSafeLow, 34);
GPIO_PIN(GPIO35, GpioOutputSafeLow, 35);
GPIO_PIN(GPIO36, GpioOutputSafeLow, 36);
GPIO_PIN(GPIO37, GpioOutputSafeLow, 37);
GPIO_PIN(GPIO38, GpioOutputSafeLow, 38);
GPIO_PIN(GPIO39, GpioOutputSafeLow, 39);
GPIO_PIN(GPIO40, GpioOutputSafeLow, 40);
GPIO_PIN(GPIO41, GpioOutputSafeLow, 41);
GPIO_PIN(GPIO45, GpioOutputSafeLow, 45);

// NOTE: GPIO42 and GPIO46 are skipped and reserved for TWAI (CAN).

// List of GPIO objects that will be used for the configurable IO pins. You
// should keep the constexpr declaration, because it will produce a compile
// error in case the list of pointers cannot be compiled into a compiler
// constant and thus would be placed into RAM instead of ROM.
constexpr const Gpio *const gpio_set[] =
{
    GPIO0_Pin::instance(),  GPIO1_Pin::instance(),  GPIO2_Pin::instance(),  //
    GPIO3_Pin::instance(),  GPIO4_Pin::instance(),  GPIO5_Pin::instance(),  //
    GPIO6_Pin::instance(),  GPIO7_Pin::instance(),  GPIO8_Pin::instance(),  //
    GPIO9_Pin::instance(),  GPIO10_Pin::instance(), GPIO11_Pin::instance(), //
    GPIO12_Pin::instance(), GPIO13_Pin::instance(), GPIO14_Pin::instance(), //
    GPIO15_Pin::instance(), GPIO16_Pin::instance(), GPIO17_Pin::instance(), //
#ifdef USE_GPIO_18_FOR_IO
    GPIO18_Pin::instance(),
#endif // USE_GPIO_18_FOR_IO
    GPIO19_Pin::instance(), GPIO20_Pin::instance(), GPIO21_Pin::instance(), //
    GPIO33_Pin::instance(), GPIO34_Pin::instance(), GPIO35_Pin::instance(), //
    GPIO36_Pin::instance(), GPIO37_Pin::instance(), GPIO38_Pin::instance(), //
    GPIO39_Pin::instance(), GPIO40_Pin::instance(), GPIO41_Pin::instance(),  //
    GPIO45_Pin::instance()
};

// Configurable IO handler.
openlcb::MultiConfiguredPC gpio(openmrn.stack()->node(), gpio_set,
                                ARRAYSIZE(gpio_set), cfg.seg().gpio());

// Create an initializer that can initialize all the GPIO pins in one shot
typedef GpioInitializer<
    GPIO0_Pin,  GPIO1_Pin,  GPIO2_Pin,  GPIO3_Pin,  GPIO4_Pin,  GPIO5_Pin,  //
    GPIO6_Pin,  GPIO7_Pin,  GPIO8_Pin,  GPIO9_Pin,  GPIO10_Pin, GPIO11_Pin, //
    GPIO12_Pin, GPIO13_Pin, GPIO14_Pin, GPIO15_Pin, GPIO16_Pin, GPIO17_Pin, //
#ifdef USE_GPIO_18_FOR_IO
    GPIO18_Pin,
#endif // USE_GPIO_18_FOR_IO
    GPIO19_Pin, GPIO20_Pin, GPIO21_Pin, GPIO33_Pin, GPIO34_Pin, GPIO35_Pin, //
    GPIO36_Pin, GPIO37_Pin, GPIO38_Pin, GPIO39_Pin, GPIO40_Pin, GPIO41_Pin, //
    GPIO45_Pin
    > GpioInit;

// The GPIO pins need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the
// producers to it.
openlcb::RefreshLoop producer_refresh_loop(openmrn.stack()->node(),
    {
        gpio.polling()
    }
);

class FactoryResetHelper : public DefaultConfigUpdateListener {
public:
    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE {
        AutoNotify n(done);
        return UPDATED;
    }

    void factory_reset(int fd) override
    {
        cfg.userinfo().name().write(fd, openlcb::SNIP_STATIC_DATA.model_name);
        cfg.userinfo().description().write(
            fd, "OpenLCB + Arduino-ESP32 on an ESP32-S2.");
    }
} factory_reset_helper;

namespace openlcb
{
    // Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = "/spiffs/cdi.xml";

    // This will stop openlcb from exporting the CDI memory space upon start.
    extern const char CDI_DATA[] = "";

    // Path to where OpenMRN should persist general configuration data.
    extern const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";

    // The size of the memory space to export over the above device.
    extern const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

    // Default to store the dynamic SNIP data is stored in the same persistant
    // data file as general configuration data.
    extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

void setup()
{
    Serial.begin(115200L);

    // Initialize the SPIFFS filesystem as our persistence layer
    if (!SPIFFS.begin())
    {
        printf("SPIFFS failed to mount, attempting to format and remount\n");
        // unmount the SPIFFS filesystem before attempting to mount/format.
        SPIFFS.end();
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

    // initialize all declared GPIO pins
    GpioInit::hw_init();

    // Start the OpenMRN stack
    openmrn.begin();

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
