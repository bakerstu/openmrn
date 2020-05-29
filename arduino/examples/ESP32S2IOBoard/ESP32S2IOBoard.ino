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

// Declare all OUTPUT pins to be used on the ESP32-S2.
GPIO_PIN(OUTPUT0, GpioOutputSafeLow, 0);
GPIO_PIN(OUTPUT1, GpioOutputSafeLow, 1);
GPIO_PIN(OUTPUT2, GpioOutputSafeLow, 2);
GPIO_PIN(OUTPUT3, GpioOutputSafeLow, 3);
GPIO_PIN(OUTPUT4, GpioOutputSafeLow, 4);
GPIO_PIN(OUTPUT5, GpioOutputSafeLow, 5);
GPIO_PIN(OUTPUT6, GpioOutputSafeLow, 6);
GPIO_PIN(OUTPUT7, GpioOutputSafeLow, 7);
GPIO_PIN(OUTPUT8, GpioOutputSafeLow, 8);
GPIO_PIN(OUTPUT9, GpioOutputSafeLow, 9);
GPIO_PIN(OUTPUT10, GpioOutputSafeLow, 10);
GPIO_PIN(OUTPUT11, GpioOutputSafeLow, 11);
GPIO_PIN(OUTPUT12, GpioOutputSafeLow, 12);
GPIO_PIN(OUTPUT13, GpioOutputSafeLow, 13);
GPIO_PIN(OUTPUT14, GpioOutputSafeLow, 14);
GPIO_PIN(OUTPUT15, GpioOutputSafeLow, 45);
// NOTE: OUTPUT15 is configured to use GPIO 45 rather than GPIO 15 due to GPIO
// 45 having a pull-down which will interfere with the GpioInputPU state used
// for INPUTs below. It is generally not recommended to enable both a pull-up
// and pull-down on the same pin, especially when one is done with a resistor.

// Declare all INPUT pins to be used on the ESP32-S2.
// NOTE: GPIO42 and GPIO46 are skipped to keep it free for TWAI (CAN).
// NOTE: GPIO18 skipped due to on-board WS2812 RGB LED.
GPIO_PIN(INPUT0, GpioInputPU, 16);
GPIO_PIN(INPUT1, GpioInputPU, 17);
GPIO_PIN(INPUT2, GpioInputPU, 19);
GPIO_PIN(INPUT3, GpioInputPU, 20);
GPIO_PIN(INPUT4, GpioInputPU, 21);
GPIO_PIN(INPUT5, GpioInputPU, 33);
GPIO_PIN(INPUT6, GpioInputPU, 34);
GPIO_PIN(INPUT7, GpioInputPU, 35);
GPIO_PIN(INPUT8, GpioInputPU, 36);
GPIO_PIN(INPUT9, GpioInputPU, 37);
GPIO_PIN(INPUT10, GpioInputPU, 38);
GPIO_PIN(INPUT11, GpioInputPU, 39);
GPIO_PIN(INPUT12, GpioInputPU, 40);
GPIO_PIN(INPUT13, GpioInputPU, 41);
GPIO_PIN(INPUT14, GpioInputPU, 15);
#ifdef USE_GPIO_18_FOR_IO
GPIO_PIN(INPUT15, GpioInputPU, 18);
#endif // USE_GPIO_18_FOR_IO

openlcb::ConfiguredProducer INPUT0_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<0>(), INPUT0_Pin());
openlcb::ConfiguredProducer INPUT1_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<1>(), INPUT1_Pin());
openlcb::ConfiguredProducer INPUT2_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<2>(), INPUT2_Pin());
openlcb::ConfiguredProducer INPUT3_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<3>(), INPUT3_Pin());
openlcb::ConfiguredProducer INPUT4_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<4>(), INPUT4_Pin());
openlcb::ConfiguredProducer INPUT5_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<5>(), INPUT5_Pin());
openlcb::ConfiguredProducer INPUT6_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<6>(), INPUT6_Pin());
openlcb::ConfiguredProducer INPUT7_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<7>(), INPUT7_Pin());
openlcb::ConfiguredProducer INPUT8_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<8>(), INPUT8_Pin());
openlcb::ConfiguredProducer INPUT9_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<9>(), INPUT9_Pin());
openlcb::ConfiguredProducer INPUT10_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<10>(), INPUT10_Pin());
openlcb::ConfiguredProducer INPUT11_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<11>(), INPUT11_Pin());
openlcb::ConfiguredProducer INPUT12_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<12>(), INPUT12_Pin());
openlcb::ConfiguredProducer INPUT13_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<13>(), INPUT13_Pin());
openlcb::ConfiguredProducer INPUT14_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<14>(), INPUT14_Pin());
#ifdef USE_GPIO_18_FOR_IO
openlcb::ConfiguredProducer INPUT15_producer(
    openmrn.stack()->node(), cfg.seg().inputs().entry<15>(), INPUT15_Pin());
#endif // USE_GPIO_18_FOR_IO

// List of GPIO objects that will be used for the output pins. You should keep
// the constexpr declaration, because it will produce a compile error in case
// the list of pointers cannot be compiled into a compiler constant and thus
// would be placed into RAM instead of ROM.
constexpr const Gpio *const output_gpio_set[] =
{
    OUTPUT0_Pin::instance(),  OUTPUT1_Pin::instance(),  OUTPUT2_Pin::instance(),  //
    OUTPUT3_Pin::instance(),  OUTPUT4_Pin::instance(),  OUTPUT5_Pin::instance(),  //
    OUTPUT6_Pin::instance(),  OUTPUT7_Pin::instance(),  OUTPUT8_Pin::instance(),  //
    OUTPUT9_Pin::instance(),  OUTPUT10_Pin::instance(), OUTPUT11_Pin::instance(), //
    OUTPUT12_Pin::instance(), OUTPUT13_Pin::instance(), OUTPUT14_Pin::instance(), //
    OUTPUT15_Pin::instance()
};

openlcb::MultiConfiguredConsumer gpio_consumers(openmrn.stack()->node(),
    output_gpio_set, ARRAYSIZE(output_gpio_set), cfg.seg().outputs());

// Create an initializer that can initialize all the GPIO pins in one shot
typedef GpioInitializer<
    OUTPUT0_Pin,  OUTPUT1_Pin,  OUTPUT2_Pin,  OUTPUT3_Pin,  OUTPUT4_Pin,
    OUTPUT5_Pin,  OUTPUT6_Pin,  OUTPUT7_Pin,  OUTPUT8_Pin,  OUTPUT9_Pin,
    OUTPUT10_Pin, OUTPUT11_Pin, OUTPUT12_Pin, OUTPUT14_Pin, OUTPUT15_Pin,
    INPUT0_Pin,   INPUT1_Pin,   INPUT2_Pin,   INPUT3_Pin,   INPUT4_Pin,
    INPUT5_Pin,   INPUT6_Pin,   INPUT7_Pin,   INPUT8_Pin,   INPUT9_Pin,
    INPUT10_Pin,  INPUT11_Pin,  INPUT12_Pin,  INPUT13_Pin,  INPUT14_Pin,
#ifdef USE_GPIO_18_FOR_IO
  , INPUT15_Pin
#endif // USE_GPIO_18_FOR_IO
    > GpioInit;

// The GPIO pins need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the
// producers to it.
openlcb::RefreshLoop producer_refresh_loop(openmrn.stack()->node(),
    {
        INPUT0_producer.polling(),
        INPUT1_producer.polling(),
        INPUT2_producer.polling(),
        INPUT3_producer.polling(),
        INPUT4_producer.polling(),
        INPUT5_producer.polling(),
        INPUT6_producer.polling(),
        INPUT7_producer.polling(),
        INPUT8_producer.polling(),
        INPUT9_producer.polling(),
        INPUT10_producer.polling(),
        INPUT11_producer.polling(),
        INPUT12_producer.polling(),
        INPUT13_producer.polling(),
        INPUT14_producer.polling()
#ifdef USE_GPIO18_FOR_IO
      , INPUT15_producer.polling()
#endif // USE_GPIO18_FOR_IO
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
