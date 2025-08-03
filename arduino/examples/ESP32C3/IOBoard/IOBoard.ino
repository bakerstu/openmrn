/** \copyright
 * Copyright (c) 2021, Mike Dunston
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
 * \file ESP32C3IOBoard.ino
 *
 * Main file for the io board application on an ESP32-C3.
 *
 * @author Mike Dunston
 * @date 2 May 2021
 */

#include <Arduino.h>
#include <SPIFFS.h>

#include <OpenMRNLite.h>
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/MultiConfiguredConsumer.hxx"
#include "utils/GpioInitializer.hxx"

// Pick an operating mode below, if you select USE_WIFI it will expose this
// node on WIFI. If USE_CAN is enabled the node will be
// available on CAN.
//
// Enabling both options will allow the ESP32 to be accessible from
// both WiFi and CAN interfaces.

#define USE_WIFI
#define USE_CAN

// uncomment the line below to have all packets printed to the Serial
// output. This is not recommended for production deployment.
//#define PRINT_PACKETS

// uncomment the line below to specify a GPIO pin that should be used to force
// a factory reset when the node starts and the GPIO pin reads LOW.
// Note: GPIO 10 is also used for IO9, care must be taken to ensure that this
// GPIO pin is not used both for FACTORY_RESET and an OUTPUT pin.
//#define FACTORY_RESET_GPIO_PIN 10

// Uncomment FIRMWARE_UPDATE_BOOTLOADER to enable the bootloader feature when
// using the CAN device.
//
// NOTE: in order for this to work you *MUST* use a partition schema that has
// two app partitions, typically labeled with "OTA" in the partition name in
// the Arduino IDE.
//#define FIRMWARE_UPDATE_BOOTLOADER

// Configuration option validation

#if defined(FIRMWARE_UPDATE_BOOTLOADER) && !defined(USE_CAN)
#error Firmware update is only supported via CAN, enable USE_CAN to use this.
#endif

#include "config.h"

/// This is the node id to assign to this device, this must be unique
/// on the TWAI bus.
static constexpr uint64_t NODE_ID = UINT64_C(0x05010101182d);

#if defined(USE_WIFI)
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

#endif // USE_WIFI

#if defined(USE_CAN)
// This is the ESP32-C3 pin connected to the SN65HVD23x/MCP2551 R (RX) pin.
// NOTE: Any pin can be used for this other than 11-17 which are connected to
// the onboard flash.
// NOTE: Adjusting this pin assignment will require updating the GPIO_PIN
// declarations below for input/outputs.
// NOTE: USB is connected to GPIO 18 (D-) and 19 (D+) and can not be changed
// to any other pins. When "USB CDC On Boot" is selected in Arduino IDE this
// pin will need to be changed as well as input/output pins changed
// accordingly.
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_18;

// This is the ESP32-C3 pin connected to the SN65HVD23x/MCP2551 D (TX) pin.
// NOTE: Any pin can be used for this other than 11-17 which are connected to
// the onboard flash.
// NOTE: Adjusting this pin assignment will require updating the GPIO_PIN
// declarations below for input/outputs.
// NOTE: USB is connected to GPIO 18 (D-) and 19 (D+) and can not be changed
// to any other pins. When "USB CDC On Boot" is selected in Arduino IDE this
// pin will need to be changed as well as input/output pins changed
// accordingly.
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_19;

#endif // USE_CAN

#if defined(FACTORY_RESET_GPIO_PIN)
static constexpr uint8_t FACTORY_RESET_COUNTDOWN_SECS = 10;
#endif // FACTORY_RESET_GPIO_PIN

/// This is the primary entrypoint for the OpenMRN/LCC stack.
OpenMRN openmrn(NODE_ID);

// ConfigDef comes from config.h and is specific to this particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

#if defined(FIRMWARE_UPDATE_BOOTLOADER)
// Include the Bootloader HAL implementation for the ESP32. This should only
// be included in one ino/cpp file.
#include "freertos_drivers/esp32/Esp32BootloaderHal.hxx"
#endif // FIRMWARE_UPDATE_BOOTLOADER

#if defined(USE_WIFI)
Esp32WiFiManager wifi_mgr(ssid, password, openmrn.stack(), cfg.seg().wifi());
#endif // USE_WIFI

#if defined(USE_CAN)
Esp32Can can_driver(CAN_TX_PIN, CAN_RX_PIN);
#endif // USE_CAN

// Declare output pins.
GPIO_PIN(IO0, GpioOutputSafeLow, 0);
GPIO_PIN(IO1, GpioOutputSafeLow, 1);
GPIO_PIN(IO2, GpioOutputSafeLow, 2);
GPIO_PIN(IO3, GpioOutputSafeLow, 3);
GPIO_PIN(IO4, GpioOutputSafeLow, 4);

// Declare input pins.
GPIO_PIN(IO5, GpioInputPU, 5);
GPIO_PIN(IO6, GpioInputPU, 6);
GPIO_PIN(IO7, GpioInputPU, 7);
GPIO_PIN(IO8, GpioInputPU, 9);
GPIO_PIN(IO9, GpioInputPU, 10);

#if defined(FACTORY_RESET_GPIO_PIN)
GPIO_PIN(FACTORY_RESET, GpioInputPU, FACTORY_RESET_GPIO_PIN);
#endif // FACTORY_RESET_GPIO_PIN

// List of GPIO objects that will be used for the output pins. You should keep
// the constexpr declaration, because it will produce a compile error in case
// the list of pointers cannot be compiled into a compiler constant and thus
// would be placed into RAM instead of ROM.
constexpr const Gpio *const outputGpioSet[] = {
    IO0_Pin::instance(), IO1_Pin::instance(), //
    IO2_Pin::instance(), IO3_Pin::instance(), //
    IO4_Pin::instance()
};

openlcb::MultiConfiguredConsumer gpio_consumers(openmrn.stack()->node(), outputGpioSet,
    ARRAYSIZE(outputGpioSet), cfg.seg().consumers());

openlcb::ConfiguredProducer IO5_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<0>(), IO5_Pin());
openlcb::ConfiguredProducer IO6_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<1>(), IO6_Pin());
openlcb::ConfiguredProducer IO7_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<2>(), IO7_Pin());
openlcb::ConfiguredProducer IO8_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<3>(), IO8_Pin());
openlcb::ConfiguredProducer IO9_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<4>(), IO9_Pin());

// Create an initializer that can initialize all the GPIO pins in one shot
typedef GpioInitializer<
#if defined(FACTORY_RESET_GPIO_PIN)
    FACTORY_RESET_Pin,                           // factory reset
#endif // FACTORY_RESET_GPIO_PIN
    IO0_Pin, IO1_Pin, IO2_Pin, IO3_Pin, IO4_Pin, // output pins
    IO5_Pin, IO6_Pin, IO7_Pin, IO8_Pin, IO9_Pin  // input pins
    > GpioInit;

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the
// producers to it.
openlcb::RefreshLoop producer_refresh_loop(openmrn.stack()->node(),
    {
        IO5_producer.polling(),
        IO6_producer.polling(),
        IO7_producer.polling(),
        IO8_producer.polling(),
        IO9_producer.polling()
    }
);

// This will perform the factory reset procedure for this node's configuration
// items.
//
// The node name and description will be set to the SNIP model name field
// value.
// Descriptions for intputs and outputs will be set to a blank string, input
// debounce parameters will be set to default values.
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
            fd, openlcb::SNIP_STATIC_DATA.model_name);
        for(int i = 0; i < openlcb::NUM_OUTPUTS; i++)
        {
            cfg.seg().consumers().entry(i).description().write(fd, "");
        }
        for(int i = 0; i < openlcb::NUM_INPUTS; i++)
        {
            cfg.seg().producers().entry(i).description().write(fd, "");
            CDI_FACTORY_RESET(cfg.seg().producers().entry(i).debounce);
        }
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

void check_for_factory_reset()
{
#if defined(FACTORY_RESET_GPIO_PIN)
    // Check the factory reset pin which should normally read HIGH (set), if it
    // reads LOW (clr) delete the cdi.xml and openlcb_config
    if (!FACTORY_RESET_Pin::instance()->read())
    {
        LOG(WARNING, "!!!! WARNING WARNING WARNING WARNING WARNING !!!!");
        LOG(WARNING, "The factory reset GPIO pin %d has been triggered.",
            FACTORY_RESET_GPIO_PIN);
        for (uint8_t sec = FACTORY_RESET_COUNTDOWN_SECS;
             sec > 0 && !FACTORY_RESET_Pin::instance()->read(); sec--)
        {
            LOG(WARNING, "Factory reset will be initiated in %d seconds.",
                sec);
            usleep(SEC_TO_USEC(1));
        }
        if (!FACTORY_RESET_Pin::instance()->read())
        {
            unlink(openlcb::CDI_FILENAME);
            unlink(openlcb::CONFIG_FILENAME);
            LOG(WARNING, "Factory reset complete");
        }
        else
        {
            LOG(WARNING, "Factory reset aborted as pin %d was not held LOW",
                FACTORY_RESET_GPIO_PIN);
        }
    }
#endif // FACTORY_RESET_GPIO_PIN
}

void setup()
{
    Serial.begin(115200L);
    uint8_t reset_reason = Esp32SocInfo::print_soc_info();
    LOG(INFO, "[Node] ID: %s", uint64_to_string_hex(NODE_ID).c_str());
    LOG(INFO, "[SNIP] version:%d, manufacturer:%s, model:%s, hw-v:%s, sw-v:%s",
        openlcb::SNIP_STATIC_DATA.version,
        openlcb::SNIP_STATIC_DATA.manufacturer_name,
        openlcb::SNIP_STATIC_DATA.model_name,
        openlcb::SNIP_STATIC_DATA.hardware_version,
        openlcb::SNIP_STATIC_DATA.software_version);

    // Initialize the SPIFFS filesystem as our persistence layer
    if (!SPIFFS.begin())
    {
        LOG(WARNING, "SPIFFS failed to mount, attempting to format and remount");
        if (!SPIFFS.begin(true))
        {
            LOG_ERROR("SPIFFS mount failed even with format, giving up!");
            while (1)
            {
                // Unable to start SPIFFS successfully, give up and wait
                // for WDT to kick in
            }
        }
    }

    // initialize all declared GPIO pins
    GpioInit::hw_init();

#if defined(FIRMWARE_UPDATE_BOOTLOADER)
    // initialize the bootloader.
    esp32_bootloader_init(reset_reason);

    // if we have a request to enter the bootloader we need to process it
    // before we startup the OpenMRN stack.
    if (request_bootloader())
    {
        esp32_bootloader_run(NODE_ID, CAN_RX_PIN, CAN_TX_PIN);
        // This line should not be reached as the esp32_bootloader_run method
        // will not return by default.
        HASSERT(false);
    }
#endif // FIRMWARE_UPDATE_BOOTLOADER

    check_for_factory_reset();

    // Create the CDI.xml dynamically
    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

    // Create the default internal configuration file
    openmrn.stack()->create_config_file_if_needed(cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

#if defined(USE_CAN)
    can_driver.begin();
    openmrn.add_can_port(&can_driver);
#endif // USE_CAN

    // Start the OpenMRN stack
    openmrn.begin();

    if (reset_reason == RTCWDT_BROWN_OUT_RESET)
    {
        openmrn.stack()->executor()->add(new CallbackExecutable([]()
        {
            openmrn.stack()->send_event(openlcb::Defs::NODE_POWER_BROWNOUT_EVENT);
        }));
    }

#if defined(PRINT_PACKETS)
    // Dump all packets as they are sent/received.
    // Note: This should not be enabled in deployed nodes as it will have
    // performance impact.
    openmrn.stack()->print_all_packets();
#endif // PRINT_PACKETS

    // start executor thread since this is required for select() to work in the
    // OpenMRN executor.
    openmrn.start_executor_thread();
}

void loop()
{
    // Call the OpenMRN executor, this needs to be done as often
    // as possible from the loop() method.
    openmrn.loop();
}

#if defined(FIRMWARE_UPDATE_BOOTLOADER)

extern "C"
{

/// Updates the state of a status LED.
///
/// @param led is the LED to update.
/// @param value is the new state of the LED.
void bootloader_led(enum BootloaderLed led, bool value)
{
    LOG(INFO, "[Bootloader] bootloader_led(%d, %d)", led, value);
}

} // extern "C"

#endif // FIRMWARE_UPDATE_BOOTLOADER