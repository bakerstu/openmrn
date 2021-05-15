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
 * \file ESP32IOBoard.ino
 *
 * Main file for the io board application on an ESP32.
 *
 * @author Mike Dunston
 * @date 13 January 2019
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include <vector>
#include <esp_spi_flash.h>

#include <OpenMRNLite.h>
#include <openlcb/TcpDefs.hxx>

#include <openlcb/MultiConfiguredConsumer.hxx>
#include <utils/GpioInitializer.hxx>
#include <freertos_drivers/arduino/CpuLoad.hxx>

// Pick an operating mode below, if you select USE_WIFI it will expose
// this node on WIFI if you select USE_TWAI, this node will be available
// on CAN.
// Enabling both options will allow the ESP32 to be accessible from
// both WiFi and CAN interfaces.

#define USE_WIFI
//#define USE_TWAI

// uncomment the line below to have all packets printed to the Serial
// output. This is not recommended for production deployment.
//#define PRINT_PACKETS

#include "config.h"

/// This is the node id to assign to this device, this must be unique
/// on the CAN bus.
static constexpr uint64_t NODE_ID = UINT64_C(0x05010101182c);

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

OVERRIDE_CONST(gridconnect_buffer_size, 3512);
//OVERRIDE_CONST(gridconnect_buffer_delay_usec, 200000);
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 2000);
OVERRIDE_CONST(gc_generate_newlines, CONSTANT_TRUE);
OVERRIDE_CONST(executor_select_prescaler, 60);
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, 2);

#endif // USE_WIFI

#if defined(USE_TWAI)
// This is the ESP32-C3 pin connected to the SN65HVD23x/MCP2551 R (RX) pin.
// Note: Any pin can be used for this other than 11-17 which are connected to
// the onboard flash.
// Note: Adjusting this pin assignment will require updating the GPIO_PIN
// declarations below for input/outputs.
constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_18;

// This is the ESP32-C3 pin connected to the SN65HVD23x/MCP2551 D (TX) pin.
// Note: Any pin can be used for this other than 11-17 which are connected to
// the onboard flash.
// Note: Adjusting this pin assignment will require updating the GPIO_PIN
// declarations below for input/outputs.
constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_19;

#endif // USE_TWAI

// This is the primary entrypoint for the OpenMRN/LCC stack.
OpenMRN openmrn(NODE_ID);

// This tracks the CPU usage of the ESP32-C3 through the usage of a hardware
// timer that records what the CPU is currently executing roughly 163 times per
// second.
CpuLoad cpu_load;

// This will report the usage to the console output.
CpuLoadLog cpu_log(openmrn.stack()->service());

// ConfigDef comes from config.h and is specific to this particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

#if defined(USE_WIFI)
Esp32WiFiManager wifi_mgr(ssid, password, openmrn.stack(), cfg.seg().wifi());
#endif // USE_WIFI

#if defined(USE_TWAI)
Esp32HardwareTwai twai(TWAI_RX_PIN, TWAI_TX_PIN);
#endif // USE_TWAI

// Declare output pins
// NOTE: pins 11-17 are connected to the onboard flash and can not be used for
// any purpose.
GPIO_PIN(IO0, GpioOutputSafeLow, 0);
GPIO_PIN(IO1, GpioOutputSafeLow, 1);
GPIO_PIN(IO2, GpioOutputSafeLow, 2);
GPIO_PIN(IO3, GpioOutputSafeLow, 3);
GPIO_PIN(IO4, GpioOutputSafeLow, 4);

// Declare input pins.
// GPIO 8 intentionally skipped due to WS2812 LED on this pin on the
// ESP32-DevKitM-1 board.
GPIO_PIN(IO5, GpioInputPU, 5);
GPIO_PIN(IO6, GpioInputPU, 6);
GPIO_PIN(IO7, GpioInputPU, 7);
GPIO_PIN(IO8, GpioInputPU, 9);
GPIO_PIN(IO9, GpioInputPU, 10);

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

// Callback function for the hardware timer configured to fire roughly 163
// times per second.
void ARDUINO_ISR_ATTR record_cpu_usage()
{
#if CONFIG_ARDUINO_ISR_IRAM
    // if the ISR is called with flash disabled we can not safely recored the
    // cpu usage.
    if (!spi_flash_cache_enabled())
    {
        return;
    }
#endif
    // Retrieves the vtable pointer from the currently running executable.
    unsigned *pp = (unsigned *)openmrn.stack()->executor()->current();
    cpuload_tick(pp ? pp[0] | 1 : 0);
}

void setup()
{
    Serial.begin(115200L);

    Esp32SocInfo::print_soc_info();

    // Register hardware timer zero to use a 1Mhz resolution and to count up
    // from zero when the timer triggers.
    auto timer = timerBegin(0, 80, true);
    // Attach our callback function to be called when the timer is ready to
    // fire. Note that the edge parameter is not used/supported on the
    // ESP32-C3.
    timerAttachInterrupt(timer, record_cpu_usage, true);
    // Configure the trigger point to be roughly 163 times per second.
    timerAlarmWrite(timer, 1000000/163, true);
    // Enable the timer.
    timerAlarmEnable(timer);

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

#if defined(USE_TWAI)
    twai.hw_init();
#endif // USE_TWAI

    // Start the OpenMRN stack
    openmrn.begin();

#if defined(PRINT_PACKETS)
    // Dump all packets as they are sent/received.
    // Note: This should not be enabled in deployed nodes as it will
    // have performance impact.
    openmrn.stack()->print_all_packets();
#endif // PRINT_PACKETS

#if defined(USE_TWAI)
    openmrn.add_can_port_async("/dev/twai/twai0");
#endif // USE_TWAI
}

void loop()
{
    // Call the OpenMRN executor, this needs to be done as often
    // as possible from the loop() method.
    openmrn.loop();
}
