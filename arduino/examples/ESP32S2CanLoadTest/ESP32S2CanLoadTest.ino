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
 * \file ESP32S2CanLoadTest.ino
 *
  * Main file for the ESP32-S2 CAN Load Test application.
 *
 * @author Mike Dunston
 * @date 2 May 2021
 */

#include <Arduino.h>
#include <SPIFFS.h>
#include <esp_spi_flash.h>
#include <USB.h>

#include <OpenMRNLite.h>
#include <openlcb/TcpDefs.hxx>

#include <openlcb/MultiConfiguredConsumer.hxx>
#include <utils/GpioInitializer.hxx>
#include <freertos_drivers/arduino/CpuLoad.hxx>

// Pick an operating mode below, if you select USE_WIFI it will expose
// this node on WIFI if you select USE_TWAI this node will be available on CAN.
// Enabling both options will allow the ESP32 to be accessible from
// both WiFi and CAN interfaces.

#define USE_WIFI
//#define USE_TWAI

// Uncomment USE_TWAI_SELECT to enable the usage of select() for the TWAI
// interface.
//#define USE_TWAI_SELECT

// uncomment the line below to have all packets printed to the Serial
// output. This is not recommended for production deployment.
//#define PRINT_PACKETS

// Uncomment the line below to configure the native USB CDC for all output from
// OpenMRNLite. When not defined the default UART0 will be used instead.
//#define USE_USB_CDC_OUTPUT

// Configuration option validation

// If USE_TWAI_SELECT or USE_TWAI_ASYNC is enabled but USE_TWAI is not, enable
// USE_TWAI.
#if defined(USE_TWAI_SELECT) && !defined(USE_TWAI)
#define USE_TWAI
#endif // USE_TWAI_SELECT && !USE_TWAI

#include "config.h"

/// This is the node id to assign to this device, this must be unique
/// on the CAN bus.
static constexpr uint64_t NODE_ID = UINT64_C(0x05010101182e);

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

// This is the name of the WiFi network (access point) to connect to.
const char *ssid = WIFI_SSID;

// Password of the wifi network.
const char *password = WIFI_PASS;

// This is the hostname which the ESP32 will advertise via mDNS, it should be
// unique.
const char *hostname = "esp32mrn";

OVERRIDE_CONST(gridconnect_buffer_size, 3512);
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 2000);
OVERRIDE_CONST(gc_generate_newlines, CONSTANT_TRUE);
OVERRIDE_CONST(executor_select_prescaler, 60);
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, 2);

#endif // USE_WIFI

#if defined(USE_TWAI)
// This is the ESP32-S2 pin connected to the SN65HVD23x/MCP2551 R (RX) pin.
// Recommended pins: 40, 41.
// Note: Any pin can be used for this other than 26-32 which are connected to
// the onboard flash.
// Note: If you are using a pin other than 40 you will likely need to adjust
// the GPIO pin definitions for the outputs.
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_40;

// This is the ESP32 pin connected to the SN65HVD23x/MCP2551 D (TX) pin.
// Recommended pins: 40, 41.
// Note: Any pin can be used for this other than 26-32 which are connected to
// the onboard flash.
// Note: If you are using a pin other than 41 you will likely need to adjust
// the GPIO pin definitions for the outputs.
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_41;

#endif // USE_TWAI

// This is the primary entrypoint for the OpenMRN/LCC stack.
OpenMRN openmrn(NODE_ID);

// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

// This tracks the CPU usage of the ESP32-S2 through the usage of a hardware
// timer that records what the CPU is currently executing roughly 163 times per
// second.
CpuLoad cpu_load;

// This reports the usage to the console output.
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
Esp32HardwareTwai twai(CAN_RX_PIN, CAN_TX_PIN);
#endif // USE_TWAI

// Declare output pins.
GPIO_PIN(IO0, GpioOutputSafeLow, 0);
GPIO_PIN(IO1, GpioOutputSafeLow, 1);
GPIO_PIN(IO2, GpioOutputSafeLow, 2);
GPIO_PIN(IO3, GpioOutputSafeLow, 3);
GPIO_PIN(IO4, GpioOutputSafeLow, 4);
GPIO_PIN(IO5, GpioOutputSafeLow, 5);
GPIO_PIN(IO6, GpioOutputSafeLow, 6);
GPIO_PIN(IO7, GpioOutputSafeLow, 7);
GPIO_PIN(IO8, GpioOutputSafeLow, 8);
GPIO_PIN(IO9, GpioOutputSafeLow, 9);
GPIO_PIN(IO10, GpioOutputSafeLow, 10);
GPIO_PIN(IO11, GpioOutputSafeLow, 11);
GPIO_PIN(IO12, GpioOutputSafeLow, 12);
GPIO_PIN(IO13, GpioOutputSafeLow, 45);

// Declare input pins
// Notes:
// GPIO 18 is reserved for the status LED.
// GPIO 19 and 20 are intentionally skipped as they are reserved for native USB.
// GPIO 43 and 44 are skipped as they are connected to UART0.
// GPIO 40 and 41 are intentionally skipped as they are reserved for TWAI.
GPIO_PIN(IO14, GpioInputPU, 13);
GPIO_PIN(IO15, GpioInputPU, 14);
GPIO_PIN(IO16, GpioInputPU, 15);
GPIO_PIN(IO17, GpioInputPU, 16);
GPIO_PIN(IO18, GpioInputPU, 17);
GPIO_PIN(IO19, GpioInputPU, 21);
GPIO_PIN(IO20, GpioInputPU, 33);
GPIO_PIN(IO21, GpioInputPU, 34);
GPIO_PIN(IO22, GpioInputPU, 35);
GPIO_PIN(IO23, GpioInputPU, 36);
GPIO_PIN(IO24, GpioInputPU, 37);
GPIO_PIN(IO25, GpioInputPU, 38);
GPIO_PIN(IO26, GpioInputPU, 39);
GPIO_PIN(IO27, GpioInputPU, 42);

// List of GPIO objects that will be used for the output pins. You should keep
// the constexpr declaration, because it will produce a compile error in case
// the list of pointers cannot be compiled into a compiler constant and thus
// would be placed into RAM instead of ROM.
constexpr const Gpio *const outputGpioSet[] = {
    IO0_Pin::instance(),  IO1_Pin::instance(),  //
    IO2_Pin::instance(),  IO3_Pin::instance(),  //
    IO4_Pin::instance(),  IO5_Pin::instance(),  //
    IO6_Pin::instance(),  IO7_Pin::instance(),  //
    IO8_Pin::instance(),  IO9_Pin::instance(),  //
    IO10_Pin::instance(), IO11_Pin::instance(), //
    IO12_Pin::instance(), IO13_Pin::instance(), //
};

openlcb::MultiConfiguredConsumer gpio_consumers(openmrn.stack()->node(), outputGpioSet,
    ARRAYSIZE(outputGpioSet), cfg.seg().consumers());

openlcb::ConfiguredProducer IO14_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<0>(), IO14_Pin());
openlcb::ConfiguredProducer IO15_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<1>(), IO15_Pin());
openlcb::ConfiguredProducer IO16_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<2>(), IO16_Pin());
openlcb::ConfiguredProducer IO17_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<3>(), IO17_Pin());
openlcb::ConfiguredProducer IO18_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<4>(), IO18_Pin());
openlcb::ConfiguredProducer IO19_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<5>(), IO19_Pin());
openlcb::ConfiguredProducer IO20_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<6>(), IO20_Pin());
openlcb::ConfiguredProducer IO21_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<7>(), IO21_Pin());
openlcb::ConfiguredProducer IO22_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<8>(), IO22_Pin());
openlcb::ConfiguredProducer IO23_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<9>(), IO23_Pin());
openlcb::ConfiguredProducer IO24_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<10>(), IO24_Pin());
openlcb::ConfiguredProducer IO25_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<11>(), IO25_Pin());
openlcb::ConfiguredProducer IO26_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<12>(), IO26_Pin());
openlcb::ConfiguredProducer IO27_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<13>(), IO27_Pin());

// Create an initializer that can initialize all the GPIO pins in one shot
typedef GpioInitializer<
    IO0_Pin,  IO1_Pin,  IO2_Pin,  IO3_Pin,  // outputs 0-3
    IO4_Pin,  IO5_Pin,  IO6_Pin,  IO7_Pin,  // outputs 4-7
    IO8_Pin,  IO9_Pin,  IO10_Pin, IO11_Pin, // outputs 8-11
    IO12_Pin, IO13_Pin,                     // outputs 12-13
    IO14_Pin, IO15_Pin, IO16_Pin, IO17_Pin, // inputs 0-3
    IO18_Pin, IO19_Pin, IO20_Pin, IO21_Pin, // inputs 4-7
    IO22_Pin, IO23_Pin, IO24_Pin, IO25_Pin, // inputs 8-11
    IO26_Pin, IO27_Pin                      // inputs 12-13
    > GpioInit;

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the
// producers to it.
openlcb::RefreshLoop producer_refresh_loop(openmrn.stack()->node(),
    {
        IO14_producer.polling(),
        IO15_producer.polling(),
        IO16_producer.polling(),
        IO17_producer.polling(),
        IO18_producer.polling(),
        IO19_producer.polling(),
        IO20_producer.polling(),
        IO21_producer.polling(),
        IO22_producer.polling(),
        IO23_producer.polling(),
        IO24_producer.polling(),
        IO25_producer.polling(),
        IO26_producer.polling(),
        IO27_producer.polling()
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

#if defined(USE_USB_CDC_OUTPUT)
// USB CDC wrapper that uses TinyUSB internally to route data to/from the USB
// CDC device driver.
USBCDC USBSerial;

// Override the log_output method from OpenMRNLite to redirect all log output
// via USB CDC.
void log_output(char* buf, int size)
{
    if (size <= 0) return;
    buf[size] = '\0';
    USBSerial.println(buf);
}
#endif // USE_USB_CDC_OUTPUT

void setup()
{
#if !defined(USE_USB_CDC_OUTPUT)
    Serial.begin(115200L);
#else
    USB.productName(openlcb::SNIP_STATIC_DATA.model_name);
    USB.manufacturerName(openlcb::SNIP_STATIC_DATA.manufacturer_name);
    USB.firmwareVersion(openlcb::CANONICAL_VERSION);
    USB.serialNumber(uint64_to_string_hex(NODE_ID).c_str());
    USB.begin();
    USBSerial.begin();
    USBSerial.setDebugOutput(true);
    // Give time for the USB peripheral to startup and be ready to use.
    delay(5000);
#endif // USE_USB_CDC_OUTPUT

    Esp32SocInfo::print_soc_info();
    LOG(INFO, "[Node] ID: %s", uint64_to_string_hex(NODE_ID).c_str());
    LOG(INFO, "[SNIP] version:%d, manufacturer:%s, model:%s, hw-v:%s, sw-v:%s"
      , openlcb::SNIP_STATIC_DATA.version
      , openlcb::SNIP_STATIC_DATA.manufacturer_name
      , openlcb::SNIP_STATIC_DATA.model_name
      , openlcb::SNIP_STATIC_DATA.hardware_version
      , openlcb::SNIP_STATIC_DATA.software_version);

    // Register hardware timer zero to use a 1Mhz resolution and to count up
    // from zero when the timer triggers.
    auto timer = timerBegin(0, 80, true);
    // Attach our callback function to be called on the timer edge signal.
    timerAttachInterrupt(timer, record_cpu_usage, true);
    // Configure the trigger point to be roughly 163 times per second.
    timerAlarmWrite(timer, 1000000/163, true);
    // Enable the timer.
    timerAlarmEnable(timer);

    // Initialize the SPIFFS filesystem as our persistence layer
    if (!SPIFFS.begin())
    {
        LOG(WARNING,
            "SPIFFS failed to mount, attempting to format and remount");
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

#if defined(USE_TWAI_SELECT)
    // add TWAI driver with select() usage
    openmrn.add_can_port_select("/dev/twai/twai0");

    // start executor thread since this is required for select() to work in the
    // OpenMRN executor.
    openmrn.start_executor_thread();
#else
    // add TWAI driver with non-blocking usage
    openmrn.add_can_port_async("/dev/twai/twai0");
#endif // USE_TWAI_SELECT
}

void loop()
{
    // Call the OpenMRN executor, this needs to be done as often
    // as possible from the loop() method.
    openmrn.loop();
}
