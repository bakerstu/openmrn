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
#include <WiFi.h>
#include <ESPmDNS.h>
#include <vector>

#include <OpenMRN.h>
#include <openlcb/TcpDefs.hxx>
#include "config.h"

constexpr uint16_t OPENMRN_TCP_PORT = 12021L;

WiFiServer openMRNServer(OPENMRN_TCP_PORT);

// Configuring WiFi accesspoint name and password
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// There are two options:
// 1) edit the sketch to set this information just below. Use quotes:
//     const char* ssid     = "linksys";
//     const char* password = "superSecret";
// 2) add a new file to the sketch folder called something.cpp with the following contents:
//     #include <OpenMRN.h>
//
//     const char DEFAULT_WIFI_NAME[] = "linksys";
//     const char DEFAULT_PASSWORD[] = "theTRUEsupers3cr3t";

/// This is the name of the WiFi network (access point) to connect to.
const char* ssid     = DEFAULT_WIFI_NAME;
/// Password of the wifi network.
const char* password = DEFAULT_PASSWORD;
const char* hostname = "esp32mrn";

static constexpr uint64_t NODE_ID = UINT64_C(0x050101011423);
OpenMRN openmrn(NODE_ID);

string somewhere("abcdef");
// ConfigDef comes from config.hxx and is specific to the particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

// Declare output pins
GPIO_PIN(IO0, GpioOutputSafeLow, 2);
GPIO_PIN(IO1, GpioOutputSafeLow, 4);
GPIO_PIN(IO2, GpioOutputSafeLow, 5);
GPIO_PIN(IO3, GpioOutputSafeLow, 16);
GPIO_PIN(IO4, GpioOutputSafeLow, 17);
GPIO_PIN(IO5, GpioOutputSafeLow, 18);
GPIO_PIN(IO6, GpioOutputSafeLow, 19);
GPIO_PIN(IO7, GpioOutputSafeLow, 23);

// Declare input pins, these are using analog pins as digital inputs
// NOTE: pins 25 and 26 can not safely be used as analog pins while
// WiFi is active.
GPIO_PIN(IO8, GpioInputPU, 32);
GPIO_PIN(IO9, GpioInputPU, 33);
GPIO_PIN(IO10, GpioInputPU, 34);
GPIO_PIN(IO11, GpioInputPU, 35);
GPIO_PIN(IO12, GpioInputPU, 36);
GPIO_PIN(IO13, GpioInputPU, 39);
GPIO_PIN(IO14, GpioInputPU, 25);
GPIO_PIN(IO15, GpioInputPU, 26);

openlcb::ConfiguredConsumer IO0_consumer(
    openmrn.stack()->node(), cfg.seg().consumers().entry<0>(), IO0_Pin());
openlcb::ConfiguredConsumer IO1_consumer(
    openmrn.stack()->node(), cfg.seg().consumers().entry<1>(), IO1_Pin());
openlcb::ConfiguredConsumer IO2_consumer(
    openmrn.stack()->node(), cfg.seg().consumers().entry<2>(), IO2_Pin());
openlcb::ConfiguredConsumer IO3_consumer(
    openmrn.stack()->node(), cfg.seg().consumers().entry<3>(), IO3_Pin());
openlcb::ConfiguredConsumer IO4_consumer(
    openmrn.stack()->node(), cfg.seg().consumers().entry<4>(), IO4_Pin());
openlcb::ConfiguredConsumer IO5_consumer(
    openmrn.stack()->node(), cfg.seg().consumers().entry<5>(), IO5_Pin());
openlcb::ConfiguredConsumer IO6_consumer(
    openmrn.stack()->node(), cfg.seg().consumers().entry<6>(), IO6_Pin());
openlcb::ConfiguredConsumer IO7_consumer(
    openmrn.stack()->node(), cfg.seg().consumers().entry<7>(), IO7_Pin());

openlcb::ConfiguredProducer IO8_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<0>(), IO8_Pin());
openlcb::ConfiguredProducer IO9_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<1>(), IO9_Pin());
openlcb::ConfiguredProducer IO10_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<2>(), IO10_Pin());
openlcb::ConfiguredProducer IO11_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<3>(), IO11_Pin());
openlcb::ConfiguredProducer IO12_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<4>(), IO12_Pin());
openlcb::ConfiguredProducer IO13_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<5>(), IO13_Pin());
openlcb::ConfiguredProducer IO14_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<6>(), IO14_Pin());
openlcb::ConfiguredProducer IO15_producer(
    openmrn.stack()->node(), cfg.seg().producers().entry<7>(), IO15_Pin());

void setup() {
    Serial.begin(115200L);

    printf("\nConnecting to: %s\n", ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // This makes the wifi much more responsive. Since we are plugged in we don't care about the increased power usage. Disable when on battery.
    WiFi.setSleep(false);

    printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());

    openmrn.stack()->print_all_packets();
    //openmrn.start_background_task();

    openMRNServer.setNoDelay(true);
    openMRNServer.begin();
    MDNS.begin(hostname);
    MDNS.addService(openlcb::TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN,
        openlcb::TcpDefs::MDNS_PROTOCOL_TCP, OPENMRN_TCP_PORT);
}

void loop() {
    if(openMRNServer.hasClient()) {
        WiFiClient client = openMRNServer.available();
        if(client) {
            openmrn.add_gridconnect_port(new Esp32WiFiClientAdapter(client));
        }
    }
    openmrn.loop();
}
