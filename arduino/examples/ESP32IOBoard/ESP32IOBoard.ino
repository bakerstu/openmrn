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

#if !defined(LED_BUILTIN)
//constexpr uint8_t LED_BUILTIN = 13;
#endif

GPIO_PIN(LED, GpioOutputSafeLow, LED_BUILTIN);

//openlcb::ConfiguredConsumer onboard_led(
//    openmrn.stack()->node(), cfg.seg().consumers().entry<0>(), LED_Pin());

class WiFiClientAdapter {
public:
    WiFiClientAdapter(WiFiClient client) : client_(client){
        client_.setNoDelay(true);
        printf("[%s] OpenMRN GridConnect client connected.\n",
            client_.remoteIP().toString().c_str());
    }
    // on the ESP32 there is no TX limit method
    size_t availableForWrite() {
        return client_.connected();
    }
    size_t write(const char *buffer, size_t len) {
        if(client_.connected()) {
            return client_.write(buffer, len);  
        }
        return 0;
    }
    size_t available() {
        if(client_.connected()) {
            return client_.available();
        }
        return 0;
    }
    size_t read(const char *buffer, size_t len) {
        size_t bytesRead = 0;
        if(client_.connected()) {
            bytesRead = client_.read((uint8_t *)buffer, len);
        }
        return bytesRead;
    }
private:
    WiFiClient client_;
};

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
            openmrn.add_gridconnect_port(new WiFiClientAdapter(client));
        }
    }
    openmrn.loop();
    //vTaskDelay(pdMS_TO_TICKS(50));
}
