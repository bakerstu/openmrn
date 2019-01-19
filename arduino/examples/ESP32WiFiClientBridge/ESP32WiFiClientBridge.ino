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
 * \file ESP32WiFiClientBridge.ino
 * 
 * Example application for the ESP32 showing how to configure a TCP/IP GridConnect
 * client adapter to the OpenMRN stack.
 *
 * @author Mike Dunston
 * @date 13 January 2019
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <vector>
#include <SPIFFS.h>

#include <OpenMRN.h>
#include <openlcb/TcpDefs.hxx>

constexpr uint16_t OPENMRN_TCP_PORT = 12021L;

WiFiServer openMRNServer(OPENMRN_TCP_PORT);

const char* ssid     = "apname";
const char* password = "password";
const char* hostname = "esp32mrn";

static constexpr uint64_t NODE_ID = UINT64_C(0x050101011423);
OpenMRN openmrn(NODE_ID);

namespace openlcb
{
    const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";
    const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
} // namespace openlcb

void setup() {
    Serial.begin(115200L);

    SPIFFS.begin(true);
    openmrn.begin();

    printf("\nConnecting to: %s\n", ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());

    openmrn.stack()->print_all_packets();
    openmrn.start_background_task();

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
}