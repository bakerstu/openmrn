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
 * \file ESP32SerialBridge.ino
 * 
 * Example application for the ESP32 showing how to configure a Serial bridge
 * GridConnect client adapter to the OpenMRN stack.
 *
 * @author Mike Dunston
 * @date 13 January 2019
 */

#include <Arduino.h>
#include <OpenMRN.h>
#include <SPIFFS.h>

constexpr uint32_t  SERIAL_BAUD     = 115200L;
constexpr uint8_t   SERIAL_RX_PIN   = 16;
constexpr uint8_t   SERIAL_TX_PIN   = 17;

static constexpr uint64_t NODE_ID = UINT64_C(0x050101011423);
OpenMRN openmrn(NODE_ID);

namespace openlcb
{
    const char *const CONFIG_FILENAME = "/spiffs/openlcb_config";
    const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
} // namespace openlcb

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial1.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
    SPIFFS.begin(true);
    openmrn.begin();

    printf("\nSerial(rx:%d, tx:%d, speed:%d) is ready to exchange grid connect packets.\n",
        SERIAL_TX_PIN, SERIAL_TX_PIN, SERIAL_BAUD);
    openmrn.add_gridconnect_port(&Serial1);
    openmrn.stack()->print_all_packets();
    openmrn.start_background_task();
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(50));
}