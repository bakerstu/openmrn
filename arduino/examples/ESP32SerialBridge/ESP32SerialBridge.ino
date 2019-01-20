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
#include "config.h"

constexpr uint32_t  SERIAL_BAUD     = 115200L;
constexpr uint8_t   SERIAL_RX_PIN   = 16;
constexpr uint8_t   SERIAL_TX_PIN   = 17;

static constexpr uint64_t NODE_ID = UINT64_C(0x050101011423);

OpenMRN openmrn(NODE_ID);

// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

// ConfigDef comes from config.hxx and is specific to the particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr openlcb::ConfigDef cfg(0);

namespace openlcb {
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

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial1.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

    // Initialize the SPIFFS filesystem as our persistence layer
    if(!SPIFFS.begin())
    {
        printf("SPIFFS failed to mount, attempting to format and remount\n");
        if(!SPIFFS.begin(true))
        {
            printf("SPIFFS mount failed even with format, giving up!\n");
            while(1)
            {
                // Unable to start SPIFFS successfully, give up and wait
                // for WDT to kick in
            }
        }
    }

    printf("\nSerial(rx:%d, tx:%d, speed:%d) is ready to exchange grid connect packets.\n",
        SERIAL_TX_PIN, SERIAL_TX_PIN, SERIAL_BAUD);

    // Create the CDI.xml dynamically
    openmrn.create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

    // Create the default internal configuration file
    openmrn.stack()->create_config_file_if_needed(cfg.seg().internal_config(),
        openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    // Start the OpenMRN stack
    openmrn.begin();

    // Add Serial1 as a bridge
    openmrn.add_gridconnect_port(new Esp32HardwareSerialAdapter(Serial1));
}

void loop() {
    openmrn.loop();
}