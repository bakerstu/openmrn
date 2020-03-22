/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file Stm32CanSerial.ino
 * 
 * Example Arduino sketch for the Stm32 showing how to make a Serial-CAN
 * adapter with an OpenLCB node.
 *
 * @author Balazs Racz
 * @date 22 March 2020
 */

#include <Arduino.h>
#include <OpenMRNLite.h>

/// Which serial port to use.
#define SERIAL_PORT Serial

/// Specify how fast the serial port should be going. In order not to lose
/// packets on a fully loaded CAN-bus, this has to be at least 460800, but the
/// default is set to 115200 for better compatibility.
#define SERIAL_BAUD_RATE 115200

/// Specifies which Stm32 Pin should be used for CAN_TX signal. The default
/// matches the pinout of the OpenLCB Dev Kit daughterboard.
#define CAN_TX_PIN PB_9

/// Specifies which Stm32 Pin should be used for CAN_RX signal. The default
/// matches the pinout of the OpenLCB Dev Kit daughterboard.
#define CAN_RX_PIN PB_8

/// This is the OpenLCB Node ID. It must be coming from the Node ID range
/// assigned to the developer (get a range assigned to you via openlcb.org).
static constexpr uint64_t NODE_ID = UINT64_C(0x050101011824);

Stm32Can Can("/dev/can0");
OpenMRN openmrn(NODE_ID);

OVERRIDE_CONST_TRUE(gc_generate_newlines);

namespace openlcb {
/// These definitions tell how the Node will appear on the OpenLCB bus for a
/// network browser.
extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,
    "OpenMRN",
    "CAN-Serial Arduino Stm32",
    BOARD_NAME,
    "1.00"
};
} // namespace openlcb

/// Arduino setup routine. Initializes the OpenLCB software and connects the
/// CAN-bus and the serial port.
void setup() {
  SERIAL_PORT.begin(SERIAL_BAUD_RATE);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  openmrn.add_gridconnect_port(&SERIAL_PORT);
  arduino_can_pinmap(CAN_TX_PIN, CAN_RX_PIN);
  openmrn.add_can_port(&Can);
  openmrn.begin();
}

/// Arduino loop routine. Calls the OpenLCB software to do its work.
void loop() {
  openmrn.loop();
}
