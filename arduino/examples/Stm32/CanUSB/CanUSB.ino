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
 * \file Stm32CanUSB.ino
 * 
 * Example Arduino sketch for the Stm32 showing how to make a USB-CAN
 * adapter with minimal size and dependencies.
 *
 * @author Balazs Racz
 * @date 23 March 2020
 */

#include <Arduino.h>
#include <OpenMRNLite.h>

/// Which serial port to use.
/// =========================
/// IMPORTANT: TO USE THIS SKETCH, YOU MUST SET Tools->USB
/// Support TO CDC (either one).
/// =========================
#define SERIAL_PORT SerialUSB

/// Specifies which Stm32 Pin should be used for CAN_TX signal. The default
/// matches the pinout of the OpenLCB Dev Kit daughterboard.
#define CAN_TX_PIN PB_9

/// Specifies which Stm32 Pin should be used for CAN_RX signal. The default
/// matches the pinout of the OpenLCB Dev Kit daughterboard.
#define CAN_RX_PIN PB_8

Stm32Can Can("/dev/can0");

OVERRIDE_CONST_TRUE(gc_generate_newlines);

/// These objects perform the crossbar switching of CAN frames.
Executor<1> openmrn_executor{NO_THREAD()};
Service openmrn_service(&openmrn_executor);
CanHubFlow openmrn_can_hub(&openmrn_service);
std::unique_ptr<Executable> can_bridge;
std::unique_ptr<Executable> serial_bridge;

/// Arduino setup routine. Initializes the CAN-bus and the serial port.
void setup() {
  // Baud rate does not matter for USB ports.
  SERIAL_PORT.begin(115200);
  arduino_can_pinmap(CAN_TX_PIN, CAN_RX_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
  // Ties the two ports into the Hub.
  can_bridge.reset(new openmrn_arduino::CanBridge(&Can, &openmrn_can_hub));
  serial_bridge.reset(new openmrn_arduino::SerialBridge<decltype(SERIAL_PORT)>(
      &SERIAL_PORT, &openmrn_can_hub));
}

/// Arduino loop routine. Calls the OpenMRN software to do its work.
void loop() {
  openmrn_executor.loop_some();
  can_bridge->run();
  serial_bridge->run();
}
