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
 * \file HardwareCanAdapter.ino
 * 
 * Example application for the ESP32 showing how to configure the hardware CAN
 * to communicate with OpenMRN
 *
 * @author Mike Dunston
 * @date 19 January 2019
 */

#include <Arduino.h>
#include <WiFi.h>
#include <OpenMRN.h>
#include <driver/can.h>
#include <freertos/task.h>

constexpr uint32_t     SERIAL_BAUD     = 115200L;

static constexpr uint64_t NODE_ID = UINT64_C(0x050101011423);
OpenMRN openmrn(NODE_ID);

void setup()
{
    Serial.begin(SERIAL_BAUD);
    WiFi.mode(WIFI_MODE_NULL);
    btStop();

    openmrn.stack()->print_all_packets();
    openmrn.start_background_task();

    openmrn.add_can_port(new Esp32HardwareCan("ESP32Can", GPIO_NUM_16, GPIO_NUM_17));
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(50));
}