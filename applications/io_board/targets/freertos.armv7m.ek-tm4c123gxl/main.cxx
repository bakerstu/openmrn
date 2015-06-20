/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file main.cxx
 *
 * Main file for the io board application on the Tiva Launchpad board.
 *
 * @author Balazs Racz
 * @date 5 Jun 2015
 */

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "os/os.h"
#include "can_frame.h"
#include "nmranet_config.h"

#include "os/TempFile.hxx"
#include "nmranet/SimpleStack.hxx"
#include "nmranet/SimpleNodeInfoMockUserFile.hxx"
#include "nmranet/ConfiguredConsumer.hxx"

#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "config.hxx"
#include "definitions.hxx"

#define SNIFF_ON_SERIAL

extern const nmranet::NodeID NODE_ID;

OVERRIDE_CONST(gc_generate_newlines, 1);
OVERRIDE_CONST(main_thread_stack_size, 2500);

nmranet::SimpleCanStack stack(NODE_ID);

const char *const nmranet::CONFIG_FILENAME = "/dev/eeprom";
const char *const nmranet::SNIP_DYNAMIC_FILENAME = nmranet::CONFIG_FILENAME;

static_assert(nmranet::ConfigDef::size() <= 256, "Need to adjust eeprom size");

GPIO_PIN(LED_GREEN, LedPin, F, 3);
GPIO_PIN(LED_BLUE, LedPin, F, 2);

GPIO_PIN(SW1, GpioInputPU, F, 4);
GPIO_PIN(SW2, GpioInputPU, F, 0);

nmranet::ConfigDef cfg(0);

nmranet::ConfiguredConsumer consumer_red(
    stack.node(), cfg.seg().consumers().entry<0>(), BLINKER_Pin());
nmranet::ConfiguredConsumer consumer_green(
    stack.node(), cfg.seg().consumers().entry<1>(), LED_GREEN_Pin());
nmranet::ConfiguredConsumer consumer_blue(
    stack.node(), cfg.seg().consumers().entry<2>(), LED_BLUE_Pin());

nmranet::ConfiguredProducer producer_sw1(
    stack.node(), cfg.seg().producers().entry<0>(), SW1_Pin());
nmranet::ConfiguredProducer producer_sw2(
    stack.node(), cfg.seg().producers().entry<1>(), SW2_Pin());

nmranet::RefreshLoop loop(stack.node(), {producer_sw1.polling(), producer_sw2.polling()});

nmranet::FileMemorySpace config_space(
    nmranet::CONFIG_FILENAME, cfg.seg().size() + cfg.seg().offset());

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    stack.memory_config_handler()->registry()->insert(
        stack.node(), nmranet::MemoryConfigDefs::SPACE_CONFIG, &config_space);

#if defined(HAVE_PHYSICAL_CAN_PORT)
    stack.add_can_port_select("/dev/can0");
#endif

// Enable this to add sniffing through the usb or serial port.
#if defined(SNIFF_ON_USB)
    stack.add_gridconnect_port("/dev/serUSB0");
#endif
#if defined(SNIFF_ON_SERIAL)
    stack.add_gridconnect_port("/dev/ser0");
#endif

    stack.loop_executor();
    return 0;
}
