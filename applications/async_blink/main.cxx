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
 * A simple application to demonstrate asynchronous interfaces.
 *
 * @author Balazs Racz
 * @date 7 Dec 2013
 */

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "os/os.h"
#include "can_frame.h"
#include "nmranet_config.h"

#include "nmranet/SimpleStack.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#ifdef TARGET_LPC11Cxx
#include "freertos_drivers/nxp/11cxx_async_can.hxx"
#endif

#ifdef BOARD_LAUNCHPAD_EK
#include "console/Console.hxx"
#endif

extern const nmranet::NodeID NODE_ID;

OVERRIDE_CONST(gc_generate_newlines, 1);

//OVERRIDE_CONST(can_tx_buffer_size, 2);
//OVERRIDE_CONST(can_rx_buffer_size, 1);

#ifdef BOARD_LAUNCHPAD_EK
OVERRIDE_CONST(main_thread_stack_size, 2500);
#elif defined(TARGET_LPC11Cxx)
OVERRIDE_CONST(main_thread_stack_size, 1200);
#endif

nmranet::SimpleCanStack stack(NODE_ID);

//static const uint64_t EVENT_ID = 0x0501010114FF2200ULL;
static const uint64_t EVENT_ID = 0x0502010202000000ULL;

class BlinkerFlow : public StateFlowBase
{
public:
    BlinkerFlow(nmranet::Node* node)
        : StateFlowBase(node->interface()),
          state_(1),
          bit_(node, EVENT_ID, EVENT_ID + 1, &state_, (uint8_t)1),
          producer_(&bit_),
          sleepData_(this)
    {
        start_flow(STATE(blinker));
    }

private:
    Action blinker()
    {
        state_ = !state_;
#ifdef __linux__
        LOG(INFO, "blink produce %d", state_);
#endif
        producer_.Update(&helper_, n_.reset(this));
        return wait_and_call(STATE(handle_sleep));
    }

    Action handle_sleep()
    {
        return sleep_and_call(&sleepData_, MSEC_TO_NSEC(1000), STATE(blinker));
    }

    uint8_t state_;
    nmranet::MemoryBit<uint8_t> bit_;
    nmranet::BitEventProducer producer_;
    nmranet::WriteHelper helper_;
    StateFlowTimer sleepData_;
    BarrierNotifiable n_;
};

extern "C" { void resetblink(uint32_t pattern); }

class LoggingBit: public nmranet::BitEventInterface
{
public:
    LoggingBit(uint64_t event_on, uint64_t event_off, const char* name)
        : BitEventInterface(event_on, event_off), name_(name), state_(false)
    {
    }

    virtual bool GetCurrentState()
    {
        return state_;
    }
    virtual void SetState(bool new_value)
    {
        state_ = new_value;
        //HASSERT(0);
#ifdef __linux__
        LOG(INFO, "bit %s set to %d", name_, state_);
#else
        resetblink(state_ ? 1 : 0);
#endif
    }

    virtual nmranet::Node* node()
    {
        return stack.node();
    }

private:
    const char* name_;
    bool state_;
};

BlinkerFlow blinker_flow(stack.node());

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char* argv[])
{
#ifdef BOARD_LAUNCHPAD_EK
    new Console(true, -1);
#endif

#ifdef __linux__
    stack.print_all_packets();
    stack.start_tcp_hub_server(12021);
#elif defined(TARGET_LPC11Cxx)
    lpc11cxx::CreateCanDriver(stack.can_hub());
#elif defined(TARGET_PIC32MX)
    stack.add_can_port_blocking("/dev/can0");
#elif defined(__FreeRTOS__)
    stack.add_can_port_async("/dev/can0");
#else
#error Define how to connect to your CAN hardware.
#endif  // default target

    // Enable this to add sniffing through the usb or serial port.
#if defined(SNIFF_ON_USB)
    stack.add_gridconnect_port("/dev/serUSB0");
#endif
#if defined(SNIFF_ON_SERIAL)
    stack.add_gridconnect_port("/dev/ser0");
#endif


    LoggingBit logger(EVENT_ID, EVENT_ID + 1, "blinker");
    nmranet::BitEventConsumer consumer(&logger);

    stack.loop_executor();
    return 0;
}
