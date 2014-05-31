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
#include "utils/GridConnectHub.hxx"
#include "executor/Executor.hxx"
#include "can_frame.h"
#include "nmranet_config.h"

#include "utils/GcTcpHub.hxx"
#include "utils/HubDevice.hxx"
#include "nmranet/IfCan.hxx"
#include "nmranet/Defs.hxx"
#include "nmranet/AliasAllocator.hxx"
#include "nmranet/EventService.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/DefaultNode.hxx"

NO_THREAD nt;
Executor<1> g_executor(nt);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);
#ifdef __linux__
GcPacketPrinter packet_printer(&can_hub0);
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

nmranet::IfCan g_if_can(&g_executor, &can_hub0, 3, 3, 2);
static nmranet::AddAliasAllocator _alias_allocator(NODE_ID, &g_if_can);
nmranet::DefaultNode g_node(&g_if_can, NODE_ID);
nmranet::EventService g_event_service(&g_if_can);

static const uint64_t EVENT_ID = 0x0501010114FF2200ULL;

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

class LoggingBit : public nmranet::BitEventInterface
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
        return &g_node;
    }

private:
    const char* name_;
    bool state_;
};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char* argv[])
{
#ifdef __linux__
    GcTcpHub hub(&can_hub0, 12021);
#else
#ifdef NNTARGET_LPC11Cxx
    lpc11cxx::CreateCanDriver(&can_pipe);
#else
    int can_fd = ::open("/dev/can0", O_RDWR);
    HASSERT(can_fd >= 0);

    FdHubPort<CanHubFlow> can_hub_port(&can_hub0, can_fd, EmptyNotifiable::DefaultInstance());
#endif  // default target
#endif  // FreeRTOS

    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    LoggingBit logger(EVENT_ID, EVENT_ID + 1, "blinker");
    nmranet::BitEventConsumer consumer(&logger);
    //BlinkerFlow blinker(&g_node);

    g_executor.thread_body();
    return 0;
}
