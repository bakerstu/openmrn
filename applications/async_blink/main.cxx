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

#include <stdio.h>
#include <unistd.h>

#include "os/os.h"
#include "utils/pipe.hxx"
#include "executor/executor.hxx"
#include "nmranet_can.h"
#include "nmranet_config.h"

#include "nmranet/AsyncIfCan.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/AsyncAliasAllocator.hxx"
#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/NMRAnetAsyncEventHandler.hxx"
#include "nmranet/NMRAnetAsyncDefaultNode.hxx"
#include "freertos_drivers/nxp/11cxx_async_can.hxx"

// DEFINE_PIPE(gc_can_pipe, 1);

Executor g_executor;

DEFINE_PIPE(can_pipe, &g_executor, sizeof(struct can_frame));

static const NMRAnet::NodeID NODE_ID = 0x050101011441ULL;

extern "C" {
const size_t WRITE_FLOW_THREAD_STACK_SIZE = 900;
extern const size_t CAN_TX_BUFFER_SIZE;
extern const size_t CAN_RX_BUFFER_SIZE;
const size_t CAN_RX_BUFFER_SIZE = 1;
const size_t CAN_TX_BUFFER_SIZE = 2;
const size_t main_stack_size = 900;
}

NMRAnet::AsyncIfCan g_if_can(&g_executor, &can_pipe, 3, 3, 2);
NMRAnet::DefaultAsyncNode g_node(&g_if_can, NODE_ID);
NMRAnet::GlobalEventFlow g_event_flow(&g_executor, 4);

static const uint64_t EVENT_ID = 0x050101011441FF00ULL;
const int main_priority = 0;

Executor* DefaultWriteFlowExecutor() {
    return &g_executor;
}

class BlinkerFlow : public ControlFlow
{
public:
    BlinkerFlow(NMRAnet::AsyncNode* node)
        : ControlFlow(node->interface()->dispatcher()->executor(), nullptr),
          state_(1),
          bit_(node, EVENT_ID, EVENT_ID + 1, &state_, (uint8_t)1),
          producer_(&bit_)
    {
        StartFlowAt(ST(blinker));
    }

private:
    ControlFlowAction blinker()
    {
        state_ = !state_;
#ifdef __linux__
        LOG(INFO, "blink produce %d", state_);
#endif
        producer_.Update(&helper_, this);
        return WaitAndCall(ST(handle_sleep));
    }

    ControlFlowAction handle_sleep()
    {
        return Sleep(&sleepData_, MSEC_TO_NSEC(1000), ST(blinker));
    }

    uint8_t state_;
    NMRAnet::MemoryBit<uint8_t> bit_;
    NMRAnet::BitEventProducer producer_;
    NMRAnet::WriteHelper helper_;
    SleepData sleepData_;
};

extern "C" { void resetblink(uint32_t pattern); }

class LoggingBit : public NMRAnet::BitEventInterface
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

    virtual NMRAnet::AsyncNode* node()
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
#ifdef TARGET_LPC11Cxx
    lpc11cxx::CreateCanDriver(&can_pipe);
#endif
    LoggingBit logger(EVENT_ID, EVENT_ID + 1, "blinker");
    NMRAnet::BitEventConsumer consumer(&logger);
    BlinkerFlow blinker(&g_node);
    g_if_can.AddWriteFlows(1, 1);
    g_if_can.set_alias_allocator(
        new NMRAnet::AsyncAliasAllocator(NODE_ID, &g_if_can));
    NMRAnet::AliasInfo info;
    g_if_can.alias_allocator()->empty_aliases()->Release(&info);
    NMRAnet::AddEventHandlerToIf(&g_if_can);
    g_executor.ThreadBody();
    return 0;
}
