/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file client.cxx
 *
 * A fake application meant to be compiled into a .js library. This library can
 * be included in html pages to add an OpenLCB stack running in the browser.
 *
 * @author Balazs Racz
 * @date 13 Sep 2015
 */

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <memory>
#include <set>

#include "os/os.h"
#include "can_frame.h"
#include "nmranet_config.h"

#include "os/TempFile.hxx"
#include "nmranet/Defs.hxx"
#include "nmranet/SimpleStack.hxx"
#include "nmranet/SimpleNodeInfoMockUserFile.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "utils/JSWebsocketClient.hxx"

const nmranet::NodeID NODE_ID = 0x0501010114DFULL;

nmranet::SimpleCanStack stack(NODE_ID);

nmranet::MockSNIPUserFile snip_user_file(
    "Javascript app", "No description");
const char *const nmranet::SNIP_DYNAMIC_FILENAME =
    nmranet::MockSNIPUserFile::snip_user_file_path;

bool stack_started = false;

void ignore_function()
{
}

class QueryFlow : public StateFlowBase
{
public:
    QueryFlow()
        : StateFlowBase(stack.service())
    {
        start_flow(STATE(wait_for_initialized));
    }

    /// Adds an event ID to be queried (once the node is up).
    void add_event(uint64_t event_id)
    {
        /// @TODO (balazs.racz) handle the case when events are inserted after
        /// the iteration has begun.
        events_.insert(event_id);
    }

    /// Starts the entire initialization again, performing a query on all known
    /// event IDs once more.
    void restart()
    {
        if (is_terminated())
        {
            start_flow(STATE(wait_for_initialized));
        }
        else
        {
            nextEvent_ = events_.begin();
        }
    }

private:
    Action wait_for_initialized()
    {
        if (!stack.node()->is_initialized())
        {
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(100), STATE(wait_for_initialized));
        }
        return call_immediately(STATE(start_iteration));
    }

    Action start_iteration()
    {
        nextEvent_ = events_.begin();
        return call_immediately(STATE(iterate));
    }

    Action iterate()
    {
        if (nextEvent_ == events_.end())
        {
            return exit();
        }
        return allocate_and_call(
            stack.node()->iface()->global_message_write_flow(),
            STATE(send_query));
    }

    Action send_query()
    {
        auto *f = stack.node()->iface()->global_message_write_flow();
        auto *b = get_allocation_result(f);
        b->data()->reset(nmranet::Defs::MTI_PRODUCER_IDENTIFY, stack.node()->node_id(),
                         nmranet::eventid_to_buffer(*nextEvent_));
        b->data()->set_flag_dst(nmranet::NMRAnetMessage::WAIT_FOR_LOCAL_LOOPBACK);
        b->set_done(n_.reset(this));
        f->send(b);
        nextEvent_++;
        return wait_and_call(STATE(delay_before_iterate));
    }

    Action delay_before_iterate() {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(5), STATE(iterate));
    }

    using EventList = std::set<uint64_t>;
    EventList events_;
    EventList::iterator nextEvent_;
    BarrierNotifiable n_;
    StateFlowTimer timer_{this};
} g_query_flow;

class JSBitEventPC : private nmranet::BitEventInterface
{
public:
    JSBitEventPC(std::string event_on, emscripten::val fn_on_cb,
        std::string event_off, emscripten::val fn_off_cb)
        : BitEventInterface(parse_event_id(event_on), parse_event_id(event_off))
        , eventOnCallback_(fn_on_cb)
        , eventOffCallback_(fn_off_cb)
        , consumer_(this)
    {
        g_query_flow.add_event(BitEventInterface::event_on());
    }

    void toggleState()
    {
        if (!hasValue_)
        {
            lastValue_ = true;
            hasValue_ = true;
            //return;
        }
        /// @TODO(balazs.racz) ideally we should be able to just call
        /// SendEventReport. However, that requires a WriteHelper and there is
        /// no option to allocate a buffer dynamically.
        lastValue_ = !lastValue_;
        auto *f = stack.node()->iface()->global_message_write_flow();
        auto *b = f->alloc();
        b->data()->reset(nmranet::Defs::MTI_EVENT_REPORT,
            stack.node()->node_id(),
            nmranet::eventid_to_buffer(lastValue_ ? event_on() : event_off()));
        f->send(b);
    }

private:
    static uint64_t parse_event_id(const string &s)
    {
        return strtoll(s.c_str(), nullptr, 16);
    }

    nmranet::Node *node() OVERRIDE
    {
        return stack.node();
    }

    void set_state(bool new_value) OVERRIDE
    {
        lastValue_ = new_value;
        hasValue_ = true;
        if (new_value)
        {
            eventOnCallback_();
        }
        else
        {
            eventOffCallback_();
        }
    }

    nmranet::EventState get_current_state() OVERRIDE
    {
        using nmranet::EventState;
        if (!hasValue_)
            return EventState::UNKNOWN;
        return lastValue_ ? EventState::VALID : EventState::INVALID;
    }

    bool hasValue_{false};
    bool lastValue_{false};
    emscripten::val eventOnCallback_;
    emscripten::val eventOffCallback_;
    nmranet::BitEventPC consumer_;
};

void start_stack()
{
    if (!stack_started) {
        stack_started = true;
        emscripten_cancel_main_loop();
        stack.loop_executor();
        EM_ASM(console.log('stack start done'););
    } else {
        // Restarting.
        stack.restart_stack();
        g_query_flow.restart();
    }
}



/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    new JSWebsocketClient(stack.can_hub(), "openmrn_websocket_server_url");

    // We delay the start of the stack until the connection is established.
    emscripten_set_main_loop(&ignore_function, 0, true);
    return 0;
}

EMSCRIPTEN_BINDINGS(js_client_main)
{
    emscripten::function("startStack", &start_stack);
    emscripten::class_<JSBitEventPC>("BitEventPC")
        .constructor<std::string, emscripten::val, std::string,
            emscripten::val>()
        .function("toggleState", &JSBitEventPC::toggleState);
}
