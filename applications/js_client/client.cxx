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

#include "os/os.h"
#include "can_frame.h"
#include "nmranet_config.h"

#include "os/TempFile.hxx"
#include "nmranet/SimpleStack.hxx"
#include "nmranet/SimpleNodeInfoMockUserFile.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "utils/JSWebsocketClient.hxx"

const nmranet::NodeID NODE_ID = 0x0501010114DFULL;

nmranet::SimpleCanStack stack(NODE_ID);

nmranet::MockSNIPUserFile snip_user_file("Default user name",
                                         "Default user description");
const char *const nmranet::SNIP_DYNAMIC_FILENAME =
    nmranet::MockSNIPUserFile::snip_user_file_path;

void start_stack()
{
    emscripten_cancel_main_loop();
    stack.loop_executor();
    EM_ASM(console.log('stack start done'););
}

void ignore_function()
{
}

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

    void SetState(bool new_value) OVERRIDE {
        lastValue_ = new_value;
        if (new_value) {
            eventOnCallback_();
        } else {
            eventOffCallback_();
        }
    }

    bool GetCurrentState() OVERRIDE {
        return lastValue_;
    }

    bool lastValue_{false};
    emscripten::val eventOnCallback_;
    emscripten::val eventOffCallback_;
    nmranet::BitEventPC consumer_;
};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    new JSWebsocketClient(stack.can_hub(), "ws://localhost:50003");

    // We delay the start of the stack until the connection is established.
    emscripten_set_main_loop(&ignore_function, 0, true);
    return 0;
}

EMSCRIPTEN_BINDINGS(js_client_main)
{
    emscripten::function("startStack", &start_stack);
    emscripten::class_<JSBitEventPC>("BitEventPC")
        .constructor<std::string, emscripten::val, std::string,
                     emscripten::val>();
}
