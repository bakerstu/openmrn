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

#include "openlcb/BootloaderClient.hxx"
#include "openlcb/Defs.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/NodeBrowser.hxx"
#include "openlcb/PIPClient.hxx"
#include "openlcb/SimpleNodeInfoMockUserFile.hxx"
#include "openlcb/SimpleStack.hxx"
#include "os/TempFile.hxx"
#include "utils/JSTcpClient.hxx"
#include "utils/JSTcpHub.hxx"
#include "utils/JSWebsocketClient.hxx"
#include "utils/JSWebsocketServer.hxx"
#include "utils/StringPrintf.hxx"
#include "utils/FileUtils.hxx"

const openlcb::NodeID NODE_ID = 0x0501010114DFULL;

openlcb::SimpleCanStack stack(NODE_ID);

openlcb::MockSNIPUserFile snip_user_file(
    "Javascript app", "No description");
const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::MockSNIPUserFile::snip_user_file_path;

bool stack_started = false;

void ignore_function()
{
}

namespace openlcb {
extern int g_bootloader_timeout_sec;
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
        b->data()->reset(openlcb::Defs::MTI_PRODUCER_IDENTIFY, stack.node()->node_id(),
                         openlcb::eventid_to_buffer(*nextEvent_));
        b->data()->set_flag_dst(openlcb::GenMessage::WAIT_FOR_LOCAL_LOOPBACK);
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

class JSBitEventPC : private openlcb::BitEventInterface
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
        b->data()->reset(openlcb::Defs::MTI_EVENT_REPORT,
            stack.node()->node_id(),
            openlcb::eventid_to_buffer(lastValue_ ? event_on() : event_off()));
        f->send(b);
    }

private:
    static uint64_t parse_event_id(const string &s)
    {
        return strtoll(s.c_str(), nullptr, 16);
    }

    openlcb::Node *node() OVERRIDE
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

    openlcb::EventState get_current_state() OVERRIDE
    {
        using openlcb::EventState;
        if (!hasValue_)
            return EventState::UNKNOWN;
        return lastValue_ ? EventState::VALID : EventState::INVALID;
    }

    bool hasValue_{false};
    bool lastValue_{false};
    emscripten::val eventOnCallback_;
    emscripten::val eventOffCallback_;
    openlcb::BitEventPC consumer_;
};

/// Wrapper of the Node ID object into a javascript-friendly function.
struct JSNodeID {
    JSNodeID(uint64_t nid)
        : nodeId_(nid)
    {
    }
    JSNodeID(const std::string& id_buffer)
        : nodeId_(openlcb::buffer_to_node_id(id_buffer))
    {
    }
    uint64_t nodeId_;
    /// @param idx index 0 to 5 (0 is MSB)
    /// @return the byte (0 to 255) at that index.
    int element(int idx)
    {
        if (idx < 0 || idx >= 6)
        {
            return -1;
        }
        return 0xFF & (nodeId_ >> ((5-idx) * 8));
    }
    /// Implementation of the javascript .toString() function.
    /// @return hex representation of the node ID, no dots, padded with zeros.
    std::string to_string()
    {
        std::string r = uint64_to_string_hex(nodeId_, 12);
        for (unsigned i = 0; i < r.size() && r[i] == ' '; ++i)
        {
            r[i] = '0';
        }
        return r;
    }
};

/// Converts a C++ node ID to a javascript object.
/// @param nid is the uint64 node ID.
/// @return a javascript-compatible wrapper.
std::shared_ptr<JSNodeID> node_id_to_js(openlcb::NodeID nid)
{
    return std::make_shared<JSNodeID>(nid);
}

/// Converts a javascript object to a C++ node ID.
/// @param js_nid is the javascript node ID wrapper.
/// @return the c++ representation of the node ID.
openlcb::NodeID js_to_node_id(std::shared_ptr<JSNodeID> js_nid)
{
    return js_nid->nodeId_;
}

/// Javascript wrapper for the Node Browser class.  This gives a callback to
/// javascript whenever a new node joins the bus, and allows pinging all live
/// nodes.
class JSNodeBrowser : public openlcb::NodeBrowser
{
public:
    /// @param callback will be called with a JSNodeID object whenever a new
    /// node joins the bus, or all existing nodes after the refresh function is
    /// called.
    JSNodeBrowser(emscripten::val callback)
        : openlcb::NodeBrowser(stack.node(),
              [callback](openlcb::NodeID id) { callback(node_id_to_js(id)); })
    {
    }

    // This is needed because emscripten Bind cannot export a function
    // inherited from the base class.
    void refresh() {
        openlcb::NodeBrowser::refresh();
    }
};

class JSPIPClient : private Notifiable
{
public:
    /// Constructor.
    JSPIPClient()
    {
    }

    /// Performs a PIP request flow.
    /// @param target is the node ID to query
    /// @param callback is a javascript function(err, data). Upon successful
    /// lookup err will be null and data will be a javascript object with ??
    void lookup(std::shared_ptr<JSNodeID> target, emscripten::val callback)
    {
        HASSERT(!isPending_);
        isPending_ = true;
        callback_ = std::move(callback);
        client_.request(
            openlcb::NodeHandle(js_to_node_id(target)), stack.node(), this);
    }

private:
    using PIPClient = openlcb::PIPClient;
    /// Callback that will be invoked by PIPClient when it's done with the
    /// lookup.
    void notify() override
    {
        isPending_ = false;
        if (client_.error_code() != PIPClient::OPERATION_SUCCESS)
        {
            callback_(
                convert_error(client_.error_code()), emscripten::val::null());
            return;
        }
        callback_(emscripten::val::null(), convert_value(client_.response()));
    }

    /// @param code the error code from the PIP client.
    /// @return javascrit error object
    emscripten::val convert_error(uint32_t code)
    {
        if (code == PIPClient::TIMEOUT)
        {
            return emscripten::val("Timed out.");
        }
        return emscripten::val(StringPrintf("LCC error %04x", code));
    }

#define HANDLE_PROTOCOL(protocol)                                              \
    do                                                                         \
    {                                                                          \
        if (response & openlcb::Defs::protocol)                                \
        {                                                                      \
            ar.call<void>("push",emscripten::val(#protocol));           \
        }                                                                      \
    } while (0)

    /// @param response is the 64-bit bit array returned by the PIP protocol.
    /// @return the javascript version of this map.
    emscripten::val convert_value(uint64_t response)
    {
        emscripten::val ar(emscripten::val::array());
        
        HANDLE_PROTOCOL(SIMPLE_PROTOCOL_SUBSET);
        HANDLE_PROTOCOL(DATAGRAM);
        HANDLE_PROTOCOL(STREAM);
        HANDLE_PROTOCOL(MEMORY_CONFIGURATION);
        HANDLE_PROTOCOL(RESERVATION);
        HANDLE_PROTOCOL(EVENT_EXCHANGE);
        HANDLE_PROTOCOL(IDENTIFICATION);
        HANDLE_PROTOCOL(LEARN_CONFIGURATION);
        HANDLE_PROTOCOL(REMOTE_BUTTON);
        HANDLE_PROTOCOL(ABBREVIATED_DEFAULT_CDI);
        HANDLE_PROTOCOL(DISPLAY_PROTOCOL);
        HANDLE_PROTOCOL(SIMPLE_NODE_INFORMATION);
        HANDLE_PROTOCOL(CDI);
        HANDLE_PROTOCOL(TRACTION_CONTROL);
        HANDLE_PROTOCOL(TRACTION_FDI);
        HANDLE_PROTOCOL(TRACTION_PROXY);
        HANDLE_PROTOCOL(TRACTION_SIMPLE_TRAIN_INFO);
        HANDLE_PROTOCOL(FUNCTION_CONFIGURATION);
        HANDLE_PROTOCOL(FIRMWARE_UPGRADE);
        HANDLE_PROTOCOL(FIRMWARE_UPGRADE_ACTIVE);

        return ar;
    }

#undef HANDLE_PROTOCOL

    emscripten::val callback_ {emscripten::val::null()};
    /// True if we are busy in a lookup.
    bool isPending_ {false};
    /// OpenLCB implementation flow.
    PIPClient client_ {stack.iface()};
};


class JSBootloaderClient : private Notifiable {
public:
    JSBootloaderClient() {}

    /// Invokes firmware upgrade on a remote node.
    /// @param node_id is the target node to upgrade.
    /// @param source_file_name contains the filename to the (binary) file to
    /// send.
    /// @param reboot_first true if the target node shall be rebooted before
    /// downloading. Usually true if the target node is in operational mode, or
    /// false if it is already in firmware upgrade mode. True is a safe choice.
    /// @param progress_cb is a function(ratio){...} which will be called with
    /// a float between 0.0 and 1.0 to mark the progress of the download. This
    /// will be called multiple times.
    /// @param done_cb is a function(err) {...} which will be called exactly
    /// once, with null if the download is successful, or an error object.
    void upgrade(std::shared_ptr<JSNodeID> node_id,
        std::string source_file_name, bool reboot_first,
        emscripten::val progress_cb, emscripten::val done_cb)
    {
        openlcb::g_bootloader_timeout_sec = 30;

        HASSERT(bn_.is_done());
        response_.error_code = 0;
        response_.error_details.clear();
        progressCb_ = std::move(progress_cb);
        doneCb_ = std::move(done_cb);
        Buffer<openlcb::BootloaderRequest> *b = client_.alloc();
        b->set_done(bn_.reset(this));
        b->data()->dst = openlcb::NodeHandle(js_to_node_id(node_id));
        b->data()->request_reboot = reboot_first ? 1 : 0;
        b->data()->data = read_file_to_string(source_file_name);
        fileSize_ = b->data()->data.size();
        b->data()->response = &response_;
        b->data()->progress_callback = [this](float f) { progressCb_(f); };
        client_.send(b);
    }

private:
    /// Called when the client is done.
    void notify() override
    {
        if (response_.error_code == 0)
        {
            progressCb_(1.0f);
            doneCb_(emscripten::val::null());
            return;
        }
        string ret = StringPrintf("Failed (%04x)", response_.error_code);
        if (!response_.error_details.empty())
        {
            ret += ": ";
            ret += response_.error_details;
        }
        doneCb_(ret);
    }

    /// Callback to javascript during download progress.
    emscripten::val progressCb_{emscripten::val::null()};
    /// Callback to javascript when downloading is done.
    emscripten::val doneCb_{emscripten::val::null()};
    /// Return value from the bootloader client.
    openlcb::BootloaderResponse response_;
    /// Helper object for notification.
    BarrierNotifiable bn_;
    /// number of bytes in the file we are sending.
    size_t fileSize_;
    /// The actual bootloader client.
    openlcb::BootloaderClient client_ {
        stack.node(), stack.dg_service(), stack.if_can()};
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

void after_start_exception() {
    stack.executor()->thread().unlock_from_thread();
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    //new JSWebsocketClient(stack.can_hub(), "openmrn_websocket_server_url");

    // We delay the start of the stack until the connection is established.
    //emscripten_set_main_loop(&ignore_function, 0, true);
    EM_ASM(console.log('in appl_main'););
    return 0;
}

/// Adds a new connection to the hub, connecting to a remote hub via TCP
/// gridconnect protocol.
/// @param host is the host name or IP address to connect to.
/// @param port is the TCP port number
void add_tcp_client(string host, int port)
{
    new JSTcpClient(stack.can_hub(), host, port);
}

/// Adds a new connection to the hub, connecting to a remote websocket server
/// via the gridconnect-over-websocket protocol.
/// @param url address of the websocket server.
void add_websocket_client(string url)
{
    new JSWebsocketClient(stack.can_hub(), url);
}

/// Starts a server listening to a TCP port for incoming gridconnect-over-TCP
/// connections.
/// @param port port number to listen on (usually 12021)
void start_tcp_hub(int port)
{
    new JSTcpHub(stack.can_hub(), port);
}

/// Starts a server listening to a TCP port for incoming HTTP to websocket
/// connections. The websocket connections use gridconnect-over-websocket to
/// connect to the LCC network.
/// @param port port number to listen on
/// @param static_dir files in this directory will be available on the HTTP
/// server started on the given port. Can be html pages (e.g. panels) and js
/// script in there.
void start_websocket_server(int port, string static_dir)
{
    new JSWebsocketServer(stack.can_hub(), port, static_dir);
}

EMSCRIPTEN_BINDINGS(js_client_main)
{
    emscripten::register_vector<std::string>("VectorString");
    emscripten::function("startStack", &start_stack);
    emscripten::function("afterStartStack", &after_start_exception);
    emscripten::class_<JSBitEventPC>("BitEventPC")
        .constructor<std::string, emscripten::val, std::string,
            emscripten::val>()
        .function("toggleState", &JSBitEventPC::toggleState);
    emscripten::class_<JSNodeBrowser>("NodeBrowser")
        .constructor<emscripten::val>()
        .function("refresh", &JSNodeBrowser::refresh);
    emscripten::class_<JSNodeID>("NodeID")
        .smart_ptr_constructor<std::shared_ptr<JSNodeID>, const std::string&>(
            "NodeID", &std::make_shared<JSNodeID>)
        //.field("internalId", &JSNodeID::internalId_)
        .function("element", &JSNodeID::element)
        .function("toString", &JSNodeID::to_string);
    emscripten::class_<JSPIPClient>("PIPClient")
        .smart_ptr_constructor("PIPClient", &std::make_shared<JSPIPClient>)
        .function("lookup", &JSPIPClient::lookup);
    emscripten::class_<JSBootloaderClient>("BootloaderClient")
        .smart_ptr_constructor(
            "BootloaderClient", &std::make_shared<JSBootloaderClient>)
        .function("upgrade", &JSBootloaderClient::upgrade);
    emscripten::function("startWebsocketServer", &start_websocket_server);
    emscripten::function("startTcpHub", &start_tcp_hub);
    emscripten::function("addWebsocketClient", &add_websocket_client);
    emscripten::function("addTcpClient", &add_tcp_client);
}
