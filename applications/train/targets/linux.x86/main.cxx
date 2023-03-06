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
 * A simple application to demonstrate a train node.
 *
 * @author Balazs Racz
 * @date 1 Feb 2015
 */

#define LOGLEVEL INFO

#include <fcntl.h>
#include <unistd.h>

#include <memory>

#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/ProtocolIdentification.hxx"
#include "openlcb/SimpleNodeInfo.hxx"
#include "openlcb/SimpleNodeInfoMockUserFile.hxx"
#include "openlcb/SimpleStack.hxx"
#include "openlcb/TractionTestTrain.hxx"
#include "openlcb/TractionTrain.hxx"
#include "os/os.h"
#include "os/sleep.h"
#include "utils/constants.hxx"

static const openlcb::NodeID NODE_ID = 0x0501010100F5ULL;

openlcb::SimpleCanStack stack(NODE_ID);

openlcb::TrainService traction_service(stack.iface());

const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::MockSNIPUserFile::snip_user_file_path;
const char *const openlcb::CONFIG_FILENAME = openlcb::SNIP_DYNAMIC_FILENAME;

int port = 12021;
const char *host = "localhost";
const char *device_path = nullptr;
const char *name = "Deadrail Train";
int address = 1732;
OVERRIDE_CONST(num_memory_spaces, 4);
OVERRIDE_CONST(local_nodes_count, 250);
OVERRIDE_CONST(local_alias_cache_size, 251);

namespace openlcb
{
const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4, "OpenMRN", "Logical train node", "No hardware here", "0.91"};
}

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s ([-i destination_host] [-p port] | [-d device_path]) "
        "[-a address] [-n name]\n\n",
        e);
    fprintf(
        stderr, "Connects to an openlcb bus and exports a virtual train.\n");
    fprintf(stderr,
        "The bus connection will be through an OpenLCB HUB on "
        "destination_host:port with OpenLCB over TCP "
        "(in GridConnect format) protocol, or through the CAN-USB device "
        "(also in GridConnect protocol) found at device_path. Device takes "
        "precedence over TCP host:port specification.");
    fprintf(stderr, "The default target is localhost:12021.\n");
    fprintf(stderr, "Address is the virtual loco address. Default 1726.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:i:d:a:n:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'i':
                host = optarg;
                break;
            case 'd':
                device_path = optarg;
                break;
            case 'a':
                address = atoi(optarg);
                break;
            case 'n':
                name = optarg;
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}



namespace openlcb
{

/// This struct encompasses all objects needed for a virtual train node.
struct TrainInstance : private IncomingMessageStateFlow
{
    /// Constructor
    ///
    /// @param addr Train (DCC) address. Will be used for generating the node
    /// ID.
    /// @param name Train Name. Will be used for SNIP.
    /// @param desc Train User Description. Will be used for SNIP.
    TrainInstance(unsigned addr, const string &name, const string &desc)
        : IncomingMessageStateFlow(stack.iface())
        , address_(addr)
        , nodeName_(name)
        , nodeDescription_(desc)
        , responseFlow_(stack.info_flow())
    {
        stack.iface()->dispatcher()->register_handler(
            this, Defs::MTI_IDENT_INFO_REQUEST, Defs::MTI_EXACT);
    }

    /// DCC address
    unsigned address_;
    /// Text name for the train. Will appear on the throttle.
    string nodeName_;
    /// Text description of the node. Will be used in the SNIP reply.
    string nodeDescription_;
    /// Common helper flow for responding to SNIP requests.
    SimpleInfoFlow *responseFlow_;
    /// This implementtion object is supposed to drive the hardware.
    LoggingTrain train_impl {address_};
    /// Virtual Train Node object.
    TrainNodeWithId node_ {&traction_service, &train_impl,
        TractionDefs::NODE_ID_DCC | 0xFF000000u | address_};
    /// Handles PIP requests for this node.
    ProtocolIdentificationHandler pip {&node_,
        Defs::EVENT_EXCHANGE | Defs::SIMPLE_NODE_INFORMATION |
            Defs::TRACTION_CONTROL};
    /// Ensures that this train responds to the IS_TRAIN event requests.
    FixedEventProducer<TractionDefs::IS_TRAIN_EVENT> isTrainEventHandler_ {
        &node_};

    /// What should be our response to SNIP requests.
    const SimpleInfoDescriptor SNIP_CUSTOM_RESPONSE[9] = {
        {SimpleInfoDescriptor::LITERAL_BYTE, 4, 0, nullptr},
        {SimpleInfoDescriptor::C_STRING, 0, 0,
            SNIP_STATIC_DATA.manufacturer_name},
        {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.model_name},
        {SimpleInfoDescriptor::C_STRING, 0, 0,
            SNIP_STATIC_DATA.hardware_version},
        {SimpleInfoDescriptor::C_STRING, 0, 0,
            SNIP_STATIC_DATA.software_version},
        {SimpleInfoDescriptor::LITERAL_BYTE, 2, 0, nullptr},
        {SimpleInfoDescriptor::C_STRING, 0, 0, nodeName_.c_str()},
        {SimpleInfoDescriptor::C_STRING, 0, 0, nodeDescription_.c_str()},
        {SimpleInfoDescriptor::END_OF_DATA, 0, 0, 0}};

    /// Custom handler for SNIP reply.
    Action entry() OVERRIDE
    {
        if (!nmsg()->dstNode)
            return release_and_exit();
        if (nmsg()->dstNode != &node_)
            return release_and_exit();
        auto *b = responseFlow_->alloc();
        b->data()->reset(
            nmsg(), SNIP_CUSTOM_RESPONSE, Defs::MTI_IDENT_INFO_REPLY);
        responseFlow_->send(b);
        return release_and_exit();
    }
}; // struct TrainInstance

} // namespace openlcb

int appl_main(int argc, char *argv[])
{
    if (false)
    {
        // This will make it appear that the device is slow to respond to
        // anything.
        stack.iface()->set_tx_hook([]() { microsleep(50000); });
    }
    parse_args(argc, argv);
    LOG(INFO, "Train name: %s", name);
    openlcb::MockSNIPUserFile snip_user_file(name, "Deadrail--description");

    if (device_path)
    {
        stack.add_gridconnect_port(device_path);
    }
    else
    {
        stack.connect_tcp_gridconnect_hub(host, port);
    }

    if (false)
    {
        // This is the simple way to create one train
        openlcb::LoggingTrain train_impl(address);
        openlcb::TrainNodeWithId train_node(&traction_service, &train_impl,
            openlcb::TractionDefs::NODE_ID_DCC | 0xFF000000u | address);
        openlcb::FixedEventProducer<openlcb::TractionDefs::IS_TRAIN_EVENT>
            is_train_event_handler(&train_node);
        openlcb::ProtocolIdentificationHandler pip(&train_node,
            openlcb::Defs::EVENT_EXCHANGE |
                openlcb::Defs::SIMPLE_NODE_INFORMATION |
                openlcb::Defs::TRACTION_CONTROL);
        openlcb::SNIPHandler snip_handler {
            stack.iface(), &train_node, stack.info_flow()};

        // Have to loop before the objects above go out of scope.
        stack.loop_executor();
    }
    else
    {
        // This is how we can create multiple trains.
        openlcb::TrainInstance some_train(
            address, name, "Deadrail--description");

        // Have to loop before the objects above go out of scope.
        stack.loop_executor();
    }

    return 0;
}
