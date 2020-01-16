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
 * A simple application to demonstrate a traction proxy node (aka command
 * station).
 *
 * @author Balazs Racz
 * @date 1 Feb 2015
 */

#define LOGLEVEL INFO

#include <unistd.h>
#include <fcntl.h>

#include <memory>

#include "openlcb/AliasAllocator.hxx"
#include "openlcb/DefaultNode.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/EventService.hxx"
#include "openlcb/IfCan.hxx"
#include "openlcb/ProtocolIdentification.hxx"
#include "openlcb/SimpleNodeInfo.hxx"
#include "openlcb/SimpleNodeInfoMockUserFile.hxx"
#include "openlcb/TractionProxy.hxx"
#include "openlcb/TractionTestTrain.hxx"
#include "openlcb/TractionTrain.hxx"
#include "os/os.h"
#include "utils/GridConnectHub.hxx"
#include "utils/Hub.hxx"
#include "utils/constants.hxx"
#include "utils/socket_listener.hxx"

NO_THREAD nt;
Executor<1> g_executor(nt);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

openlcb::IfCan g_if_can(&g_executor, &can_hub0, 3, 3, 2);
static const openlcb::NodeID NODE_ID = 0x050101011807ULL;
static openlcb::AddAliasAllocator g_alias_allocator(NODE_ID, &g_if_can);
openlcb::DefaultNode g_node(&g_if_can, NODE_ID);
openlcb::SimpleInfoFlow gInfoFlow(&g_if_can);
openlcb::SNIPHandler snip(&g_if_can, &g_node, &gInfoFlow);

openlcb::ProtocolIdentificationHandler
pip(&g_node, openlcb::Defs::EVENT_EXCHANGE |
                 openlcb::Defs::SIMPLE_NODE_INFORMATION |
                 openlcb::Defs::TRACTION_PROXY);

openlcb::EventService g_event_service(&g_if_can);
openlcb::TrainService traction_service(&g_if_can);
openlcb::TractionProxyService traction_proxy(&traction_service, &g_node);

namespace openlcb
{
const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4, "OpenMRN", "Virtual command station", "No hardware here", "0.92"};

MockSNIPUserFile snip_user_file("User name (CS)", "User description (CS)");

const char *const SNIP_DYNAMIC_FILENAME = MockSNIPUserFile::snip_user_file_path;
}

int port = 12021;
const char *host = "localhost";
const char *device_path = nullptr;
int address = 1726;

void usage(const char *e)
{
    fprintf(stderr,
            "Usage: %s ([-i destination_host] [-p port] | [-d device_path]) "
            "           [-a address]\n",
            e);
    fprintf(stderr,
            "Connects to an openlcb bus and exports a virtual train.\n");
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
    while ((opt = getopt(argc, argv, "hp:i:d:a:")) >= 0)
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
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

namespace openlcb
{

struct TrainNodeImpl
{
    std::unique_ptr<openlcb::TrainImpl> impl;
    std::unique_ptr<openlcb::TrainNode> node;
    std::unique_ptr<EventHandler> is_train_event;
    std::unique_ptr<openlcb::IncomingMessageStateFlow> pip_handler;
};

map<uint16_t, TrainNodeImpl> trains;

Node *allocate_train_node(uint8_t system, uint8_t addr_hi, uint8_t addr_lo,
                          TrainService *traction_service)
{
    uint16_t address = addr_hi;
    address <<= 8;
    address |= addr_lo;

    TrainNodeImpl &n = trains[address];
    if (!n.node)
    {
        n.impl.reset(new openlcb::LoggingTrain(address));
        n.node.reset(
            new openlcb::TrainNodeForProxy(traction_service, n.impl.get()));
        n.is_train_event.reset(new openlcb::FixedEventProducer<
            openlcb::TractionDefs::IS_TRAIN_EVENT>(n.node.get()));
        n.pip_handler.reset(new openlcb::ProtocolIdentificationHandler(
            n.node.get(),
            openlcb::Defs::EVENT_EXCHANGE | openlcb::Defs::TRACTION_CONTROL));
    }
    return n.node.get();
}
}

int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    int conn_fd = 0;
    if (device_path)
    {
        conn_fd = ::open(device_path, O_RDWR);
    }
    else
    {
        conn_fd = ConnectSocket("localhost", 12021);
    }
    HASSERT(conn_fd >= 0);
    create_gc_port_for_can_hub(&can_hub0, conn_fd);

    g_if_can.add_addressed_message_support();
    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    g_executor.thread_body();
    return 0;
}
