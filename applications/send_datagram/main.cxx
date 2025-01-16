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
 * An application that sends an OpenLCB datagram from command line.
 *
 * @author Balazs Racz
 * @date 27 Feb 2015
 */

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <unistd.h>

#include <memory>

#include "os/os.h"
#include "utils/constants.hxx"
#include "utils/Hub.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/GcTcpHub.hxx"
#include "utils/Crc.hxx"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

#include "openlcb/IfCan.hxx"
#include "openlcb/DatagramCan.hxx"
#include "openlcb/BootloaderClient.hxx"
#include "openlcb/If.hxx"
#include "openlcb/AliasAllocator.hxx"
#include "openlcb/DefaultNode.hxx"
#include "openlcb/NodeInitializeFlow.hxx"
#include "openlcb/DatagramHandlerDefault.hxx"
#include "utils/socket_listener.hxx"
#include "utils/StringPrintf.hxx"

#include "freertos/bootloader_hal.h"

NO_THREAD nt;
Executor<1> g_executor(nt);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

static const openlcb::NodeID NODE_ID = 0x05010101181EULL;

openlcb::IfCan g_if_can(&g_executor, &can_hub0, 3, 3, 2);
openlcb::CanDatagramService g_datagram_can(&g_if_can, 10, 2);
openlcb::InitializeFlow g_init_flow(&g_if_can);

static openlcb::AddAliasAllocator g_alias_allocator(NODE_ID, &g_if_can);
openlcb::DefaultNode g_node(&g_if_can, NODE_ID);

namespace openlcb
{
Pool *const g_incoming_datagram_allocator = mainBufferPool;
}

int port = 12021;
const char *host = "localhost";
const char *device_path = nullptr;
const char *filename = nullptr;
uint64_t destination_nodeid = 0;
unsigned destination_alias = 0;
vector<string> payload_strings;
bool wait_for_response = false;

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s ([-i destination_host] [-p port] | [-d device_path]) "
        "(-n nodeid | -a alias) [-w] (-g payload_hex)...\n",
        e);
    fprintf(stderr, "Connects to an openlcb bus and sends a datagram to a "
                    "specific node on the bus.\n");
    fprintf(stderr,
        "The bus connection will be through an OpenLCB HUB on "
        "destination_host:port with OpenLCB over TCP "
        "(in GridConnect format) protocol, or through the CAN-USB device "
        "(also in GridConnect protocol) found at device_path. Device takes "
        "precedence over TCP host:port specification.");
    fprintf(stderr, "The default target is localhost:12021.\n");
    fprintf(stderr, "\nnodeid should be a 12-char hex string with 0x prefix "
                    "and no separators, like '-b 0x05010101141F'\n");
    fprintf(stderr, "\nalias should be a 3-char hex string with 0x prefix and "
                    "no separators, like '-a 0x3F9'\n");
    fprintf(stderr, "\npayload is a string of hex digits that define the "
                    "datagram payload. This payload usually starts with the "
                    "datagram type byte. Required. Example: '-g 20A9' to send "
                    "a memory config datagram with reboot request. "
                    "You can supply multiple -g arguments to send multiple "
                    "datagrams\n");
    fprintf(stderr, "\n-w will wait for a response datagram and print it.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hi:p:d:n:a:g:w")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'i':
                host = optarg;
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'd':
                device_path = optarg;
                break;
            case 'n':
                destination_nodeid = strtoll(optarg, nullptr, 16);
                break;
            case 'a':
                destination_alias = strtoul(optarg, nullptr, 16);
                break;
            case 'g':
                payload_strings.push_back(optarg);
                break;
            case 'w':
                wait_for_response = true;
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
    if (payload_strings.empty() || (!destination_nodeid && !destination_alias))
    {
        usage(argv[0]);
    }
}

using openlcb::DatagramPayload;
using openlcb::DatagramClient;
using openlcb::Defs;
using openlcb::NodeHandle;
using openlcb::DefaultDatagramHandler;
using openlcb::DatagramService;

class DatagramPrinter : public DefaultDatagramHandler {
public:
    DatagramPrinter(DatagramService* s) : DefaultDatagramHandler(s) {}

    Action entry() OVERRIDE {
        string s;
        for (unsigned i = 0; i < size(); ++i) {
            s += StringPrintf("%02X ", payload()[i]);
        }
        fprintf(stderr,"Response datagram: %s\n", s.c_str());
        n_.notify();
        return respond_ok(0);
    }

    void wait() {
        n_.wait_for_notification();
    }

private:
    SyncNotifiable n_;
};

DatagramPrinter printer(&g_datagram_can);

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
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
        conn_fd = ConnectSocket(host, port);
    }
    HASSERT(conn_fd >= 0);
    create_gc_port_for_can_hub(&can_hub0, conn_fd);

    g_if_can.add_addressed_message_support();
    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    g_executor.start_thread("g_executor", 0, 1024);

    // Waits for stack to be up.
    while (!g_node.is_initialized())
        usleep(10000);
    LOG(INFO, "Node initialized.");

    // Parses datagram into a payload.
    SyncNotifiable n;
    BarrierNotifiable bn;

    DatagramClient *client = g_datagram_can.client_allocator()->next_blocking();

    for (const auto &payload_string : payload_strings)
    {
        LOG(INFO, "Sending datagram %s", payload_string.c_str());
        DatagramPayload payload;
        unsigned ofs = 0;
        while (ofs + 2 <= payload_string.size())
        {
            payload.push_back(
                strtoul(payload_string.substr(ofs, 2).c_str(), nullptr, 16));
            ofs += 2;
        }
        if (wait_for_response) {
            g_datagram_can.registry()->insert(&g_node, payload[0], &printer);
            g_datagram_can.registry()->insert(&g_node, payload[0] ^ 1, &printer);
        }
        Buffer<openlcb::GenMessage> *b;
        mainBufferPool->alloc(&b);

        NodeHandle dst;
        dst.alias = destination_alias;
        dst.id = destination_nodeid;
        b->data()->reset(Defs::MTI_DATAGRAM, g_node.node_id(), dst, payload);
        b->set_done(bn.reset(&n));

        client->write_datagram(b);
        n.wait_for_notification();
        fprintf(stderr, "Datagram send result: %04x\n", client->result());
        if (!(client->result() & DatagramClient::OK_REPLY_PENDING)) {
            LOG(INFO, "Target node indicates no response pending.");
        }
        if (wait_for_response) {
            printer.wait();
            g_datagram_can.registry()->erase(&g_node, payload[0], &printer);
            g_datagram_can.registry()->erase(&g_node, payload[0] ^ 1, &printer);
        }
    }

    g_datagram_can.client_allocator()->typed_insert(client);
    exit(0); // do not call destructors.
    return 0;
}
