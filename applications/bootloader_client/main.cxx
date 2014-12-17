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
 * An application which acts as an openlcb hub with the GC protocol.
 *
 * @author Balazs Racz
 * @date 3 Aug 2013
 */

#include <stdio.h>
#include <unistd.h>
#include <getopt.h>

#include <memory>

#include "os/os.h"
#include "utils/constants.hxx"
#include "utils/Hub.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/GcTcpHub.hxx"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

#include "nmranet/IfCan.hxx"
#include "nmranet/DatagramCan.hxx"
#include "nmranet/BootloaderClient.hxx"
#include "nmranet/If.hxx"
#include "nmranet/AliasAllocator.hxx"
#include "nmranet/DefaultNode.hxx"
#include "utils/socket_listener.hxx"

NO_THREAD nt;
Executor<1> g_executor(nt);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

static const nmranet::NodeID NODE_ID = 0x05010101141FULL;

nmranet::IfCan g_if_can(&g_executor, &can_hub0, 3, 3, 2);
nmranet::CanDatagramService g_datagram_can(&g_if_can, 10, 2);
static nmranet::AddAliasAllocator g_alias_allocator(NODE_ID, &g_if_can);
nmranet::DefaultNode g_node(&g_if_can, NODE_ID);

namespace nmranet {
Pool* const g_incoming_datagram_allocator = mainBufferPool;
}

int port = 12021;
const char *host = "localhost";
const char *filename = nullptr;
uint64_t destination_nodeid = 0;
uint64_t destination_alias = 0;
int memory_space_id = 0xF1;

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s [-d destination_host] [-p port] [-s "
                    "memory_space_id] (-n nodeid | -a "
                    "alias) -f filename\n",
        e);
    fprintf(stderr, "Connects to destination_host:port with OpenLCB over TCP "
                    "(in GridConnect format) protocol, and performs the "
                    "bootloader protocol on openlcb node with id nodeid with "
                    "the contents of a given file.\n");
    fprintf(stderr, "The default target is localhost:12021.\n");
    fprintf(stderr, "nodeid should be a 12-char hex string with 0x prefix and "
                    "no separators, like '-b 0x05010101141F'\n");
    fprintf(stderr, "alias should be a 3-char hex string with 0x prefix and no "
                    "separators, like '-a 0x3F9'\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:d:n:a:s:f:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'd':
                host = optarg;
                break;
            case 'f':
                filename = optarg;
                break;
            case 'n':
                destination_nodeid = strtoll(optarg, nullptr, 16);
                break;
            case 'a':
                destination_alias = strtoul(optarg, nullptr, 16);
                break;
            case 's':
                memory_space_id = strtol(optarg, nullptr, 16);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
    if (!filename || (!destination_nodeid && !destination_alias))
    {
        usage(argv[0]);
    }
}

nmranet::BootloaderClient bootloader_client(&g_node, &g_datagram_can, &g_if_can);
nmranet::BootloaderResponse response;

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);

    int conn_fd = ConnectSocket("localhost", 12021);
    HASSERT(conn_fd >= 0);
    create_gc_port_for_can_hub(&can_hub0, conn_fd);

    g_if_can.add_addressed_message_support();
    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    g_executor.start_thread("g_executor", 0, 1024);
    usleep(400000);

    SyncNotifiable n;
    BarrierNotifiable bn(&n);
    Buffer<nmranet::BootloaderRequest> *b;
    mainBufferPool->alloc(&b);

    b->set_done(&bn);
    b->data()->dst.alias = destination_alias;
    b->data()->dst.id = destination_nodeid;
    b->data()->memory_space = memory_space_id;
    b->data()->offset = 0;
    b->data()->response = &response;

    FILE* f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "Could not open file %s: %s\n", filename, strerror(errno));
        exit(1);
    }
    char buf[1024];
    size_t nr;
    while ((nr = fread(buf, 1, sizeof(buf), f)) > 0) {
        b->data()->data.append(buf, nr);
    }
    fclose(f);
    printf("Read %d bytes from file %s. Writing to memory space 0x%02x\n", b->data()->data.size(), filename, memory_space_id);

    bootloader_client.send(b);
    n.wait_for_notification();
    printf("Result: %04x  %s\n", response.error_code, response.error_details.c_str());

    return 0;
}
