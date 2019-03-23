/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * A simple application that connect using the OpenLCB-Tcp protocol.
 *
 * @author Balazs Racz
 * @date 23 March 2019
 */

#define LOGLEVEL INFO

#include "executor/Executor.hxx"
#include "nmranet_config.h"
#include "openlcb/BlinkerFlow.hxx"
#include "openlcb/DefaultNode.hxx"
#include "openlcb/EventService.hxx"
#include "openlcb/IfTcp.hxx"
#include "openlcb/NodeInitializeFlow.hxx"
#include "openlcb/ProtocolIdentification.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/SocketClient.hxx"
#include "utils/SocketClientParams.hxx"

const openlcb::NodeID NODE_ID = 0x050101011876;
const openlcb::EventId EVENT_ID = 0x0501010118760000;

Executor<5> g_executor("executor", 0, 2048);
Executor<1> g_connect_executor("connect_executor", 0, 2048);
Service g_service(&g_executor);
HubFlow g_device(&g_service);
openlcb::IfTcp g_if(NODE_ID, &g_device, config_local_nodes_count());
openlcb::InitializeFlow g_init_flow(&g_if);
openlcb::DefaultNode g_node(&g_if, NODE_ID);
openlcb::EventService g_event_service(&g_if);
constexpr uint64_t PIP_SUPPORT = openlcb::Defs::EVENT_EXCHANGE;
openlcb::ProtocolIdentificationHandler g_pip_handler(&g_node, PIP_SUPPORT);

BlinkerFlow blinker(&g_node, EVENT_ID);

int upstream_port = 12000;
const char *upstream_host = "localhost";

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s [-u upstream_host] [-q upstream_port] \n\n", e);
    fprintf(stderr,
        "TCP client demo.\nConnects to a TCP protocol hub, and sends some "
        "example events.\n\nArguments:\n");
    fprintf(stderr,
        "\t-u upstream_host   is the host name for an upstream "
        "hub.\n");
    fprintf(stderr,
        "\t-q upstream_port   is the port number for the upstream hub.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hu:q:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'u':
                upstream_host = optarg;
                break;
            case 'q':
                upstream_port = atoi(optarg);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

void connect_callback(int fd, Notifiable *on_error)
{
    LOG(INFO, "Connected to hub.");
    // we leak this.
    new HubDeviceSelect<HubFlow>(&g_device, fd, on_error);
    // will delete itself
    new openlcb::ReinitAllNodes(&g_if);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    SocketClient socket_client(&g_service, &g_connect_executor,
        &g_connect_executor,
        SocketClientParams::from_static(upstream_host, upstream_port),
        &connect_callback);

    while (1)
    {
        sleep(1);
    }

    return 0;
}
