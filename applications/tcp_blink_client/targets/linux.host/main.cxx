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
#include "openlcb/SimpleStack.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/SocketClient.hxx"
#include "utils/SocketClientParams.hxx"
#include "openlcb/SimpleNodeInfoMockUserFile.hxx"

openlcb::MockSNIPUserFile snip_user_file("Default user name",
                                         "Default user description");
const char *const openlcb::SNIP_DYNAMIC_FILENAME = openlcb::MockSNIPUserFile::snip_user_file_path;

const openlcb::NodeID NODE_ID = 0x050101011876;
const openlcb::EventId EVENT_ID = 0x0501010118760000;

Executor<1> g_connect_executor("connect_executor", 0, 2048);
openlcb::SimpleTcpStack stack(NODE_ID);

BlinkerFlow blinker(stack.node(), EVENT_ID);

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
    stack.add_tcp_port_select(fd, on_error);
    stack.restart_stack();
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    SocketClient socket_client(stack.service(), &g_connect_executor,
        &g_connect_executor,
        SocketClientParams::from_static(upstream_host, upstream_port),
        &connect_callback);

    stack.loop_executor();
    return 0;
}
