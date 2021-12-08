/** \copyright
 * Copyright (c) 2021, Robert Heller
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
 * An application which acts as an openlcb hub with the Tcp protocol.
 *
 * @author Robert Heller
 * @date 8 Nov 2021
 */


#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <unistd.h>

#include <memory>

#include "executor/Executor.hxx"
#include "executor/Service.hxx"
#include "os/os.h"
#include "utils/ClientConnection.hxx"
#include "openlcb/TcpHub.hxx"
#include "utils/Hub.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/constants.hxx"

Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);
openlcb::TcpHubFlow tcp_hub0(&g_service);

int port = 12000;
int upstream_port = 12000;
const char *upstream_host = nullptr;
bool timestamped = false;
bool export_mdns = false; 
const char* mdns_name = "openmrn_tcphub";

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s [-p port] [-u upstream_host] "
        "[-q upstream_port] [-m] [-n mdns_name] "
        "[-t] [-l]\n\n",
        e);
    fprintf(stderr,
        "Tcp HUB.\nListens to a specific TCP port, "
        "reads Tcp packets from the incoming connections using "
        "the OpenLCB Tcp/Ip protocol, and forwards all incoming "
        "packets to all other participants.\n\nArguments:\n");
    fprintf(stderr, "\t-p port     specifies the port number to listen on, "
                    "default is 12000.\n");
    fprintf(stderr, "\t-u upstream_host   is the host name for an upstream "
                    "hub. If specified, this hub will connect to an upstream "
                    "hub.\n");
    fprintf(stderr,
            "\t-q upstream_port   is the port number for the upstream hub.\n");
    fprintf(stderr,
            "\t-t prints timestamps for each packet.\n");
#ifdef HAVE_AVAHI_CLIENT
    fprintf(stderr,
            "\t-m exports the current service on mDNS.\n");
    fprintf(stderr,
            "\t-n mdns_name sets the exported mDNS name. Implies -m.\n");
#endif
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:u:q:tmn:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'u':
                upstream_host = optarg;
                break;
            case 'q':
                upstream_port = atoi(optarg);
                break;
            case 't':
                timestamped = true;
                break;
            case 'm':
                export_mdns = true;
                break;
            case 'n':
                mdns_name = optarg;
                export_mdns = true;
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    openlcb::TcpHub hub(&tcp_hub0, port);
    vector<std::unique_ptr<ConnectionClient>> connections;

#ifdef HAVE_AVAHI_CLIENT
    void mdns_client_start();
    void mdns_publish(const char *name, uint16_t port);

    if (export_mdns)
    {
        mdns_client_start();
        mdns_publish(mdns_name, port);
    }
#endif
    
#if 0
    if (upstream_host)
    {
        connections.emplace_back(new UpstreamConnectionClient(
                                     "upstream", &tcp_hub0, upstream_host, upstream_port));
    }
#endif

    while (1)
    {
        for (const auto &p : connections)
        {
            p->ping();
        }
        sleep(1);
    }
    return 0;
}