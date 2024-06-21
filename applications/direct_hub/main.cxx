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
 * An application which acts as an openlcb hub with the GC protocol, using the
 * DirectHub infrastructure.
 *
 * @author Balazs Racz
 * @date 31 Dec 2023
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
#include "utils/DirectHub.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/SocketCan.hxx"
#include "utils/constants.hxx"

Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);

std::unique_ptr<ByteDirectHubInterface> g_direct_hub {create_hub(&g_executor)};

CanHubFlow can_hub0(&g_service);

OVERRIDE_CONST(gc_generate_newlines, 1);
OVERRIDE_CONST(gridconnect_tcp_snd_buffer_size, 8192);
OVERRIDE_CONST(gridconnect_tcp_rcv_buffer_size, 8192);
OVERRIDE_CONST(gridconnect_tcp_notsent_lowat_buffer_size, 1024);

int port = 12021;
const char *device_path = nullptr;
const char *socket_can_path = nullptr;
int upstream_port = 12021;
const char *upstream_host = nullptr;
bool timestamped = false;
bool export_mdns = false;
const char *mdns_name = "openmrn_hub";
bool printpackets = false;

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s [-p port] [-d device_path] [-u upstream_host] "
        "[-q upstream_port] [-m] [-n mdns_name] "
#if defined(__linux__)
        "[-s socketcan_interface] "
#endif
        "[-t] [-l]\n\n",
        e);
    fprintf(stderr,
        "GridConnect CAN HUB.\nListens to a specific TCP port, "
        "reads CAN packets from the incoming connections using "
        "the GridConnect protocol, and forwards all incoming "
        "packets to all other participants.\n\nArguments:\n");
    fprintf(stderr,
        "\t-p port     specifies the port number to listen on, "
        "default is 12021.\n");
    fprintf(stderr,
        "\t-d device   is a path to a physical device doing "
        "serial-CAN or USB-CAN. If specified, opens device and "
        "adds it to the hub.\n");
#if defined(__linux__)
    fprintf(stderr,
        "\t-s socketcan_interface   is a socketcan device (e.g. 'can0'). "
        "If specified, opens device and adds it to the hub.\n");
#endif
    fprintf(stderr,
        "\t-u upstream_host   is the host name for an upstream "
        "hub. If specified, this hub will connect to an upstream "
        "hub.\n");
    fprintf(stderr,
        "\t-q upstream_port   is the port number for the upstream hub.\n");
    fprintf(stderr, "\t-t prints timestamps for each packet.\n");
    fprintf(stderr, "\t-l print all packets.\n");
#ifdef HAVE_AVAHI_CLIENT
    fprintf(stderr, "\t-m exports the current service on mDNS.\n");
    fprintf(
        stderr, "\t-n mdns_name sets the exported mDNS name. Implies -m.\n");
#endif
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:d:s:u:q:tlmn:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'd':
                device_path = optarg;
                break;
#if defined(__linux__)
            case 's':
                socket_can_path = optarg;
                break;
#endif
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
            case 'l':
                printpackets = true;
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

void create_legacy_bridge() {
    static bool is_created = false;
    if (!is_created) {
        is_created = true;
        create_gc_to_legacy_can_bridge(g_direct_hub.get(), &can_hub0);
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
    // GcPacketPrinter packet_printer(&can_hub0, timestamped);
    GcPacketPrinter *packet_printer = NULL;
    if (printpackets)
    {
        create_legacy_bridge();
        packet_printer = new GcPacketPrinter(&can_hub0, timestamped);
    }
    fprintf(stderr, "packet_printer points to %p\n", packet_printer);
    create_direct_gc_tcp_hub(g_direct_hub.get(), port);
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

#if defined(__linux__)
    if (socket_can_path)
    {
        int s = socketcan_open(socket_can_path, 1);
        if (s >= 0)
        {
            create_legacy_bridge();
            new HubDeviceSelect<CanHubFlow>(&can_hub0, s);
            fprintf(stderr, "Opened SocketCan %s: fd %d\n", socket_can_path, s);
        }
        else
        {
            fprintf(stderr, "Failed to open SocketCan %s.\n", socket_can_path);
        }
    }
#endif

    if (upstream_host)
    {
        connections.emplace_back(new UpstreamConnectionClient(
            "upstream", g_direct_hub.get(), upstream_host, upstream_port));
    }

    if (device_path)
    {
        connections.emplace_back(new DeviceConnectionClient(
            "device", g_direct_hub.get(), device_path));
    }

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
