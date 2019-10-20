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

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <unistd.h>

#if defined(__linux__) || defined(__MACH__)
#include <net/if.h>
#include <termios.h> /* tc* functions */
#endif

#if defined(__linux__)
#include "utils/HubDeviceSelect.hxx"
#include <linux/sockios.h>
#include <sys/ioctl.h>
#endif

#include <memory>

#include "os/os.h"
#include "utils/constants.hxx"
#include "utils/Hub.hxx"
#include "utils/GcTcpHub.hxx"
#include "utils/ClientConnection.hxx"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

OVERRIDE_CONST(gc_generate_newlines, 1);
OVERRIDE_CONST(gridconnect_buffer_size, 1300);
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 2000);


int port = 12021;
const char *device_path = nullptr;
const char *socket_can_path = nullptr;
int upstream_port = 12021;
const char *upstream_host = nullptr;
bool timestamped = false;
bool export_mdns = false;
const char* mdns_name = "openmrn_hub";
bool printpackets = false;

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s [-p port] [-d device_path] [-u upstream_host] "
                    "[-q upstream_port] [-m] [-n mdns_name] [-t] [-l]\n\n",
            e);
    fprintf(stderr, "GridConnect CAN HUB.\nListens to a specific TCP port, "
                    "reads CAN packets from the incoming connections using "
                    "the GridConnect protocol, and forwards all incoming "
                    "packets to all other participants.\n\nArguments:\n");
    fprintf(stderr, "\t-p port     specifies the port number to listen on, "
                    "default is 12021.\n");
    fprintf(stderr, "\t-d device   is a path to a physical device doing "
                    "serial-CAN or USB-CAN. If specified, opens device and "
                    "adds it to the hub.\n");
    fprintf(stderr, "\t-s socketcan device   is a path to a socketcan device. "
                    "If specified, opens device and adds it to the hub.\n");
    fprintf(stderr, "\t-u upstream_host   is the host name for an upstream "
                    "hub. If specified, this hub will connect to an upstream "
                    "hub.\n");
    fprintf(stderr,
            "\t-q upstream_port   is the port number for the upstream hub.\n");
    fprintf(stderr,
            "\t-t prints timestamps for each packet.\n");
    fprintf(stderr,
            "\t-l print all packets.\n");
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
            case 's':
                socket_can_path = optarg;
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
            case 'l':
                printpackets = true;
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
    //GcPacketPrinter packet_printer(&can_hub0, timestamped);
    GcPacketPrinter *packet_printer = NULL;
    if (printpackets) {
        packet_printer = new GcPacketPrinter(&can_hub0, timestamped);
    }
    fprintf(stderr,"packet_printer points to %p\n",packet_printer);
    GcTcpHub hub(&can_hub0, port);
    vector<std::unique_ptr<ConnectionClient>> connections;

#ifdef HAVE_AVAHI_CLIENT
    void mdns_client_start();
    void mdns_publish(const char *name, uint16_t port);

    if (export_mdns) {
        mdns_client_start();
        mdns_publish(mdns_name, port);
    }
#endif
    
    if (upstream_host)
    {
        connections.emplace_back(new UpstreamConnectionClient(
                                     "upstream", &can_hub0, upstream_host, upstream_port));
    }

    if (device_path)
    {
        connections.emplace_back(
            new DeviceConnectionClient("device", &can_hub0, device_path));
    }

    if (socket_can_path)
    {
        int s;
        struct sockaddr_can addr;
        struct ifreq ifr;
        int loopback = 1;

        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        // Set the blocking limit to the minimum allowed, typically 1024 in Linux
        int sndbuf = 0;
        setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        // turn on/off loopback
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

        // setup error notifications
        can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB |
            CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_TRX | CAN_ERR_ACK |
            CAN_ERR_BUSOFF | CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
        strcpy(ifr.ifr_name, socket_can_path);

        ::ioctl(s, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        bind(s, (struct sockaddr *)&addr, sizeof(addr));

        new HubDeviceSelect<CanHubFlow>(&can_hub0, s);
//        connections.emplace_back(port);
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
