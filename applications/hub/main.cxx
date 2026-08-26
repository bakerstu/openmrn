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
#include <errno.h>

#include <memory>

#include "executor/Executor.hxx"
#include "executor/Service.hxx"
#include "os/os.h"
#include "utils/ClientConnection.hxx"
#include "utils/GcTcpHub.hxx"
#include "utils/Hub.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/SocketCan.hxx"
#include "utils/constants.hxx"
#include "openlcb/FilteringCanHubFlow.hxx"
#include "HubMetrics.hxx"
#include "MonitoringServer.hxx"
#include "MetricsPort.hxx"

Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);
openlcb::FilteringCanHubFlow can_hub0(&g_service);

OVERRIDE_CONST(gc_generate_newlines, 1);
OVERRIDE_CONST(gridconnect_buffer_size, 1300);
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 2000);
OVERRIDE_CONST(gridconnect_bridge_max_incoming_packets, 5);
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, 5);
OVERRIDE_CONST(gridconnect_tcp_snd_buffer_size, 8192);
OVERRIDE_CONST(gridconnect_tcp_rcv_buffer_size, 8192);
OVERRIDE_CONST(gridconnect_tcp_notsent_lowat_buffer_size, 1024);

OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);


int port = 12021;
const char *device_path = nullptr;
const char *socket_can_path = nullptr;
int upstream_port = 12021;
const char *upstream_host = nullptr;
bool timestamped = false;
bool export_mdns = false;
const char* mdns_name = "openmrn_hub";
bool printpackets = false;
int monitoring_port = 8080;
bool enable_monitoring = true;
int stats_dump_interval = 0;  // 0 = disabled

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s [-p port] [-d device_path] [-u upstream_host] "
        "[-q upstream_port] [-m] [-n mdns_name] "
#if defined(__linux__)
        "[-s socketcan_interface] "
#endif
        "[-t] [-l] [-M monitoring_port] [--no-monitoring] [--stats-dump N]\n\n",
        e);
    fprintf(stderr,
        "GridConnect CAN HUB.\nListens to a specific TCP port, "
        "reads CAN packets from the incoming connections using "
        "the GridConnect protocol, and forwards all incoming "
        "packets to all other participants.\n\nArguments:\n");
    fprintf(stderr, "\t-p port     specifies the port number to listen on, "
                    "default is 12021.\n");
    fprintf(stderr, "\t-d device   is a path to a physical device doing "
                    "serial-CAN or USB-CAN. If specified, opens device and "
                    "adds it to the hub.\n");
#if defined(__linux__)
    fprintf(stderr, "\t-s socketcan_interface   is a socketcan device (e.g. 'can0'). "
                    "If specified, opens device and adds it to the hub.\n");
#endif
    fprintf(stderr, "\t-u upstream_host   is the host name for an upstream "
                    "hub. If specified, this hub will connect to an upstream "
                    "hub.\n");
    fprintf(stderr,
            "\t-q upstream_port   is the port number for the upstream hub.\n");
    fprintf(stderr,
            "\t-t prints timestamps for each packet.\n");
    fprintf(stderr,
            "\t-l print all packets.\n");
    fprintf(stderr,
            "\t-M monitoring_port   enables HTTP monitoring server on specified port (default: 8080).\n");
    fprintf(stderr,
            "\t--no-monitoring      disables HTTP monitoring server.\n");
    fprintf(stderr,
            "\t--stats-dump N       prints statistics summary to stderr every N seconds.\n");
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
    int option_index = 0;
    
    static struct option long_options[] = {
        {"no-monitoring", no_argument, 0, 1000},
        {"stats-dump", required_argument, 0, 1001},
        {0, 0, 0, 0}
    };
    
    while ((opt = getopt_long(argc, argv, "hp:d:s:u:q:tlmn:M:", long_options, &option_index)) >= 0)
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
            {
                char *endptr;
                errno = 0;
                long val = strtol(optarg, &endptr, 10);
                if (errno != 0 || *endptr != '\0' || val < 1 || val > 65535)
                {
                    fprintf(stderr, "Invalid port number: %s\n", optarg);
                    usage(argv[0]);
                }
                port = (int)val;
                break;
            }
            case 'M':
            {
                char *endptr;
                errno = 0;
                long val = strtol(optarg, &endptr, 10);
                if (errno != 0 || *endptr != '\0' || val < 1 || val > 65535)
                {
                    fprintf(stderr, "Invalid monitoring port number: %s\n", optarg);
                    usage(argv[0]);
                }
                monitoring_port = (int)val;
                enable_monitoring = true;
                break;
            }
            case 'u':
                upstream_host = optarg;
                break;
            case 'q':
            {
                char *endptr;
                errno = 0;
                long val = strtol(optarg, &endptr, 10);
                if (errno != 0 || *endptr != '\0' || val < 1 || val > 65535)
                {
                    fprintf(stderr, "Invalid upstream port number: %s\n", optarg);
                    usage(argv[0]);
                }
                upstream_port = (int)val;
                break;
            }
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
            case 1000: // --no-monitoring
                enable_monitoring = false;
                break;
            case 1001: // --stats-dump
            {
                char *endptr;
                errno = 0;
                long val = strtol(optarg, &endptr, 10);
                if (errno != 0 || *endptr != '\0' || val < 0)
                {
                    fprintf(stderr, "Invalid stats dump interval: %s\n", optarg);
                    usage(argv[0]);
                }
                stats_dump_interval = (int)val;
                break;
            }
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
    
    // Create metrics object
    HubMetrics metrics;
    
    // Create metrics port to track packet statistics
    MetricsPort metrics_port(can_hub0.service(), &metrics);
    can_hub0.register_port(metrics_port.get_port());
    can_hub0.set_port_promiscuous(metrics_port.get_port(), true);
    
    // Create packet printer if requested
    GcPacketPrinter *packet_printer = NULL;
    if (printpackets) {
        packet_printer = new GcPacketPrinter(&can_hub0, timestamped);
        can_hub0.set_port_promiscuous(packet_printer->get_port(), true);
    }
    
    // Create hub
    GcTcpHub hub(&can_hub0, port);
    vector<std::unique_ptr<ConnectionClient>> connections;
    
    // Start monitoring server if enabled
    MonitoringServer *monitoring_server = nullptr;
    if (enable_monitoring)
    {
        monitoring_server = new MonitoringServer(&metrics, monitoring_port);
        if (!monitoring_server->start())
        {
            fprintf(stderr, "Failed to start monitoring server\n");
            delete monitoring_server;
            monitoring_server = nullptr;
        }
    }

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
                                     "upstream", &can_hub0, upstream_host, upstream_port));
    }

    if (device_path)
    {
        connections.emplace_back(
            new DeviceConnectionClient("device", &can_hub0, device_path));
    }

    // Track initial connection count
    metrics.increment_connections(); // For main TCP hub listener
    
    fprintf(stderr, "Hub started. TCP port: %d", port);
    if (enable_monitoring && monitoring_server && monitoring_server->is_ready())
    {
        fprintf(stderr, ", Monitoring: http://127.0.0.1:%d/status", monitoring_port);
    }
    fprintf(stderr, "\n");
    
    // Main loop with stats dumping
    time_t last_stats_dump = time(nullptr);
    while (1)
    {
        // Note: GcTcpHub doesn't expose individual connection events,
        // so full connection tracking would require modifying the hub class
        
        // Ping connections to keep them alive
        for (const auto &p : connections)
        {
            p->ping();
        }
        
        // Dump stats if enabled
        if (stats_dump_interval > 0)
        {
            time_t now = time(nullptr);
            if (now - last_stats_dump >= stats_dump_interval)
            {
                metrics.print_summary();
                last_stats_dump = now;
            }
        }
        
        sleep(1);
    }
    
    // Cleanup (never reached, but good practice)
    if (monitoring_server)
    {
        delete monitoring_server;
    }
    
    return 0;
}
