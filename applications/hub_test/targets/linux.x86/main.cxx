/** \copyright
 * Copyright (c) 2023, Balazs Racz
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
 * Main file for the hub tester application.
 *
 * @author Balazs Racz
 * @date 23 Dec 2023
 */

#include "nmranet_config.h"
#include "os/os.h"

#include "utils/ClientConnection.hxx"
#include "utils/StringPrintf.hxx"

OVERRIDE_CONST(gc_generate_newlines, 1);

Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);

struct Link
{
    const char *device_path = nullptr;
    int upstream_port = 12021;
    const char *upstream_host = nullptr;
    std::unique_ptr<CanHubFlow> can_hub {new CanHubFlow(&g_service)};
};

/// We generate packets to this interface.
Link output_port;
/// Traffic to generate, default is to saturate the link.
int pkt_per_sec = -1;

/// Temporary variable used during args parsing, to hold the -Q value until the
/// next -U comes.
int in_upstream_port = 12021;
/// All the input ports to listen to for incoming traffic.
std::vector<Link> input_ports;

/// All network connections (both input and outputs).
vector<std::unique_ptr<ConnectionClient>> connections;

/// Used for error printing the usage.
const char* arg0 = "hub_test";

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s (-d device_path | [-q upstream_port] -u upstream_host) [-s "
        "speed]\n\t(-D device_path | [-Q upstream_port] -U "
        "upstream_host)...\n\n",
        e);
    fprintf(stderr,
        "\tdevice_path   is a path to a physical device doing "
        "serial-CAN or USB-CAN.\n");
    fprintf(stderr,
        "\tupstream_host   is the host name for an upstream "
        "hub.\n");
    fprintf(
        stderr, "\tupstream_port   is the port number for the upstream hub.\n");
    fprintf(stderr,
        "\t-d -q -u specifies the output port, -D -Q -U specifies input ports. "
        "-Q must be before -U. Multiple input ports can be specified.\n");
    fprintf(stderr,
        "\t-s speed   is the packets/sec to generate. Set to -1 for utomatic "
        "(saturation).\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hd:u:q:s:D:U:Q:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'd':
                output_port.device_path = optarg;
                break;
            case 'u':
                output_port.upstream_host = optarg;
                break;
            case 'q':
                output_port.upstream_port = atoi(optarg);
                break;
            case 'D':
                input_ports.emplace_back();
                input_ports.back().device_path = optarg;
                break;
            case 'U':
                input_ports.emplace_back();
                input_ports.back().upstream_host = optarg;
                input_ports.back().upstream_port = in_upstream_port;
                break;
            case 'Q':
                in_upstream_port = atoi(optarg);
                break;
            case 's':
                pkt_per_sec = atoi(optarg);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

/// Establishes a new connection.
void add_link(Link *link)
{
    static int id = 0;
    if (link->upstream_host)
    {
        connections.emplace_back(
            new UpstreamConnectionClient(StringPrintf("upstream%d", id),
                link->can_hub.get(), link->upstream_host, link->upstream_port));
    }
    else

        if (link->device_path)
    {
        connections.emplace_back(
            new DeviceConnectionClient(StringPrintf("device%d", id),
                link->can_hub.get(), link->device_path));
    }
    else
    {
        usage(arg0);
    }
    ++id;
}

class PacketGenTimer : public ::Timer
{
public:
    PacketGenTimer()
        : Timer(g_executor.active_timers())
    { }

    long long timeout() override
    {
        /// @todo send output
        // stack.send_event(0x0501010114DD1234);
        return RESTART;
    }
} pkt_gen_timer;

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    arg0 = argv[0];
    parse_args(argc, argv);

    g_executor.start_thread("executor_thread", 0, 5000);

    if (pkt_per_sec > 0)
    {
        long long diff = 1000000000ULL / pkt_per_sec;
        pkt_gen_timer.start(diff);
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
