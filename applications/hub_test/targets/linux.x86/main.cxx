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
#include "utils/LimitedPool.hxx"

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

/// How many pending buffers we can allow for the send flow.
static constexpr unsigned SEND_PARALLELISM = 10;

/// The output frames will go wioth this CAN ID.
static constexpr uint32_t SEND_HEADER = 0x195b4ffe;
/// The output frames will go with this CAN data bytes. This is NMRA ID 1,
/// which is not assigned. Lower four bytes are the id.
static constexpr uint8_t SEND_PAYLOAD[8] = {0x9, 0x0, 0x01, 0x39, 0, 0, 0, 0};

/// Temporary variable used during args parsing, to hold the -Q value until the
/// next -U comes.
int in_upstream_port = 12021;
/// All the input ports to listen to for incoming traffic.
std::vector<Link> input_ports;

/// All network connections (both input and outputs).
vector<std::unique_ptr<ConnectionClient>> connections;

/// Used for error printing the usage.
const char *arg0 = "hub_test";

// Dynamic variables tracking the traffic.

/// Index of the next packet to generate.
unsigned g_next_packet = 1;

/// How many buffers have been sent and waiting for the notification on them.
unsigned g_pending_buffers = 0;

/// How many packets we have sent to the output iface.
unsigned g_num_packets_sent = 0;
/// How many sent packets got their barrier notified.
unsigned g_num_packets_accepted = 0;

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
    else if (link->device_path)
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

struct PacketInfo : private Notifiable {
    unsigned index_;

    /// Timestamp when the timer ticked to send this packet.
    long long timerTs_{0};
    /// Timestamp when the flow started working on this packet.
    long long flowTs_{0};
    /// Timestamp when the buffer was handed over to the output hub.
    long long sendTs_{0};
    /// Timestamp when the notifiable on the buffer triggered.
    long long confirmTs_{0};
    
    /// Send frame notification barrier.
    BarrierNotifiable bn_{this};
private:
    void notify() override {
        confirmTs_ = os_get_time_monotonic();
        g_pending_buffers--;
        g_num_packets_accepted++;
    }
};

std::map<unsigned, PacketInfo> g_packet_data;

struct SendPacketRequest
{
    /// Packet number to output (sequence number).
    unsigned index;
    /// When was this packet generated by the timer.
    uint64_t generateTsNsec;
};

class SendFlow : public StateFlow<Buffer<SendPacketRequest>, QList<1>>
{
public:
    SendFlow()
        : StateFlow<Buffer<SendPacketRequest>, QList<1>>(&g_service)
    { }

    Action entry() override
    {
        pinfo_ = &g_packet_data[message()->data()->index];
        pinfo_->index_ = message()->data()->index;
        pinfo_->flowTs_ = os_get_time_monotonic();
        pinfo_->timerTs_ = message()->data()->generateTsNsec;
        return allocate_and_call(
            output_port.can_hub.get(), STATE(have_buffer), &pool_);
    }

    Action have_buffer()
    {
        BufferType *b = get_allocation_result(output_port.can_hub.get());
        b->set_done(&pinfo_->bn_);
        auto &f = *b->data()->mutable_frame();
        SET_CAN_FRAME_ID_EFF(f, SEND_HEADER);
        f.can_dlc = 8;
        memcpy(f.data, SEND_PAYLOAD, 8);
        uint32_t idx_be = htobe32(pinfo_->index_);
        memcpy(f.data + 4, &idx_be, 4);
        b->data()->skipMember_ = nullptr;
        output_port.can_hub->send(b);
        pinfo_->sendTs_ = os_get_time_monotonic();
        g_num_packets_sent++;
        return release_and_exit();
    }

private:
    using BufferType = CanHubFlow::buffer_type;
    LimitedPool pool_{sizeof(BufferType), SEND_PARALLELISM};
    PacketInfo* pinfo_;
} g_send_flow;

class PacketGenTimer : public ::Timer
{
public:
    PacketGenTimer()
        : Timer(g_executor.active_timers())
    { }

    long long timeout() override
    {
        auto *b = g_send_flow.alloc();
        b->data()->index = g_next_packet++;
        b->data()->generateTsNsec = this->schedule_time();
        g_send_flow.send(b);
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
        long long diff = SEC_TO_NSEC(1) / pkt_per_sec;
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
