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

#include <sys/ioctl.h>

#include "nmranet_config.h"
#include "os/os.h"
#include "os/sleep.h"

#include "utils/Buffer.hxx"
#include "utils/ClientConnection.hxx"
#include "utils/LimitedPool.hxx"
#include "utils/StringPrintf.hxx"
#include "utils/Stats.hxx"

OVERRIDE_CONST(gc_generate_newlines, 1);
OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, 2);
//OVERRIDE_CONST(gridconnect_tcp_snd_buffer_size, 8192);
//OVERRIDE_CONST(gridconnect_tcp_rcv_buffer_size, 8192);
OVERRIDE_CONST(gridconnect_tcp_notsent_lowat_buffer_size, 1460);

// Maximum 100 TCP packets per second.
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 10000);
// Or one full packet.
OVERRIDE_CONST(gridconnect_buffer_size, 1460);

Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);

class Receiver;

struct Link
{
    /// Config (cmdline): device file to use
    const char *device_path = nullptr;
    /// Config (cmdline) TCP target port number
    int upstream_port = 12021;
    /// Config (cmdline) TCP target hostname
    const char *upstream_host = nullptr;
    /// The CAN hub for this link.
    std::unique_ptr<CanHubFlow> can_hub {new CanHubFlow(&g_service)};
    /// The receiver object keeping track of the inputs and stats. Will be
    /// created for every link, but only used for the input ports.
    std::unique_ptr<Receiver> receiver;
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

/// How many live links are there (upstream + downstream).
int g_num_live_links = 1;

/// Index of the next packet to generate.
uint32_t g_next_packet = 1;

/// How many buffers have been sent and waiting for the notification on them.
unsigned g_pending_buffers = 0;

/// How many packets we have sent to the output iface.
unsigned g_num_packets_sent = 0;
/// How many sent packets got their (send) barrier notified.
unsigned g_num_packets_accepted = 0;
/// How many sent packets got their (receive) barrier notified and removed from
/// storage.
unsigned g_num_packets_all_received = 0;

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

/// Call this function when a packet has all its receivers confirmed.
/// @param index packet number.
void packet_complete(unsigned index);

struct PacketInfo : private Notifiable
{
    unsigned index_;

    /// Timestamp when the timer ticked to send this packet.
    long long timerTs_ {0};
    /// Timestamp when the flow started working on this packet.
    long long flowTs_ {0};
    /// Timestamp when the buffer was handed over to the output hub.
    long long sendTs_ {0};
    /// Timestamp when the notifiable on the buffer triggered.
    long long confirmTs_ {0};

    /// Send frame notification barrier.
    BarrierNotifiable bn_ {this};

    /// Number of receivers that have not yet gotten this message.
    unsigned pendingReceivers_ {0};

    /// Notifies that this packet was received by a receiver link.
    void notify_reception()
    {
        if (pendingReceivers_)
        {
            --pendingReceivers_;
        }
        if (!pendingReceivers_)
        {
            // Confirmed by all receivers. Removes from storage.
            g_num_packets_all_received++;
            packet_complete(index_);
        }
    }

private:
    void notify() override
    {
        confirmTs_ = os_get_time_monotonic();
        g_pending_buffers--;
        g_num_packets_accepted++;
    }
};

std::map<unsigned, PacketInfo> g_packet_data;

class Receiver : public CanHubPortInterface
{
public:
    Receiver(CanHubFlow *hub)
    {
        hub->register_port(this);
    }

    void send(message_type *buf, unsigned prio) override
    {
        auto ts = os_get_time_monotonic();
        auto rb = get_buffer_deleter(buf);
        const struct can_frame &f = buf->data()->frame();
        if (f.can_dlc != 8 || !IS_CAN_FRAME_EFF(f))
        {
            // not interesting frame
            ++numUnknownFrames_;
            return;
        }
        auto id = GET_CAN_FRAME_ID_EFF(f);
        if (id != SEND_HEADER)
        {
            // not interesting frame
            ++numUnknownFrames_;
            return;
        }
        if (memcmp(f.data, SEND_PAYLOAD, 4) != 0)
        {
            // not interesting frame
            ++numUnknownFrames_;
            return;
        }
        ++numFrames_;
        uint32_t cnt = 0;
        memcpy(&cnt, f.data + 4, 4);
        cnt = be32toh(cnt);
        if (cnt == nextPacket_)
        {
            ++numInOrder_;
            ++nextPacket_;
        }
        else if (cnt > nextPacket_)
        {
            numMissed_ = cnt - nextPacket_;
            nextPacket_ = cnt + 1;
        }
        else
        {
            numOutOfOrder_++;
            // we don't adjust next packet here
        }
        auto it = g_packet_data.find(cnt);
        if (it != g_packet_data.end())
        {
            long long rtt_usec = NSEC_TO_USEC(ts - it->second.confirmTs_);
            rttUsec_.add(rtt_usec);
            it->second.notify_reception();
        }
    }

    /// Gets a stats line for this input.
    string get_stats()
    {
        string ret;
        ret += StringPrintf("\tReceiver: +%d recv, +%d unk, +%d OK, +%d "
                            "out-of-order, +%d missing",
            numFrames_, numUnknownFrames_, numInOrder_, numOutOfOrder_,
            numMissed_);
        numFrames_ = numUnknownFrames_ = numInOrder_ = numOutOfOrder_ =
            numMissed_ = 0;

        ret += StringPrintf("|RTT %.1f msec +- %.1f\n",
            rttUsec_.favg()/1000, rttUsec_.stddev() / 1000);

        rttUsec_.clear();
        return ret;
    }

public:
    /// Number of incoming frames that we don't recognize.
    unsigned numUnknownFrames_ = 0;
    /// Number of successful frames received.
    unsigned numFrames_ = 0;
    /// Number of in-order frames.
    unsigned numInOrder_ = 0;
    /// Number of frames missed. These may be lost or will arrive later out of
    /// order.
    unsigned numMissed_ = 0;
    /// Number of frames that arrived late / out of order.
    unsigned numOutOfOrder_ = 0;

private:
    /// Which packet index are we expecting next.
    uint32_t nextPacket_ = 1;
    /// Statistics about the RTT of the packet (from send-confirm to
    /// receive).
    Stats rttUsec_;
};

/// Establishes a new connection.
void add_link(Link *link, bool is_receiver)
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
    if (is_receiver)
    {
        link->receiver.reset(new Receiver(link->can_hub.get()));
    }
    ++id;
}

void packet_complete(unsigned index)
{
    g_packet_data.erase(index);
}

struct SendPacketRequest
{
    /// Packet number to output (sequence number).
    unsigned index = 0;
    /// When was this packet generated by the timer.
    uint64_t generateTsNsec = 0;
};

/// Implements the state machine to acquire a buffer and send an outgoing
/// packet.
class SendFlow : public StateFlow<Buffer<SendPacketRequest>, QList<1>>
{
public:
    SendFlow()
        : StateFlow<Buffer<SendPacketRequest>, QList<1>>(&g_service)
    { }

    Action entry() override
    {
        auto it = g_packet_data.find(message()->data()->index);
        if (it != g_packet_data.end())
        {
            LOG(FATAL,
                "duplicate packet index %d, %d, old send Ts %lld, new send ts "
                "%" PRId64,
                message()->data()->index, it->second.index_,
                it->second.timerTs_, message()->data()->generateTsNsec);
            DIE("duplicate packet");
        }
        HASSERT(it == g_packet_data.end());
        pinfo_ = &g_packet_data[message()->data()->index];
        pinfo_->index_ = message()->data()->index;
        pinfo_->flowTs_ = os_get_time_monotonic();
        pinfo_->timerTs_ = message()->data()->generateTsNsec;
        return allocate_and_call(
            output_port.can_hub.get(), STATE(have_buffer), &pool_);
    }

    Action have_buffer()
    {
        g_pending_buffers++;
        auto *b = get_allocation_result(output_port.can_hub.get());
        b->set_done(&pinfo_->bn_);
        auto &f = *b->data()->mutable_frame();
        SET_CAN_FRAME_ID_EFF(f, SEND_HEADER);
        f.can_dlc = 8;
        memcpy(f.data, SEND_PAYLOAD, 8);
        uint32_t idx_be = htobe32(pinfo_->index_);
        memcpy(f.data + 4, &idx_be, 4);
        b->data()->skipMember_ = nullptr;
        pinfo_->pendingReceivers_ = std::max(0, g_num_live_links - 1);
        g_num_packets_sent++;
        pinfo_->sendTs_ = os_get_time_monotonic();
        output_port.can_hub->send(b);
        return release_and_exit();
    }

private:
    LimitedPool pool_ {
        sizeof(Buffer<HubContainer<CanFrameContainer>>), SEND_PARALLELISM};
    PacketInfo *pinfo_;
} g_send_flow;

/// This timer triggers packets to be sent by sending messages to the
/// SendFlow.
class PacketGenTimer : public ::Timer
{
public:
    PacketGenTimer()
        : Timer(g_executor.active_timers())
    { }

    long long timeout() override
    {
        auto *b = g_send_flow.alloc();
        HASSERT(b);
        b->data()->index = g_next_packet++;
        b->data()->generateTsNsec = this->schedule_time();
        g_send_flow.send(b);
        return RESTART;
    }
} pkt_gen_timer;

/// This timer prints stats every second. It is responsible for generating
/// deltas from absolute counters.
class StatsTimer : public ::Timer
{
public:
    StatsTimer()
        : Timer(g_executor.active_timers())
    { }

    long long timeout() override
    {
        print_stats();
        return RESTART;
    }

    void print_stats()
    {
        string ret;
        ret += "Send: ";
        unsigned d = update_diff(g_next_packet, &next_packet);
        ret += StringPrintf("|+%" PRIu32 " gen", d);
        d = update_diff(g_num_packets_accepted, &num_packets_accepted);
        ret += StringPrintf("|+%d send complete", d);
        d = update_diff(g_num_packets_all_received, &num_packets_all_received);
        ret += StringPrintf("|+%d recv complete", d);
        int send_q = -2;
        ::ioctl(connections[0]->fd(), TIOCOUTQ, &send_q);
        // @todo add stats about send queues.
        ret += StringPrintf(
            "||sendq %d|pending buf %u| sent>acc %u| acc>recv %d|", send_q,
            g_pending_buffers, g_num_packets_sent - g_num_packets_accepted,
            g_num_packets_accepted - g_num_packets_all_received);
        ret += "\n";
        for (const auto &lnk : input_ports)
        {
            ret += lnk.receiver->get_stats();
        }
        // @todo print using hubdeviceselect instead of blocking output.
        printf("%s\n", ret.c_str());
    }

private:
    /// Creates a delta from a counter.
    /// @param global_value current value of the global counter.
    /// @param local_value a local shadow variable for the same counter. It
    /// will be updated with the global value every time the function gets
    /// called.
    /// @return diff (number of counts elapsed since last call).
    template <typename T> T update_diff(T global_value, T *local_value)
    {
        T diff = global_value - *local_value;
        *local_value = global_value;
        return diff;
    }

    /// Index of the next packet to generate.
    uint32_t next_packet = 1;
    /// How many buffers have been sent and waiting for the notification on
    /// them.
    // unsigned pending_buffers = 0;

    /// How many packets we have sent to the output iface.
    // unsigned num_packets_sent = 0;
    /// How many sent packets got their (send) barrier notified.
    unsigned num_packets_accepted = 0;
    /// How many sent packets got their (receive) barrier notified and removed
    /// from storage.
    unsigned num_packets_all_received = 0;

} stats_timer;

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    arg0 = argv[0];
    parse_args(argc, argv);
    add_link(&output_port, false);
    for (auto &l : input_ports)
    {
        add_link(&l, true);
    }

    // g_executor.start_thread("executor_thread", 0, 5000);

    if (pkt_per_sec > 0)
    {
        long long diff = SEC_TO_NSEC(1) / pkt_per_sec;
        pkt_gen_timer.start(diff);
    }
    microsleep(100);
    stats_timer.start(SEC_TO_NSEC(1));

    while (1)
    {
        int num_links = 0;
        for (const auto &p : connections)
        {
            if (p->ping())
            {
                num_links++;
            }
        }
        g_num_live_links = num_links;
        sleep(1);
    }

    return 0;
}
