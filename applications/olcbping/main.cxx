/** \copyright
 * Copyright (c) 2026, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * An application that pings a remote device or event over the OpenLCB bus.
 *
 * @author Balazs Racz
 * @date 30 Aug 2026
 */

#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/SimpleNodeInfoMockUserFile.hxx"
#include "openlcb/Convert.hxx"
#include "openlcb/Defs.hxx"
#include "utils/constants.hxx"
#include "utils/format_utils.hxx"
#include "utils/StringPrintf.hxx"

OVERRIDE_CONST(gc_generate_newlines, 1);

openlcb::MockSNIPUserFile snip_user_file(
    "OpenLCB Ping", "Command line ping application");
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::MockSNIPUserFile::snip_user_file_path;
extern const char *const openlcb::CONFIG_FILENAME = "/dev/null";
extern const size_t openlcb::CONFIG_FILE_SIZE = 0;

enum PingMode
{
    MODE_UNSPECIFIED = 0,
    MODE_ADDRESSED,
    MODE_GLOBAL,
    MODE_PRODUCER,
    MODE_CONSUMER
};

struct Options
{
    const char *host = "localhost";
    int port = 12021;
    const char *device_path = nullptr;
    openlcb::NodeID local_node_id = 0x050101011803ULL;
    openlcb::NodeID target_node_id = 0;
    uint64_t target_event_id = 0;
    PingMode mode = MODE_UNSPECIFIED;
    double interval_sec = 0.2;
    unsigned count = 0;
} g_options;

static uint64_t parse_hex_id(const char *str)
{
    if (!str) return 0;
    std::string clean;
    for (const char *p = str; *p; ++p)
    {
        if (isxdigit(*p))
        {
            clean.push_back(*p);
        }
    }
    if (clean.empty()) return 0;
    return strtoull(clean.c_str(), nullptr, 16);
}

static std::string format_node_handle(openlcb::NodeHandle h, openlcb::SimpleCanStack *stack)
{
    if (h.id != 0)
    {
        return node_id_to_string(h.id);
    }
    if (stack && stack->if_can() && h.alias != 0)
    {
        openlcb::NodeID id = stack->if_can()->remote_aliases()->lookup(h.alias);
        if (id != 0)
        {
            return node_id_to_string(id);
        }
    }
    if (h.alias != 0)
    {
        return StringPrintf("0x%03X", h.alias);
    }
    return node_id_to_string(0);
}



static const char *mti_to_string(openlcb::Defs::MTI mti)
{
    switch (mti)
    {
        case openlcb::Defs::MTI_PRODUCER_IDENTIFIED_VALID: return "Producer Identified (Valid)";
        case openlcb::Defs::MTI_PRODUCER_IDENTIFIED_INVALID: return "Producer Identified (Invalid)";
        case openlcb::Defs::MTI_PRODUCER_IDENTIFIED_RESERVED: return "Producer Identified (Reserved)";
        case openlcb::Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN: return "Producer Identified (Unknown)";
        case openlcb::Defs::MTI_PRODUCER_IDENTIFIED_RANGE: return "Producer Identified (Range)";
        case openlcb::Defs::MTI_CONSUMER_IDENTIFIED_VALID: return "Consumer Identified (Valid)";
        case openlcb::Defs::MTI_CONSUMER_IDENTIFIED_INVALID: return "Consumer Identified (Invalid)";
        case openlcb::Defs::MTI_CONSUMER_IDENTIFIED_RESERVED: return "Consumer Identified (Reserved)";
        case openlcb::Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN: return "Consumer Identified (Unknown)";
        case openlcb::Defs::MTI_CONSUMER_IDENTIFIED_RANGE: return "Consumer Identified (Range)";
        default: return "Identified";
    }
}

class OlcbPingFlow : public StateFlowBase
{
public:
    OlcbPingFlow(openlcb::SimpleCanStack *stack, const Options &options)
        : StateFlowBase(stack->service())
        , stack_(stack)
        , options_(options)
        , handler_(this, &OlcbPingFlow::handle_message)
        , timer_(this)
    {
        register_handlers();
        start_flow(STATE(wait_for_node_init));
    }

    ~OlcbPingFlow()
    {
        stack_->iface()->dispatcher()->unregister_handler_all(&handler_);
    }

    void print_stats()
    {
        printf("\n--- %s ping statistics ---\n",
               (options_.mode == MODE_PRODUCER || options_.mode == MODE_CONSUMER)
                   ? event_id_to_string(options_.target_event_id).c_str()
                   : node_id_to_string(options_.target_node_id).c_str());
        double loss_pct = 0.0;
        if (sent_count_ > 0)
        {
            loss_pct = (double)(sent_count_ - (received_count_ > sent_count_ ? sent_count_ : received_count_)) / sent_count_ * 100.0;
        }
        printf("%u packets transmitted, %u responses received, %.1f%% packet loss\n",
               sent_count_, received_count_, loss_pct);
        if (received_count_ > 0)
        {
            double avg_rtt = sum_rtt_ / received_count_;
            printf("rtt min/avg/max = %.3f/%.3f/%.3f ms\n",
                   min_rtt_, avg_rtt, max_rtt_);
        }
        fflush(stdout);
    }

private:
    void register_handlers()
    {
        if (options_.mode == MODE_ADDRESSED || options_.mode == MODE_GLOBAL)
        {
            stack_->iface()->dispatcher()->register_handler(
                &handler_, openlcb::Defs::MTI_VERIFIED_NODE_ID_NUMBER, 0x0FFE);
            stack_->iface()->dispatcher()->register_handler(
                &handler_, openlcb::Defs::MTI_OPTIONAL_INTERACTION_REJECTED, openlcb::Defs::MTI_EXACT);
        }
        else if (options_.mode == MODE_PRODUCER)
        {
            stack_->iface()->dispatcher()->register_handler(
                &handler_, openlcb::Defs::MTI_PRODUCER_IDENTIFIED_VALID, 0x0FFC);
            stack_->iface()->dispatcher()->register_handler(
                &handler_, openlcb::Defs::MTI_PRODUCER_IDENTIFIED_RANGE, openlcb::Defs::MTI_EXACT);
        }
        else if (options_.mode == MODE_CONSUMER)
        {
            stack_->iface()->dispatcher()->register_handler(
                &handler_, openlcb::Defs::MTI_CONSUMER_IDENTIFIED_VALID, 0x0FFC);
            stack_->iface()->dispatcher()->register_handler(
                &handler_, openlcb::Defs::MTI_CONSUMER_IDENTIFIED_RANGE, openlcb::Defs::MTI_EXACT);
        }
    }

    Action wait_for_node_init()
    {
        if (!stack_->node()->is_initialized())
        {
            return sleep_and_call(&timer_, MSEC_TO_NSEC(20), STATE(wait_for_node_init));
        }
        print_header();
        return call_immediately(STATE(send_ping));
    }

    void print_header()
    {
        const char *mode_name = "addressed";
        if (options_.mode == MODE_GLOBAL) mode_name = "global";
        else if (options_.mode == MODE_PRODUCER) mode_name = "producer";
        else if (options_.mode == MODE_CONSUMER) mode_name = "consumer";

        if (options_.mode == MODE_PRODUCER || options_.mode == MODE_CONSUMER)
        {
            printf("PING event %s (mode: %s)\n",
                   event_id_to_string(options_.target_event_id).c_str(),
                   mode_name);
        }
        else
        {
            printf("PING node %s (mode: %s)\n",
                   node_id_to_string(options_.target_node_id).c_str(),
                   mode_name);
        }
        fflush(stdout);
    }

    Action send_ping()
    {
        if (options_.count > 0 && sent_count_ >= options_.count)
        {
            return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(finish));
        }

        sent_count_++;
        uint32_t seq = sent_count_;
        long long now = os_get_time_monotonic();

        sent_pings_.push_back({seq, now});

        switch (options_.mode)
        {
            case MODE_ADDRESSED:
            {
                openlcb::NodeHandle dst(options_.target_node_id);
                stack_->send_message_to(
                    openlcb::Defs::MTI_VERIFY_NODE_ID_ADDRESSED,
                    dst,
                    openlcb::EMPTY_PAYLOAD);
                break;
            }
            case MODE_GLOBAL:
            {
                auto *b = stack_->node()->iface()->global_message_write_flow()->alloc();
                b->data()->reset(
                    openlcb::Defs::MTI_VERIFY_NODE_ID_GLOBAL,
                    stack_->node()->node_id(),
                    openlcb::node_id_to_buffer(options_.target_node_id));
                stack_->node()->iface()->global_message_write_flow()->send(b);
                break;
            }
            case MODE_PRODUCER:
            {
                auto *b = stack_->node()->iface()->global_message_write_flow()->alloc();
                b->data()->reset(
                    openlcb::Defs::MTI_PRODUCER_IDENTIFY,
                    stack_->node()->node_id(),
                    openlcb::eventid_to_buffer(options_.target_event_id));
                stack_->node()->iface()->global_message_write_flow()->send(b);
                break;
            }
            case MODE_CONSUMER:
            {
                auto *b = stack_->node()->iface()->global_message_write_flow()->alloc();
                b->data()->reset(
                    openlcb::Defs::MTI_CONSUMER_IDENTIFY,
                    stack_->node()->node_id(),
                    openlcb::eventid_to_buffer(options_.target_event_id));
                stack_->node()->iface()->global_message_write_flow()->send(b);
                break;
            }
            default:
                break;
        }

        long long interval_nsec = (long long)(options_.interval_sec * 1e9);
        return sleep_and_call(&timer_, interval_nsec, STATE(send_ping));
    }

    void handle_message(Buffer<openlcb::GenMessage> *b)
    {
        auto d = get_buffer_deleter(b);
        openlcb::GenMessage *msg = b->data();
        long long now_nsec = os_get_time_monotonic();

        bool match = false;
        std::string detail_str;
        openlcb::NodeID responding_node_id = msg->src.id;

        if (options_.mode == MODE_ADDRESSED || options_.mode == MODE_GLOBAL)
        {
            openlcb::NodeID payload_node_id = 0;
            if (msg->payload.size() >= 6)
            {
                payload_node_id = openlcb::buffer_to_node_id(msg->payload);
            }
            if (responding_node_id == options_.target_node_id || payload_node_id == options_.target_node_id)
            {
                match = true;
                if (responding_node_id == 0)
                {
                    responding_node_id = payload_node_id;
                }
                detail_str = "Verified Node ID";
            }
            else if (msg->mti == openlcb::Defs::MTI_OPTIONAL_INTERACTION_REJECTED)
            {
                if (msg->src.id == options_.target_node_id)
                {
                    match = true;
                    detail_str = "Interaction Rejected";
                }
            }
        }
        else if (options_.mode == MODE_PRODUCER || options_.mode == MODE_CONSUMER)
        {
            if (msg->payload.size() >= 8)
            {
                uint64_t evt = openlcb::data_to_eventid(msg->payload.data());
                if (evt == options_.target_event_id)
                {
                    match = true;
                    detail_str = mti_to_string(msg->mti);
                }
            }
        }

        if (match && !sent_pings_.empty())
        {
            const auto &latest_ping = sent_pings_.back();
            double rtt_ms = (now_nsec - latest_ping.send_time_nsec) / 1000000.0;
            if (rtt_ms < 0) rtt_ms = 0;

            received_count_++;
            if (rtt_ms < min_rtt_) min_rtt_ = rtt_ms;
            if (rtt_ms > max_rtt_) max_rtt_ = rtt_ms;
            sum_rtt_ += rtt_ms;

            openlcb::NodeHandle h = msg->src;
            if (h.id == 0 && responding_node_id != 0)
            {
                h.id = responding_node_id;
            }
            std::string src_node_str = format_node_handle(h, stack_);

            if (options_.mode == MODE_ADDRESSED || options_.mode == MODE_GLOBAL)
            {
                printf("64 bytes from %s: openlcb_seq=%u time=%.3f ms\n",
                       src_node_str.c_str(),
                       latest_ping.seq,
                       rtt_ms);
            }
            else
            {
                printf("%s for event %s from node %s: openlcb_seq=%u time=%.3f ms\n",
                       detail_str.c_str(),
                       event_id_to_string(options_.target_event_id).c_str(),
                       src_node_str.c_str(),
                       latest_ping.seq,
                       rtt_ms);
            }
            fflush(stdout);
        }
    }

    Action finish()
    {
        print_stats();
        _exit(0);
        return exit();
    }

    struct SentPing
    {
        uint32_t seq;
        long long send_time_nsec;
    };

    openlcb::SimpleCanStack *stack_;
    Options options_;
    openlcb::MessageHandler::GenericHandler handler_;
    StateFlowTimer timer_;
    std::vector<SentPing> sent_pings_;
    unsigned sent_count_ = 0;
    unsigned received_count_ = 0;
    double min_rtt_ = 1e9;
    double max_rtt_ = 0;
    double sum_rtt_ = 0;
};

static OlcbPingFlow *g_ping_flow = nullptr;

static void sigint_handler(int signum)
{
    if (g_ping_flow)
    {
        g_ping_flow->print_stats();
    }
    _exit(0);
}

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s ([-i host] [-p port] | [-d device_path]) "
        "(-n node_id | -e event_id) [-a|-g|-P|-C|-m mode] [-I interval] [-c count]\n\n",
        e);
    fprintf(stderr, "Pings a remote node or event over OpenLCB bus and prints RTT.\n\n");
    fprintf(stderr, "Connection options:\n");
    fprintf(stderr, "  -i host / -u host   GridConnect TCP hub host (default: localhost)\n");
    fprintf(stderr, "  -p port / -q port   GridConnect TCP hub port (default: 12021)\n");
    fprintf(stderr, "  -d device_path      Serial device path (e.g. /dev/ttyACM0)\n");
    fprintf(stderr, "  -S src_node_id      Local node ID (default: 0x050101011499)\n\n");
    fprintf(stderr, "Target options:\n");
    fprintf(stderr, "  -n node_id          Target Node ID (hex, e.g. 05.01.01.01.14.09)\n");
    fprintf(stderr, "  -e event_id         Target Event ID (hex, e.g. 05.01.01.01.14.09.00.01)\n\n");
    fprintf(stderr, "Ping Mode options:\n");
    fprintf(stderr, "  -a, -m addressed    Addressed verify node ID packet (default for -n)\n");
    fprintf(stderr, "  -g, -m global       Global verify node ID packet\n");
    fprintf(stderr, "  -P, -m producer     Identify producer packet (default for -e)\n");
    fprintf(stderr, "  -C, -m consumer     Identify consumer packet\n\n");
    fprintf(stderr, "Timing & count:\n");
    fprintf(stderr, "  -I, -t interval     Ping interval in seconds (fractional, default: 0.2)\n");
    fprintf(stderr, "  -c count            Stop after sending count pings (default: 0 = infinite)\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hi:u:p:q:d:n:e:m:t:I:c:S:gPCa")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'i':
            case 'u':
                g_options.host = optarg;
                break;
            case 'p':
            case 'q':
                g_options.port = atoi(optarg);
                break;
            case 'd':
                g_options.device_path = optarg;
                break;
            case 'n':
                g_options.target_node_id = parse_hex_id(optarg);
                break;
            case 'e':
                g_options.target_event_id = parse_hex_id(optarg);
                break;
            case 'S':
            case 's':
                g_options.local_node_id = parse_hex_id(optarg);
                break;
            case 'a':
                g_options.mode = MODE_ADDRESSED;
                break;
            case 'g':
                g_options.mode = MODE_GLOBAL;
                break;
            case 'P':
                g_options.mode = MODE_PRODUCER;
                break;
            case 'C':
                g_options.mode = MODE_CONSUMER;
                break;
            case 'm':
                if (strcasecmp(optarg, "addressed") == 0) g_options.mode = MODE_ADDRESSED;
                else if (strcasecmp(optarg, "global") == 0) g_options.mode = MODE_GLOBAL;
                else if (strcasecmp(optarg, "producer") == 0) g_options.mode = MODE_PRODUCER;
                else if (strcasecmp(optarg, "consumer") == 0) g_options.mode = MODE_CONSUMER;
                else {
                    fprintf(stderr, "Unknown mode: %s\n", optarg);
                    usage(argv[0]);
                }
                break;
            case 'I':
            case 't':
                g_options.interval_sec = atof(optarg);
                break;
            case 'c':
                g_options.count = atoi(optarg);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }

    if (g_options.interval_sec <= 0)
    {
        g_options.interval_sec = 0.2;
    }

    if (g_options.mode == MODE_UNSPECIFIED)
    {
        if (g_options.target_node_id != 0)
        {
            g_options.mode = MODE_ADDRESSED;
        }
        else if (g_options.target_event_id != 0)
        {
            g_options.mode = MODE_PRODUCER;
        }
    }

    if (g_options.target_node_id == 0 && g_options.target_event_id == 0)
    {
        fprintf(stderr, "Error: Must specify either target node ID (-n) or event ID (-e).\n\n");
        usage(argv[0]);
    }

    if ((g_options.mode == MODE_ADDRESSED || g_options.mode == MODE_GLOBAL) && g_options.target_node_id == 0)
    {
        fprintf(stderr, "Error: Node ping modes (addressed/global) require a target node ID (-n).\n\n");
        usage(argv[0]);
    }

    if ((g_options.mode == MODE_PRODUCER || g_options.mode == MODE_CONSUMER) && g_options.target_event_id == 0)
    {
        fprintf(stderr, "Error: Event ping modes (producer/consumer) require a target event ID (-e).\n\n");
        usage(argv[0]);
    }
}

int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    signal(SIGINT, sigint_handler);

    openlcb::SimpleCanStack stack(g_options.local_node_id);

    if (g_options.device_path)
    {
#if defined(__linux__) || defined(__MACH__)
        stack.add_gridconnect_tty(g_options.device_path);
#else
        stack.add_gridconnect_port(g_options.device_path);
#endif
    }
    else
    {
        stack.connect_tcp_gridconnect_hub(g_options.host, g_options.port);
    }

    OlcbPingFlow flow(&stack, g_options);
    g_ping_flow = &flow;

    stack.loop_executor();
    return 0;
}
