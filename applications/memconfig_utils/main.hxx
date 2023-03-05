/** \copyright
 * Copyright (c) 2013 - 2023, Balazs Racz
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
 * \file main.hxx
 *
 * An application for downloading an entire memory space from a node.
 *
 * @author Balazs Racz
 * @date 7 Sep 2017
 */

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <unistd.h>

#include <memory>

#include "os/os.h"
#include "utils/constants.hxx"
#include "utils/Hub.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/GcTcpHub.hxx"
#include "utils/Crc.hxx"
#include "utils/FileUtils.hxx"
#include "utils/format_utils.hxx"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

#include "openlcb/IfCan.hxx"
#include "openlcb/DatagramCan.hxx"
#include "openlcb/BootloaderClient.hxx"
#include "openlcb/If.hxx"
#include "openlcb/AliasAllocator.hxx"
#include "openlcb/DefaultNode.hxx"
#include "openlcb/NodeInitializeFlow.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "openlcb/MemoryConfigClient.hxx"
#include "utils/socket_listener.hxx"

NO_THREAD nt;
Executor<1> g_executor(nt);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

OVERRIDE_CONST(gc_generate_newlines, 1);

static const openlcb::NodeID NODE_ID = 0x05010101181FULL;

openlcb::IfCan g_if_can(&g_executor, &can_hub0, 3, 3, 2);
openlcb::InitializeFlow g_init_flow{&g_service};
openlcb::CanDatagramService g_datagram_can(&g_if_can, 10, 2);
static openlcb::AddAliasAllocator g_alias_allocator(NODE_ID, &g_if_can);
openlcb::DefaultNode g_node(&g_if_can, NODE_ID);
openlcb::MemoryConfigHandler g_memcfg(&g_datagram_can, &g_node, 10);
openlcb::MemoryConfigClient g_memcfg_cli(&g_node, &g_memcfg);

namespace openlcb
{
Pool *const g_incoming_datagram_allocator = mainBufferPool;
}

static int port = 12021;
static const char *host = "localhost";
static const char *device_path = nullptr;
static const char *filename = nullptr;
static uint64_t destination_nodeid = 0;
static uint64_t destination_alias = 0;
static int memory_space_id = openlcb::MemoryConfigDefs::SPACE_CONFIG;
static uint32_t offset = 0;
static constexpr uint32_t NLEN = (uint32_t)-1;
static uint32_t len = NLEN;
static bool partial_read = false;
static bool do_read = false;
static bool do_write = false;

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s ([-i destination_host] [-p port] | [-d serial_port]) [-s "
        "memory_space_id] [-o offset] [-l len] [-c csum_algo] (-r|-w)  "
        "(-n nodeid | -a alias) -f filename\n",
        e);
    fprintf(stderr,
        "Connects to an openlcb bus and performs memory configuration protocol "
        "operations on openlcb node with id `nodeid` with the contents of a "
        "given file or arguments.\n");
    fprintf(stderr,
        "The bus connection will be through an OpenLCB HUB on "
        "destination_host:port with OpenLCB over TCP "
        "(in GridConnect format) protocol, or through the CAN-USB device "
        "(also in GridConnect protocol) found at serial_port. Device takes "
        "precedence over TCP host:port specification.");
    fprintf(stderr, "\tThe default target is localhost:12021.\n");
    fprintf(stderr, "\tnodeid should be a 12-char hex string with 0x prefix and "
                    "no separators, like '-n 0x05010101141F'\n");
    fprintf(stderr, "\talias should be a 3-char hex string with 0x prefix and no "
                    "separators, like '-a 0x3F9'\n");
    fprintf(stderr,
        "\tmemory_space_id defines which memory space to use "
        "data into. Default is '-s 0x%02x'.\n",
        openlcb::MemoryConfigDefs::SPACE_CONFIG);
    fprintf(stderr, "\t-r or -w  defines whether to read or write.\n");
    fprintf(stderr,
        "\tIf offset and len are skipped for a read, then the entire memory "
        "space will be downloaded.\n");
#ifdef __EMSCRIPTEN__
    fprintf(stderr, "\t-D lists available serial ports.\n");
#endif    
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:i:d:n:a:s:f:rwo:l:D")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'i':
                host = optarg;
                break;
            case 'd':
                device_path = optarg;
                break;
            case 'f':
                filename = optarg;
                break;
            case 'n':
                destination_nodeid = strtoll(optarg, nullptr, 16);
                break;
            case 'a':
                destination_alias = strtoul(optarg, nullptr, 16);
                break;
            case 's':
                memory_space_id = strtol(optarg, nullptr, 16);
                break;
            case 'o':
                offset = strtol(optarg, nullptr, 10);
                break;
            case 'l':
                len = strtol(optarg, nullptr, 10);
                break;
            case 'r':
                do_read = true;
                break;
            case 'w':
                do_write = true;
                break;
#ifdef __EMSCRIPTEN__
            case 'D':
                JSSerialPort::list_ports();
                break;
#endif                
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
    partial_read = do_read && ((offset != 0) || (len != NLEN));
    if ((!filename && !partial_read) ||
        (!destination_nodeid && !destination_alias))
    {
        usage(argv[0]);
    }
    if ((do_read ? 1 : 0) + (do_write ? 1 : 0) != 1)
    {
        fprintf(stderr, "Must set exactly one of option -r and option -w.\n\n");
        usage(argv[0]);
    }
}


class HelperFlow : public StateFlowBase {
public:
    HelperFlow() : StateFlowBase(&g_service) {
        start_flow(STATE(wait_for_boot));
    }

    Action wait_for_boot()
    {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(400), STATE(send_request));
    }

    /// Application business logic.
    Action send_request()
    {
        openlcb::NodeHandle dst;
        dst.alias = destination_alias;
        dst.id = destination_nodeid;
        HASSERT((!!do_read) + (!!do_write) == 1);
        if (do_write)
        {
            auto payload = read_file_to_string(filename);
            printf("Read %" PRIdPTR
                   " bytes from file %s. Writing to memory space 0x%02x\n",
                payload.size(), filename, memory_space_id);
            return invoke_subflow_and_wait(&g_memcfg_cli, STATE(write_done),
                openlcb::MemoryConfigClientRequest::WRITE, dst, memory_space_id,
                0, std::move(payload));
        }

        if (do_read && partial_read)
        {
            printf("Loading from space 0x%02x offset %u length %d\n",
                   (unsigned)memory_space_id, (unsigned)offset, (int)len);
            return invoke_subflow_and_wait(&g_memcfg_cli, STATE(part_read_done),
                openlcb::MemoryConfigClientRequest::READ_PART, dst,
                memory_space_id, offset, len);
        }
        else if (do_read)
        {
            auto cb = [](openlcb::MemoryConfigClientRequest *rq) {
                static size_t last_len = rq->payload.size();
                if ((last_len & ~1023) != (rq->payload.size() & ~1023))
                {
                    printf("Loaded %d bytes\n", (int)rq->payload.size());
                    last_len = rq->payload.size();
                }
            };
            printf("Loading memory space 0x%02x\n",
                   (unsigned)memory_space_id);
            return invoke_subflow_and_wait(&g_memcfg_cli, STATE(read_done),
                openlcb::MemoryConfigClientRequest::READ, dst, memory_space_id,
                std::move(cb));
        }

        printf("Nothing to do.");
        return call_immediately(STATE(flow_done));
    }        

    /// Invoked when a write operation is complete. Prints result and
    /// terminates.
    Action write_done() {
        auto b = get_buffer_deleter(full_allocation_result(&g_memcfg_cli));
        printf("Result: %04x\n", b->data()->resultCode);
        hasError_ = b->data()->resultCode != 0;
        return call_immediately(STATE(flow_done));
    }

    /// Invoked when a partial read operation is complete. Prints result and
    /// terminates.
    Action part_read_done() {
        auto b = get_buffer_deleter(full_allocation_result(&g_memcfg_cli));
        printf("Result: %04x\n", b->data()->resultCode);
        printf("Data: %s\n", string_to_hex(b->data()->payload).c_str());
        hasError_ = b->data()->resultCode != 0;
        return call_immediately(STATE(flow_done));
    }

    /// Invoked when a read operation is complete. Prints result and
    /// terminates.
    Action read_done() {
        auto b = get_buffer_deleter(full_allocation_result(&g_memcfg_cli));
        printf("Result: %04x\n", b->data()->resultCode);
        write_string_to_file(filename, b->data()->payload);
        fprintf(stderr, "Written %" PRIdPTR " bytes to file %s.\n",
                b->data()->payload.size(), filename);
        hasError_ = b->data()->resultCode != 0;
        return call_immediately(STATE(flow_done));
    }

    /// Terminates the process.
    Action flow_done()
    {
        fflush(stdout);
#ifdef __EMSCRIPTEN__
        EM_ASM(process.exit());
#endif
        _exit(hasError_ ? 1 : 0);

        return exit();
    }

    StateFlowTimer timer_{this};
    bool hasError_ = false;
} helper_flow;


/// Runs the executor. Never returns.
void execute() {
    g_if_can.add_addressed_message_support();
    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    g_executor.thread_body();
}
