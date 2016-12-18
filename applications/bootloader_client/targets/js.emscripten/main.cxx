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
 * An application for updating the firmware of a remote node on the bus.
 *
 * @author Balazs Racz
 * @date 3 Aug 2013
 */

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <memory>

#include "executor/Executor.hxx"
#include "executor/Service.hxx"
#include "executor/StateFlow.hxx"
#include "freertos/bootloader_hal.h"
#include "openlcb/AliasAllocator.hxx"
#include "openlcb/BootloaderClient.hxx"
#include "openlcb/DatagramCan.hxx"
#include "openlcb/DefaultNode.hxx"
#include "openlcb/If.hxx"
#include "openlcb/IfCan.hxx"
#include "openlcb/NodeInitializeFlow.hxx"
#include "os/os.h"
#include "utils/Crc.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/Hub.hxx"
#include "utils/JSTcpClient.hxx"
#include "utils/FileUtils.hxx"
#include "utils/constants.hxx"

NO_THREAD nt;
Executor<1> g_executor(nt);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

static const openlcb::NodeID NODE_ID = 0x05010101181FULL;

openlcb::IfCan g_if_can(&g_executor, &can_hub0, 3, 3, 2);
openlcb::InitializeFlow g_init_flow{&g_service};
openlcb::CanDatagramService g_datagram_can(&g_if_can, 10, 2);
static openlcb::AddAliasAllocator g_alias_allocator(NODE_ID, &g_if_can);
openlcb::DefaultNode g_node(&g_if_can, NODE_ID);

namespace openlcb
{
Pool *const g_incoming_datagram_allocator = mainBufferPool;
}

int port = 12021;
const char *host = "localhost";

const char *filename = nullptr;
uint64_t destination_nodeid = 0;
uint64_t destination_alias = 0;
int memory_space_id = openlcb::MemoryConfigDefs::SPACE_FIRMWARE;
const char *checksum_algorithm = nullptr;
bool request_reboot = false;
bool request_reboot_after = true;

OVERRIDE_CONST(gc_generate_newlines, 1);

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s [-i destination_host] [-p port] [-s "
                    "memory_space_id] [-c csum_algo] [-r] [-t] (-n nodeid | -a "
                    "alias) -f filename\n",
        e);

    fprintf(stderr, "Connects to an openlcb bus and performs the "
                    "bootloader protocol on openlcb node with id nodeid with "
                    "the contents of a given file.\n");
    fprintf(stderr, "The bus connection will be through an OpenLCB HUB on "
                    "destination_host:port with OpenLCB over TCP "
                    "(in GridConnect format) protocol.");
    fprintf(stderr, "The default target is localhost:12021.\n");
    fprintf(stderr, "nodeid should be a 12-char hex string with 0x prefix and "
                    "no separators, like '-b 0x05010101141F'\n");
    fprintf(stderr, "alias should be a 3-char hex string with 0x prefix and no "
                    "separators, like '-a 0x3F9'\n");
    fprintf(stderr, "memory_space_id defines which memory space to write the "
                    "data into. Default is '-s 0xEF'.\n");
    fprintf(stderr, "csum_algo defines the checksum algorithm to use. If "
                    "omitted, no checksumming is done before writing the "
                    "data.\n");
    fprintf(stderr,
        "-r request the target to enter bootloader mode before sending data\n");
    fprintf(stderr, "Unless -t is specified the target will be rebooted after "
                    "flashing complete.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:rtn:a:s:f:c:")) >= 0)
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
            case 'c':
                checksum_algorithm = optarg;
                break;
            case 'r':
                request_reboot = true;
                break;
            case 't':
                request_reboot_after = false;
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
    if (!filename || (!destination_nodeid && !destination_alias))
    {
        usage(argv[0]);
    }
}

void maybe_checksum(string *firmware)
{
    if (!checksum_algorithm)
        return;
    string algo = checksum_algorithm;
    if (algo == "tiva123")
    {
        struct app_header hdr;
        memset(&hdr, 0, sizeof(hdr));
        // magic constant that comes from the size of the interrupt table. The
        // actual target has this in memory_map.ld.
        uint32_t offset = 0x270;
        if (firmware->size() < offset + sizeof(hdr))
        {
            fprintf(stderr, "Failed to checksum: firmware too small.\n");
            exit(1);
        }
        if (memcmp(&hdr, &(*firmware)[offset], sizeof(hdr)))
        {
            fprintf(stderr,
                "Failed to checksum: location of checksum is not empty.\n");
            exit(1);
        }
        hdr.app_size = firmware->size();
        crc3_crc16_ibm(
            &(*firmware)[8], (offset - 8) & ~3, (uint16_t *)hdr.checksum_pre);
        crc3_crc16_ibm(&(*firmware)[offset + sizeof(hdr)],
            (firmware->size() - offset - sizeof(hdr)) & ~3,
            (uint16_t *)hdr.checksum_post);
        memcpy(&(*firmware)[offset], &hdr, sizeof(hdr));
        printf("Checksummed firmware with algorithm tiva123\n");
        uint32_t reset_handler;
        memcpy(&reset_handler, firmware->data() + 52, 4);
        if (!reset_handler)
        {
            fprintf(stderr,
                "Firmware does not contain any entry vector at offset 52.\n");
            exit(1);
        }
    }
    else
    {
        fprintf(stderr,
            "Unknown checksumming algo %s. Known algorithms are: tiva123.\n",
            checksum_algorithm);
        exit(1);
    }
}

openlcb::BootloaderClient bootloader_client(
    &g_node, &g_datagram_can, &g_if_can);
openlcb::BootloaderResponse response;

class BootloaderClientStateFlow : public StateFlowBase
{
public:
    BootloaderClientStateFlow()
        : StateFlowBase(&g_service)
    {
        start_flow(STATE(wait_for_boot));
    }

private:
    Action wait_for_boot()
    {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(400), STATE(fill_request));
    }

    Action fill_request()
    {
        Buffer<openlcb::BootloaderRequest> *b;
        mainBufferPool->alloc(&b);

        b->set_done(bn_.reset(this));
        b->data()->dst.alias = destination_alias;
        b->data()->dst.id = destination_nodeid;
        b->data()->memory_space = memory_space_id;
        b->data()->offset = 0;
        b->data()->response = &response;
        b->data()->request_reboot = request_reboot ? 1 : 0;
        b->data()->request_reboot_after = request_reboot_after ? 1 : 0;
        b->data()->data = read_file_to_string(filename);

        printf("Read %" PRIdPTR
               " bytes from file %s. Writing to memory space 0x%02x\n",
            b->data()->data.size(), filename, memory_space_id);
        maybe_checksum(&b->data()->data);

        bootloader_client.send(b);
        return wait_and_call(STATE(bootload_done));
    }

    Action bootload_done()
    {
        printf("Result: %04x  %s\n", response.error_code,
            response.error_details.c_str());
        fflush(stdout);
#ifdef __EMSCRIPTEN__
        EM_ASM(process.exit());
#endif
        ::exit(0);
    }

    StateFlowTimer timer_{this};
    BarrierNotifiable bn_;
} bootload_state_flow;

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    std::unique_ptr<JSTcpClient> client;
    client.reset(new JSTcpClient(&can_hub0, host, port));

    g_if_can.add_addressed_message_support();
    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    g_executor.thread_body();
    return 0;
}
