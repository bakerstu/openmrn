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
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

#include "nmranet/IfCan.hxx"
#include "nmranet/DatagramCan.hxx"
#include "nmranet/BootloaderClient.hxx"
#include "nmranet/If.hxx"
#include "nmranet/AliasAllocator.hxx"
#include "nmranet/DefaultNode.hxx"
#include "nmranet/NodeInitializeFlow.hxx"
#include "utils/socket_listener.hxx"

#include "freertos/bootloader_hal.h"

NO_THREAD nt;
Executor<1> g_executor(nt);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

static const nmranet::NodeID NODE_ID = 0x05010101181FULL;

nmranet::IfCan g_if_can(&g_executor, &can_hub0, 3, 3, 2);
nmranet::InitializeFlow g_init_flow{&g_service};
nmranet::CanDatagramService g_datagram_can(&g_if_can, 10, 2);
static nmranet::AddAliasAllocator g_alias_allocator(NODE_ID, &g_if_can);
nmranet::DefaultNode g_node(&g_if_can, NODE_ID);

namespace nmranet
{
Pool *const g_incoming_datagram_allocator = mainBufferPool;
extern long long DATAGRAM_RESPONSE_TIMEOUT_NSEC;
}

int port = 12021;
const char *host = "localhost";
const char *device_path = nullptr;
const char *filename = nullptr;
uint64_t destination_nodeid = 0;
uint64_t destination_alias = 0;
int memory_space_id = nmranet::MemoryConfigDefs::SPACE_FIRMWARE;
const char *checksum_algorithm = nullptr;
bool request_reboot = false;
bool request_reboot_after = true;
bool skip_pip = false;

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s ([-i destination_host] [-p port] | [-d device_path]) [-s "
        "memory_space_id] [-c csum_algo] [-r] [-t] [-x] [-w dg_timeout] (-n nodeid | -a "
        "alias) -f filename\n",
        e);
    fprintf(stderr, "Connects to an openlcb bus and performs the "
                    "bootloader protocol on openlcb node with id nodeid with "
                    "the contents of a given file.\n");
    fprintf(stderr,
        "The bus connection will be through an OpenLCB HUB on "
        "destination_host:port with OpenLCB over TCP "
        "(in GridConnect format) protocol, or through the CAN-USB device "
        "(also in GridConnect protocol) found at device_path. Device takes "
        "precedence over TCP host:port specification.");
    fprintf(stderr, "The default target is localhost:12021.\n");
    fprintf(stderr, "nodeid should be a 12-char hex string with 0x prefix and "
                    "no separators, like '-b 0x05010101141F'\n");
    fprintf(stderr, "alias should be a 3-char hex string with 0x prefix and no "
                    "separators, like '-a 0x3F9'\n");
    fprintf(stderr, "memory_space_id defines which memory space to write the "
                    "data into. Default is '-s 0xF0'.\n");
    fprintf(stderr, "csum_algo defines the checksum algorithm to use. If "
                    "omitted, no checksumming is done before writing the "
                    "data.\n");
    fprintf(stderr,
        "-r request the target to enter bootloader mode before sending data\n");
    fprintf(stderr, "Unless -t is specified the target will be rebooted after "
                    "flashing complete.\n");
    fprintf(stderr, "-x skips the PIP request and uses streams.\n");
    fprintf(stderr,
        "-w dg_timeout sets how many seconds to wait for a datagram reply.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:i:rtd:n:a:s:f:c:xw:")) >= 0)
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
            case 'w':
                nmranet::DATAGRAM_RESPONSE_TIMEOUT_NSEC = SEC_TO_NSEC(strtoul(optarg, nullptr, 10));
                break;
            case 'c':
                checksum_algorithm = optarg;
                break;
            case 'r':
                request_reboot = true;
                break;
            case 'x':
                skip_pip = true;
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

nmranet::BootloaderClient bootloader_client(
    &g_node, &g_datagram_can, &g_if_can);
nmranet::BootloaderResponse response;

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
        if (!reset_handler) {
            fprintf(stderr,
                "Firmware does not contain any entry vector at offset 52.\n");
            exit(1);
        }
    }
    else if (algo == "esp8266")
    {
        struct app_header hdr;
        memset(&hdr, 0, sizeof(hdr));
        string nfirmware(4096, 0);
        nfirmware += *firmware;

        hdr.app_size = firmware->size() + 4096;
        crc3_crc16_ibm(
            nullptr, 0, (uint16_t *)hdr.checksum_pre);
        hdr.checksum_pre[2] = 0x73a92bd1;
        hdr.checksum_pre[3] = 0x5a5a55aa;

        unsigned post_size = hdr.app_size - sizeof(hdr);
        crc3_crc16_ibm(
            &nfirmware[sizeof(hdr)], post_size, (uint16_t *)hdr.checksum_post);
        hdr.checksum_post[2] = 0x73a92bd1;
        hdr.checksum_post[3] = 0x5a5a55aa;

        memcpy(&nfirmware[0], &hdr, sizeof(hdr));
        swap(*firmware, nfirmware);
        printf("Checksummed firmware with algorithm esp8266 (p sz %u\n", post_size);
    }
    else
    {
        fprintf(stderr, "Unknown checksumming algo %s. Known algorithms are: "
                        "tiva123, esp8266.\n",
            checksum_algorithm);
        exit(1);
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
    int conn_fd = 0;
    if (device_path)
    {
        conn_fd = ::open(device_path, O_RDWR);
    }
    else
    {
        conn_fd = ConnectSocket(host, port);
    }
    HASSERT(conn_fd >= 0);
    create_gc_port_for_can_hub(&can_hub0, conn_fd);

    g_if_can.add_addressed_message_support();
    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    g_executor.start_thread("g_executor", 0, 1024);
    usleep(400000);

    SyncNotifiable n;
    BarrierNotifiable bn(&n);
    Buffer<nmranet::BootloaderRequest> *b;
    mainBufferPool->alloc(&b);

    b->set_done(&bn);
    b->data()->dst.alias = destination_alias;
    b->data()->dst.id = destination_nodeid;
    b->data()->memory_space = memory_space_id;
    b->data()->offset = 0;
    b->data()->response = &response;
    b->data()->request_reboot = request_reboot ? 1 : 0;
    b->data()->request_reboot_after = request_reboot_after ? 1 : 0;
    b->data()->skip_pip = skip_pip ? 1 : 0;
    b->data()->data = read_file_to_string(filename);

    printf("Read %" PRIdPTR
           " bytes from file %s. Writing to memory space 0x%02x\n",
        b->data()->data.size(), filename, memory_space_id);
    maybe_checksum(&b->data()->data);

    bootloader_client.send(b);
    n.wait_for_notification();
    printf("Result: %04x  %s\n", response.error_code,
        response.error_details.c_str());

    return 0;
}
