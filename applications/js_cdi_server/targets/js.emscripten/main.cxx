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

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <memory>

#include "os/os.h"
#include "utils/constants.hxx"
#include "utils/Hub.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/GcTcpHub.hxx"
#include "utils/JSTcpHub.hxx"
#include "utils/JSTcpClient.hxx"
#include "utils/FileUtils.hxx"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/SimpleNodeInfoMockUserFile.hxx"

namespace openlcb {
extern const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,               "OpenMRN", "CDI test server",
    "node.js", "1.00"};
}

openlcb::MockSNIPUserFile snip_user_file(
    "Default user name", "Default user description");
const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::MockSNIPUserFile::snip_user_file_path;

const uint64_t node_id_base = 0x050101011800ULL;
uint64_t node_id = node_id_base | 0xF3;

OVERRIDE_CONST(gc_generate_newlines, 1);

int port = -1;
int upstream_port = 12021;
const char *upstream_host = nullptr;
const char *cdi_file = nullptr;

namespace openlcb {
/// This symbol contains the embedded text of the CDI xml file.
const char CDI_DATA[128*1024] = {0,};
}

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s [-p port] [-u hub_host [-q hub_port]] [-n id] -x cdi_file\n\n", e);
    fprintf(stderr, "\n\nArguments:\n");
    fprintf(stderr,
            "\t-c filename   is the filename for the CDI (xml text).\n");
    fprintf(stderr, "\t-p port     Opens a TCP server on this port and listens for clients connecting with a GridConnect protocol (like a hub).\n");
    /*    fprintf(stderr, "\t-d device   is a path to a physical device doing "
                        "serial-CAN or USB-CAN. If specified, opens device and "
                        "adds it to the hub.\n");*/
    fprintf(stderr, "\t-u upstream_host   is the host name for a TCP GridConnect "
                    "hub. If specified, this program will connect to that hub for the CAN-bus.\n");
    fprintf(stderr,
            "\t-q upstream_port   is the port number for the upstream hub.\n");
    fprintf(stderr,
            "\t-n id   sets the lowest 8 bits of the node id. Valid values: 0..255. Default: 243.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:u:q:x:n:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            /*            case 'd':
                            device_path = optarg;
                            break;*/
            case 'p':
                port = atoi(optarg);
                break;
            case 'u':
                upstream_host = optarg;
                break;
            case 'q':
                upstream_port = atoi(optarg);
                break;
            case 'n':
                node_id = node_id_base + atoi(optarg);
                break;
            case 'x':
                cdi_file = optarg;
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
    if (!cdi_file) {
        fprintf(stderr, "cdi_file is not specified\n");
        usage(argv[0]);
    }
    string contents = read_file_to_string(cdi_file);
    if (contents.size() + 1 <= sizeof(openlcb::CDI_DATA)) {
        memcpy((char*)openlcb::CDI_DATA, contents.c_str(), contents.size() + 1);
    } else {
        DIE("CDI file too large.");
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
    openlcb::SimpleCanStack stack(node_id);
    const size_t configlen = 16*1024;
    uint8_t* cdispace = new uint8_t[configlen];
    memset(cdispace, 0, configlen);
    openlcb::ReadWriteMemoryBlock ramblock(cdispace, configlen);
    stack.memory_config_handler()->registry()->insert(nullptr, openlcb::MemoryConfigDefs::SPACE_CONFIG, &ramblock);
    GcPacketPrinter packet_printer(stack.can_hub(), false);
    std::unique_ptr<JSTcpHub> hub;
    if (port > 0) {
        hub.reset(new JSTcpHub(stack.can_hub(), port));
    }
    std::unique_ptr<JSTcpClient> client;
    if (upstream_host) {
        client.reset(new JSTcpClient(stack.can_hub(), upstream_host, upstream_port));
    }
    /*    int dev_fd = 0;
    while (1)
    {
        if (device_path && !dev_fd)
        {
            dev_fd = ::open(device_path, O_RDWR);
            if (dev_fd > 0)
            {
                // Sets up the terminal in raw mode. Otherwise linux might echo
                // characters coming in from the device and that will make
                // packets go back to where they came from.
                HASSERT(!tcflush(dev_fd, TCIOFLUSH));
                struct termios settings;
                HASSERT(!tcgetattr(dev_fd, &settings));
                cfmakeraw(&settings);
                HASSERT(!tcsetattr(dev_fd, TCSANOW, &settings));
                LOG(INFO, "Opened device %s.\n", device_path);
                create_gc_port_for_can_hub(&can_hub0, dev_fd);
            }
            else
            {
                LOG(ERROR, "Failed to open device %s: %s\n", device_path,
                    strerror(errno));
            }
        }
        sleep(1);
        }*/
    stack.loop_executor();
    return 0;
}
