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
 * Main file for the io board application on a host target.
 *
 * @author Balazs Racz
 * @date 21 Jun 2015
 */

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/SimpleNodeInfoMockUserFile.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "freertos_drivers/common/GpioWrapper.hxx"

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);

struct DummyPin
{
    static bool get()
    {
        return false;
    }
    static bool is_output()
    {
        return true;
    }
    static void set(bool v)
    {
        printf("Pin value: %s\n", v ? "on" : "off");
    }
};

openlcb::MockSNIPUserFile snip_user_file(
    "Default user name", "Default user description");
const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::MockSNIPUserFile::snip_user_file_path;

uint64_t node_id = 0;
const char *hostname = "localhost";
int port = 12021;

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s -n node_id [-d hostname] [-p port]\n\n", e);
    fprintf(stderr, "\t-n node_id is required, defines the OpenLCB node ID. "
                    "Example: -n 0x0501010118F0\n");
    fprintf(stderr,
        "\t-d hostname   is the host name for a GridConnect TCP hub.\n");
    fprintf(stderr, "\t-p port     specifies the port number to connect to, "
                    "default is 12021.\n");
    exit(1);
};

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:d:n:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'd':
                hostname = optarg;
                break;
            case 'n':
                node_id = strtoll(optarg, nullptr, 16);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
    if (!node_id)
    {
        usage(argv[0]);
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

    // Sets up the stack with the dynamic node ID and a fixed consumer.
    openlcb::SimpleCanStack stack(node_id);
    openlcb::GPIOBit bit(stack.node(), 0x0501010118010203, 0x0501010118010204,
                         GpioWrapper<DummyPin>::instance());
    openlcb::BitEventConsumer consumer(&bit);

    // Connects to a TCP hub.
    stack.connect_tcp_gridconnect_hub(hostname, port);
    // Causes all packets to be dumped to stdout.
    stack.print_all_packets();
    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}
