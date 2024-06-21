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
 * Main file for the load generator application for the load-test suite.
 *
 * @author Balazs Racz
 * @date 5 Jun 2015
 */

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"

#include "config.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/LoggingGPIO.hxx"
#include "utils/ClientConnection.hxx"

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 2500);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID = 0x050101011419ULL;

// Sets up a comprehensive OpenLCB stack for a single virtual node. This stack
// contains everything needed for a usual peripheral node -- all
// CAN-bus-specific components, a virtual node, PIP, SNIP, Memory configuration
// protocol, ACDI, CDI, a bunch of memory spaces, etc.
openlcb::SimpleCanStack stack(NODE_ID);

// ConfigDef comes from config.hxx and is specific to the particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
openlcb::ConfigDef cfg(0);
// Defines weak constants used by the stack to tell it which device contains
// the volatile configuration information. This device name appears in
// HwInit.cxx that creates the device drivers.
extern const char *const openlcb::CONFIG_FILENAME = "/tmp/load_config_eeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE = 256;
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

// None of these pins exist in Linux.
typedef DummyPinWithRead LED_RED_Pin;
typedef DummyPinWithRead LED_GREEN_Pin;
typedef DummyPinWithRead LED_BLUE_Pin;
typedef DummyPinWithRead SW1_Pin;
typedef DummyPinWithRead SW2_Pin;


const char *device_path = nullptr;
int upstream_port = 12021;
const char *upstream_host = nullptr;
int pkt_per_sec = 0;

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s [-d device_path] [-u upstream_host] "
                    "[-q upstream_port] -s speed\n\n",
            e);
    fprintf(stderr, "\t-d device   is a path to a physical device doing "
                    "serial-CAN or USB-CAN. If specified, opens device and "
                    "adds it to the hub.\n");
    fprintf(stderr, "\t-u upstream_host   is the host name for an upstream "
                    "hub. If specified, this hub will connect to an upstream "
                    "hub.\n");
    fprintf(stderr,
            "\t-q upstream_port   is the port number for the upstream hub.\n");
    fprintf(stderr,
            "\t-s speed   is the packets/sec to generate.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hd:u:q:s:")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'd':
                device_path = optarg;
                break;
            case 'u':
                upstream_host = optarg;
                break;
            case 'q':
                upstream_port = atoi(optarg);
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

class PacketGenTimer : public ::Timer {
public:
    PacketGenTimer() : Timer(stack.executor()->active_timers()) {}

    long long timeout() override {
        stack.send_event(0x0501010114DD1234);
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
    parse_args(argc, argv);
    vector<std::unique_ptr<ConnectionClient>> connections;

    if (upstream_host)
    {
        connections.emplace_back(new UpstreamConnectionClient(
            "upstream", stack.can_hub(), upstream_host, upstream_port));
    }

    if (device_path)
    {
        connections.emplace_back(
            new DeviceConnectionClient("device", stack.can_hub(), device_path));
    }

    stack.create_config_file_if_needed(cfg.seg().internal_config(), openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

    stack.start_executor_thread("executor_thread", 0, 5000);

    if (pkt_per_sec > 0) {
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
