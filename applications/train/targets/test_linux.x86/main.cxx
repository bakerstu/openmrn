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
 * A simple application to demonstrate a train node.
 *
 * @author Balazs Racz
 * @date 1 Feb 2015
 */

#define LOGLEVEL INFO

#include <unistd.h>
#include <fcntl.h>

#include <memory>

#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/ProtocolIdentification.hxx"
#include "openlcb/SimpleNodeInfo.hxx"
#include "openlcb/SimpleNodeInfoMockUserFile.hxx"
#include "openlcb/SimpleStack.hxx"
#include "openlcb/TractionTestTrain.hxx"
#include "openlcb/TractionTrain.hxx"
#include "os/os.h"
#include "utils/constants.hxx"
#include "config.hxx"

static const openlcb::NodeID NODE_ID = 0x0501010118F5ULL;

openlcb::ConfigDef cfg(0);
namespace openlcb
{
extern const char *const CONFIG_FILENAME = "openlcb_test_train_config";
// The size of the memory space to export over the above device.
extern const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();
extern const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}
const char kFdiXml[] =
    R"(<?xml version='1.0' encoding='UTF-8'?>
<?xml-stylesheet type='text/xsl' href='xslt/fdi.xsl'?>
<fdi xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xsi:noNamespaceSchemaLocation='http://openlcb.org/trunk/prototypes/xml/schema/fdi.xsd'>
<segment space='249'><group><name/>
<function size='1' kind='binary'>
<name>Light</name>
<number>0</number>
</function>
<function size='1' kind='momentary'>
<name>Blue</name>
<number>1</number>
</function>
</group></segment></fdi>)";

openlcb::LoggingTrain trainImpl(1732);
openlcb::SimpleTrainCanStack stack(&trainImpl, kFdiXml, NODE_ID);

int port = 12021;
const char *host = "localhost";
const char *device_path = nullptr;
int address = 1726;
OVERRIDE_CONST(num_memory_spaces, 6);

void usage(const char *e)
{
    fprintf(stderr,
            "Usage: %s ([-i destination_host] [-p port] | [-d device_path]) "
            "           [-a address]\n",
            e);
    fprintf(stderr,
            "Connects to an openlcb bus and exports a virtual train.\n");
    fprintf(stderr,
            "The bus connection will be through an OpenLCB HUB on "
            "destination_host:port with OpenLCB over TCP "
            "(in GridConnect format) protocol, or through the CAN-USB device "
            "(also in GridConnect protocol) found at device_path. Device takes "
            "precedence over TCP host:port specification.");
    fprintf(stderr, "The default target is localhost:12021.\n");
    fprintf(stderr, "Address is the virtual loco address. Default 1726.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:i:d:a:")) >= 0)
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
            case 'a':
                address = atoi(optarg);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

int appl_main(int argc, char *argv[])
{
    stack.create_config_file_if_needed(cfg.seg().internal_data(),
                                       openlcb::EXPECTED_VERSION, openlcb::CONFIG_FILE_SIZE);
    parse_args(argc, argv);
    if (device_path)
    {
        stack.add_gridconnect_port(device_path);
    }
    else
    {
        stack.connect_tcp_gridconnect_hub(host, port);
    }

    stack.loop_executor();
    return 0;
}
