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

#include "freertos/bootloader_hal.h"

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
static bool do_read = false;
static bool do_write = false;

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s ([-i destination_host] [-p port] | [-d device_path]) [-s "
        "memory_space_id] [-c csum_algo] (-r|-w)  (-n nodeid | -a "
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
    fprintf(stderr, "memory_space_id defines which memory space to use "
                    "data into. Default is '-s 0xF0'.\n");
    fprintf(stderr, "-r or -w  defines whether to read or write.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:d:n:a:s:f:rw")) >= 0)
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
            case 'r':
                do_read = true;
                break;
            case 'w':
                do_write = true;
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
    if ((do_read ? 1 : 0) + (do_write ? 1 : 0) != 1)
    {
        fprintf(stderr, "Must set exactly one of option -r and option -w.\n\n");
        usage(argv[0]);
    }
}


/*
namespace openlcb
{

struct MemConfigUtilsResponse;

struct MemConfigUtilsRequest
{
    enum Command
    {
        READ,
        WRITE
    };
    Command cmd;
    /// Node to send the bootload request to.
    NodeHandle dst;
    uint8_t memory_space;
    string payload;
    MemConfigUtilsResponse *response = nullptr;
};

struct MemConfigUtilsResponse
{
    int error_code = 0;
    string error_details;
    string payload;
};

class MemConfigUtilsFlow
    : public StateFlow<Buffer<MemConfigUtilsRequest>, QList<1>>
{
    static constexpr unsigned SIZE = 64;

public:
    MemConfigUtilsFlow(Node *node, DatagramService *datagram_service)
        : StateFlow<Buffer<MemConfigUtilsRequest>, QList<1>>(datagram_service)
        , node_(node)
        , datagramService_(datagram_service)
    {
    }

    Action entry() override
    {
        return allocate_and_call(
            STATE(got_dg_client), datagramService_->client_allocator());
    }

    Action got_dg_client()
    {
        dgClient_ =
            full_allocation_result(datagramService_->client_allocator());
        offset = 0;

        if (request()->cmd == MemConfigUtilsRequest::READ)
        {
            return call_immediately(STATE(send_read_dg));
        }
        else HASSERT(0);

            if (request()->cmd == MemConfigUtilsRequest::WRITE)
        {
            return call_immediately(STATE(send_write_dg));
            }
    }

    /// Datagram handler that listens to the incoming memoryconfig datagram for
    /// the write stream response message.
    class ResponseHandler : public DefaultDatagramHandler
    {
    public:
        ResponseHandler(MemConfigUtilsFlow *parent)
            : DefaultDatagramHandler(parent->datagram_service())
            , parent_(parent)
        {
        }

        Action entry() override
        {
            IncomingDatagram *datagram = message()->data();

            if (datagram->dst != parent_->node() ||
                !parent_->node()->iface()->matching_node(
                    parent_->dst(), datagram->src) ||
                datagram->payload.size() < 6 ||
                datagram->payload[0] != DatagramDefs::CONFIGURATION ||
                ((datagram->payload[1] & 0xF4) !=
                    MemoryConfigDefs::COMMAND_WRITE_STREAM_REPLY))
            {
                // Uninteresting datagram.
                return respond_reject(DatagramDefs::PERMANENT_ERROR);
            }
            return respond_ok(DatagramDefs::FLAGS_NONE);
        }

        Action ok_response_sent() override
        {
            parent_->response_datagram_arrived(transfer_message());
            return exit();
        }

    private:
        MemConfigUtilsFlow *parent_;
    } responseHandler_{this};

    void response_datagram_arrived(Buffer<IncomingDatagram> *datagram)
    {
        if (responseDatagram_)
        {
            LOG_ERROR("Multiple response datagrams arrived from the "
                       "target node.");
            responseDatagram_->unref();
        }
        responseDatagram_ = datagram;
        if (sleeping_)
        {
            timer_.trigger();
        } // else we will be woken up by the datagram client.
    }

    void register_response_handler()
    {
        datagramService_->registry()->insert(
            node_, DatagramDefs::CONFIGURATION, &responseHandler_);
        responseRegistered_ = true;
    }

    void unregister_response_handler()
    {
        if (responseRegistered_)
        {
            responseRegistered_ = false;
            datagramService_->registry()->erase(
                node_, DatagramDefs::CONFIGURATION, &responseHandler_);
        }
    }

    Action send_read_dg()
    {
        if (offset >= request()->payload.size())
        {
            return completed();
        }

        Buffer<GenMessage> *b;
        mainBufferPool->alloc(&b);
        DatagramPayload payload;
        payload.push_back(DatagramDefs::CONFIGURATION);
        payload.push_back(MemoryConfigDefs::COMMAND_READ);
        payload.push_back(offset >> 24);
        payload.push_back(offset >> 16);
        payload.push_back(offset >> 8);
        payload.push_back(offset);
        payload.push_back(request()->memory_space);
        payload.push_back(SIZE);
        b->data()->reset(Defs::MTI_DATAGRAM, node_->node_id(),
            message()->data()->dst, payload);
        b->set_done(n_.reset(this));

        register_response_handler();

        dgClient_->write_datagram(b);
        return wait_and_call(STATE(read_sent));
    }

    Action read_sent() {
        uint32_t dg_result =
            dgClient_->result() & DatagramClient::RESPONSE_CODE_MASK;
        if (dg_result != DatagramClient::OPERATION_SUCCESS) {
            return complete(dg_result, "Read rejected.");
        }
        if (responseDatagram_) {
            return call_immediately(STATE(read_done));
        } else {
            sleeping_ = true;
            return sleep_and_call(&timer_, SEC_TO_NSEC(3), STATE(read_done));
        }
    }

    Action read_done()
    {
        unregister_response_handler();
        sleeping_ = false;
        if (!responseDatagram_)
        {
            return return_error(DatagramClient::RESEND_OK,
                "Timed out waiting for response datagram.");
        }
        const auto &payload = responseDatagram_->data()->payload;
        if ((payload[1] & 0xFC) == MemoryConfigDefs::COMMAND_READ_FAILED)
        {
            uint16_t error_code = DatagramClient::PERMANENT_ERROR;
            unsigned error_ofs = 6;
            if (payload[1] == MemoryConfigDefs::COMMAND_READ_FAILED)
            {
                ++error_ofs;
            }
            LOG(WARNING,
                "payload length %" PRIdPTR " error offset %u data %02x %02x",
                payload.size(), error_ofs, payload[error_ofs],
                payload[error_ofs + 1]);
            error_code =
                (payload[error_ofs] << 8) | ((uint8_t)payload[error_ofs + 1]);
            error_ofs += 2;
            return complete(
                error_code, "Read rejected " + payload.substr(error_ofs));
        }
        if ((payload[1] & 0xFC) == MemoryConfigDefs::COMMAND_READ_REPLY)
        {
            unsigned data_ofs =
                (payload[1] == MemoryConfigDefs::COMMAND_READ_REPLY ? 7 : 6);
            string part = payload.substr(data_ofs);
            response()->payload.append(part);
            LOG(WARNING, "Offset %zd received data ", offset);
            offset = response()->payload.size();
            return call_immediately(send_read_dg());
        }
    }

    Action completed(int code = 0, string details = "")
    {
        datagramService_->client_allocator()->typed_insert(dgClient_);
        response()->error_code = code;
        response()->error_details = details;
        return release_and_exit();
    }

    MemConfigUtilsRequest *request()
    {
        return message()->data();
    }

    MemConfigUtilsRequest *response()
    {
        return request()->response;
    }

    const NodeHandle& dst() {
        return request()->dst;
    }

    Node* node() { return node_; }

    DatagramService *datagram_service() { return datagramService_; }
    
private:
    Node *node_;
    DatagramService *datagramService_;
    DatagramClient* dgClient_;
    size_t offset;
    StateFlowTimer timer_{this};
    Buffer<IncomingDatagram> *responseDatagram_ = nullptr;
    bool sleeping_ = false;
};

} // namespace openlcb

openlcb::MemConfigUtilsFlow flow(&g_node, &g_datagram_can);

*/


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

    openlcb::NodeHandle dst;
    dst.alias = destination_alias;
    dst.id = destination_nodeid;
    HASSERT(do_read);
    auto b = invoke_flow(&g_memcfg_cli, openlcb::MemoryConfigClientRequest::READ, dst, memory_space_id);

    if (0 && do_write)
    {
        b->data()->payload = read_file_to_string(filename);
        //b->data()->cmd = openlcb::MemoryConfigClientRequest::WRITE;
        printf("Read %" PRIdPTR
               " bytes from file %s. Writing to memory space 0x%02x\n",
            b->data()->payload.size(), filename, memory_space_id);
    }

    printf("Result: %04x\n", b->data()->resultCode);

    if (do_read)
    {
        write_string_to_file(filename, b->data()->payload);
        fprintf(stderr, "Written %" PRIdPTR " bytes to file %s.\n",
            b->data()->payload.size(), filename);
    }

    return 0;
}
