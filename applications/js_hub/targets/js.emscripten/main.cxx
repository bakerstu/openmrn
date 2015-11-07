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
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

Executor<1> g_executor{NO_THREAD()};
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);
GcPacketPrinter packet_printer(&can_hub0, false);

OVERRIDE_CONST(gc_generate_newlines, 1);

int port = 12021;
const char *device_path = nullptr;
const char *static_dir = "";
int upstream_port = 12021;
const char *upstream_host = nullptr;

int ws_port = -1;

void usage(const char *e)
{
    fprintf(stderr,
        "Usage: %s [-p port] [-d device_path] [-w websocket_port [-l path_to_web]] [-u upstream_host [-q upstream_port]]\n\n", e);
    fprintf(stderr, "GridConnect CAN HUB.\nListens to a specific TCP port, "
                    "reads CAN packets from the incoming connections using "
                    "the GridConnect protocol, and forwards all incoming "
                    "packets to all other participants.\n\nArguments:\n");
    fprintf(stderr, "\t-p port     specifies the port number to listen on, "
                    "default is 12021.\n");
    /*    fprintf(stderr, "\t-d device   is a path to a physical device doing "
                        "serial-CAN or USB-CAN. If specified, opens device and "
                        "adds it to the hub.\n");*/
    fprintf(stderr, "\t-w websocket_port     Opens a webserver on this port "
                    "and serves up a websocket connection to the same "
                    "CAN-bus.\n");
    fprintf(stderr, "\t-l path_to_web     Exports the contents of this directory thorugh the websocket's webserver.\n");
    fprintf(stderr, "\t-u upstream_host   is the host name for an upstream "
                    "hub. If specified, this hub will connect to an upstream "
                    "hub.\n");
    fprintf(stderr,
            "\t-q upstream_port   is the port number for the upstream hub.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:w:l:u:q:")) >= 0)
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
            case 'w':
                ws_port = atoi(optarg);
                break;
            case 'l':
                static_dir = optarg;
                break;
            case 'u':
                upstream_host = optarg;
                break;
            case 'q':
                upstream_port = atoi(optarg);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

class JSWebsocketServer
{
public:
    JSWebsocketServer(CanHubFlow *hflow, int port, string static_dir)
        : canHub_(hflow)
    {
        if (!static_dir.empty()) {
            string script = "Module.static_dir = '" + static_dir + "';\n";
            emscripten_run_script(script.c_str());
        }
        EM_ASM_(
            {
                var WebSocketServer = require('websocket').server;
                var http = require('http');
                var ecstatic = require('ecstatic');
                if (Module.static_dir) {
                    var serverImpl = ecstatic({ root: Module.static_dir });
                } else {
                    var serverImpl = function(request, response){
                    // process HTTP request. Since we're writing just
                    // WebSockets server we don't have to implement anything.
                    };
                }
                var server = http.createServer(serverImpl);
                console.log('try to listen on ', $0);
                server.listen($0, function()
                    {
                        console.log(
                            'websocket server: listening on port ' + $0);
                    });
                console.log('ws: listen done ', $0);

                // create the server
                wsServer = new WebSocketServer({httpServer : server});

                // WebSocket server
                wsServer.on('request', function(request)
                    {
                        var connection = request.accept(null, request.origin);
                        var client_port = new Module.JSHubPort($1,
                            function(gc_text)
                            {
                                var json = JSON.stringify(
                                    {type : 'gc_can_frame', data : gc_text});
                                connection.sendUTF(json);
                            });
                        connection.on('message', function(message)
                            {
                                try
                                {
                                    var json = JSON.parse(message.utf8Data);
                                }
                                catch (e)
                                {
                                    console.log(
                                        'This doesn\'t look like a valid JSON: ',
                                        message.data, ' raw msg ' ,message);
                                    return;
                                }
                                if (json.type === 'gc_can_frame')
                                {
                                    // Send can frame data to the hub port
                                    client_port.recv(json.data);
                                } else {
                                    console.log('Unknown type ', message.type);
                                }
                            });
                        connection.on('close', function(connection)
                            {
                                console.log('websocket client disconnected');
                                client_port.delete();
                            });
                    });
            },
            port, (unsigned long)canHub_);
    }

private:
    CanHubFlow *canHub_;
};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    JSTcpHub hub(&can_hub0, port);
    std::unique_ptr<JSWebsocketServer> ws;
    if (ws_port > 0) {
        ws.reset(new JSWebsocketServer(&can_hub0, ws_port, static_dir));
    }
    std::unique_ptr<JSTcpClient> client;
    if (upstream_host) {
        client.reset(new JSTcpClient(&can_hub0, upstream_host, upstream_port));
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
    g_executor.thread_body();
    return 0;
}
