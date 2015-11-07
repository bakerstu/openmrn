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

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <termios.h> /* tc* functions */
#include <unistd.h>

#include <memory>

#include "os/os.h"
#include "utils/constants.hxx"
#include "utils/Hub.hxx"
#include "utils/GridConnectHub.hxx"
#include "utils/GcTcpHub.hxx"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

OVERRIDE_CONST(gc_generate_newlines, 1);

int port = 12021;
const char *device_path = nullptr;
int upstream_port = 12021;
const char *upstream_host = nullptr;
bool timestamped = false;

void usage(const char *e)
{
    fprintf(stderr, "Usage: %s [-p port] [-d device_path] [-u upstream_host] "
                    "[-q upstream_port] [-t]\n\n",
            e);
    fprintf(stderr, "GridConnect CAN HUB.\nListens to a specific TCP port, "
                    "reads CAN packets from the incoming connections using "
                    "the GridConnect protocol, and forwards all incoming "
                    "packets to all other participants.\n\nArguments:\n");
    fprintf(stderr, "\t-p port     specifies the port number to listen on, "
                    "default is 12021.\n");
    fprintf(stderr, "\t-d device   is a path to a physical device doing "
                    "serial-CAN or USB-CAN. If specified, opens device and "
                    "adds it to the hub.\n");
    fprintf(stderr, "\t-u upstream_host   is the host name for an upstream "
                    "hub. If specified, this hub will connect to an upstream "
                    "hub.\n");
    fprintf(stderr,
            "\t-q upstream_port   is the port number for the upstream hub.\n");
    fprintf(stderr,
            "\t-t prints timestamps for each packet.\n");
    exit(1);
}

void parse_args(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "hp:d:u:q:t")) >= 0)
    {
        switch (opt)
        {
            case 'h':
                usage(argv[0]);
                break;
            case 'd':
                device_path = optarg;
                break;
            case 'p':
                port = atoi(optarg);
                break;
            case 'u':
                upstream_host = optarg;
                break;
            case 'q':
                upstream_port = atoi(optarg);
                break;
            case 't':
                timestamped = true;
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                usage(argv[0]);
        }
    }
}

class ConnectionClient
{
public:
    /** Test the connection whether it is alive; establish the connection if it
     * is dead. */
    virtual void ping() = 0;
};

class DeviceClosedNotify : public Notifiable
{
public:
    DeviceClosedNotify(int *fd, string name)
        : fd_(fd)
        , name_(name)
    {
    }
    void notify() override
    {
        LOG_ERROR("Connection to %s closed.", name_.c_str());
        *fd_ = -1;
    }

private:
    int *fd_;
    string name_;
};

class FdConnectionClient : public ConnectionClient
{
public:
    FdConnectionClient(const string &name) : closedNotify_(&fd_, name)
    {
    }

    void ping() OVERRIDE
    {
        if (fd_ < 0)
        {
            try_connect();
        }
    }

protected:
    virtual void try_connect() = 0;

    int fd_{-1};
    DeviceClosedNotify closedNotify_;
};

class DeviceConnectionClient : public FdConnectionClient
{
public:
    DeviceConnectionClient(const string &name, const string &dev)
        : FdConnectionClient(name)
        , dev_(dev)
    {
    }

private:
    void try_connect() OVERRIDE
    {
        fd_ = ::open(dev_.c_str(), O_RDWR);
        if (fd_ >= 0)
        {
            // Sets up the terminal in raw mode. Otherwise linux might echo
            // characters coming in from the device and that will make
            // packets go back to where they came from.
            HASSERT(!tcflush(fd_, TCIOFLUSH));
            struct termios settings;
            HASSERT(!tcgetattr(fd_, &settings));
            cfmakeraw(&settings);
            cfsetspeed(&settings, B115200);
            HASSERT(!tcsetattr(fd_, TCSANOW, &settings));
            LOG(INFO, "Opened device %s.\n", device_path);
            create_gc_port_for_can_hub(&can_hub0, fd_, &closedNotify_);
        }
        else
        {
            LOG_ERROR("Failed to open device %s: %s\n", device_path,
                      strerror(errno));
            fd_ = -1;
        }
    }

    string dev_;
};

class UpstreamConnectionClient : public FdConnectionClient
{
public:
    UpstreamConnectionClient(const string &name, const string &host, int port)
        : FdConnectionClient(name)
        , host_(host)
        , port_(port)
    {
    }

private:
    void try_connect() OVERRIDE
    {
        fd_ = ConnectSocket(upstream_host, upstream_port);
        if (fd_ >= 0)
        {
            LOG_ERROR("Connected to %s:%d\n", host_.c_str(), port_);
            create_gc_port_for_can_hub(&can_hub0, fd_, &closedNotify_);
        }
        else
        {
            LOG_ERROR("Failed to connect to %s:%d: %s\n", host_.c_str(), port_,
                      strerror(errno));
            fd_ = -1;
        }
    }

    string host_;
    int port_;
};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    GcPacketPrinter packet_printer(&can_hub0, timestamped);
    GcTcpHub hub(&can_hub0, port);
    vector<std::unique_ptr<ConnectionClient>> connections;

    if (upstream_host)
    {
        connections.emplace_back(new UpstreamConnectionClient(
            "upstream", upstream_host, upstream_port));
    }

    if (device_path)
    {
        connections.emplace_back(
            new DeviceConnectionClient("device", device_path));
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
