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

#include "main.hxx"

/// Handle the device/socket connection and reconnection upon disconnect.
class ConnectionMonitor : StateFlowBase
{
public:
    /// Constructor.
    ConnectionMonitor()
        : StateFlowBase(&g_service)
    {
        connect();
        HASSERT(fd_ >= 0);
        create_gc_port_for_can_hub(&can_hub0, fd_, this);
        wait_and_call(STATE(disconnected));
    }

    /// Disconnect occurred.
    Action disconnected()
    {
        LOG(INFO, "disconnected()");
        /// @todo cleanup main to use a SimpleStack, then replace this with
        /// SimpleStack::restart_stack().
        g_if_can.remote_aliases()->clear();
        // Spawn a thread so that the executor is not blocked.
        new (&thread_) OSThread("ConnectSocket", 0, 1024, connect_thread, this);
        return wait_and_call(STATE(disconnected));
    }

private:
    /// Make a connection to the device or socket.
    void connect()
    {
        if (device_path)
        {
            fd_ = ::open(device_path, O_RDWR);
        }
        else
        {
            fd_ = ConnectSocket(host, port);
        }
    }

    /// In context helper thread for making socket connections.
    void connect_thread()
    {
        do
        {
            // This sleep serves two functions:
            // 1. Some delay between reconnect attempts.
            // 2. Allows some extra time for the client to exit upon completion
            //    before attempting a new connection.
            sleep(1);
            connect();
        } while (fd_ < 0);
        LOG(INFO, "reconnected()");
        create_gc_port_for_can_hub(&can_hub0, fd_, this);
    }

    /// Helper thread for making socket connections.
    static void *connect_thread(void *arg)
    {
        static_cast<ConnectionMonitor*>(arg)->connect_thread();
        return nullptr;
    }

    OSThread thread_; ///< thread object
    int fd_; ///< file descriptor for the connection
};

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    parse_args(argc, argv);
    if (process_dump())
    {
        return 0;
    }
    new ConnectionMonitor();

    g_if_can.add_addressed_message_support();
    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    g_executor.start_thread("g_executor", 0, 1024);
    usleep(400000);

    SyncNotifiable n;
    BarrierNotifiable bn(&n);
    Buffer<openlcb::BootloaderRequest> *b = fill_request();

    b->set_done(&bn);
    maybe_checksum(&b->data()->data);

    bootloader_client.send(b);
    n.wait_for_notification();
    printf("Result: %04x  %s\n", response.error_code,
        response.error_details.c_str());
    if (response.error_code != 0)
    {
        exit(1);
    }
    exit(0);
    return 0;
}
