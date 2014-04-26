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

#include <stdio.h>
#include <unistd.h>

#include <memory>

#include "os/os.h"
//#include "utils/pipe.hxx"
#include "utils/gc_pipe.hxx"
#include "utils/PipeFlow.hxx"
#include "utils/HubDevice.hxx"
#include "utils/socket_listener.hxx"
#include "nmranet_can.h"
#include "executor/Executor.hxx"
#include "executor/Service.hxx"

// DEFINE_PIPE(gc_can_pipe, 1);

extern "C" {
// extern int CAN_PIPE_BUFFER_COUNT;
// int CAN_PIPE_BUFFER_COUNT = 32;
}

Executor<1> g_executor("g_executor", 0, 1024);
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);
GcPacketPrinter packet_printer(&can_hub0);

// DEFINE_PIPE(can_pipe, &g_executor, sizeof(struct can_frame));
// DEFINE_PIPE(display_pipe, &g_executor, 1);

extern "C" {
extern int GC_GENERATE_NEWLINES;
int GC_GENERATE_NEWLINES = 1;
}

struct ClientInfo : public Notifiable
{
    ClientInfo(int fd)
        : gcHub_(&g_service)
        , bridge_(GCAdapterBase::CreateGridConnectAdapter(&gcHub_, &can_hub0,
                                                          false))
        , gcWrite_(&gcHub_, fd, this)
    {
    }

    HubFlow gcHub_;
    std::unique_ptr<GCAdapterBase> bridge_;
    FdHubPort<HubFlow> gcWrite_;

    void notify() OVERRIDE
    {
        // We get this call when something is wrong with the FDs and we need to
        // close the connection.
        delete this;
    }
};

void NewConnection(int fd)
{
    new ClientInfo(fd);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    SocketListener listener(8082, NewConnection);
    while (1)
    {
        sleep(1);
    }
    return 0;
}
