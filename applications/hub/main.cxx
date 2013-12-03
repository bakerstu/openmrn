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

#include "os/os.h"
#include "utils/pipe.hxx"
#include "utils/gc_pipe.hxx"
#include "utils/socket_listener.hxx"
#include "nmranet_can.h"

// DEFINE_PIPE(gc_can_pipe, 1);

DEFINE_PIPE(can_pipe, sizeof(struct can_frame));

struct ClientInfo
{
    int fd;
    char thread_name[30];
    Pipe* client_pipe;
    GCAdapterBase* bridge;
};

void NewConnection(int fd)
{
    ClientInfo* c = new ClientInfo(); // @TODO(balazs.racz): this is leaked.
    sprintf(c->thread_name, "thread_fd_%d", fd);
    c->fd = fd;
    c->client_pipe = new Pipe(1);
    c->client_pipe->AddPhysicalDeviceToPipe(fd, fd, c->thread_name, 0);
    c->bridge = GCAdapterBase::CreateGridConnectAdapter(c->client_pipe,
                                                        &can_pipe, false);
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char* argv[])
{
    SocketListener listener(8082, NewConnection);
    while (1) {
        sleep(1);
    }
    return 0;
}
