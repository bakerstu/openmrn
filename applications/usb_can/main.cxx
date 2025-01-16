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
 * A simple application to act as a USB-CAN bridge. This application creates no
 * OpenLCB node, just forwards CAN packets to/from the host, in gridconnect
 * format.
 *
 * @author Balazs Racz
 * @date 29 Apr 2014
 */

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "can_frame.h"
#include "executor/Executor.hxx"
#include "nmranet_config.h"
#include "os/os.h"
#include "utils/GridConnectHub.hxx"
#include "utils/Hub.hxx"
#include "utils/HubDevice.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/blinker.h"

Executor<1> g_executor(NO_THREAD{});
Service g_service(&g_executor);
CanHubFlow can_hub0(&g_service);

OVERRIDE_CONST(gc_generate_newlines, 0);
OVERRIDE_CONST(can_tx_buffer_size, 8);
OVERRIDE_CONST(can_rx_buffer_size, 8);
OVERRIDE_CONST(serial_tx_buffer_size, 64);
OVERRIDE_CONST(serial_rx_buffer_size, 64);
#ifdef BOARD_LAUNCHPAD_EK
OVERRIDE_CONST(main_thread_stack_size, 2500);
#else
OVERRIDE_CONST(main_thread_stack_size, 900);
#endif

OVERRIDE_CONST(gridconnect_bridge_max_incoming_packets, 5);
OVERRIDE_CONST(gridconnect_port_max_incoming_packets, 2);

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char* argv[])
{
    int serial_fd = ::open("/dev/serUSB0", O_RDWR); // or /dev/ser0
    HASSERT(serial_fd >= 0);
    ::fcntl(serial_fd, F_SETFL, O_NONBLOCK);

    create_gc_port_for_can_hub(&can_hub0, serial_fd, nullptr, true);

    int can_fd = ::open("/dev/can0", O_RDWR);
    HASSERT(can_fd >= 0);
    ::fcntl(can_fd, F_SETFL, O_NONBLOCK);

    HubDeviceSelect<CanHubFlow> can_hub_port(
        &can_hub0, can_fd, EmptyNotifiable::DefaultInstance());

    g_executor.thread_body();
    return 0;
}
