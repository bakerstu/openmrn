/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * Application that dumps all DCC packets to USB-serial.
 *
 * @author Balazs Racz
 * @date 4 Sep 2018
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include "os/os.h"
#include "executor/Executor.hxx"
#include "dcc/DccDebug.hxx"
#include "utils/constants.hxx"
#include "utils/StringPrintf.hxx"

Executor<1> executor("executor", 0, 2048);

// We reserve a lot of buffer for transmit to cover for small hiccups in the
// host reading data.
OVERRIDE_CONST(serial_tx_buffer_size, 2048);

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    setblink(0);
    int fd = ::open("/dev/nrz0", O_RDONLY);
    HASSERT(fd >= 0);
    int wfd = ::open("/dev/serUSB0", O_RDWR);
    HASSERT(wfd >= 0);
    int cnt = 0;
    while (1)
    {
        DCCPacket packet_data;
        int sret = ::read(fd, &packet_data, sizeof(packet_data));
        HASSERT(sret == sizeof(packet_data));
        long long t = os_get_time_monotonic();
        string txt = StringPrintf("\n%02d.%06d %04d ",
            (unsigned)((t / 1000000000) % 100),
            (unsigned)((t / 1000) % 1000000), cnt % 10000);
        ::write(wfd, txt.data(), txt.size());
        txt = dcc::packet_to_string(packet_data);
        ::write(wfd, txt.data(), txt.size());
        // we purposefully do not check the return value, because if there was
        // not enough space in the serial write buffer, we need to throw away
        // data.
        ++cnt;
        resetblink((cnt >> 3) & 1);
    }
    return 0;
}
