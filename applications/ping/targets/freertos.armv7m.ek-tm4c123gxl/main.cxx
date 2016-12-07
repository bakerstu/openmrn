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
 * Ping client for the tiva 123.
 *
 * @author Balazs Racz
 * @date 5 Jun 2015
 */

#include <memory>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "os/os.h"
#include "nmranet_config.h"

#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "freertos_drivers/ti/TivaDev.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "hardware.hxx"

#include "protocol.hxx"
#include "utils.hxx"

#include "inc/hw_ints.h"

int req_bytes = 1;
int resp_bytes = 1;

// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 2500);

const char SERIAL_PORT[] = "/dev/ser1";
const char USB_PORT[] = "/dev/serUSB0";

/** UART 4 serial driver instance */
static TivaUart uart4("/dev/ser1", UART4_BASE, INT_UART4);

GPIO_HWPIN(UART4RX, GpioHwPin, C, 4, U4RX, UART);
GPIO_HWPIN(UART4TX, GpioHwPin, C, 5, U4TX, UART);

typedef GpioInitializer<UART4RX_Pin, UART4TX_Pin> UartInitializer;


extern "C" {
void uart4_interrupt_handler(void)
{
    uart4.interrupt_handler();
}
}

#define printstat(x, bef, after) testfn(bef, after)

void run_client(int fd)
{
    Request r;
    r.payload_length = req_bytes;
    r.response_length = resp_bytes;
    long long before_send = os_get_time_monotonic();
    long long first_hdr;
    if (!rw_repeated(fd, &r, sizeof(r), &first_hdr, write, "write"))
    {
        diewith(ERR_WRITE_1);
    }
    long long after_hdr = os_get_time_monotonic();
    int maxlen = std::max(r.payload_length, r.response_length);
    std::unique_ptr<uint8_t[]> payload(new uint8_t[maxlen]);
    memset(payload.get(), 0xAA, maxlen);
    long long first_write;
    if (!rw_repeated(
            fd, payload.get(), r.payload_length, &first_write, write, "write"))
    {
        diewith(ERR_WRITE_2);
    }
    long long write_done = os_get_time_monotonic();

    long long first_response;
    if (!rw_repeated(fd, payload.get(), r.response_length, &first_response,
            read, "read"))
    {
        diewith(ERR_READ_1);
    }
    long long all_response = os_get_time_monotonic();

    LatencyResponse l;
    l.latency_usec = (all_response - before_send) / 1000;

    if (!rw_repeated(fd, &l, sizeof(l), nullptr, write, "write"))
    {
        diewith(ERR_WRITE_3);
    }

    printstat("before hdr->first hdr", before_send, first_hdr);
    printstat("first hdr->after_hdr", first_hdr, after_hdr);
    printstat("after_hdr->first_write", after_hdr, first_write);
    printstat("first_write->write_done", first_write, write_done);

    // printf("\n");
    printstat("all write", before_send, write_done);
    // printf("\n");
    printstat("write_done->first_resp", write_done, first_response);
    printstat("first_resp->all_resp", first_response, all_response);

    // printf("\n");
    printstat("e2e", before_send, all_response);
}


/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    UartInitializer::hw_init();
    int fd = -1;
    if (SW1_Pin::get()) {
        fd = ::open(SERIAL_PORT, O_RDWR);
    } else {
        fd = ::open(USB_PORT, O_RDWR);
    }
    while (true)
    {
        do
        {
            usleep(200000);
        } while (SW2_Pin::get());
        run_client(fd);
    }
    ::close(fd);

    return 0;
}
