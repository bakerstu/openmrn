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

#include "main.hxx"

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "utils/JSSerialPort.hxx"
#include "utils/JSTcpClient.hxx"

class BootloaderClientStateFlow : public StateFlowBase
{
public:
    BootloaderClientStateFlow()
        : StateFlowBase(&g_service)
    {
        start_flow(STATE(wait_for_boot));
    }

private:
    Action wait_for_boot()
    {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(400), STATE(send_request));
    }

    Action send_request()
    {
        Buffer<openlcb::BootloaderRequest> *b = fill_request();

        b->set_done(bn_.reset(this));
        bootloader_client.send(b);
        return wait_and_call(STATE(bootload_done));
    }

    Action bootload_done()
    {
        printf("Result: %04x  %s\n", response.error_code,
            response.error_details.c_str());
        fflush(stdout);
#ifdef __EMSCRIPTEN__
        EM_ASM(process.exit());
#endif
        ::exit(response.error_code == 0 ? 0 : 1);
    }

    StateFlowTimer timer_{this};
    BarrierNotifiable bn_;
} bootload_state_flow;

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
    std::unique_ptr<JSSerialPort> dev;
    std::unique_ptr<JSTcpClient> client;
    if (device_path)
    {
        dev.reset(new JSSerialPort(&can_hub0, device_path));
    }
    else
    {
        client.reset(new JSTcpClient(&can_hub0, host, port));
    }
    g_if_can.add_addressed_message_support();
    // Bootstraps the alias allocation process.
    g_if_can.alias_allocator()->send(g_if_can.alias_allocator()->alloc());

    g_executor.thread_body();
    return 0;
}
