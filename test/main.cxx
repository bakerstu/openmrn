/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file main.c
 * This file represents the entry point to a simple test program.
 *
 * @author Stuart W. Baker
 * @date 16 September 2012
 */

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include "os/os.h"
#include "os/OS.hxx"
#include "if/nmranet_if.h"
#include "core/nmranet_node.h"
#include "core/nmranet_event.h"

const char *nmranet_manufacturer = "Stuart W. Baker";
const char *nmranet_hardware_rev = "N/A";
const char *nmranet_software_rev = "0.1";
const size_t main_stack_size = 512;
const int main_priority = 0;

/** Entry point to program.
 * @param argc number of command line arguments
 * @param argv array of command line aguments
 * @return 0, should never return
 */
extern "C" int nmranet_main(int argc, char *argv[])
{
    NMRAnetIF *nmranet_if;
    //nmranet_init(0x02010d000000);

    if (argc >= 2)
    {
        nmranet_if = nmranet_gc_if_init(0x02010d000000, argv[1]);
    }
    else
    {
        nmranet_if = nmranet_gc_if_init(0x02010d000000, "/dev/ttyUSB1");
    }
    
    node_t node = nmranet_node_create(0x02010d000001, NODE_ID_EXACT_MASK, nmranet_if, "Virtual Node", NULL);
    nmranet_node_initialized(node);
    nmranet_event_consumer(node, 0x0502010202650013, EVENT_STATE_INVALID);
    nmranet_event_producer(node, 0x0502010202650012, EVENT_STATE_INVALID);
    nmranet_event_producer(node, 0x0502010202650013, EVENT_STATE_VALID);

    for (;;)
    {
#if 1
        //nmranet_node_wait(node);
        //printf("wokeup\n");
        uint64_t event;
        do
        {
            event = nmranet_event_consume(node);
            if (event == 0x0502010202650013)
            {
                printf("we got the right one\n");
            }
        }
        while (event != 0);
#endif
        sleep(2);
        nmranet_event_produce(node, 0x0502010202650012, EVENT_STATE_INVALID);
        nmranet_event_produce(node, 0x0502010202650012, EVENT_STATE_VALID);
        sleep(2);
        nmranet_event_produce(node, 0x0502010202650013, EVENT_STATE_INVALID);
        nmranet_event_produce(node, 0x0502010202650013, EVENT_STATE_VALID);
        
    }

    return 0;
}
