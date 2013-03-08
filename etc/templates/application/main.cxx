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
#include <fcntl.h>
#include <inttypes.h>

#include "os/os.h"
#include "os/OS.hxx"
#include "if/nmranet_if.h"
#include "core/nmranet_node.h"
#include "core/nmranet_event.h"
#include "core/nmranet_datagram.h"
#include "nmranet_config.h"

const char *nmranet_manufacturer = "Stuart W. Baker";
const char *nmranet_hardware_rev = "N/A";
const char *nmranet_software_rev = "0.1";
const size_t main_stack_size = 2048;
const int main_priority = 0;
const size_t ALIAS_POOL_SIZE = 2;
const size_t DOWNSTREAM_ALIAS_CACHE_SIZE = 2;
const size_t UPSTREAM_ALIAS_CACHE_SIZE = 2;
const size_t DATAGRAM_POOL_SIZE = 10;
const size_t CAN_RX_BUFFER_SIZE = 1;
const size_t CAN_TX_BUFFER_SIZE = 32;
const size_t SERIAL_RX_BUFFER_SIZE = 16;
const size_t SERIAL_TX_BUFFER_SIZE = 16;

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    printf("hello world\n");

    NMRAnetIF *nmranet_if;

    if (argc >= 2)
    {
        nmranet_if = nmranet_gc_if_init(0x02010d000000ULL, argv[1]);
    }
    else
    {
#if defined (__FreeRTOS__)

        nmranet_if = nmranet_can_if_init(0x02010d000000ULL, "/dev/can0", read, write);
#else
        nmranet_if = nmranet_gc_if_init(0x02010d000000ULL, "/dev/ttyUSB1");
#endif
    }
    
    if (nmranet_if == NULL)
    {
        printf("Unable to open NMRAnet Interface.\n");
        return 0;
    }
    
    node_t node = nmranet_node_create(0x02010d000001ULL, nmranet_if, "Virtual Node", NULL);
    nmranet_node_user_description(node, "Test Node");

    nmranet_event_consumer(node, 0x0502010202000000ULL, EVENT_STATE_INVALID);
    nmranet_event_consumer(node, 0x0502010202650013ULL, EVENT_STATE_INVALID);
    nmranet_event_consumer(node, 0x0502010202650012ULL, EVENT_STATE_INVALID);
    nmranet_event_producer(node, 0x0502010202000000ULL, EVENT_STATE_INVALID);
    nmranet_event_producer(node, 0x0502010202650012ULL, EVENT_STATE_INVALID);
    nmranet_event_producer(node, 0x0502010202650013ULL, EVENT_STATE_VALID);

    nmranet_node_initialized(node);
#if 1
    uint8_t data[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    //node_handle_t dst = {0, 0x014};
    node_handle_t dst = {0x050201020265ULL, 0};
    nmranet_datagram_produce(node, dst, DATAGRAM_TRAIN_CONTROL, data, 16, 0);
    nmranet_datagram_produce(node, dst, DATAGRAM_TRAIN_CONTROL, data, 16, 3000000000LL);
#endif
    for (;;)
    {
        if (argc < 3)
        {    
            sleep(2);
            nmranet_event_produce(node, 0x0502010202650012ULL, EVENT_STATE_INVALID);
            nmranet_event_produce(node, 0x0502010202650012ULL, EVENT_STATE_VALID);
            sleep(2);
            nmranet_event_produce(node, 0x0502010202650013ULL, EVENT_STATE_INVALID);
            nmranet_event_produce(node, 0x0502010202650013ULL, EVENT_STATE_VALID);
        }
        int result = nmranet_node_wait(node, MSEC_TO_NSEC(300));

        if (result)
        {
            for (size_t i = nmranet_event_pending(node); i > 0; i--)
            {
                uint64_t event = nmranet_event_consume(node);
#if !defined (__FreeRTOS__)
                printf("we got event: 0x%016" PRIx64 "\n", event);
#else
                event = event ? 1 : 0; /* suppress compiler warning, unused variable */
#endif
            }
            for (size_t i = nmranet_datagram_pending(node); i > 0; i--)
            {
                datagram_t datagram = nmranet_datagram_consume(node);
                nmranet_datagram_release(datagram);
            }            
        }
    }

    return 0;
}
