/** \copyright
 * Copyright (c) 2016, Sidney McHarg
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
 * \file FreeRTOSTCPSocket.cxx
 * This file provides the sockets interface to the FreeRTOSPlus TCP stack.
 * Based on work by Stuart W. Baker
 *
 * @author Sidney McHarg
 * @date 21 March 2016
 */

#include <new>
#include <stdio.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// FreeRTOSTCP includes
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#include "FreeRTOSTCP.hxx"
#include "FreeRTOSTCPSocket.hxx"

static SocketSet_t socket_set = NULL;

static QueueHandle_t close_queue = NULL;

/// Called before the TCP stack is initialized in FreeRTOS.
bool network_layer_preinit(void);

/// Callback from FreeRTOS network event handler. Called on the network thread.
/// @param eNetworkEvent see freertos-TCP documentation.
extern "C" void vApplicationIPNetworkEventHook(
    eIPCallbackEvent_t eNetworkEvent);

/*
 * FreeRTOSTCP::FreeRTOSTCP()
 */
FreeRTOSTCP::FreeRTOSTCP()
    : ipAddress(0)
    , wakeup(NULL)
{
}

/*
 * FreeRTOSTCP::start()
 */
void FreeRTOSTCP::start()
{
    network_layer_preinit();
    os_thread_create(NULL, "FreeRTOSTCP Task", configMAX_PRIORITIES - 1, 2048,
        net_task_entry, NULL);
}

/*
 * FreeRTOSTCP::net_task()
 */
void FreeRTOSTCP::net_task()
{
    int sel_result, result, tries;
    struct freertos_sockaddr address;

    socket_set = FreeRTOS_CreateSocketSet();
    HASSERT(socket_set != NULL);

    close_queue = xQueueCreate(10, sizeof(Socket_t));
    HASSERT(close_queue != 0);

    tries = 0;
    wakeup = FREERTOS_INVALID_SOCKET;
    while ((++tries <= 5) && (wakeup == FREERTOS_INVALID_SOCKET))
    {
        wakeup = FreeRTOS_socket(
            FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);
        if (wakeup == FREERTOS_INVALID_SOCKET)
        {
            // no memory or networking stack not fully operational
            if (++tries < 5)
                sleep(1);
        }
    }
    HASSERT(wakeup != FREERTOS_INVALID_SOCKET);

    address.sin_port = FreeRTOS_htons(8000);
    address.sin_addr = FreeRTOS_htonl(0); // INADDR_ANY;
    result = FreeRTOS_bind(wakeup, &address, sizeof(address));
    HASSERT(result >= 0);

    FreeRTOS_FD_SET(wakeup, socket_set, eSELECT_READ | eSELECT_INTR);

    for (; /* forever */;)
    {
        const TickType_t timeout = pdMS_TO_TICKS(30000);

        sel_result = FreeRTOS_select(socket_set, timeout);

        if (sel_result < 0)
        {
            continue;
        }

        if (sel_result & eSELECT_INTR)
        {
            // wakeup: process close_queue entries
            Socket_t sd;

            while (xQueueReceive(close_queue, &sd, 0) == pdTRUE)
            {
                portENTER_CRITICAL();
                FreeRTOS_FD_CLR(sd, socket_set,
                    eSELECT_READ | eSELECT_WRITE | eSELECT_EXCEPT);
                delete FreeRTOSTCPSocket::get_instance_from_sd(sd);
                FreeRTOSTCPSocket::remove_instance_from_sd(sd);
                portEXIT_CRITICAL();
                FreeRTOS_shutdown(sd, FREERTOS_SHUT_RDWR);
                FreeRTOS_closesocket(sd);
            }
        }

        // there should be a better to way to go about determining which sockets
        // are
        // in the socketset used for the call in select
        for (int i = MAX_SOCKETS - 1; i >= 0; --i)
        {
            FreeRTOSTCPSocket *s = nullptr;
            Socket_t sd;
            BaseType_t evt;
            sd = wakeup;

            if (i == 0)
            {
                s = nullptr;
                sd = wakeup;
            }
            else
            {
                s = FreeRTOSTCPSocket::get_sd_by_index(i);
                if (s == nullptr)
                {
                    continue;
                }
                sd = s->sd;
            }

            /* obtain event mask in evt */
            evt = FreeRTOS_FD_ISSET(sd, socket_set);
            if (evt == 0)
            {
                continue;
            }

            if (evt & eSELECT_READ)
            {
                if (sd == wakeup)
                {
                    /* this is the socket we use as a signal, so discard any
                     * data received */
                    char data[4];
                    FreeRTOS_recvfrom(
                        wakeup, data, sizeof(data), 0, nullptr, nullptr);
                }
                else
                {
                    // have something to read
                    FreeRTOS_FD_CLR(sd, socket_set, eSELECT_READ);
                    s->readActive = true;
                    s->select_wakeup(&s->selInfoRd);
                }
            }

            if (evt & eSELECT_WRITE)
            {
                if (sd == wakeup)
                {
                    FreeRTOS_FD_CLR(sd, socket_set, eSELECT_WRITE);
                    continue;
                }
                // was the test for the write fdset
                FreeRTOS_FD_CLR(sd, socket_set, eSELECT_WRITE);
                s->writeActive = false;
                s->select_wakeup(&s->selInfoWr);
            }
            if (evt & eSELECT_EXCEPT)
            {
                // was the test for the except fdset
                FreeRTOS_FD_CLR(sd, socket_set, eSELECT_EXCEPT);
                /* currently we don't handle any errors */
                ;
            }
        }
    }
}

/*
 * FreeRTOSTCP::select_wakeup()
 */
void FreeRTOSTCP::select_wakeup(Socket_t data)
{
    HASSERT(wakeup >= 0);

#if ipconfigSUPPORT_SIGNALS == 1
    // needs FreeRTOSIPConfig to include SUPPORT_SIGNALS = 1
    if (data != nullptr)
    {
        // place request in wake_queue
        xQueueSend(close_queue, &data, 0);
    }
    FreeRTOS_SignalSocket(wakeup);
#else
#error "FreeRTOSIPConfig.h must include ipconfigSUPPORT_SIGNALS = 1"
#endif
}

/*
 * FreeRTOSTCP::fd_set_read()
 */
void FreeRTOSTCP::fd_set_read(Socket_t socket)
{
    FreeRTOS_FD_SET(socket, socket_set, eSELECT_READ);
    select_wakeup();
}

/*
 * FreeRTOSTCP::fd_set_write()
 */
void FreeRTOSTCP::fd_set_write(Socket_t socket)
{
    FreeRTOS_FD_SET(socket, socket_set, eSELECT_WRITE);
    select_wakeup();
}

extern "C" void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
    return;
}
