/** \copyright
 * Copyright (c) 2019, Mike Dunston
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
 * \file OS.cxx
 * This file represents a C++ language abstraction of common operating
 * system calls.
 *
 * @author Mike Dunston
 * @date 22 Feb 2019
 */

#if defined (ESP32)

#include "os/os.h"
#include <Arduino.h>
#include <freertos/task.h>
#include <map>

// since vTaskSetApplicationTaskTag is not enabled by default on ESP32 use a
// std::map as a holder for thread private data.
static std::map<os_thread_t, ThreadPriv *> threadPrivHolder;

extern "C" {
ThreadPriv *getCurrentThreadPriv()
{
    os_thread_t threadHandle = xTaskGetCurrentTaskHandle();
    ThreadPriv *priv = nullptr;
    if(threadPrivHolder.find(threadHandle) == threadPrivHolder.end())
    {
        // no thread priv allocated previously, allocate it now
        // this should only happen for threads not started/owned by OpenMRN.
        priv = (ThreadPriv *)malloc(sizeof(ThreadPriv));
        priv->entry = nullptr;
        priv->arg = nullptr;
        priv->selectEventBit = 0;
        threadPrivHolder[threadHandle] = priv;
    }
    else
    {
        priv = threadPrivHolder[threadHandle];
    }
    return priv;
}

void saveThreadPriv(ThreadPriv *priv)
{
    os_thread_t threadHandle = xTaskGetCurrentTaskHandle();
    threadPrivHolder[threadHandle] = priv;
}

void eraseCurrentThreadPriv()
{
    os_thread_t threadHandle = xTaskGetCurrentTaskHandle();
    threadPrivHolder.erase(threadHandle);
}

} // extern "C"
#endif