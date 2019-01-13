/** @copyright
 * Copyright (c) 2018, Stuart W Baker
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
 * @file SimpleLinkPthread.cxx
 * This file represents a C language abstraction of common operating
 * system calls specific to the Simplelink SDK pthread support.
 *
 * @author Stuart W. Baker
 * @date 29 December 2018
 */

#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

#include "nmranet_config.h"

#include "os/os.h"
#include "os/os_private.h"

extern "C"
{
#if defined (__FreeRTOS__)
typedef struct
{
    TaskHandle_t freeRTOSTask;
} PthreadObj;

//
// ::os_thread_start_entry_hook()
//
void os_thread_start_entry_hook(void)
{
    // make sure that pthread_self() struct did not rot by asserting
    // against the FreeRTOS task handle
    PthreadObj *thread_handle = static_cast<PthreadObj*>(pthread_self());
    HASSERT(thread_handle->freeRTOSTask == xTaskGetCurrentTaskHandle());
}

//
// ::os_thread_start_exit_hook()
//
void os_thread_start_exit_hook(void *context)
{
    pthread_exit(context);
}
#else
#error Unsupported OS
#endif // __FreeRTOS__

/** Entry point to a thread, posix variant
 * @param arg metadata for entering the thread
 * @return unused, should never return.
 */
static void *os_thread_start_posix(void *arg)
{
    os_thread_start(arg);

    return NULL;
}

//
// ::os_thread_create_helper()
//
int os_thread_create_helper(os_thread_t *thread, const char *name, int priority,
                            size_t stack_size, void *priv)
{
    HASSERT(thread);
    pthread_attr_t attr;

    int result = pthread_attr_init(&attr);
    if (result != 0)
    {
        return result;
    }
    result = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (result != 0)
    {
        return result;
    }

    result = pthread_attr_setstacksize(&attr, stack_size);
    if (result != 0)
    {
        return result;
    }

    struct sched_param sched_param;
    sched_param.sched_priority = priority;
    result = pthread_attr_setschedparam(&attr, &sched_param);
    if (result != 0)
    {
        return result;
    }

    pthread_t pthread;
    result = pthread_create(&pthread, &attr, os_thread_start_posix, priv);
    if (result != 0)
    {
        return result;
    }

#if defined (__FreeRTOS__)
    // Hack:  Though not pretty, it allows us to get the FreeRTOS
    //        task handle from the pthread handle.
    *thread = static_cast<PthreadObj*>(pthread)->freeRTOSTask;

    // Hack:  The pthread API doesn't support setting the FreeRTOS task name
    //        directly.  Call pcTaskGetName() and override the const pointer.
    char *task_name = static_cast<char*>(pcTaskGetName(*thread));
    strncpy(task_name, name, configMAX_TASK_NAME_LEN - 1);
    task_name[configMAX_TASK_NAME_LEN - 1] = '\0';
#else
#error Unsupported OS
#endif

    return result;
}

} // extern "C"

