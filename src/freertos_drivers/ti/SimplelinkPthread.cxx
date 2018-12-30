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
extern void hw_postinit(void);

/// default stdin
extern const char *STDIN_DEVICE;

/// default stdout
extern const char *STDOUT_DEVICE;

/// default stderr
extern const char *STDERR_DEVICE;

/// Private metadata for starting a thread.
typedef struct
{
    void *(*entry)(void*); ///< thread entry point
    void *arg; ///< argument to thread
    TaskList *taskList; //< task list instance
} OSThreadStartPriv;

/// Entry point to a thread.
/// @param arg metadata for entering the thread
static void *os_thread_start(void *arg)
{
    OSThreadStartPriv *priv = (OSThreadStartPriv*)arg;
    priv->taskList->task = xTaskGetCurrentTaskHandle();
    add_thread_to_task_list(priv->taskList);
    vTaskSetThreadLocalStoragePointer(NULL, TLS_INDEX_SELECT_EVENT_BIT, NULL);

    void *result = (*priv->entry)(priv->arg);

    del_thread_from_task_list(xTaskGetCurrentTaskHandle());

    // We purposesly do not free priv->taskist.  Though it is technically a
    // leak, we keep it around for diagnostic purposes.

    free(arg);
    pthread_exit(result);
    return NULL;
}
#else
#error Unsupported OS
#endif // __FreeRTOS__

//
// ::os_thread_create()
//
int os_thread_create(os_thread_t *thread, const char *name, int priority,
                     size_t stack_size, void *(*start_routine) (void *),
                     void *arg)
{
    static unsigned int count = 0;
    char auto_name[10];

    if (name == NULL)
    {
        strcpy(auto_name, "thread.");
        auto_name[9] = '\0';
        auto_name[8] = '0' + (count % 10);
        auto_name[7] = '0' + (count / 10);
        count++;
        name = auto_name;
    }

    if (priority == 0)
    {
        priority = sched_get_priority_max(SCHED_FIFO) / 2;
    }
    else if (priority > sched_get_priority_max(SCHED_FIFO))
    {
        priority = sched_get_priority_max(SCHED_FIFO);
    }

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

#if defined (__FreeRTOS__)
    // TI's pthread port does not support setting names.

    OSThreadStartPriv *priv =
        (OSThreadStartPriv*)malloc(sizeof(OSThreadStartPriv));

    priv->taskList = (TaskList*)malloc(sizeof(TaskList));

    priv->taskList->name =
        (char*)malloc(sizeof(char) * configMAX_TASK_NAME_LEN);

    priv->entry = start_routine;
    priv->arg = arg;

    priv->taskList->unused = stack_size;
    strncpy(priv->taskList->name, name, configMAX_TASK_NAME_LEN - 1);
    priv->taskList->name[configMAX_TASK_NAME_LEN - 1] = '\0';
#else
#error Unsupported OS
#endif

    pthread_t pthread;
    result = pthread_create(&pthread, &attr, os_thread_start, priv);
    if (result != 0)
    {
        return result;
    }

#if defined (__FreeRTOS__)
    if (thread != NULL)
    {
        // This is a hack.  Though not pretty, it allows us to get the FreeRTOS
        // task handle from the pthread handle.
        typedef struct
        {
            TaskHandle_t freeRTOSTask;
        } PthreadObj;
        *thread = ((PthreadObj*)pthread)->freeRTOSTask;
    }
#else
#error Unsupported OS
#endif

    return result;
}

// 
// ::main_thread()
//
static void *main_thread(void *unused)
{
    char *argv[2] = {(char*)"openmrn", NULL};
#if defined (__FreeRTOS__)
    taskYIELD();
#else
#error Unsupported OS
#endif
    hw_postinit();

    appl_main(1, argv);

    abort();
    return NULL;
}

// 
// ::main()
//
int main(int argc, char *argv[])
{
    /* initialize the processor hardware */
    hw_init();

    /* stdin */
    if (open(STDIN_DEVICE, O_RDWR) < 0)
    {
        open("/dev/null", O_RDWR);
    }
    /* stdout */
    if (open(STDOUT_DEVICE, O_RDWR) < 0)
    {
        open("/dev/null", O_RDWR);
    }
    /* stderr */
    if (open(STDERR_DEVICE, O_WRONLY) < 0)
    {
        open("/dev/null", O_WRONLY);
    }

    int priority;
    
    if (config_main_thread_priority() == 0xdefa01)
    {
        priority = 0;
    }
    else
    {
        priority = config_main_thread_priority();
    }

    os_thread_t thread;
    os_thread_create(&thread, "thread.main", priority,
                     config_main_thread_stack_size(), main_thread, NULL);

    vTaskStartScheduler();
}

} // extern "C"

