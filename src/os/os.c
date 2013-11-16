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
 * \file os.c
 * This file represents a C language abstraction of common operating
 * system calls.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#if !defined (GCC_MEGA_AVR)
#include <unistd.h>
#endif
#if defined (__FreeRTOS__)
#include "devtab.h"
#include "FreeRTOS.h"
#include "task.h"
#else
#if defined(__WIN32__)
#include <winsock2.h>
#include <ws2tcpip.h> /* socklen_t */
#else
#include <sys/select.h>
#include <sched.h>
#endif
#include <time.h>
#include <signal.h>
#endif

#include "utils/macros.h"
#include "os/os.h"

/** default stdin */
extern const char *STDIN_DEVICE;

/** default stdout */
extern const char *STDOUT_DEVICE;

/** default stderr */
extern const char *STDERR_DEVICE;

#if !defined (__FreeRTOS__)
/* forward prototype */
static void os_timer_delete_locked(os_timer_t timer);
#endif

/** Timer structure */
typedef struct timer
{
#if !defined (__FreeRTOS__)
    struct timer *next; /**< next timer in the list */
#endif
    long long (*callback)(void*, void*); /**< timer's callback */
    void *data1; /**< timer's callback data */
    void *data2; /**< timer's callback data */
    long long when; /**< when in nanoseconds timer should expire */
    long long period; /**< period in nanoseconds for timer */
} Timer;

#if defined (__FreeRTOS__)
/** Task list entriy */
typedef struct task_list
{
    xTaskHandle task; /**< list entry data */
    char * name; /**< name of task */
    size_t unused; /**< number of bytes left unused in the stack */
    struct task_list *next; /**< next link in the list */
} TaskList;

/** List of all the tasks in the system */
static TaskList taskList;

/** Mutex for os_thread_once. */
static os_mutex_t onceMutex = OS_MUTEX_INITIALIZER;

/** Default hardware initializer.  This function is defined weak so that
 * a given board can stub in an intiailization specific to it.
 */
void hw_init(void) __attribute__ ((weak));
void hw_init(void)
{
}

__attribute__ ((weak))
struct _reent* allocate_reent(void)
{
    struct _reent* data = malloc(sizeof(struct _reent));
    _REENT_INIT_PTR(data);
    return data;
}

struct _reent* timerReent = NULL;

/** One time intialization routine
 * @param once one time instance
 * @param routine method to call once
 * @return 0 upon success
 */
int os_thread_once(os_thread_once_t *once, void (*routine)(void))
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* The scheduler has started so we should use locking */
        os_mutex_lock(&onceMutex);
        if (once->state == OS_THREAD_ONCE_NEVER)
        {
            once->state = OS_THREAD_ONCE_INPROGRESS;
            os_mutex_unlock(&onceMutex);
            routine();
            os_mutex_lock(&onceMutex);
            once->state = OS_THREAD_ONCE_DONE;
        }

        while (once->state == OS_THREAD_ONCE_INPROGRESS)
        {
            /* avoid dead lock waiting for PTHREAD_ONCE_DONE state */
            os_mutex_unlock(&onceMutex);
            /** @todo should we sleep here? */
            os_mutex_lock(&onceMutex);
        }
        os_mutex_unlock(&onceMutex);
    }
    else
    {
        /* this is for static constructures before the scheduler is started */
        if (once->state == OS_THREAD_ONCE_NEVER)
        {
            once->state = OS_THREAD_ONCE_INPROGRESS;
            routine();
            once->state = OS_THREAD_ONCE_DONE;
        }
   }

    return 0;
}

/** Callback function for handling of FreeRTOS timers.
 * @param timer timer to handle
 */
static void timer_callback(xTimerHandle timer)
{
    /* we must handle our struct _reent here at its first oportunity */
    _impure_ptr = timerReent;

    portTickType ticks;
    Timer *t = pvTimerGetTimerID(timer);
    do
    {
        long long next_period = (*t->callback)(t->data1, t->data2);
        switch (next_period)
        {
            case OS_TIMER_NONE:
                /* no need to restart timer */
                return;
            default:
                t->period = next_period;
                /* fall through */
            case OS_TIMER_RESTART:
                t->when += t->period;
                break;
        }
        long long now = os_get_time_monotonic();
        long long delay = t->when - now;
        if (delay < 0) delay = 0;
        ticks = (delay * configTICK_RATE_HZ) / (1000 * 1000 * 1000);
    } while (ticks == 0);
    xTimerChangePeriod(timer, ticks, portMAX_DELAY);
}
#else

#if defined (__WIN32__)
/** Windows does not support pipes, so we made our own with a pseudo socketpair.
 * @param fildes fildes[0] is open for reading, filedes[1] is open for writing
 * @return 0 upon success, else -1 with errno set to indicate error
 */
int pipe(int fildes[2])
{
    struct sockaddr_in addr;  
    int listener, connector, acceptor;
    socklen_t addrlen = sizeof(addr);

    if ((listener = socket(AF_INET, SOCK_STREAM, 0)) <= 0)
    {
        errno = EMFILE;
        return -1;
    }
    if ((connector = socket(AF_INET, SOCK_STREAM, 0)) <= 0)
    {
        closesocket(listener);
        errno = EMFILE;
        return -1;
    }
    
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0; 

    int reuse = 0;
    if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, 
                   (char*)&reuse, (socklen_t)sizeof(reuse)) < 0)
    {
        closesocket(listener);
        closesocket(connector);
        errno = EMFILE;
        return -1;
    }

    if (bind(listener, (const struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        closesocket(listener);
        closesocket(connector);
        errno = EMFILE;
        return -1;
    }
    
    if  (getsockname(listener, (struct sockaddr*)&addr, &addrlen) < 0)
    {
        closesocket(listener);
        closesocket(connector);
        errno = EMFILE;
        return -1;
    }

    if (listen(listener, 1) < 0)
    {
        closesocket(listener);
        closesocket(connector);
        errno = EMFILE;
        return -1;
    }

    if (connect(connector, (const struct sockaddr*)&addr, addrlen) < 0)
    {
        closesocket(listener);
        closesocket(connector);
        errno = EMFILE;
        return -1;
    }
   
    if ((acceptor = accept(listener, NULL, NULL)) < 0)
    {
        closesocket(listener);
        closesocket(connector);
        errno = EMFILE;
        return  -1;
    }

    int flag = 1;
    setsockopt(connector, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int));
    setsockopt(acceptor, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int));

    fildes[0] = connector;
    fildes[1] = acceptor;
    closesocket(listener);
    return 0;
}
#endif

static Timer *active = NULL; /**< list of active timers */
static int timerfds[2]; /**< pipe used for refreshing the timer list */

/** One time initialization for timers. */
static os_thread_once_t timer_once = OS_THREAD_ONCE_INIT;

/** Mutex for timers. */
static os_mutex_t timerMutex = OS_MUTEX_INITIALIZER;

/** Insert a timer into the active timer list.
 * @param timer timer to put in the list
 */
static void insert_timer(Timer *timer)
{
    Timer *tp = active;
    Timer *last = NULL;
    while (tp)
    {
        if (timer->when <= tp->when)
        {
            break;
        }
        last = tp;
        tp = tp->next;
    }
    if (last)
    {
        timer->next = last->next;
        last->next = timer;
    }
    else
    {
        timer->next = active;
        active = timer;
        /* wakeup and refresh timer list */
        char data = 1;
        int result = write(timerfds[1], &data, 1);
        HASSERT(result == 1);
    }
}

/** Remove a timer from the active timer list.
 * @param timer timer to remove from the list
 */
static void remove_timer(Timer *timer)
{
    /* Search the active list for this timer */
    Timer *tp = active;
    Timer *last = NULL;
    while (tp)
    {
        if (tp == timer)
        {
            /* Found the timer */
            break;
        }

        last = tp;
        tp = tp->next;
    }

    if (tp)
    {
        /* Remove the timer from the active list */
        if (last) {
            last->next = tp->next;
        } else {
            active = tp->next;
        }
    }
}

/** Thread for handling timer callbacks.
 * @param arg unused argument
 * @return never return
 */
static void *timer_thread(void* arg)
{
    struct timeval tv;

    for ( ; /* forever */ ; )
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(timerfds[0], &rfds);

        /* flush the pipe */
        ssize_t bytes_read;
        do
        {
            char buf[16];
            bytes_read = read(timerfds[0], buf, 16);
        } while (bytes_read > 0);

        long long now = os_get_time_monotonic();
        
        os_mutex_lock(&timerMutex);
        if (active)
        {
            if (active->when <= now)
            {
                /* remove timer from head of list */
                Timer *t = active;
                active = t->next;

                long long next_period = (*t->callback)(t->data1, t->data2);

                switch (next_period)
                {
                    case OS_TIMER_NONE:
                        break;
                    default:
                        t->period = next_period;
                        /* fall through */
                    case OS_TIMER_RESTART:
                        t->when += t->period;
                        insert_timer(t);
                        break;
                    case OS_TIMER_DELETE:
                        os_timer_delete_locked(t);
                        break;
                }
                os_mutex_unlock(&timerMutex);
                continue;
            }
            long long wait = active->when - now;
            os_mutex_unlock(&timerMutex);

            tv.tv_sec = wait / 1000000000LL;
            tv.tv_usec = (wait % 1000000000LL) / 1000LL;

            select(timerfds[0] + 1, &rfds, NULL, NULL, &tv);
        }
        else
        {
            os_mutex_unlock(&timerMutex);
            /* no active timers as of yet */
            select(timerfds[0] + 1, &rfds, NULL, NULL, NULL);
        }
    }
    return NULL;
}

/** Startup the timer thread.
 */
static void os_timer_init(void)
{
    os_thread_t thread_handle;

    int result = pipe(timerfds);
    HASSERT(result == 0);
#if defined (__WIN32__)
    u_long arg = 1;
    ioctlsocket(timerfds[0], FIONBIO, &arg);
#else
    fcntl(timerfds[0], F_SETFL, O_NONBLOCK);
#endif
    os_thread_create(&thread_handle, "thread.timer", 0, 4096, timer_thread, NULL);
}
#endif

/** Create a new timer.
 * @param callback callback associated with timer
 * @param data1 data to pass along with callback
 * @param data2 data to pass along with callback
 * @return timer handle on success, else NULL
 */
os_timer_t os_timer_create(long long (*callback)(void*, void*), void *data1, void* data2)
{
    HASSERT(callback != NULL);
    Timer *timer = malloc(sizeof(Timer));

    timer->callback = callback;
    timer->data1 = data1;
    timer->data2 = data2;
    timer->period = 0;
#if defined (__FreeRTOS__)
    return xTimerCreate(NULL, portMAX_DELAY, pdFALSE, timer, timer_callback);
#else
    os_thread_once(&timer_once, os_timer_init);
    return timer;
#endif
}

/** Delete a timer.
 * @param timer timer to delete
 */
void os_timer_delete(os_timer_t timer)
{
    HASSERT(timer != NULL);

#if defined (__FreeRTOS__)
    Timer *t = pvTimerGetTimerID(timer);
    xTimerDelete(timer, portMAX_DELAY);
#else
    Timer *t = timer;
    os_mutex_lock(&timerMutex);
    remove_timer(t);
    os_mutex_unlock(&timerMutex);
#endif
    free(t);
}

#if !defined (__FreeRTOS__)
/** Delete a timer with the mutex already locked.
 * @param timer timer to delete
 */
static void os_timer_delete_locked(os_timer_t timer)
{
    HASSERT(timer != NULL);

    Timer *t = timer;
    remove_timer(t);
    free(t);
}
#endif

/** Start a timer.
 * @param timer timer to start
 * @param period period in nanoseconds before expiration
 */
void os_timer_start(os_timer_t timer, long long period)
{
    HASSERT(timer != NULL);

#if defined (__FreeRTOS__)
    Timer          *t = pvTimerGetTimerID(timer);
    long long now = os_get_time_monotonic();
    t->when = now + period;
    t->period = period;
    portTickType ticks = (period * configTICK_RATE_HZ) / (1000 * 1000 * 1000);
    xTimerChangePeriod(timer, ticks, portMAX_DELAY);
#else
    Timer          *t = timer;
    long long timeout = os_get_time_monotonic() + period;

    os_mutex_lock(&timerMutex);
    /* Remove timer from the active list */
    remove_timer(t);

    t->when = timeout;
    t->period = period;
    /* insert the timer in the list */
    insert_timer(t);
    os_mutex_unlock(&timerMutex);
#endif
}

/** Stop a timer.
 * @param timer timer to stop
 */
void os_timer_stop(os_timer_t timer)
{
    HASSERT(timer != NULL);
#if defined (__FreeRTOS__)
    xTimerStop(timer, portMAX_DELAY);
#else
    Timer *t = timer;

    os_mutex_lock(&timerMutex);
    remove_timer(t);
    os_mutex_unlock(&timerMutex);
#endif
}

#if defined (__FreeRTOS__)
#if defined (TARGET_LPC2368) || defined(TARGET_LPC1768)
extern const char __ETHRAM_segment_start__;
static const char* sstack_start = &__ETHRAM_segment_start__;
extern const char __stacks_min__;

/** Custom malloc function for stack spaces.
 *  \param length the length of block to allocate
 *  \returns a pointer to the newly allocated block.
 *
 *  There is currently no way to free blocks allocated with this function, so
 *  only suitable for stacks of threads that are running throughout the entire
 *  life of the application.
 */
const void* stack_malloc(unsigned long length) {
  const char* old_stack_start = sstack_start;
  const char* new_stack_start = sstack_start + length;
  if (new_stack_start > &__stacks_min__) {
    diewith(BLINK_DIE_OUTOFMEMSTACK);
  }
  sstack_start = new_stack_start;
  return old_stack_start;
}
#elif defined(TARGET_LPC11Cxx)
__attribute__((noinline)) const void* stack_malloc(unsigned long length) {
    return malloc(length);
}
#else
#define stack_malloc malloc
#endif
#endif  // FreeRTOS

/** Entry point to a thread.
 * @param metadata for entering the thread
 */
#if defined (__FreeRTOS__)
static void os_thread_start(void *arg)
{
    ThreadPriv *priv = arg;
    vTaskSetApplicationTaskTag(NULL, arg);
    _impure_ptr = priv->reent;
    (*priv->entry)(priv->arg);
    free(priv);
}
#endif

/** Create a thread.
 * @param thread handle to the created thread
 * @param name name of thread, NULL for an auto generated name
 * @param priority priority of created thread, 0 means default
 * @param stack_size size in bytes of the created thread's stack
 * @param start_routine entry point of the thread
 * @param arg entry parameter to the thread
 * @return 0 upon success or error number upon failure
 */
int os_thread_create(os_thread_t *thread, const char *name, int priority,
                     size_t stack_size,
                     void *(*start_routine) (void *), void *arg)
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

#if defined (__FreeRTOS__)
    ThreadPriv *priv = malloc(sizeof(ThreadPriv));
    
    priv->entry = start_routine;
    priv->arg = arg;
    priv->reent = allocate_reent();
    
    if (priority == 0)
    {
        priority = configMAX_PRIORITIES / 2;
    }
    else
    {
        priority = configMAX_PRIORITIES - priority;
    }

    TaskList *current = &taskList;
    vTaskSuspendAll();
    while (current->next != NULL)
    {
        current = current->next;
    }
    
    TaskList *task_new = malloc(sizeof(TaskList));
    task_new->task = NULL;
    task_new->next = NULL;
    task_new->unused = stack_size;
    current->next = task_new;
    xTaskResumeAll();

    if (thread)
    {
        xTaskGenericCreate(os_thread_start,
                           (const signed char *const)name,
                           stack_size/sizeof(portSTACK_TYPE),
                           priv,
                           priority,
                           (xTaskHandle*)thread,
                           (long unsigned int*)stack_malloc(stack_size),
                           NULL);
        task_new->task = *thread;
        task_new->name = (char*)pcTaskGetTaskName(*thread);
    }
    else
    {
        xTaskHandle task_handle;
        xTaskGenericCreate(os_thread_start,
                           (const signed char *const)name,
                           stack_size/sizeof(portSTACK_TYPE),
                           priv,
                           priority,
                           (xTaskHandle*)&task_handle,
                           (long unsigned int*) stack_malloc(stack_size),
                           NULL);
        task_new->task = task_handle;
        task_new->name = (char*)pcTaskGetTaskName(task_handle);
    }

    return 0;
#else
    pthread_attr_t attr;

    int result = pthread_attr_init(&attr);
    if (result != 0)
    {
        return result;
    }
#if !defined(__linux__) && !defined(__MACH__) /* Linux allocates stack as needed */
    struct sched_param sched_param;
    result = pthread_attr_setstacksize(&attr, stack_size);
    if (result != 0)
    {
        return result;
    }

    sched_param.sched_priority = 0; /* default priority */
    result = pthread_attr_setschedparam(&attr, &sched_param);
    if (result != 0)
    {
        return result;
    }

    result = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (result != 0)
    {
        return result;
    }

    result = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (result != 0)
    {
        return result;
    }
#endif
    result = pthread_create(thread, &attr, start_routine, arg);

    pthread_setname_np(*thread, name);

    return result;
#endif
}

#if defined (__FreeRTOS__)
/* standard C library hooks for multi-threading */

/** Lock access to malloc.
 */
void __malloc_lock(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        vTaskSuspendAll();
    }
}

/** Unlock access to malloc.
 */
void __malloc_unlock(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xTaskResumeAll();
    }
}

/** Implementation of standard sleep().
 * @param seconds number of seconds to sleep
 */
unsigned sleep(unsigned seconds)
{
    vTaskDelay(seconds * configTICK_RATE_HZ);
    return 0;
}

/** Implementation of standard usleep().
 * @param usec number of microseconds to sleep
 */
int usleep(useconds_t usec)
{
    vTaskDelay((usec * configTICK_RATE_HZ) / (1000 * 1000));
    return 0;
}

void abort(void)
{
#if defined(TARGET_LPC2368) || defined(TARGET_LPC11Cxx) || defined(TARGET_LPC1768)
    blinker_pattern = BLINK_DIE_ABORT;
#endif
    for (;;)
    {
    }
}

extern char *heap_end;
char *heap_end = 0;
caddr_t _sbrk_r(struct _reent *reent, ptrdiff_t incr)
{
    extern char __cs3_heap_start;
    extern char __cs3_heap_end; /* Defined by the linker */
    char *prev_heap_end;
    if (heap_end == 0)
    {
        heap_end = &__cs3_heap_start;
    }
    prev_heap_end = heap_end;
    if (heap_end + incr > &__cs3_heap_end)
    {
        /* Heap and stack collistion */
        diewith(BLINK_DIE_OUTOFMEM);
        return 0;
    }
    heap_end += incr;
    return (caddr_t) prev_heap_end;
}

/** This method is called if a stack overflows its boundries.
 * @param task task handle for violating task
 * @param name name of violating task
 */
void vApplicationStackOverflowHook(xTaskHandle task, signed portCHAR *name)
{
    diewith(BLINK_DIE_STACKOVERFLOW);
}

/** This method will be called repeatedly from the idle task. If needed, it can
 * be overridden in hw_init.c.
 */
void hw_idle_hook(void) __attribute__((weak));

void hw_idle_hook(void)
{
}

/** Here we will monitor the other tasks.
 */
void vApplicationIdleHook( void )
{
    vTaskSuspendAll();
    xTaskResumeAll();
    hw_idle_hook();
    for (TaskList *tl = &taskList; tl != NULL; tl = tl->next)
    {
        if (tl->task)
        {
            tl->name = (char*)pcTaskGetTaskName(tl->task);
            tl->unused = uxTaskGetStackHighWaterMark(tl->task) * sizeof(portSTACK_TYPE);
        }
    }
}

/** Stack size of the main thread */
extern const size_t main_stack_size;


/** priority of the main thread */
extern const int main_priority;

/** Entry point to the main thread.
 * @param arg unused argument
 * @return NULL;
 */
void main_thread(void *arg)
{
    ThreadPriv *priv = arg;
    char *argv[2] = {"nmranet", NULL};
    vTaskSetApplicationTaskTag(NULL, arg);
    _impure_ptr = priv->reent;

    /* setup the monitoring entries for the timer and idle tasks */
    taskList.next = malloc(sizeof(TaskList)*2);
    taskList.next->task = xTimerGetTimerDaemonTaskHandle();
    taskList.next->unused = uxTaskGetStackHighWaterMark(taskList.next->task);
    taskList.next->next = taskList.next + 1;
    taskList.next->next->task = xTaskGetIdleTaskHandle();
    taskList.next->next->unused = uxTaskGetStackHighWaterMark(taskList.next->next->task);
    taskList.next->next->next = NULL;

    appl_main(1, argv);
    // If the main thread returns, FreeRTOS usually crashes the CPU in a
    // hard-to-debug state. Let's avoid that.
    abort();
}
#endif

#if !defined (__MINGW32__)
int main(int argc, char *argv[]) __attribute__ ((weak));
#endif

/** Entry point to program.
 * @param argc number of command line arguments
 * @param argv array of command line aguments
 * @return 0, should never return
 */
int main(int argc, char *argv[])
{
#if defined (__FreeRTOS__)
    /* initialize the processor hardware */
    hw_init();

    timerReent = allocate_reent();

    ThreadPriv *priv = malloc(sizeof(ThreadPriv));
    xTaskHandle task_handle;
    int priority;
    priv->reent = _impure_ptr;
    priv->entry = NULL;
    priv->arg = NULL;
    
    if (main_priority == 0)
    {
        priority = configMAX_PRIORITIES / 2;
    }
    else
    {
        priority = configMAX_PRIORITIES - main_priority;
    }
    
    /* initialize all the devices */
    for (devtab_t *dev = &DEVTAB[0]; dev != &DEVTAB_END; dev++)
    {
        dev->init(dev);
    }

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

    /* start the main thread */
    xTaskGenericCreate(main_thread,
                       (signed char *)"thread.main",
                       main_stack_size/sizeof(portSTACK_TYPE),
                       priv,
                       priority,
                       &task_handle,
                       (long unsigned int*)stack_malloc(main_stack_size),
                       NULL
                       );
    taskList.task = task_handle;
    taskList.unused = main_stack_size;
    taskList.name = "thread.main";

    vTaskStartScheduler();
#else
#if defined (__WIN32__)
    /* enable Windows networking */
    WSADATA wsa_data;
    WSAStartup(WINSOCK_VERSION, &wsa_data);
#endif
    return appl_main(argc, argv);
#endif
}
