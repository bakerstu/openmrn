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

#include <time.h>
#include <signal.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <sys/select.h>
#include "os/os.h"

/** Timer structure */
typedef struct timer
{
    struct timer *next; /**< next timer in the list */
    long long (*callback)(void*, void*); /**< timer's callback */
    void *data1; /**< timer's callback data */
    void *data2; /**< timer's callback data */
    long long when; /**< when in nanoseconds timer should expire */
    long long period; /**< period in nanoseconds for timer */
} Timer;

static Timer *active = NULL; /**< list of active timers */
static int timerfds[2]; /**< pipe used for refreshing the timer list */

/** One time initialization for timers. */
static os_thread_once_t timer_once = OS_THREAD_ONCE_INIT;

/** Mutex for timers. */
static os_mutex_t timerMutex = OS_MUTEX_INITIALIZER;

/** Define the signal we will use for timers
 */
#define WAKEUP_SIGNAL SIGRTMIN

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
        if (result < 0)
        {
            abort();
        }
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
    struct timespec ts;

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

#if defined (__nuttx__)
        clock_gettime(CLOCK_REALTIME, &ts);
#else
        clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
        long long now = ((long long)ts.tv_sec * 1000000000LL) + ts.tv_nsec;
        
        os_mutex_lock(&timerMutex);
        if (active)
        {
            if (active->when <= now)
            {
                /* remove timer from head of list */
                Timer *t = active;
                active = t->next;
                os_mutex_unlock(&timerMutex);

                long long next_period = (*t->callback)(t->data1, t->data2);

                os_mutex_lock(&timerMutex);
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
    if (result != 0)
    {
        abort();
    }
    fcntl(timerfds[0], F_SETFL, O_NONBLOCK);
    os_thread_create(&thread_handle, 0, 4096, timer_thread, NULL);
}

/** Create a new timer.
 * @param callback callback associated with timer
 * @param data1 data to pass along with callback
 * @param data2 data to pass along with callback
 * @return timer handle on success, else NULL
 */
os_timer_t os_timer_create(long long (*callback)(void*, void*), void *data1, void* data2)
{
    if (callback == NULL)
    {
        abort();
    }

    os_thread_once(&timer_once, os_timer_init);

    Timer *timer = malloc(sizeof(Timer));

    timer->callback = callback;
    timer->data1 = data1;
    timer->data2 = data2;
    timer->period = 0;

    return timer;
}

/** Delete a timer.
 * @param timer timer to delete
 */
void os_timer_delete(os_timer_t timer)
{
    if (timer == NULL)
    {
        abort();
    }

    Timer *t = timer;

    os_mutex_lock(&timerMutex);
    remove_timer(t);
    os_mutex_unlock(&timerMutex);
    free(t);
}

/** Start a timer.
 * @param timer timer to start
 * @param period period in nanoseconds before expiration
 */
void os_timer_start(os_timer_t timer, long long period)
{
    struct timespec ts;
    long long       timeout;
    Timer          *t = timer;

    if (timer == NULL)
    {
        abort();
    }

#if defined (__nuttx__)
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
    timeout = ((long long)ts.tv_sec * 1000000000LL) + ts.tv_nsec + period;

    os_mutex_lock(&timerMutex);
    /* Remove timer from the active list */
    remove_timer(t);

    t->when = timeout;
    t->period = period;
    /* insert the timer in the list */
    insert_timer(t);
    os_mutex_unlock(&timerMutex);
}

/** Delete a timer.
 * @param timer timer to stop
 */
void os_timer_stop(os_timer_t timer)
{
    if (timer == NULL)
    {
        abort();
    }

    Timer *t = timer;

    os_mutex_lock(&timerMutex);
    remove_timer(t);
    os_mutex_unlock(&timerMutex);
}


/** Create a thread.
 * @param thread handle to the created thread
 * @param priority priority of created thread
 * @param stack_size size in bytes of the created thread's stack
 * @param start_routine entry point of the thread
 * @param arg entry parameter to the thread
 * @return 0 upon success or error number upon failure
 */
int os_thread_create(os_thread_t *thread, int priority,
                     size_t stack_size,
                     void *(*start_routine) (void *), void *arg)
{
    pthread_attr_t attr;

    int result = pthread_attr_init(&attr);
    if (result != 0)
    {
        return result;
    }
#if !defined(__linux__) /* Linux allocates stack as needed */
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

    return result;
}


