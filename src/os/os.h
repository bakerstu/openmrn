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
 * \file os.h
 * This file represents a C language abstraction of common operating
 * system calls.
 *
 * @author Stuart W. Baker
 * @date 28 May 2012
 */

#ifndef _OS_OS_H_
#define _OS_OS_H_

#include <sys/time.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <limits.h>
#include <stdint.h>

#if defined (__FreeRTOS__)
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <event_groups.h>
#elif defined(ESP_NONOS)
#else
#include <pthread.h>
#include <semaphore.h>
#endif

#if defined (__MACH__)
#include <mach/mach_time.h>
#endif

#if defined (__WIN32__)
#include <sys/time.h>
#include <unistd.h>
#endif

#include "utils/macros.h"

#ifdef __cplusplus
extern "C" {
#endif


#ifndef OS_INLINE
/// Forces one definition of each inline function to be compiled.
#define OS_INLINE extern inline __attribute__((__gnu_inline__))
#endif

/** Entry point to application.
 * @param argc number of arguments
 * @param argv list of arguments
 * @return 0 upon success.
 */
int appl_main(int argc, char *argv[]);

#if defined (__FreeRTOS__)

extern void hw_init(void);

/** Stack size of the main thread */
extern const size_t main_stack_size;

/** priority of the main thread */
extern const int main_priority;

typedef xTaskHandle os_thread_t; /**< thread handle */
typedef struct
{
    xSemaphoreHandle sem; /**< FreeRTOS mutex handle */
    char recursive; /**< recursive mutex if set */
} os_mutex_t; /**< mutex handle */
typedef xQueueHandle os_mq_t; /**< message queue handle */
typedef struct
{
    unsigned char state; /**< keep track if already executed */
} os_thread_once_t; /**< one time initialization type */
typedef xSemaphoreHandle os_sem_t; /**< semaphore handle */
typedef struct thread_priv
{
    struct _reent *reent; /**< newlib thread specific data (errno, etc...) */
    EventBits_t selectEventBit; /**< bit used for waking up from select */
    void *(*entry)(void*); /**< thread entry point */
    void *arg; /** argument to thread */
} ThreadPriv; /**< thread private data */
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
typedef struct {
    int locked;
    uint8_t recursive;
} os_mutex_t;
typedef struct
{
    unsigned char state; /**< keep track if already executed */
} os_thread_once_t; /**< one time initialization type */
typedef struct {
    unsigned counter;
} os_sem_t;

typedef unsigned os_thread_t;
typedef void *os_mq_t; /**< message queue handle */

#else
typedef pthread_t os_thread_t; /**< thread handle */
typedef pthread_mutex_t os_mutex_t; /**< mutex handle */
typedef void *os_mq_t; /**< message queue handle */
typedef pthread_once_t os_thread_once_t; /**< one time initialization type */
/** Some Operating Systems do not support timeouts with semaphores */
typedef struct
{
    /// Condition variable.
    pthread_cond_t cond;
    /// Mutex protectin the counter
    pthread_mutex_t mutex;
    /// How many counts doe the semaphore store.
    int counter;
} os_sem_t;
#endif

#ifndef container_of
/** Get a pointer to the parent structure of one of its members.
 * @param _ptr original member pointer
 * @param _type parent structure type
 * @param _member name of member within structure
 * @return pointer to the parent structure
 */
#define container_of(_ptr, _type, _member)                  \
({                                                       \
    const typeof( ((_type *)0)->_member ) *__mptr = (_ptr); \
    (_type *)( (char *)__mptr - offsetof(_type,_member) );  \
})
#endif

/** Get the monotonic time since the system started.
 * @return time in nanoseconds since system start
 */
extern long long os_get_time_monotonic(void);

#if defined (__FreeRTOS__) || defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
/** @ref os_thread_once states.
 */
enum
{
    OS_THREAD_ONCE_NEVER = 0, ///< not yet executed
    OS_THREAD_ONCE_INPROGRESS, ///< execution in progress
    OS_THREAD_ONCE_DONE ///< execution complete
};
/** initial value for one time intitialization instance */
#define OS_THREAD_ONCE_INIT { OS_THREAD_ONCE_NEVER }
#else
/** initial value for one time intitialization instance */
#define OS_THREAD_ONCE_INIT PTHREAD_ONCE_INIT
#endif

#if defined (__FreeRTOS__) || defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
/** One time intialization routine
 * @param once one time instance
 * @param routine method to call once
 * @return 0 upon success
 */
int os_thread_once(os_thread_once_t *once, void (*routine)(void));
#else
/** One time intialization routine
 * @param once one time instance
 * @param routine method to call once
 * @return 0 upon success
 */
OS_INLINE int os_thread_once(os_thread_once_t *once, void (*routine)(void))
{
    return pthread_once(once, routine);
}
#endif

#define OS_PRIO_MIN 1 /**< lowest thread priority supported by abstraction */
#define OS_PRIO_DEFAULT 0 /**< default thread priority */
#define OS_PRIO_MAX 32 /**< highest thread priority suported by abstraction */

#define OS_MQ_NONE     0 /**< error code for no error for message queues */
#define OS_MQ_TIMEDOUT 1 /**< error code for timedout for message queues */
#define OS_MQ_EMPTY    2 /**< error code for the queue being empty */
#define OS_MQ_FULL     3 /**< error code for queue being full */

#if defined LLONG_MAX
#define OS_WAIT_FOREVER LLONG_MAX /**< maximum timeout period */
#else
#define OS_WAIT_FOREVER __LONG_LONG_MAX__ /**< maximum timeout period */
#endif

/** Convert a nanosecond value to a microsecond value.
 * @param _nsec nanosecond value to convert
 * @return microsecond value
 */
#define NSEC_TO_USEC(_nsec) (((long long)_nsec) / 1000LL)

/** Convert a nanosecond value to a millisecond value.
 * @param _nsec nanosecond value to convert
 * @return milliosecond value
 */
#define NSEC_TO_MSEC(_nsec) (((long long)_nsec) / 1000000LL)

/** Convert a nanosecond value to a second value.
 * @param _nsec nanosecond value to convert
 * @return second value
 */
#define NSEC_TO_SEC(_nsec) (((long long)_nsec) / 1000000000LL)

/** Convert a microsecond value to a nanosecond value.
 * @param _usec microsecond value to convert
 * @return nanosecond value
 */
#define USEC_TO_NSEC(_usec) (((long long)_usec) * 1000LL)

/** Convert a microsecond value to a millisecond value.
 * @param _usec microsecond value to convert
 * @return millisecond value
 */
#define USEC_TO_MSEC(_usec) (((long long)_usec) / 1000LL)

/** Convert a microsecond value to a second value.
 * @param _usec microsecond value to convert
 * @return second value
 */
#define USEC_TO_SEC(_usec) (((long long)_usec) / 1000000LL)

/** Convert a millisecond value to a nanosecond value.
 * @param _msec millisecond value to convert
 * @return nanosecond value
 */
#define MSEC_TO_NSEC(_msec) (((long long)_msec) * 1000000LL)

/** Convert a millisecond value to a microsecond value.
 * @param _msec millisecond value to convert
 * @return microsecond value
 */
#define MSEC_TO_USEC(_msec) (((long long)_msec) * 1000LL)

/** Convert a millisecond value to a second value.
 * @param _msec millisecond value to convert
 * @return second value
 */
#define MSEC_TO_SEC(_msec) (((long long)_msec) / 1000LL)

/** Convert a second value to a nanosecond value.
 * @param _sec second value to convert
 * @return nanosecond value
 */
#define SEC_TO_NSEC(_sec) (((long long)_sec) * 1000000000LL)

/** Convert a second value to a microsecond value.
 * @param _sec second value to convert
 * @return microsecond value
 */
#define SEC_TO_USEC(_sec) (((long long)_sec) * 1000000LL)

/** Convert a second value to a millisecond value.
 * @param _sec second value to convert
 * @return millisecond value
 */
#define SEC_TO_MSEC(_sec) (((long long)_sec) * 1000LL)

/** Creates a thread.
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
                     void *(*start_routine) (void *), void *arg);

/** Destroy a thread.
 * @param thread handle to the created thread
 */
void os_thread_cancel(os_thread_t thread);

/** Return a handle to the calling thread.
 * @return a handle to the calling thread
 */
OS_INLINE os_thread_t os_thread_self(void)
{
#if defined (__FreeRTOS__)
    return xTaskGetCurrentTaskHandle();
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    return 0xdeadbeef;
#else
    return pthread_self();
#endif
}

/** Return the current thread priority.
 * @param thread handle to thread of interest
 * @return current thread priority
 */
OS_INLINE int os_thread_getpriority(os_thread_t thread)
{
#if defined (__FreeRTOS__)
    return uxTaskPriorityGet(thread);
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    return 2;
#else
    struct sched_param params;
    int policy;
    pthread_getschedparam(thread, &policy, &params);
    return params.sched_priority;
#endif
}

#if defined (__FreeRTOS__)
/** Static initializer for mutexes */
#define OS_MUTEX_INITIALIZER {NULL, 0}
/** Static initializer for recursive mutexes */
#define OS_RECURSIVE_MUTEX_INITIALIZER {NULL, 1}
#elif defined(__EMSCRIPTEN__)
/** Static initializer for mutexes */
#define OS_MUTEX_INITIALIZER {0, 0}
/** Static initializer for recursive mutexes */
#define OS_RECURSIVE_MUTEX_INITIALIZER {0, 1}
#else
/** Static initializer for mutexes */
#define OS_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER

#if defined (__nuttx__)
/** Static initializer for recursive mutexes */
#define OS_RECURSIVE_MUTEX_INITIALIZER {0, SEM_INITIALIZER(1), PTHREAD_MUTEX_RECURSIVE, 0}
#elif defined (__MACH__)
#define OS_RECURSIVE_MUTEX_INITIALIZER PTHREAD_RECURSIVE_MUTEX_INITIALIZER
#else
/** Static initializer for recursive mutexes */
#define OS_RECURSIVE_MUTEX_INITIALIZER PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP
#endif
#endif

#ifdef __EMSCRIPTEN__
extern void os_emscripten_yield();
#endif

/** Initialize mutex.
 * @param mutex address of mutex handle to initialize
 * @return 0 upon succes or error number upon failure
 */
OS_INLINE int os_mutex_init(os_mutex_t *mutex)
{
#if defined (__FreeRTOS__)
    mutex->recursive = 0;
    mutex->sem = xSemaphoreCreateMutex();

    return 0;    
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    mutex->locked = 0;
    mutex->recursive = 0;
    return 0;
#else
    return pthread_mutex_init(mutex, NULL);
#endif
}

/** Initialize recursive mutex.
 * @param mutex address of mutex handle to initialize
 * @return 0 upon succes or error number upon failure
 */
OS_INLINE int os_recursive_mutex_init(os_mutex_t *mutex)
{
#if defined (__FreeRTOS__)
    mutex->recursive = 1;
    mutex->sem = xSemaphoreCreateRecursiveMutex();

    return 0;    
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    mutex->locked = 0;
    mutex->recursive = 1;
    return 0;
#else
    pthread_mutexattr_t attr;
    int result;

    result = pthread_mutexattr_init(&attr);
    if (result != 0)
    {
        return result;
    }

    result = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    if (result != 0)
    {
        return result;
    }

    return pthread_mutex_init(mutex, &attr);
#endif
}

/** Destroy a mutex.
 * @param mutex address of mutex handle to destroy
 * @return 0 upon succes or error number upon failure
 */
OS_INLINE int os_mutex_destroy(os_mutex_t *mutex)
{
#if defined (__FreeRTOS__)
    vSemaphoreDelete(mutex->sem);

    return 0;    
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    mutex->locked = 0;
    return 0;
#else
    return pthread_mutex_destroy(mutex);
#endif
}

/** Lock a mutex.
 * @param mutex address of mutex handle to lock
 * @return 0 upon succes or error number upon failure
 */
OS_INLINE int os_mutex_lock(os_mutex_t *mutex)
{
#if (__FreeRTOS__)
    vTaskSuspendAll();
    if (mutex->sem == NULL)
    {
        if (mutex->recursive)
        {
            mutex->sem = xSemaphoreCreateRecursiveMutex();
        }
        else
        {
            mutex->sem = xSemaphoreCreateMutex();
        }
    }
    xTaskResumeAll();

    if (mutex->recursive)
    {
        xSemaphoreTakeRecursive(mutex->sem, portMAX_DELAY);
    }
    else
    {
        xSemaphoreTake(mutex->sem, portMAX_DELAY);
    }
    return 0;
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    if (mutex->locked && !mutex->recursive)
    {
        DIE("Mutex deadlock.");
    }
    mutex->locked++;
    return 0;
#else
    return pthread_mutex_lock(mutex);
#endif
}

/** Unock a mutex.
 * @param mutex address of mutex handle to unlock
 * @return 0 upon succes or error number upon failure
 */
OS_INLINE int os_mutex_unlock(os_mutex_t *mutex)
{
#if defined (__FreeRTOS__)
    if (mutex->recursive)
    {
        xSemaphoreGiveRecursive(mutex->sem);
    }
    else
    {
        xSemaphoreGive(mutex->sem);
    }
    return 0;
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    if (mutex->locked <= 0)
    {
        DIE("Unlocking a not locked mutex");
    }
    --mutex->locked;
    return 0;
#else
    return pthread_mutex_unlock(mutex);
#endif
}

/** Initialize a semaphore.
 * @param sem address of semaphore to initialize
 * @param value initial value of semaphore
 * @return 0 upon success
 */
OS_INLINE int os_sem_init(os_sem_t *sem, unsigned int value)
{
#if defined (__FreeRTOS__)
    *sem = xSemaphoreCreateCounting(LONG_MAX, value);
    if (!*sem) {
      abort();
    }
    return 0;
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    sem->counter = value;
    return 0;
#else
    pthread_cond_init(&sem->cond, NULL);
    pthread_mutex_init(&sem->mutex, NULL);
    sem->counter = value;
    return 0;
#endif
}

/** Destroy a semaphore.
 * @param sem address of semaphore to destroy
 * @return 0 upon success
 */
OS_INLINE int os_sem_destroy(os_sem_t *sem)
{
#if defined (__FreeRTOS__)
    vSemaphoreDelete(*sem);
    return 0;
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    return 0;
#else
    pthread_cond_destroy(&sem->cond);
    pthread_mutex_destroy(&sem->mutex);
    return 0;
#endif
}

/** Post a semaphore.
 * @param sem address of semaphore to increment
 * @return 0 upon success
 */
OS_INLINE int os_sem_post(os_sem_t *sem)
{
#if defined (__FreeRTOS__)
    xSemaphoreGive(*sem);
    return 0;
#elif defined(__EMSCRIPTEN__) || defined(ESP_NONOS)
    sem->counter++;
    return 0;
#else
    pthread_mutex_lock(&sem->mutex);
    sem->counter++;
    pthread_cond_signal(&sem->cond);
    pthread_mutex_unlock(&sem->mutex);
    return 0;
#endif
}

#if defined (__FreeRTOS__)
/** Post a semaphore from the ISR context.
 * @param sem address of semaphore to increment
 * @param woken is the task woken up
 * @return 0 upon success
 */
OS_INLINE int os_sem_post_from_isr(os_sem_t *sem, int *woken)
{
    portBASE_TYPE local_woken;
    xSemaphoreGiveFromISR(*sem, &local_woken);
    *woken |= local_woken;
    return 0;
}
#endif

/** Wait on a semaphore.
 * @param sem address of semaphore to decrement
 * @return 0 upon success
 */
OS_INLINE int os_sem_wait(os_sem_t *sem)
{
#if defined (__FreeRTOS__)
    xSemaphoreTake(*sem, portMAX_DELAY);
    return 0;
#elif defined(__EMSCRIPTEN__)
    while (!sem->counter)
    {
        os_emscripten_yield();
    }
    --sem->counter;
    return 0;
#elif defined(ESP_NONOS)
    if (!sem->counter) {
        DIE("Semaphore deadlock.");
    }
    --sem->counter;
    return 0;
#else
    pthread_mutex_lock(&sem->mutex);
    while (sem->counter == 0)
    {
        pthread_cond_wait(&sem->cond, &sem->mutex);
    }
    sem->counter--;
    pthread_mutex_unlock(&sem->mutex);
    return 0;
#endif
}

#ifndef ESP_NONOS
/** Wait on a semaphore with a timeout.
 * @param sem address of semaphore to decrement
 * @param timeout in nanoseconds, else OS_WAIT_FOREVER to wait forever
 * @return 0 upon success, else -1 with errno set to indicate error
 */
OS_INLINE int os_sem_timedwait(os_sem_t *sem, long long timeout)
{
    if (timeout == OS_WAIT_FOREVER)
    {
        return os_sem_wait(sem);
    }
#if defined (__FreeRTOS__)
    if (xSemaphoreTake(*sem, timeout >> NSEC_TO_TICK_SHIFT) == pdTRUE)
    {
        return 0;
    }
    else
    {
        errno = ETIMEDOUT;
        return -1;
    }
#elif defined(__EMSCRIPTEN__)
    long long end_time = 0;
    do
    {
        if (sem->counter)
        {
            --sem->counter;
            return 0;
        }
        else if (end_time)
        {
            errno = ETIMEDOUT;
            return -1;
        }
        end_time = os_get_time_monotonic() + timeout;
        while (!sem->counter && os_get_time_monotonic() < end_time)
        {
            os_emscripten_yield();
        }
    } while(1);
#else
    struct timeval tv;
    struct timespec ts;
    gettimeofday(&tv, NULL);
    timeout += ((long long)tv.tv_sec * 1000000000LL) + ((long long) tv.tv_usec * 1000LL);
    ts.tv_sec = timeout / 1000000000LL;
    ts.tv_nsec = timeout % 1000000000LL;
    pthread_mutex_lock(&sem->mutex);
    while (sem->counter == 0)
    {
        if (pthread_cond_timedwait(&sem->cond, &sem->mutex, &ts) == ETIMEDOUT)
        {
            pthread_mutex_unlock(&sem->mutex);
            errno = ETIMEDOUT;
            return -1;
        }
    }
    sem->counter--;
    pthread_mutex_unlock(&sem->mutex);
    return 0;
#endif
}

#endif // #ifndef ESP_NONOS


#if !defined (__FreeRTOS__)
/** Private data structure for a queue, do not use directly
 */
typedef struct queue_priv
{
    os_sem_t semSend; /**< able to send semaphore */
    os_sem_t semReceive; /**< able to receive semaphore */
    char *buffer; /**< queue data */
    size_t itemSize; /**< size of each item in the queue */
    size_t bytes; /**< number of bytes that make up the queue */
    unsigned int indexSend; /**< current index for send */
    unsigned int indexReceive; /**< current index for receive */
    os_mutex_t mutex; /**< mutex to protect queue operations */
} QueuePriv;
#endif

/** Create a new message queue.
 * @param length length in number of messages of the queue
 * @param item_size size in number of bytes of a message
 * @return handle to the created queue, NULL on failure
 */
OS_INLINE os_mq_t os_mq_create(size_t length, size_t item_size)
{
#if defined (__FreeRTOS__)
    return xQueueCreate(length, item_size);
#else
    QueuePriv *q = (QueuePriv*)malloc(sizeof(QueuePriv));
    if (!q)
    {
        errno = ENOMEM;
        return NULL;
    }
    os_sem_init(&q->semSend, length);
    os_sem_init(&q->semReceive, 0);
    q->buffer = (char*)malloc(length * item_size);
    q->itemSize = item_size;
    q->bytes = length * item_size;
    q->indexSend = 0;
    q->indexReceive = 0;
    os_mutex_init(&q->mutex);

    return q;
#endif
}

/** Blocking send of a message to a queue.
 * @param queue queue to send message to
 * @param data message to copy into queue
 */
OS_INLINE void os_mq_send(os_mq_t queue, const void *data)
{
#if defined (__FreeRTOS__)
    xQueueSend(queue, data, portMAX_DELAY);
#else
    QueuePriv *q = (QueuePriv*)queue;
    
    os_sem_wait(&q->semSend);

    os_mutex_lock(&q->mutex);
    memcpy(q->buffer + q->indexSend, data, q->itemSize);
    q->indexSend += q->itemSize;
    if (q->indexSend >= q->bytes)
    {
        q->indexSend = 0;
    }
    os_mutex_unlock(&q->mutex);
    os_sem_post(&q->semReceive);
#endif
}

/** Send a message to a queue with a timeout.
 * @param queue queue to send message to
 * @param data message to copy into queue
 * @param timeout time in nanoseconds to wait for queue to be able to accept message
 * @return OS_MQ_NONE on success, OS_MQ_TIMEDOUT on timeout
 */
OS_INLINE int os_mq_timedsend(os_mq_t queue, const void *data, long long timeout)
{
#if defined (__FreeRTOS__)
    portTickType ticks = (timeout >> NSEC_TO_TICK_SHIFT);
    
    if (xQueueSend(queue, data, ticks) != pdTRUE)
    {
        return OS_MQ_TIMEDOUT;
    }
#else
    DIE("unimplemented.");
#endif
    return OS_MQ_NONE;
}


/** Blocking receive a message from a queue.
 * @param queue queue to receive message from
 * @param data location to copy message from the queue
 */
OS_INLINE void os_mq_receive(os_mq_t queue, void *data)
{
#if defined (__FreeRTOS__)
    xQueueReceive(queue, data, portMAX_DELAY);
#else
    QueuePriv *q = (QueuePriv*)queue;
    
    os_sem_wait(&q->semReceive);

    os_mutex_lock(&q->mutex);
    memcpy(q->buffer + q->indexReceive, data, q->itemSize);
    q->indexReceive += q->itemSize;
    if (q->indexReceive >= q->bytes)
    {
        q->indexReceive = 0;
    }
    os_mutex_unlock(&q->mutex);
    os_sem_post(&q->semSend);
#endif
}

/** Receive a message from a queue.
 * @param queue queue to receive message from
 * @param data location to copy message from the queue
 * @param timeout time in nanoseconds to wait for queue to have a message available
 * @return OS_MQ_NONE on success, OS_MQ_TIMEDOUT on timeout
 */
OS_INLINE int os_mq_timedreceive(os_mq_t queue, void *data, long long timeout)
{
#if defined (__FreeRTOS__)
    portTickType ticks = (timeout >> NSEC_TO_TICK_SHIFT);

    if (xQueueReceive(queue, data, ticks) != pdTRUE)
    {
        return OS_MQ_TIMEDOUT;
    }
#else
    HASSERT(0);    
#endif
    return OS_MQ_NONE;
}

/** Send of a message to a queue from ISR context.
 * @param queue queue to send message to
 * @param data message to copy into queue
 * @param woken is the task woken up
 * @return OS_MQ_NONE on success, else OS_MQ_FULL
 */
OS_INLINE int os_mq_send_from_isr(os_mq_t queue, const void *data, int *woken)
{
#if defined (__FreeRTOS__)
    portBASE_TYPE local_woken;
    if (xQueueSendFromISR(queue, data, &local_woken) != pdTRUE)
    {
        return OS_MQ_FULL;
    }
    *woken |= local_woken;
#else
    HASSERT(0);    
#endif
    return OS_MQ_NONE;
}

/** Check if a queue is full from ISR context.
 * @param queue is the queue to check
 * @return non-zero if the queue is full.
 */
OS_INLINE int os_mq_is_full_from_isr(os_mq_t queue)
{
#if defined (__FreeRTOS__)
    return xQueueIsQueueFullFromISR(queue);
#else
    HASSERT(0);    
#endif
    return 1;
}


/** Receive a message from a queue from ISR context.
 * @param queue queue to receive message from
 * @param data location to copy message from the queue
 * @param woken is the task woken up
 * @return OS_MQ_NONE on success, else OS_MQ_FULL
 */
OS_INLINE int os_mq_receive_from_isr(os_mq_t queue, void *data, int *woken)
{
#if defined (__FreeRTOS__)
    portBASE_TYPE local_woken;
    if (xQueueReceiveFromISR(queue, data, &local_woken) != pdTRUE)
    {
        return OS_MQ_EMPTY;
    }
    *woken |= local_woken;
#else
    HASSERT(0);    
#endif
    return OS_MQ_NONE;
}

/** Return the number of messages pending in the queue.
 * @param queue queue to check
 * @return number of messages in the queue
 */
OS_INLINE int os_mq_num_pending(os_mq_t queue)
{
#if defined (__FreeRTOS__)
    return uxQueueMessagesWaiting(queue);
#else
    HASSERT(0);    
    return 0;
#endif
}

/** Return the number of messages pending in the queue from ISR context.
 * @param queue queue to check
 * @return number of messages in the queue
 */
OS_INLINE int os_mq_num_pending_from_isr(os_mq_t queue)
{
#if defined (__FreeRTOS__)
    return uxQueueMessagesWaitingFromISR(queue);
#else
    HASSERT(0);    
    return 0;
#endif
}

/** Return the number of spaces available in the queue.
 * @param queue queue to check
 * @return number of spaces available
 */
OS_INLINE int os_mq_num_spaces(os_mq_t queue)
{
#if defined (__FreeRTOS__)
    return uxQueueSpacesAvailable(queue);
#else
    HASSERT(0);
    return 0;
#endif
}

#if defined (__FreeRTOS__)
/** Some of the older ports of FreeRTOS don't yet have this macro, so define it.
 */
#if !defined (portEND_SWITCHING_ISR)
#define portEND_SWITCHING_ISR(_woken) \
    if( _woken )                      \
    {                                 \
        portYIELD_FROM_ISR();         \
    }
#endif

#ifdef TARGET_PIC32MX

void __attribute__((nomips16)) os_isr_exit_yield_test(int woken);

#else
/** Test if we have woken up a higher priority task as the end of an interrupt.
 * @param _woken test value
 */
#define os_isr_exit_yield_test(_woken) \
do                                     \
{                                      \
    portEND_SWITCHING_ISR(_woken);     \
} while(0);

#endif // PIC32 or general
#endif

/** Get the monotonic time since the system started.
 * @return time in nanoseconds since system start
 */
extern long long os_get_time_monotonic(void);

#if defined (__WIN32__)
/** Implementation of standard sleep().
 * @param seconds number of seconds to sleep
 */
OS_INLINE unsigned sleep(unsigned seconds)
{
    usleep(seconds * 1000);
    return 0;
}
#endif

/* This section of code is required because CodeSourcery's mips-gcc
 * distribution contains a strangely compiled NewLib (in the unhosted-libc.a
 * version) that does not forward these function calls to the implementations
 * we have. We are thus forced to override their weak definition of these
 * functions. */
#if defined(TARGET_PIC32MX) || defined(ESP_NONOS)
#include "reent.h"

OS_INLINE int open(const char* b, int flags, ...) {
    return _open_r(_impure_ptr, b, flags, 0);
}
OS_INLINE int close(int fd) {
    return _close_r(_impure_ptr, fd);
}
OS_INLINE ssize_t read(int fd, void* buf, size_t count) {
    return _read_r(_impure_ptr, fd, buf, count);
}
OS_INLINE ssize_t write(int fd, const void* buf, size_t count) {
    return _write_r(_impure_ptr, fd, buf, count);
}
OS_INLINE off_t lseek(int fd, off_t offset, int whence) {
    return _lseek_r(_impure_ptr, fd, offset, whence);
}
OS_INLINE int fstat(int fd, struct stat* buf) {
    return _fstat_r(_impure_ptr, fd, buf);
}

#endif

#ifdef __cplusplus
}
#endif

#endif /* _OS_OS_H_ */
