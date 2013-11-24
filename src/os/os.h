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

#ifndef _os_h_
#define _os_h_

#include <sys/time.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <limits.h>

#if defined (__FreeRTOS__)
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
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

#ifdef __cplusplus
extern "C" {
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
typedef xTimerHandle os_timer_t; /**< timer handle */
typedef struct
{
    unsigned char state; /**< keep track if already executed */
} os_thread_once_t; /**< one time initialization type */
typedef xSemaphoreHandle os_sem_t; /**< semaphore handle */
typedef struct thread_priv
{
    struct _reent *reent; /**< newlib thread specific data (errno, etc...) */
    void *(*entry)(void*); /**< thread entry point */
    void *arg; /** argument to thread */
} ThreadPriv; /**< thread private data */
#else
typedef pthread_t os_thread_t; /**< thread handle */
typedef pthread_mutex_t os_mutex_t; /**< mutex handle */
typedef void *os_mq_t; /**< message queue handle */
typedef void *os_timer_t; /**< timer handle */
typedef pthread_once_t os_thread_once_t; /**< one time initialization type */
/** Some Operating Systems do not support timeouts with semaphores */
typedef struct
{
    pthread_cond_t cond;
    pthread_mutex_t mutex;
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

#if defined (__FreeRTOS__)
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

#if defined (__FreeRTOS__)
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
static inline int os_thread_once(os_thread_once_t *once, void (*routine)(void))
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

#define OS_TIMER_NONE 0LL /**< do not restart a timer */
#define OS_TIMER_RESTART 1LL /**< restart a timer with the last period */
#define OS_TIMER_DELETE -1LL /**< delete the timer */
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
                     void *(*start_routine) (void *), void *arg);

/** Destroy a thread.
 * @param thread handle to the created thread
 */
void os_thread_cancel(os_thread_t thread);

#if defined (__FreeRTOS__)
/** Static initializer for mutexes */
#define OS_MUTEX_INITIALIZER {NULL, 0}
/** Static initializer for recursive mutexes */
#define OS_RECURSIVE_MUTEX_INITIALIZER {NULL, 1}
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

/** Create a new timer.
 * @param callback callback associated with timer
 * @param data1 data to pass along with callback
 * @param data2 data to pass along with callback
 * @return timer handle on success, else NULL
 */
os_timer_t os_timer_create(long long (*callback)(void*, void*), void *data1, void* data2);

/** Delete a timer.
 * @param timer timer to delete
 */
void os_timer_delete(os_timer_t timer);

/** Start a timer.  This method shall not be used on a timer within its own
 * callback function.
 * @param timer timer to start
 * @param period period in nanoseconds before expiration
 */
void os_timer_start(os_timer_t timer, long long period);

/** Delete a timer.  This method shall not be used on a timer within its own
 * callback function.
 * @param timer timer to stop
 */
void os_timer_stop(os_timer_t timer);

/** Initialize mutex.
 * @param mutex address of mutex handle to initialize
 * @return 0 upon succes or error number upon failure
 */
static inline int os_mutex_init(os_mutex_t *mutex)
{
#if defined (__FreeRTOS__)
    mutex->recursive = 0;
    mutex->sem = xSemaphoreCreateMutex();

    return 0;    
#else
    return pthread_mutex_init(mutex, NULL);
#endif
}

/** Initialize recursive mutex.
 * @param mutex address of mutex handle to initialize
 * @return 0 upon succes or error number upon failure
 */
static inline int os_recursive_mutex_init(os_mutex_t *mutex)
{
#if defined (__FreeRTOS__)
    mutex->recursive = 1;
    mutex->sem = xSemaphoreCreateRecursiveMutex();

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
static inline int os_mutex_destroy(os_mutex_t *mutex)
{
#if defined (__FreeRTOS__)
    vSemaphoreDelete(mutex->sem);

    return 0;    
#else
    return pthread_mutex_destroy(mutex);
#endif
}

/** Lock a mutex.
 * @param mutex address of mutex handle to lock
 * @return 0 upon succes or error number upon failure
 */
static inline int os_mutex_lock(os_mutex_t *mutex)
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
#else
    return pthread_mutex_lock(mutex);
#endif
}

/** Unock a mutex.
 * @param mutex address of mutex handle to unlock
 * @return 0 upon succes or error number upon failure
 */
static inline int os_mutex_unlock(os_mutex_t *mutex)
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
#else
    return pthread_mutex_unlock(mutex);
#endif
}

/** Initialize a semaphore.
 * @param sem address of semaphore to initialize
 * @param value initial value of semaphore
 * @return 0 upon success
 */
static inline int os_sem_init(os_sem_t *sem, unsigned int value)
{
#if defined (__FreeRTOS__)
#if defined (GCC_ARMCM3) || defined(TARGET_LPC2368) || defined(TARGET_LPC11Cxx) || defined(TARGET_LPC1768) || defined(TARGET_PIC32MX)
    *sem = xSemaphoreCreateCounting(LONG_MAX, value);
    if (!*sem) {
      abort();
    }
#else
    #error Need to define your semaphore handling
#endif
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
static inline int os_sem_destroy(os_sem_t *sem)
{
#if defined (__FreeRTOS__)
#if defined (GCC_ARMCM3) || defined(TARGET_LPC2368) || defined(TARGET_LPC11Cxx) || defined(TARGET_LPC1768) || defined(TARGET_PIC32MX)
    vSemaphoreDelete(*sem);
#else
    #error Need to define your semaphore handling
#endif
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
static inline int os_sem_post(os_sem_t *sem)
{
#if defined (__FreeRTOS__)
    xSemaphoreGive(*sem);
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
static inline int os_sem_post_from_isr(os_sem_t *sem)
{
    portBASE_TYPE woken;
    xSemaphoreGiveFromISR(*sem, &woken);
    return 0;
}
#endif

/** Wait on a semaphore.
 * @param sem address of semaphore to decrement
 * @return 0 upon success
 */
static inline int os_sem_wait(os_sem_t *sem)
{
#if defined (__FreeRTOS__)
    xSemaphoreTake(*sem, portMAX_DELAY);
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

/** Wait on a semaphore with a timeout.
 * @param sem address of semaphore to decrement
 * @param timeout in nanoseconds, else OS_WAIT_FOREVER to wait forever
 * @return 0 upon success, else -1 with errno set to indicate error
 */
static inline int os_sem_timedwait(os_sem_t *sem, long long timeout)
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
static inline os_mq_t os_mq_create(size_t length, size_t item_size)
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
static inline void os_mq_send(os_mq_t queue, const void *data)
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
static inline int os_mq_timedsend(os_mq_t queue, const void *data, long long timeout)
{
#if defined (__FreeRTOS__)
    portTickType ticks = (timeout >> NSEC_TO_TICK_SHIFT);

    if (xQueueSend(queue, data, ticks) != pdTRUE)
    {
        return OS_MQ_TIMEDOUT;
    }
#endif
    return OS_MQ_NONE;
}


/** Blocking receive a message from a queue.
 * @param queue queue to receive message from
 * @param data location to copy message from the queue
 */
static inline void os_mq_receive(os_mq_t queue, void *data)
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
static inline int os_mq_timedreceive(os_mq_t queue, void *data, long long timeout)
{
#if defined (__FreeRTOS__)
    portTickType ticks = (timeout >> NSEC_TO_TICK_SHIFT);

    if (xQueueReceive(queue, data, ticks) != pdTRUE)
    {
        return OS_MQ_TIMEDOUT;
    }    
#endif
    return OS_MQ_NONE;
}

/** Send of a message to a queue from ISR context.
 * @param queue queue to send message to
 * @param data message to copy into queue
 * @param woken is the task woken up
 * @return OS_MQ_NONE on success, else OS_MQ_FULL
 */
static inline int os_mq_send_from_isr(os_mq_t queue, const void *data, int *woken)
{
#if defined (__FreeRTOS__)
    portBASE_TYPE local_woken;
    if (xQueueSendFromISR(queue, data, &local_woken) != pdTRUE)
    {
        return OS_MQ_FULL;
    }
    *woken |= local_woken;
#endif
    return OS_MQ_NONE;
}

/** Check if a queue is full from ISR context.
 * @param queue is the queue to check
 * @return non-zero if the queue is full.
 */
static inline int os_mq_is_full_from_isr(os_mq_t queue)
{
#if defined (__FreeRTOS__)
    return xQueueIsQueueFullFromISR(queue);
#endif
    return 1;
}


/** Receive a message from a queue from ISR context.
 * @param queue queue to receive message from
 * @param data location to copy message from the queue
 * @param woken is the task woken up
 * @return OS_MQ_NONE on success, else OS_MQ_FULL
 */
static inline int os_mq_receive_from_isr(os_mq_t queue, void *data, int *woken)
{
#if defined (__FreeRTOS__)
    portBASE_TYPE local_woken;
    if (xQueueReceiveFromISR(queue, data, &local_woken) != pdTRUE)
    {
        return OS_MQ_EMPTY;
    }
    *woken |= local_woken;
#endif
    return OS_MQ_NONE;
}

/** Return the number of messages pending in the queue.
 * @return number of messages in the queue
 */
static inline int os_mq_num_pending(os_mq_t queue)
{
#if defined (__FreeRTOS__)
    return uxQueueMessagesWaiting(queue);
#else
    return 0;
#endif
}

/** Return the number of messages pending in the queue from ISR context.
 * @return number of messages in the queue
 */
static inline int os_mq_num_pending_from_isr(os_mq_t queue)
{
#if defined (__FreeRTOS__)
    return uxQueueMessagesWaitingFromISR(queue);
#else
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

/** Test if we have woken up a higher priority task as the end of an interrupt.
 * @param _woken test value
 */
#define os_isr_exit_yield_test(_woken) \
do                                     \
{                                      \
    portEND_SWITCHING_ISR(_woken);     \
} while(0);
#endif

/** Get the monotonic time since the system started.
 * @return time in nanoseconds since system start
 */
static inline long long os_get_time_monotonic(void)
{
    static long long last = 0;
    long long time;
#if defined (__FreeRTOS__)
    portTickType tick = xTaskGetTickCount();
    time = ((long long)tick) << NSEC_TO_TICK_SHIFT;
#elif defined (__MACH__)
    /* get the timebase info */
    mach_timebase_info_data_t info;
    mach_timebase_info(&info);
    
    /* get the timestamp */
    time = (long long)mach_absolute_time();
    
    /* convert to nanoseconds */
    time *= info.numer;
    time /= info.denom;
#elif defined (__WIN32__)
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time = ((long long)tv.tv_sec * 1000LL * 1000LL * 1000LL) +
           ((long long)tv.tv_usec * 1000LL);
#else
    struct timespec ts;
#if defined (__nuttx__)
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
    time = ((long long)ts.tv_sec * 1000000000LL) + ts.tv_nsec;
    
#endif
    /* This logic ensures that every successive call is one value larger
     * than the last.  Each call returns a unique value.
     */
    if (time <= last)
    {
        last++;
    }
    else
    {
        last = time;
    }

    return last;
}

#if defined (__WIN32__)
/** Implementation of standard sleep().
 * @param seconds number of seconds to sleep
 */
static inline unsigned sleep(unsigned seconds)
{
    usleep(seconds * 1000);
    return 0;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* _os_h_ */
