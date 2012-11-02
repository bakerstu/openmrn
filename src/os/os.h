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

#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef pthread_t os_thread_t; /**< thread handle */
typedef pthread_mutex_t os_mutex_t; /**< mutex handle */
typedef void *os_mq_t; /**< message queue handle */
typedef void *os_timer_t; /**< timer handle */
typedef pthread_once_t os_thread_once_t; /**< one time initialization type */
typedef sem_t os_sem_t; /**< semaphore handle */

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

/** initial value for one time intitialization instance */
#define OS_THREAD_ONCE_INIT PTHREAD_ONCE_INIT

/** One time intialization routine
 * @param once one time instance
 * @param routine method to call once
 * @return 0 upon success
 */
static inline int os_thread_once(os_thread_once_t *once, void (*routine)(void))
{
    return pthread_once(once, routine);
}

#define OS_PRIO_MIN 1 /**< lowest thread priority supported by abstraction */
#define OS_PRIO_DEFAULT 0 /**< default thread priority */
#define OS_PRIO_MAX 32 /**< highest thread priority suported by abstraction */

#define OS_MQ_NONE 0 /**< error code for no error for message queues */
#define OS_MQ_TIMEDOUT 1 /**< error code for timedout for message queues */

#define OS_TIMER_NONE 0LL /**< do not restart a timer */
#define OS_TIMER_RESTART 1LL /**< restart a timer with the last period */

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
 * @param priority priority of created thread
 * @param stack_size size in bytes of the created thread's stack
 * @param start_routine entry point of the thread
 * @param arg entry parameter to the thread
 * @return 0 upon success or error number upon failure
 */
int os_thread_create(os_thread_t *thread, int priority,
                     size_t stack_size,
                     void *(*start_routine) (void *), void *arg);

/** Static initializer for mutexes */
#define OS_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER

/** Static initializer for recursive mutexes */
#define OS_RECURSIVE_MUTEX_INITIALIZER PTHREAD_RECURSIVE_MUTEX_INITIALIZER

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
    return pthread_mutex_init(mutex, NULL);
}

/** Initialize recursive mutex.
 * @param mutex address of mutex handle to initialize
 * @return 0 upon succes or error number upon failure
 */
static inline int os_recursive_mutex_init(os_mutex_t *mutex)
{
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
}

/** Lock a mutex.
 * @param mutex address of mutex handle to lock
 * @return 0 upon succes or error number upon failure
 */
static inline int os_mutex_lock(os_mutex_t *mutex)
{
   return pthread_mutex_lock(mutex);
}

/** Unock a mutex.
 * @param mutex address of mutex handle to unlock
 * @return 0 upon succes or error number upon failure
 */
static inline int os_mutex_unlock(os_mutex_t *mutex)
{
   return pthread_mutex_unlock(mutex);
}

/** Initialize a semaphore.
 * @param sem address of semaphore to initialize
 * @param value initial value of semaphore
 * @return 0 upon success
 */
static inline int os_sem_init(os_sem_t *sem, unsigned int value)
{
    return sem_init(sem, 0, value);
}

/** Post a semaphore.
 * @param sem address of semaphore to increment
 * @return 0 upon success
 */
static inline int os_sem_post(os_sem_t *sem)
{
    return sem_post(sem);
}

/** Wait on a semaphore.
 * @param sem address of semaphore to decrement
 * @return 0 upon success
 */
static inline int os_sem_wait(os_sem_t *sem)
{
    return sem_wait(sem);
}

/** Private data structure for a queue, do not use directly
 */
typedef struct queue_priv
{
    sem_t semSend; /**< able to send semaphore */
    sem_t semReceive; /**< able to receive semaphore */
    char *buffer; /**< queue data */
    size_t itemSize; /**< size of each item in the queue */
    size_t bytes; /**< number of bytes that make up the queue */
    unsigned int indexSend; /**< current index for send */
    unsigned int indexReceive; /**< current index for receive */
    os_mutex_t mutex; /**< mutex to protect queue operations */
} QueuePriv;

/** Create a new message queue.
 * @param length length in number of messages of the queue
 * @param item_size size in number of bytes of a message
 * @return handle to the created queue, NULL on failure
 */
static inline os_mq_t os_mq_create(size_t length, size_t item_size)
{
    QueuePriv *q = (QueuePriv*)malloc(sizeof(QueuePriv));
    if (!q)
    {
        errno = ENOMEM;
        return NULL;
    }
    sem_init(&q->semSend, 0, length);
    sem_init(&q->semReceive, 0, 0);
    q->buffer = (char*)malloc(length * item_size);
    q->itemSize = item_size;
    q->bytes = length * item_size;
    q->indexSend = 0;
    q->indexReceive = 0;
    os_mutex_init(&q->mutex);

    return q;
}

/** Blocking send of a message to a queue.
 * @param queue queue to send message to
 * @param data message to copy into queue
 */
static inline void os_mq_send(os_mq_t queue, const void *data)
{
    QueuePriv *q = (QueuePriv*)queue;
    
    sem_wait(&q->semSend);

    pthread_mutex_lock(&q->mutex);
    memcpy(q->buffer + q->indexSend, data, q->itemSize);
    q->indexSend += q->itemSize;
    if (q->indexSend >= q->bytes)
    {
        q->indexSend = 0;
    }
    pthread_mutex_unlock(&q->mutex);
    sem_post(&q->semReceive);
}

/** Send a message to a queue with a timeout.
 * @param queue queue to send message to
 * @param data message to copy into queue
 * @param timeout time to wait for queue to be able to accept message
 * @return OS_MQ_NONE on success, OS_MQ_TIMEDOUT on timeout
 */
static inline int os_mq_timedsend(os_mq_t queue, const void *data, const struct timespec *timeout)
{
    return OS_MQ_NONE;
}


/** Receive a message from a queue.
 * @param queue queue to receive message from
 * @param data location to copy message from the queue
 */
static inline void os_mq_receive(os_mq_t queue, void *data)
{
    QueuePriv *q = (QueuePriv*)queue;
    
    sem_wait(&q->semReceive);

    pthread_mutex_lock(&q->mutex);
    memcpy(q->buffer + q->indexReceive, data, q->itemSize);
    q->indexReceive += q->itemSize;
    if (q->indexReceive >= q->bytes)
    {
        q->indexReceive = 0;
    }
    pthread_mutex_unlock(&q->mutex);
    sem_post(&q->semSend);
}

/** Receive a message from a queue.
 * @param queue queue to receive message from
 * @param data location to copy message from the queue
 * @param timeout time to wait for queue to have a message available
 * @return OS_MQ_NONE on success, OS_MQ_TIMEDOUT on timeout
 */
static inline int os_mq_timedreceive(os_mq_t queue, void *data, const struct timespec *timeout)
{
    return OS_MQ_NONE;
}

/** Get the monotonic time since the system started.
 * @return time in nanoseconds since system start
 */
static inline long long os_get_time_monotonic(void)
{
    struct timespec ts;
#if defined (__nuttx__)
    clock_gettime(CLOCK_REALTIME, &ts);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
    return ((long long)ts.tv_sec * 1000000000LL) + ts.tv_nsec;
}

#ifdef __cplusplus
}
#endif

#endif /* _os_h_ */
