/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file BufferQueue.hxx
 * This file provides an implementation buffered queues.
 *
 * @author Stuart W. Baker
 * @date 3 August 2013
 */

#ifndef _UTILS_QUEUE_HXX_
#define _UTILS_QUEUE_HXX_

#include <cstdint>
#include <cstdlib>
#include <cstdarg>

#include "executor/Executable.hxx"
#include "utils/Atomic.hxx"
#include "executor/Notifiable.hxx"
#include "os/OS.hxx"
#include "utils/MultiMap.hxx"
#include "utils/Queue.hxx"
#include "utils/macros.h"

class DynamicPool;
class FixedPool;
class Pool;
template <class T> class Buffer;
class BufferBase;

namespace NMRAnet
{
class AsyncIfTest;
}

/** Abstract interface to all Queues of all types.
 */
class QInterface
{
public:
    /** Result of pulling an item from the queue based on priority.
     */
    struct Result
    {
        /** Defualt Constructor.
         */
        Result()
            : item(NULL)
            , index(0)
        {
        }

        /** Explicit initializer constructor.
         * @param item item presented in result
         * @param index index presented in result
         */
        Result(QMember *item, unsigned index)
            : item(item)
            , index(index)
        {
        }

        /** Default Destructor.
         */
        ~Result()
        {
        }

        QMember *item;  /**< item pulled from queue */
        unsigned index; /**< index of item pulled from queue */
    };

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index in the list to operate on
     */
    virtual void insert(QMember *item, unsigned index) = 0;

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    virtual QMember *next(unsigned index) = 0;

    /** Get an item from the front of the queue queue in priority order.
     * @return item retrieved from queue + index, NULL if no item available
     */
    virtual Result next() = 0;

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    virtual size_t pending(unsigned index) = 0;

    /** Get the total number of pending items in all queues in the list.
     * @param index in the list to operate on
     * @return number of total pending items in all queues in the list
     */
    virtual size_t pending() = 0;

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    virtual bool empty(unsigned index) = 0;

    /** Test if all the queues are empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    virtual bool empty() = 0;

protected:
    /** Default Constructor.
     */
    QInterface()
    {
    }

    /** Default Destructor.
     */
    virtual ~QInterface()
    {
    }

private:
    DISALLOW_COPY_AND_ASSIGN(QInterface);
};

/** QInterface that enqueues items based on priority.  FIFO ordering for
 * entries of the same priority.
 */
class QPriority : public QInterface,
                  private MultiMap<unsigned, QMember *>,
                  private Atomic
{
public:
    /** Default Constructor.
     */
    QPriority()
        : QInterface()
        , MultiMap<unsigned, QMember *>()
    {
    }

    /** Constructor.
     * @param entries number of entries in a fixed size queue
     */
    QPriority(size_t entries)
        : QInterface()
        , MultiMap<unsigned, QMember *>(entries)
    {
    }

    /** Destructor.
     */
    ~QPriority()
    {
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index in the list to operate on
     */
    void insert(QMember *item, unsigned index)
    {
        AtomicHolder h(this);
        MultiMap<unsigned, QMember *>::Iterator it = upper_bound(index);
        MultiMap<unsigned, QMember *>::insert(
            it, MultiMap<unsigned, QMember *>::Pair(index, item));
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        AtomicHolder h(this);
        MultiMap<unsigned, QMember *>::Iterator it;
        it = MultiMap<unsigned, QMember *>::find(index);
        if (it != MultiMap<unsigned, QMember *>::end())
        {
            QMember *result = (*it).second;
            MultiMap<unsigned, QMember *>::erase(it);
            return result;
        }

        return NULL;
    }

    /** Get an item from the front of the queue in priority order.
     * @return item retrieved from queue + index, NULL if no item available
     */
    Result next()
    {
        AtomicHolder h(this);
        MultiMap<unsigned, QMember *>::Iterator it;
        it = MultiMap<unsigned, QMember *>::begin();
        if (it != MultiMap<unsigned, QMember *>::end())
        {
            Result result((*it).second, (*it).first);
            MultiMap<unsigned, QMember *>::erase(it);
            return result;
        }

        return Result();
    }

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        return MultiMap<unsigned, QMember *>::count(index);
    }

    /** Get the total number of pending items in all queues in the list.
     * @param index in the list to operate on
     * @return number of total pending items in all queues in the list
     */
    size_t pending()
    {
        return MultiMap<unsigned, QMember *>::size();
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        return (pending(index) == 0);
    }

    /** Test if all the queues are empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty()
    {
        return (pending() == 0);
    }

private:
    DISALLOW_COPY_AND_ASSIGN(QPriority);
};

/** This class implements a linked list "queue" of buffers.  It may be
 * instantiated to use the mainBufferPool for its memory pool, or optionally
 * another BufferPool instance may be specified for its memory pool.
 */
class Q : public QInterface, public Atomic
{
public:
    /** Default Constructor.
     */
    Q()
        : QInterface()
        , head(NULL)
        , tail(NULL)
        , count(0)
    {
    }

    /** Default destructor.
     */
    ~Q()
    {
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index unused parameter
     */
    void insert(QMember *item, unsigned index = 0);

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        return next().item;
    }

    /** Get an item from the front of the queue.
     * @return @ref Result structure with item retrieved from queue, NULL if
     *         no item available
     */
    Result next();

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        return pending();
    };

    /** Get the number of pending items in the queue.
     * @return number of pending items in the queue
     */
    size_t pending()
    {
        return count;
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        return empty();
    }

    /** Test if the queue is empty.
     * @return true if empty, else false
     */
    bool empty()
    {
        return (head == NULL);
    }

private:
    /** head item in queue */
    QMember *head;

    /** tail item in queue */
    QMember *tail;

    /** number of items in queue */
    size_t count;

    DISALLOW_COPY_AND_ASSIGN(Q);
};

/** Asynchronous specialization of @ref Q.
 */
class QAsync : public Q
{
public:
    /** Default Constructor.
     */
    QAsync()
        : Q()
        , waiting(true)
    {
    }

    /** Default destructor.
     */
    ~QAsync()
    {
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index unused parameter
     */
    void insert(QMember *item, unsigned index = 0);

    /** Get an item from the front of the queue.
     * @param flow Executable that will wait on the item
     * @return item retrieved from queue, NULL if no item available
     */
    void next_async(Executable *flow);

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        return next().item;
    }

    /** Get an item from the front of the queue.
     * @return @ref Result structure with item retrieved from queue, NULL if
     *         no item available
     */
    Result next()
    {
        AtomicHolder h(this);
        return waiting ? Result() : Q::next();
    }

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        return pending();
    };

    /** Get the number of pending items in the queue.
     * @return number of pending items in the queue
     */
    size_t pending()
    {
        AtomicHolder h(this);
        return waiting ? 0 : Q::pending();
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        return empty();
    }

    /** Test if the queue is empty.
     * @return true if empty, else false
     */
    bool empty()
    {
        AtomicHolder h(this);
        return waiting ? true : Q::empty();
    }

private:
    /** true if someone is waiting for an insertion */
    bool waiting;

    DISALLOW_COPY_AND_ASSIGN(QAsync);
};

/** A list of queues.  Index 0 is the highest priority queue with increasingly
 * higher indexes having increasingly lower priority.
 */
template <unsigned ITEMS> class QList : public QInterface
{
public:
    /** Default Constructor.
     * @param size number of queues in the list
     */
    QList()
        : QInterface()
        , list()
    {
    }

    /** Destructor.
     */
    ~QList()
    {
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index in the list to operate on
     */
    void insert(QMember *item, unsigned index)
    {
        if (index >= ITEMS)
        {
            index = ITEMS - 1;
        }
        list[index].insert(item);
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        return list[index].next().item;
    }

    /** Get an item from the front of the queue queue in priority order.
     * @return item retrieved from queue + index, NULL if no item available
     */
    Result next()
    {
        for (unsigned i = 0; i < ITEMS; ++i)
        {
            QMember *result = list[i].next(0);
            if (result)
            {
                return Result(result, i);
            }
        }
        return Result();
    }

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        return list[index].pending();
    }

    /** Get the total number of pending items in all queues in the list.
     * @return number of total pending items in all queues in the list
     */
    size_t pending()
    {
        size_t result = 0;
        for (unsigned i = 0; i < ITEMS; ++i)
        {
            result += list[i].pending();
        }
        return result;
    }

    size_t size()
    {
        return pending();
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        return list[index].empty();
    }

    /** Test if all the queues are empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty()
    {
        for (unsigned i = 0; i < ITEMS; ++i)
        {
            if (!list[i].empty())
            {
                return false;
            }
        }
        return true;
    }

private:
    /** the list of queues */
    Q list[ITEMS];

    DISALLOW_COPY_AND_ASSIGN(QList);
};

/** A list of queues.
 */
template <unsigned items>
class QListProtected : public QList<items>, private Atomic
{
public:
    /** Default Constructor.
     * @param size number of queues in the list
     */
    QListProtected()
        : QList<items>()
    {
    }

    /** Destructor.
     */
    ~QListProtected()
    {
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @return item retrieved from queue, NULL if no item available
     */
    void insert(QMember *q, unsigned index = 0)
    {
        AtomicHolder h(this);
        QList<items>::insert(q, index);
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @return item retrieved from queue, NULL if no item available
     */
    void insert_locked(QMember *q, unsigned index = 0)
    {
        QList<items>::insert(q, index);
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        AtomicHolder h(this);
        return QList<items>::next(index);
    }

    /** Translate the Result type */
    typedef typename QList<items>::Result Result;

    /** Get an item from the front of the queue queue in priority order.
     * @return item retrieved from queue + index, NULL if no item available
     */
    Result next()
    {
        AtomicHolder h(this);
        return QList<items>::next();
    }
};

/** Pool of previously allocated, but currently unused, items. */
class Pool
{
public:
    /** Get a free item out of the pool.
     * @param result pointer to a pointer to the result
     * @param flow if !NULL, then the alloc call is considered async and will
     *        behave as if @ref alloc_async() was called.
     */
    template <class BufferType>
    void alloc(Buffer<BufferType> **result, Executable *flow = NULL)
    {
#ifdef DEBUG_BUFFER_MEMORY
        g_current_alloc = &&alloc;
        alloc:
#endif
        *result = static_cast<Buffer<BufferType> *>(
            alloc_untyped(sizeof(Buffer<BufferType>), flow));
        if (*result && !flow)
        {
            new (*result) Buffer<BufferType>(this);
        }
    }

    /** Get a free item out of the pool.
     * @param flow Executable to notify upon allocation
     */
    template <class BufferType> void alloc_async(Executable *flow)
    {
        Buffer<BufferType> *buffer;
        alloc(&buffer, flow);
    }

    /** Cast the result of an asynchronous allocation and perform a placement
     * new on it.
     * @param base untyped buffer
     * @param result pointer to a pointer to the cast result
     */
    template <class BufferType>
    static void alloc_async_init(BufferBase *base, Buffer<BufferType> **result)
    {
        HASSERT(base);
        HASSERT(sizeof(Buffer<BufferType>) == base->size());
        *result = static_cast<Buffer<BufferType> *>(base);
        new (*result) Buffer<BufferType>(base->pool());
    }

    /** Number of free items in the pool.
     * @return number of free items in the pool
     */
    virtual size_t free_items() = 0;

    /** Number of free items in the pool for a given allocation size.
     * @param size size of interest
     * @return number of free items in the pool for a given allocation size
     */
    virtual size_t free_items(size_t size) = 0;

protected:
    /** Default Constructor.
     */
    Pool()
        : totalSize(0)
    {
    }

    /** default destructor.
     */
    virtual ~Pool()
    {
    }

    virtual BufferBase *alloc_untyped(size_t size, Executable *flow) = 0;

    /** Release an item back to the free pool.
     * @param item pointer to item to release
     */
    virtual void free(BufferBase *item) = 0;

    /** keep track of total allocated size of memory */
    size_t totalSize;

private:
    /** Allow BufferBase to access this class */
    friend class BufferBase;

    /** Allow Buffer to access this class */
    template <class T> friend class Buffer;

    DISALLOW_COPY_AND_ASSIGN(Pool);
};

#if 0
/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.
 */
template <class T> class QueueWait : public Q <T>, public OSSem
{
public:
    /** Default Constructor. */
    QueueWait()
        : Q<T>(),
          OSSem(0)
    {
    }

    /** Default destructor.
     */
    ~QueueWait()
    {
    }

    /** Add an item to the back of the queue.
     * @param item item to add to queue
     */
    void insert(T *item)
    {
        Q<T>::insert(item);
        post();
    }

    /** Get an item from the front of the queue.
     * @return item retrieved from queue
     */
    T *next()
    {
        T *result = Q<T>::next();
        if (result != NULL)
        {
            /* decrement semaphore */
            OSSem::wait();
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @return item retrieved from queue, else NULL with errno set:
     *         EINTR - woken up asynchronously
     */
    T *wait()
    {
        OSSem::wait();
        T *result = Q<T>::next();
        if(result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @return item retrieved from queue, else NULL with errno set:
     *         ETIMEDOUT - timeout occured, EINTR woken up asynchronously
     */
    T *timedwait(long long timeout)
    {
        if (OSSem::timedwait(timeout) != 0)
        {
            errno = ETIMEDOUT;
            return NULL;
        }

        T *result = Q<T>::next();
        if (result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wakeup anyone waiting on the wait queue.
     */
    void wakeup()
    {
        post();
    }

private:

    DISALLOW_COPY_AND_ASSIGN(QueueWait);
};

/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.
 */
template <class T> class QueueProtectedWait : public QProtected <T>, public OSSem
{
public:
    /** Default Constructor. */
    QueueProtectedWait()
        : QProtected<T>(),
          OSSem(0)
    {
    }

    /** Default destructor.
     */
    ~QueueProtectedWait()
    {
    }

    /** Add an item to the back of the queue.
     * @param item item to add to queue
     */
    void insert(T *item)
    {
        QProtected<T>::insert(item);
        post();
    }

#ifdef __FreeRTOS__
    /** Add an item to the back of the queue, callable from interrupt context.
     * @param item item to add to queue
     */
    void insert_from_isr(T *item)
    {
        QProtected<T>::insert_locked(item);
        post_from_isr();
    }
#endif

    /** Get an item from the front of the queue.
     * @return item retrieved from queue
     */
    T *next()
    {
        T *result = QProtected<T>::next();
        if (result != NULL)
        {
            /* decrement semaphore */
            OSSem::wait();
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @return item retrieved from queue, else NULL with errno set:
     *         EINTR - woken up asynchronously
     */
    T *wait()
    {
        OSSem::wait();
        T *result = QProtected<T>::next();
        if(result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @return item retrieved from queue, else NULL with errno set:
     *         ETIMEDOUT - timeout occured, EINTR - woken up asynchronously
     */
    T *timedwait(long long timeout)
    {
        if (OSSem::timedwait(timeout) != 0)
        {
            errno = ETIMEDOUT;
            return NULL;
        }

        T *result = QProtected<T>::next();
        if (result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wakeup someone waiting on the wait queue.
     */
    void wakeup()
    {
        post();
    }

private:
    DISALLOW_COPY_AND_ASSIGN(QueueProtectedWait);
};

#endif

/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.  The priority of pulling items out of
 * of the list is fixed to look at index 0 first and the highest index last.
 */
template <unsigned items>
class QListProtectedWait : public QListProtected<items>, public OSSem
{
public:
    /** Default Constructor.
     * @param size number of queues in the list
     */
    QListProtectedWait()
        : QListProtected<items>()
        , OSSem(0)
    {
    }

    /** Default destructor.
     */
    ~QListProtectedWait()
    {
    }

    /** Add an item to the back of the queue.
     * @param item item to add to queue
     * @param index in the list to operate on
     */
    void insert(QMember *item, unsigned index)
    {
        QListProtected<items>::insert(item, index);
        post();
    }

#ifdef __FreeRTOS__
    /** Add an item to the back of the queue.
     * @param item item to add to queue
     * @param index in the list to operate on
     */
    void insert_from_isr(QMember *item, unsigned index)
    {
        QListProtected<items>::insert_locked(item, index);
        this->post_from_isr();
    }
#endif

    /** Translate the Result type */
    typedef typename QListProtected<items>::Result Result;

    /** Get an item from the front of the queue.
     * @return item retrieved from one of the queues
     */
    Result next()
    {
        Result result = QListProtected<items>::next();
        if (result.item != NULL)
        {
            /* decrement semaphore */
            OSSem::wait();
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @return item retrieved from queue, else NULL with errno set:
     *         EINTR - woken up asynchronously
     */
    Result wait()
    {
        OSSem::wait();
        Result result = QListProtected<items>::next();
        if (result.item == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wait for an item from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @param priority pass back the priority of the queue pulled from
     * @return item retrieved from queue, else NULL with errno set:
     *         ETIMEDOUT - timeout occured, EINTR - woken up asynchronously
     */
    Result timedwait(long long timeout)
    {
        if (OSSem::timedwait(timeout) != 0)
        {
            errno = ETIMEDOUT;
            return {NULL, 0};
        }

        Result result = QListProtected<items>::next();
        if (result.item == NULL)
        {
            errno = EINTR;
        }
        return result;
    }

    /** Wakeup anyone waiting on the wait queue.
     */
    void wakeup()
    {
        post();
    }

private:
    DISALLOW_COPY_AND_ASSIGN(QListProtectedWait);
};

/** Decrement count.
 */
template <class T> void Buffer<T>::unref()
{
    HASSERT(sizeof(Buffer<T>) <= size_);
    if (--count_ == 0)
    {
        this->~Buffer();
        pool_->free(this);
    }
}

#endif /* _UTILS_QUEUE_HXX_ */
