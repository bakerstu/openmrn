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

#ifndef _BufferQueue_hxx_
#define _BufferQueue_hxx_

#include <new>
#include <cstdint>
#include <cstdlib>
#include <cstdarg>

#include "executor/notifiable.hxx"
#include "os/OS.hxx"
#include "utils/macros.h"
#include "utils/Queue.hxx"


class DynamicPool;
class FixedPool;
class Pool;
template <class T> class Buffer;
class BufferBase;


/** main buffer pool instance */
extern DynamicPool *mainBufferPool;



class BufferBase : public QMember
{
public:
    uint16_t references()
    {
        return count_;
    }

    void set_done(Notifiable *done)
    {
        done_ = done;
    }

    size_t size()
    {
        return size_;
    }

protected:
    /** Get a pointer to the pool that this buffer belongs to.
     * @return pool that this buffer belongs to
     */
    Pool *pool()
    {
        return pool_;
    }
    
    /** size of data in bytes */
    uint16_t size_;

    /** number of references in use */
    uint16_t count_;
    
    /** Reference to the pool from whence this buffer came */
    Pool *pool_;
    
    Notifiable *done_;
    
    /** Constructor.
     * @param size size of buffer data
     * @param pool pool this buffer belong to
     */
    BufferBase(size_t size, Pool *pool)
        : QMember(),
          size_(size),
          count_(1),
          pool_(pool),
          done_(NULL)
    {
    }

    /** Destructor.
     */
    ~BufferBase()
    {
    }

    DISALLOW_COPY_AND_ASSIGN(BufferBase);
};

/** Base class for all QMember types that hold data in an expandable format
 */
template <class T> class Buffer : public BufferBase
{
public:
    /** The type of payload this buffer contains. */
    typedef T value_type;

    /** Add another reference to the buffer.
     * @return total number of references to this point
     */
    Buffer<T> *ref()
    {
        ++count_;
        return this;
    }

    /** Decrement count.
     */
    inline void unref();
    
    /** get a pointer to the start of the data.
     */
    T *data()
    {
        return &data_;
    }
    
private:    
    /** Constructor.
     * @param pool pool this buffer belong to
     */
    Buffer(Pool *pool)
        : BufferBase(sizeof(Buffer<T>), pool),
          data_()
    {
    }

    /** Destructor.
     */
    ~Buffer()
    {
    }

    DISALLOW_COPY_AND_ASSIGN(Buffer);

    /** Allow DynamicPool access to our constructor */
    friend class DynamicPool;

    /** Allow FixedPool access to our constructor */
    friend class FixedPool;

    /** user data */
    T data_;
};

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
            : item(NULL),
              index(0)
        {
        }
        
        /** Explicit initializer constructor.
         * @param item item presented in result
         * @param index index presented in result
         */
        Result(QMember *item, unsigned index)
            : item(item),
              index(index)
        {
        }
        
        
        /** Default Destructor.
         */
        ~Result()
        {
        }
        
        QMember *item; /**< item pulled from queue */
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
};

/** This class implements a linked list "queue" of buffers.  It may be
 * instantiated to use the mainBufferPool for its memory pool, or optionally
 * another BufferPool instance may be specified for its memory pool.
 */
class Q : public QInterface
{
public:
    /** Default Constructor.
     */
    Q()
        : QInterface(),
          head(NULL),
          tail(NULL),
          count(0)
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
    void insert(QMember *q, unsigned index = 0)
    {
        if (head == NULL)
        {
            head = tail = q;
        }
        else
        {
            tail->next = q;
            tail = q;
        }
        q->next = NULL;
        ++count;
    }

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
        if (head == NULL)
        {
            return Result();
        }
        --count;
        QMember* qm = head;
        head = (qm->next);

        return Result(qm, 0);
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
        : QInterface(),
          list()
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
     * @param index in the list to operate on
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

#if 0
/** This is a specialization of the Q which uses a mutex for insertion and
 * removal.
 */
template <class T> class QProtected : public Q <T>
{
public:
    /** Default Constructor.
     */
    QProtected()
        : Q<T>(),
          mutex()
    {
    }

    /** Default destructor.
     */
    ~QProtected()
    {
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     */
    void insert(T *q)
    {
        mutex.lock();
        Q<T>::insert(q);
        mutex.unlock();
    }

    /** Get an item from the front of the queue.
     * @return item retrieved from queue, NULL if no item available
     */
    T *next()
    {
        T *q;

        mutex.lock();
        q = Q<T>::next();
        mutex.unlock();

        return q;
    }

protected:

private:
    /** @todo (Stuart Baker) For free RTOS, we may want to consider a different
     * (smaller) locking mechanism
     */
    /** Mutual exclusion for Queue */
    OSMutex mutex;

    DISALLOW_COPY_AND_ASSIGN(QProtected);
};
#endif

/** A list of queues.
 */
template <unsigned items> class QListProtected : public QList <items>
{
public:
    /** Default Constructor.
     * @param size number of queues in the list
     */
    QListProtected()
        : QList<items>(),
          mutex()
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
        mutex.lock();
        QList<items>::insert(q, index);
        mutex.unlock();
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue, NULL if no item available
     */
    QMember *next(unsigned index)
    {
        mutex.lock();
        QMember *result = QList<items>::next(index);
        mutex.unlock();
        return result;
    }

    /** Translate the Result type */
    typedef typename QList<items>::Result Result;
    
    /** Get an item from the front of the queue queue in priority order.
     * @return item retrieved from queue + index, NULL if no item available
     */
    Result next()
    {
        mutex.lock();
        Result result = QList<items>::next();
        mutex.unlock();
        return result;
    }

private:    
    /** @todo (Stuart Baker) For free RTOS, we may want to consider a different
     * (smaller) locking mechanism
     */
    /** Mutual exclusion for Queue */
    OSMutex mutex;
};

/** Pool of previously allocated, but currently unused, items. */
class Pool
{
public:

protected:
    /** Default Constructor */
    Pool()
        : mutex(true),
          totalSize(0)
    {
    }

    /** default destructor */
    ~Pool()
    {
    }

    /** Release an item back to the free pool.
     * @param item pointer to item to release
     */
    virtual void free(BufferBase *item)
    {
    }

    /** Mutual exclusion for buffer pool */
    OSMutex mutex;

    /** keep track of total allocated size of memory */
    size_t totalSize;
    
private:
    /** Allow BufferBase to access this class */
    template <class T> friend class Buffer;
    
    DISALLOW_COPY_AND_ASSIGN(Pool);
};

/** This is a struct for storing info about a specific size item in the
 * DynamicPool.
 */
class Bucket : public Q
{
public:
    /** Allocate a Bucket array off of the heap initialized with sizes.
     * @param s size of first bucket
     * @param ... '0' terminated list of additional buckets
     * @todo (Stuart Baker) fix such that sizes do not need to be in strict ascending order
     */
    static Bucket *init(int s, ...)
    {
        va_list ap, aq;
        va_start(ap, s);
        va_copy(aq, ap);
        int count = 1;
        int current = s;
        
        while (current != 0)
        {
            ++count;
            int next = va_arg(ap, int);
            HASSERT(next > current || next == 0);
            current = next;
        }
        
        Bucket *bucket = (Bucket*)malloc(sizeof(Bucket) * count);
        Bucket *now = bucket;

        for (int i = 0; i < count; ++i)
        {
            new (now) Bucket(va_arg(aq, int));
            now++;
        }
        
        va_end(aq);
        va_end(ap);
        return bucket;
    }

    /** destroy a bucket created with init.
     * @param bucket Bucket array to destroy
     */
    static void destroy(Bucket *bucket)
    {
        free(bucket);
    }
    
    /** Get the size of the bucket.
     * @return size of bucket
     */
    size_t size()
    {
        return size_;
    }

    /** Pull out any pending Executables.
     * @return next Qmember pending on an item in the bucket
     */
    QMember *executables()
    {
        return pending_.next().item;
    }
    
private:
    /** Constructor.
     */
    Bucket(size_t size)
        : Q(),
          size_(size),
          pending_()
    {
    }
    
    /** Destructor.
     */
    ~Bucket()
    {
    }
    
    size_t size_; /**< size of entry */

    /** list of anyone waiting for an item in the bucket */
    Q pending_;    
};

/** A specialization of a pool which can allocate new elements dynamically
 * upon request.
 */
class DynamicPool : public Pool
{
public:
    /** Constructor.
     * @param sizes array of bucket sizes for the pool
     */
    DynamicPool(Bucket sizes[])
        : Pool(),
          totalSize(0),
          buckets(sizes)
    {
    }

    /** default destructor */
    ~DynamicPool()
    {
        Bucket::destroy(buckets);
    }

    /** Get a free item out of the pool.
     * @param result pointer to a pointer to the result
     * @param flow if !NULL, then the alloc call is considered async and will
     *        behave as if @ref alloc_async() was called.
     */
    template <class BufferType> void alloc(Buffer<BufferType> **result,
                                           Executable *flow = NULL)
    {
        *result = NULL;

        for (Bucket *current = buckets; current->size() != 0; ++current)
        {
            if (sizeof(Buffer<BufferType>) <= current->size())
            {
                mutex.lock();
                *result = static_cast<Buffer<BufferType>*>(current->next().item);
                if (*result == NULL)
                {
                    *result = (Buffer<BufferType>*)malloc(current->size());
                    totalSize += current->size();
                }
                new (*result) Buffer<BufferType>(this);
                mutex.unlock();
                return;
            }
        }
         
        /* big items are just malloc'd freely */
        *result = (Buffer<BufferType>*)malloc(sizeof(Buffer<BufferType>));
        new (*result) Buffer<BufferType>(this);
        mutex.lock();
        totalSize += sizeof(Buffer<BufferType>);
        mutex.unlock();
    }

    /** Get a free item out of the pool.
     * @param flow Executable to notify upon allocation
     */
    template <class BufferType> void alloc_async(Executable *flow)
    {
        Buffer<BufferType> *buffer;
        alloc<BufferType>(&buffer);
        /* This pool will malloc indefinitely to create more buffers.
         * We will always have the result.
         */
        flow->alloc_result(buffer);
    }

    /** Cast the result of an asynchronous allocation and perform a placement
     * new on it.
     * @param base untyped buffer
     * @param result pointer to a pointer to the cast result
     */
    template <class BufferType> static void alloc_async_init(BufferBase *base, Buffer<BufferType> **result)
    {
        HASSERT(sizeof(Buffer<BufferType>) <= base->size());
        *result = static_cast<Buffer<BufferType>*>(base);
        new (*result) Buffer<BufferType>();
    }

protected:
    /** keep track of total allocated size of memory */
    size_t totalSize;
    
    /** Free buffer queue */
    Bucket *buckets;
    
private:
    /** Release an item back to the free pool.
     * @param item pointer to item to release
     * @param size size of buffer to free
     */
    void free(BufferBase *item)
    {
        for (Bucket *current = buckets; current->size() != 0; ++current)
        {
            if (item->size() <= current->size())
            {
                current->insert(item);
                mutex.unlock();
                return;
            }
        }
        mutex.lock();
        /* big items are just freed */
        totalSize -= item->size();
        mutex.unlock();
        ::free(item);
    }

    /** Default constructor.
     */
    DynamicPool();

    DISALLOW_COPY_AND_ASSIGN(DynamicPool);
};

/** Pool of fixed number of items which can be allocated up on request.
 */
class FixedPool : public Pool
{
public:
    /** Used in static pools to tell if this item is a member of the pool.
     * @param item to test validity on
     * @return true if the item is in the pool, else return false;
     */
    bool valid(QMember *item)
    {
        if ((char*)item >= mempool && (char*)item < (mempool + (items * itemSize)))
        {
            return true;
        }
        return false;
    }

    /** Get a free item out of the pool.
     * @param result pointer to a pointer to the result
     * @param flow if !NULL, then the alloc call is considered async and will
     *        behave as if @ref alloc_async() was called.
     */
    template <class BufferType> void alloc(Buffer<BufferType> **result,
                                           Executable *flow = NULL)
    {
        mutex.lock();
        if (empty == false)
        {
            *result = static_cast<Buffer<BufferType>*>(queue.next(0));
            if (*result)
            {
                new (*result) Buffer<BufferType>(this);
                totalSize += itemSize;
                mutex.unlock();
                if (flow)
                {
                    flow->alloc_result(*result);
                }
                return;
            }
            empty = true;
        }
        *result = NULL;
        if (flow)
        {
            queue.insert(flow);
        }
        mutex.unlock();
    }

    /** Get a free item out of the pool.
     * @param flow Executable to notify upon allocation
     */
    template <class BufferType> void alloc_async(Executable *flow)
    {
        Buffer<BufferType> *buffer;
        alloc<BufferType>(&buffer, flow);
    }

    /** Constructor for a fixed size pool.
     * @param item_size size of each item in the pool
     * @param items number of items in the pool
     */
    FixedPool(size_t item_size, size_t items)
        : Pool(),
          totalSize(0),
          mempool(new char[(item_size * items)]),
          itemSize(item_size),
          items(items),
          empty(false)
    {
        HASSERT(item_size != 0 && items != 0);
        QMember *current = (QMember*)mempool;
        for (size_t i = 0; i < items; ++i)
        {
            queue.insert(current);
            current = (QMember*)((char*)current + item_size);
        }
    }

    /** default destructor */
    ~FixedPool()
    {
        delete [] mempool;
    }

protected:
    /** keep track of total allocated size of memory */
    size_t totalSize;
    
    /** Free buffer queue */
    Q queue;
    
    /** First buffer in a pre-allocated array pool */
    char *mempool;
    
    /** item Size for fixed pools */
    size_t itemSize;
    
    /** total number of items in the queue */
    size_t items;
    
    /** is the pool empty */
    bool empty;
    
private:
    /** Release an item back to the free pool.
     * @param item pointer to item to release
     * @param size size of buffer to free
     */
    void free(BufferBase *item)
    {
        HASSERT(valid(item));
        HASSERT(item->size() <= itemSize);
        if (empty == true)
        {
            Executable *waiting = static_cast<Executable*>(queue.next().item);
            if (waiting)
            {
                waiting->alloc_result(item);
            }
            if (queue.pending() == 0)
            {
                empty = false;
            }
        }
        else
        {
            queue.insert(item);
        }
    }

    /** Default Constructor.
     */
    FixedPool();

    DISALLOW_COPY_AND_ASSIGN(FixedPool);
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
template <unsigned items> class QListProtectedWait : public QListProtected <items>, public OSSem
{
public:
    /** Default Constructor.
     * @param size number of queues in the list
     */
    QListProtectedWait()
        : QListProtected<items>(),
          OSSem(0)
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
        if(result.item == NULL)
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

#if 0
/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.
 */
class BufferQueueWait : public QueueWait <Buffer>
{
public:
    /** Default Constructor, use mainBufferPool for buffer allocation. */
    BufferQueueWait()
        : QueueWait<Buffer>()
    {
    }

    /** Default destructor.
     */
    ~BufferQueueWait()
    {
    }

private:

    DISALLOW_COPY_AND_ASSIGN(BufferQueueWait);
};

/** Get a free buffer out of the mainBufferPool pool.
 * @param size minimum size in bytes the buffer must hold
 * @return pointer to the newly allocated buffer
 */
inline Buffer *buffer_alloc(size_t size)
{
    return mainBufferPool->alloc(size);
}

/** Release a buffer back to the mainBufferPool free buffer pool.
 * @param buffer pointer to buffer to release
 */
inline void buffer_free(Buffer *buffer)
{
    mainBufferPool->free(buffer);
}

/** Advance the position of the buffer.
 * @param bytes number of bytes to advance.
 * @return pointer to the new position (next available byte)
 */
inline void *BufferManager::advance(size_t bytes)
{
    /** @todo (Stuart Baker) do we really need a mutex lock here? */
    //pool->mutex.lock();
    left -= bytes;    
    //pool->mutex.unlock();
    return &data()[size_ - left];
}

/** Add another reference to the buffer.
 * @return total number of references to this point
 */
inline unsigned int BufferManager::reference()
{
    /** (Stuart Baker) we need a mutex lock here.  Maybe in the derived class. */
    //pool->mutex.lock();
    ++count;
    //pool->mutex.unlock();
    return count;
}
#endif

/** Decrement count.
 */
template <class T> void Buffer<T>::unref()
{
    HASSERT(sizeof(Buffer<T>) == size_);
    if (--count_ == 0)
    {
        this->~Buffer();
        pool_->free(this);
    }
}
    

#endif /* _BufferQueue_hxx_ */
