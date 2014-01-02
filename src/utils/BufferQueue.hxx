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

#include <os/OS.hxx>

template <class T> class DynamicPool;
template <class T> class Pool;
class Buffer;

/** main buffer pool instance */
extern DynamicPool<Buffer> *mainBufferPool;


/** Essentially a "next" pointer container.
 */
class QMember
{
protected:
    /** Decrement count.  This method is here to allow a QMember to not
     * have to implement a reference count, potentially saving 4 bytes of
     * space.
     * @return new count value
     */
    virtual unsigned int dec_count()
    {
        return 0;
    }
    
    /** Constructor.
     */
    QMember()
        : next(NULL)
    {
    }
    
    /** Destructor.
     */ 
    ~QMember()
    {
    }
    
    /** pointer to the next member in the queue */
    QMember *next;

    /** This class is a helper of Q */
    template <class T> friend class Q;

    /** This class is a helper of DynamicPool */
    template <class T> friend class DynamicPool;
};

/** Base class for all QMember types that hold data in an expandable format
 */
class BufferManager : public QMember
{
public:
    /** Free this buffer to the Pool from whence it came.
     */
    virtual void free() = 0;

    /** Advance the position of the buffer.
     * @param bytes number of bytes to advance.
     * @return pointer to the new position (next available byte)
     */
    virtual void *advance(size_t bytes);
    
    /** reset the buffer position back to beginning.
     * @return pointer to the new position (next available byte)
     */
    void *zero()
    {
        left = size_;
        return data();
    }    

    /** Get a pointer to the first position (byte) of the buffer.
     * @return pointer to the first position (byte)
     */
    void *start()
    {
        return data();
    }

    /** Get a pointer to the current position of the buffer.
     * @return pointer to the current position (next available byte)
     */
    void *position()
    {
        return &data()[size_ - left];
    }

    /** Get the size of the buffer in bytes.
     * @return size of the buffer in bytes
     */
    size_t size() const
    {
        return size_;
    }

    /** Get the number of unused bytes in the buffer.
     * @return number of unused bytes
     */
    size_t available() const
    {
        return left;    
    }

    /** Get the number of used bytes in the buffer.
     * @return number of used bytes
     */
    size_t used() const
    {
        return size_ - left;    
    }

    /** Add another reference to the buffer.
     * @return total number of references to this point
     */
    unsigned int reference();

    /** Decrement count.
     */
    unsigned int dec_count()
    {
        return --count;
    }

protected:
    /** get a pointer to the start of the data.
     */
    virtual char *data() = 0;
    
    /** Like a constructor, but in this case, we allocate extra space for the
     * user data.
     * @param size size of user data in bytes
     * @param align_size alignment size if allocated as an array
     * @param items number of items to allocate
     * @return newly allocated buffer or addres of first item in an array of
     *         allocated buffers, HASSERT() on failure
     */
    static BufferManager *alloc(size_t size, size_t align_size, size_t items = 1)
    {
        HASSERT(items != 0);
        BufferManager *buffer = (BufferManager*)malloc(align_size * items);
        HASSERT(buffer != NULL);
        BufferManager *result = buffer;
        for (size_t i = 0; i < items; ++i)
        {
            buffer->next = NULL;
            buffer->size_ = size;
            buffer->left = size;
            buffer->count = 1;
            buffer = (BufferManager*)((char*)buffer + align_size);
        }
        return result;
    }

    /** Like a constructor, but in this case, we re-purpose an existing buffer
     * with no new memory allocation.
     * @param buffer instance of buffer to reinitialize
     * @param size size of user data in bytes
     * @return newly reinitialized buffer, HASSERT() on failure
     */
    static BufferManager *init(BufferManager *buffer, size_t size)
    {
        HASSERT(buffer->size_ == size);
        buffer->next = NULL;
        buffer->left = size;
        buffer->count = 1;
        return buffer;
    }

    /** size of data in bytes */
    size_t size_;

    /** amount for free space left in the buffer */
    size_t left;

    /** number of references in use */
    unsigned int count;
    
    /** Constructor.
     */
    BufferManager(size_t size)
        : QMember(),
          size_(size),
          left(size),
          count(1)
    {
    }

    /** Destructor.
     */
    ~BufferManager()
    {
    }

private:
    /** Constructor.
     */
    BufferManager();
    
    DISALLOW_COPY_AND_ASSIGN(BufferManager);
};

/** Buffer structure that contains both metadata and user data */
class Buffer : public BufferManager
{
public:
    /** Free this buffer to the BufferPool from whence it came.
     */
    inline void free();

    /** Expand the buffer size.  Exercise caution when using this API.  If anyone
     * else is holding onto a reference of this, their reference will be corrupted.
     * @param size size buffer after expansion.
     * @return newly expanded buffer with old buffer data moved
     */
    Buffer *expand(size_t size);

    /** Set the unique identifier for the buffer.
     * @param identifier 32-bit unique identifier
     */
    void id(uint32_t identifier)
    {
        id_ = identifier;
    }

    /** Get the unique identifier for the buffer.
     * @return 32-bit unique identifier
     */
    uint32_t id()
    {
        return id_;
    }

    /** Get a pointer to the pool that this buffer belongs to.
     * @return pool that this buffer belongs to
     */
    Pool<Buffer> *pool()
    {
        return pool_;
    }
    
private:
    /** get a pointer to the start of the data.
     */
    char *data()
    {
        return data_;
    }
    
    /** The total size of an array element of a Buffer for given payload.
     * @param size payload size
     */
    static size_t sizeof_type(size_t size)
    {
        return sizeof(Buffer) + (((size/sizeof(long)) + (size % sizeof(long) ? 1 : 0)) * sizeof(long));
    }

    /** Like a constructor, but in this case, we allocate extra space for the
     * user data.
     * @param pool Pool instance from which this buffer will come
     * @param size size of user data in bytes
     * @param items number of items to allocate
     * @return newly allocated buffer or addres of first item in an array of
     *         allocated buffers, HASSERT() on failure
     */
    static Buffer *alloc(Pool<Buffer> *pool, size_t size, size_t items = 1)
    {
        HASSERT(pool != NULL);

        size_t align_size = sizeof_type(size);
        Buffer *buffer = (Buffer*)malloc(align_size * items);
        Buffer *result = buffer;
        for (size_t i = 0; i < items; ++i)
        {
            new (buffer) Buffer(size, pool);
            buffer = (Buffer*)((char*)buffer + align_size);
        }
        return result;
    }

    /** Like a constructor, but in this case, we re-purpose an existing buffer
     * with no new memory allocation.
     * @param buffer instance of buffer to reinitialize
     * @param size size of user data in bytes
     * @return newly reinitialized buffer, HASSERT() on failure
     */
    static Buffer *init(Buffer *buffer, size_t size)
    {
        HASSERT(buffer->pool_ != NULL);
        HASSERT(buffer->size_ == size);
        BufferManager::init(buffer, size);
        buffer->id_ = 0;
        return buffer;
    }

    /** pointer to Pool instance that this buffer belongs to */
    Pool<Buffer> *pool_;

    /** message ID for uniquely identifying this buffer in a queue */
    uint32_t id_;
    
    /** user data */
    char data_[];

    /** This class is a helper of Pool */
    template <class T> friend class Pool;
    
    /** This class is a helper of Pool */
    template <class T> friend class FixedPool;
    
    /** This class is a helper of Pool */
    template <class T> friend class DynamicPool;
    
    /** Constructor.
     * @param size size of buffer data in bytes
     * @param pool pool that this buffer belongs to
     */
    Buffer(size_t size, Pool<Buffer> *pool)
        : BufferManager(size),
          pool_(pool),
          id_(0)
    {
    }
    
    /** Destructor.
     */
    ~Buffer()
    {
    }

    /** Default Constructor.
     */
    Buffer();

    DISALLOW_COPY_AND_ASSIGN(Buffer);
};

/** This class implements a linked list "queue" of buffers.  It may be
 * instantiated to use the mainBufferPool for its memory pool, or optionally
 * another BufferPool instance may be specified for its memory pool.
 */
template <class T> class Q
{
public:
    /** Default Constructor.
     */
    Q()
        : head(NULL),
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
     */
    void insert(T *q)
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
        count++;
    }

    /** Get an item from the front of the queue.
     * @return item retrieved from queue
     */
    T *next()
    {
        T *q;

        if (head == NULL)
        {
            return NULL;
        }
        q = head;
        head = static_cast<T*>(q->next);
        count--;

        return q;
    }

    /** Get the number of pending items in the queue.
     * @return number of pending items in the queue
     */
    size_t pending()
    {
        return count;
    }

    /** Test if the queue is empty.
     * @return true if empty, else false
     */
    bool empty()
    {
        return (head == NULL);
    }
    
protected:

private:
    /** head item in queue */
    T *head;
    
    /** tail item in queue */
    T *tail;
    
    /** number of items in queue */
    size_t count;

    DISALLOW_COPY_AND_ASSIGN(Q);
};

/** A list of queues.
 */
template <class T> class QList
{
public:
    /** Default Constructor.
     * @param size number of queues in the list
     */
    QList(size_t size = 1)
        : list(new Q<T>[size]),
          size_(size)
    {
    }

    /** Destructor.
     */
    ~QList()
    {
        delete [] list;
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index in the list to operate on
     */
    void insert(T *q, unsigned index)
    {
        list[index].insert(q);
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue
     */
    T *next(unsigned index)
    {
        return list[index].next();
    }

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        return list[index].pending();
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        return list[index].empty();
    }

    /** Get the number of queues in the list
     * @return number of queues in the list
     */
    size_t size()
    {
        return size_;
    }
    
private:
    /** the list of queues */
    Q<T> *list;

    /** number of queues in the lists */
    size_t size_;
};

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
     * @return item retrieved from queue
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

/** A list of queues.
 */
template <class T> class QListProtected
{
public:
    /** Default Constructor.
     * @param size number of queues in the list
     */
    QListProtected(size_t size = 1)
        : list(new Q<T>[size]),
          size_(size),
          mutex()
    {
    }

    /** Destructor.
     */
    ~QListProtected()
    {
        delete [] list;
    }

    /** Add an item to the back of the queue.
     * @param item to add to queue
     * @param index in the list to operate on
     */
    void insert(T *q, unsigned index)
    {
        mutex.lock();
        list[index].insert(q);
        mutex.unlock();
    }

    /** Get an item from the front of the queue.
     * @param index in the list to operate on
     * @return item retrieved from queue
     */
    T *next(unsigned index)
    {
        mutex.lock();
        T *result = list[index].next();
        mutex.unlock();
        return result;
    }

    /** Get the number of pending items in the queue.
     * @param index in the list to operate on
     * @return number of pending items in the queue
     */
    size_t pending(unsigned index)
    {
        mutex.lock();
        size_t result = list[index].pending();
        mutex.unlock();
        return result;
    }

    /** Test if the queue is empty.
     * @param index in the list to operate on
     * @return true if empty, else false
     */
    bool empty(unsigned index)
    {
        mutex.lock();
        bool result = list[index].empty();
        mutex.unlock();
        return result;
    }

    /** Get the number of queues in the list
     * @return number of queues in the list
     */
    size_t size()
    {
        return size_;
    }
    
private:
    /** the list of queues */
    Q<T> *list;
    
    /** number of queues in the lists */
    size_t size_;

    /** @todo (Stuart Baker) For free RTOS, we may want to consider a different
     * (smaller) locking mechanism
     */
    /** Mutual exclusion for Queue */
    OSMutex mutex;
};

/** Pool of previously allocated, but currently unused, items. */
template <class T> class Pool
{
public:
    /** Get a free item out of the pool.
     * @param size minimum size in bytes the item must hold
     * @return pointer to the newly allocated item
     */
    virtual T *alloc(size_t size)
    {
        return NULL;
    }

    /** Release an item back to the free pool.
     * @param item pointer to item to release
     */
    virtual void free(T *item)
    {
    }

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

    /** Mutual exclusion for buffer pool */
    OSMutex mutex;

    /** keep track of total allocated size of memory */
    size_t totalSize;
    
private:
    DISALLOW_COPY_AND_ASSIGN(Pool);
};

/** A specialization of a pool which can allocate new elements dynamically
 * upon request.
 */
template <class T> class DynamicPool : public Pool <T>
{
public:
    /** This is a struct for storing info about a specific size item in the
     * DynamicPool.
     */
    class Bucket
    {
    public:
        /** Allocate a Bucket array off of the heap initialized with sizes.
         * @param s size of first bucket
         * @param '0' terminated list of additional buckets
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
            
            Bucket *bucket = new Bucket[count];

            for (int i = 0; i < count; ++i)
            {
                bucket[i].size_ = va_arg(aq, int);
                bucket[i].first_ = NULL;
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
            delete [] bucket;
        }
        
    private:
        /** Constructor.
         */
        Bucket()
        {
        }
        
        /** Destructor.
         */
        ~Bucket()
        {
        }
        
        size_t size_; /**< size of entry */
        T* first_; /**< first item in list of entries */
        
        /** Allow Dynamic Pool access to private members */
        template <class C> friend class DynamicPool;
    };
    
    /** Get a free item out of the pool.
     * @param size minimum size in bytes the item must hold
     * @return pointer to the newly allocated item
     */
    T *alloc(size_t size)
    {
        T *item = NULL;

        for (Bucket *current = buckets; current->size_ != 0; current++)
        {
            if (size <= current->size_)
            {
                Pool<T>::mutex.lock();
                if (current->first_ != NULL)
                {
                    item = current->first_;
                    current->first_ = static_cast<T*>(item->next);
                    (void)T::init(item, current->size_);
                }
                else
                {
                    item = T::alloc(this, current->size_);

                    totalSize += current->size_ + sizeof(T);
                    //DEBUG_PRINTF("cxx buffer total size: %zu\n", totalSize);
                }
                Pool<T>::mutex.unlock();
                return item;
            }
        }
         
        /* big items are just malloc'd freely */
        item = T::alloc(this, size);
        Pool<T>::mutex.lock();
        totalSize += size + sizeof(T);
        Pool<T>::mutex.unlock();
        //DEBUG_PRINTF("cxx buffer total size: %zu\n", totalSize);
        return item;
     }

    /** Release an item back to the free pool.
     * @param item pointer to item to release
     */
    void free(T *item)
    {
        HASSERT(this == item->pool());

        if (item->dec_count() == 0)
        {
            for (Bucket *current = buckets; current->size_ != 0; current++)
            {
                if (item->size() <= current->size_)
                {
                    Pool<T>::mutex.lock();
                    item->next = current->first_;
                    current->first_ = item;
                    Pool<T>::mutex.unlock();
                    return;
                }
            }
            Pool<T>::mutex.lock();
            /* big items are just freed */
            totalSize -= item->size();
            totalSize -= sizeof(Buffer);
            Pool<T>::mutex.unlock();
            //DEBUG_PRINTF("buffer total size: %zu\n", totalSize);
            free(item);
        }
    }

    /** Constructor.
     * @param sizes array of bucket sizes for the pool
     */
    DynamicPool(Bucket sizes[])
        : Pool<T>(),
          totalSize(0),
          buckets(sizes),
          first(NULL),
          itemSize(0)
    {
    }

    /** default destructor */
    ~DynamicPool()
    {
        Bucket::destroy(buckets);
    }

protected:
    /** keep track of total allocated size of memory */
    size_t totalSize;
    
    /** Free buffer queue */
    Bucket *buckets;
    
    /** First buffer in a pre-allocated array pool */
    T *first;
    
    /** item Size for fixed pools */
    size_t itemSize;
    
    /** total number of items in the queue */
    size_t items;
    
private:
    /** Default constructor.
     */
    DynamicPool();

    DISALLOW_COPY_AND_ASSIGN(DynamicPool);
};

/** Pool of fixed number of items which can be allocated up on request.
 */
template <class T> class FixedPool : public Pool <T>
{
public:
    /** Used in static pools to tell if this buffer is a member of the pool.
     * @param buffer buffer to test validity on
     * @return true if the buffer is in the pool, or this is not a fixed pool,
     *         else return false;
     */
    bool valid(T *item)
    {
        if (item >= first &&
            item <= (T*)((char*)first + (items * T::sizeof_type(itemSize))))
        {
            return true;
        }
        return false;
    }

    /** Get a free item out of the pool.
     * @param size minimum size in bytes the item must hold
     * @return pointer to the newly allocated item
     */
    virtual T *alloc()
    {
        Pool<T>::mutex.lock();
        T *item = queue.next();
        if (item != NULL)
        {
            (void)T::init(item, itemSize);
            totalSize++;
            //DEBUG_PRINTF("static buffer total size: %zu\n", totalSize);
        }
        Pool<T>::mutex.unlock();
        return item;
    }

    /** Release an item back to the free pool.
     * @param item pointer to item to release
     */
    virtual void free(T *item)
    {
        HASSERT(this == item->pool());

        Pool<T>::mutex.lock();
        if (item->dec_count() == 0)
        {
            totalSize--;
            queue.insert(item);
            //DEBUG_PRINTF("static buffer total used: %zu\n", totalSize);
        }

        Pool<T>::mutex.unlock();
    }

    /** Constructor for a fixed size pool.
     * @param item_size size of each item in the pool
     * @param items number of items in the pool
     */
    FixedPool(size_t item_size, size_t items)
        : Pool<T>(),
          //mutex(true),
          totalSize(0),
          first (T::alloc(this, item_size, items)),
          itemSize(item_size),
          items(items)
    {
        T *current = first;
        for (size_t i = 0; i < items; ++i)
        {
            queue.insert(current);
            current = (T*)((char*)current + T::sizeof_type(item_size));
        }
    }

    /** default destructor */
    ~FixedPool()
    {
        /** @todo (Stuart Baker) need to free the itmes alloced */
    }

protected:
    /** keep track of total allocated size of memory */
    size_t totalSize;
    
    /** Free buffer queue */
    Q<T> queue;
    
    /** First buffer in a pre-allocated array pool */
    T *first;
    
    /** item Size for fixed pools */
    size_t itemSize;
    
    /** total number of items in the queue */
    size_t items;
    
private:
    /** Default Constructor.
     */
    FixedPool();

    DISALLOW_COPY_AND_ASSIGN(FixedPool);
};

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
        
    /** Wait for a buffer from the front of the queue.
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
    
    /** Wait for a buffer from the front of the queue.
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
    
    /** Wakeup anyone waiting on the wait queue.
     */
    void wakeup()
    {
        post();
    }
    
private:

    DISALLOW_COPY_AND_ASSIGN(QueueProtectedWait);
};

/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.
 */
template <class T> class QueueListProtectedWait : public QListProtected <T>, public OSSem
{
public:
    /** Default Constructor.
     * @param size number of queues in the list
     */
    QueueListProtectedWait(size_t size)
        : QListProtected<T>(size),
          OSSem(0)
    {
    }

    /** Default destructor.
     */
    ~QueueListProtectedWait()
    {
    }

    /** Add an item to the back of the queue.
     * @param item item to add to queue
     * @param index in the list to operate on
     */
    void insert(T *item, unsigned index)
    {
        QListProtected<T>::insert(item, index);
        post();
    }

    /** Get an item from the front of the queue.
     * @return item retrieved from one of the queues
     */
    T *next()
    {
        T *result = NULL;
        for (size_t i = 0; i < QListProtected<T>::size(); ++i)
        {
            result = QListProtected<T>::next();
            if (result)
            {
                break;
            }
        }
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
        T *result = NULL;
        for (size_t i = 0; i < QListProtected<T>::size(); ++i)
        {
            result = QListProtected<T>::next();
            if (result)
            {
                break;
            }
        }
        if(result == NULL)
        {
            errno = EINTR;
        }
        return result;
    }
    
    /** Wait for a buffer from the front of the queue.
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
        
        T *result = NULL;
        for (size_t i = 0; i < QListProtected<T>::size(); ++i)
        {
            result = QListProtected<T>::next();
            if (result)
            {
                break;
            }
        }
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

    DISALLOW_COPY_AND_ASSIGN(QueueListProtectedWait);
};

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

/** Free this buffer to the BufferPool from whence it came.
 */
inline void Buffer::free()
{
    HASSERT(pool_ != NULL);
    pool_->free(this);
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
    //pool->mutex.lock();
    ++count;
    //pool->mutex.unlock();
    return count;
}

#endif /* _BufferQueue_hxx_ */
