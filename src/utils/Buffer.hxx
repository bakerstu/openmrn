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
 * \file Buffer.hxx
 * This file provides buffers with allocation and freelist/reuse mechanisms.
 *
 * @author Stuart W. Baker
 * @date 3 August 2013
 */

#ifndef _UTILS_BUFFER_HXX_
#define _UTILS_BUFFER_HXX_

// Enable this to collect the pointer of all buffers live.
//#define DEBUG_BUFFER_MEMORY

#include <new>
#include <cstdint>
#include <cstdlib>
#include <cstdarg>

#include "executor/Executable.hxx"
#include "executor/Notifiable.hxx"
#include "os/OS.hxx"
#include "utils/Atomic.hxx"
#include "utils/MultiMap.hxx"
#include "utils/QMember.hxx"
#include "utils/Queue.hxx"
#include "utils/macros.h"

class DynamicPool;
class FixedPool;
class Pool;
template <class T> class Buffer;
class BufferBase;

namespace nmranet
{
class AsyncIfTest;
}

/** main buffer pool instance */
extern DynamicPool *mainBufferPool;

/** Initializes the main buffer pool. The first call is not thread safe, later
 * calls are noops. It is recommended to call this one or more times from the
 * static initialization. */
Pool* init_main_buffer_pool();

/** This pointer will be saved for debugging the current allocation source. */
extern void* g_current_alloc;

/// Abstract base class for all Buffers. This class contains all shared
/// components that are not template-dependent.
class BufferBase : public QMember
{
public:
    uint16_t references()
    {
        return count_;
    }

    void set_done(BarrierNotifiable *done)
    {
        done_ = done;
    }

    /** Creates a new child notifiable of the current done notifiable. @return
     * the new notifiable, or NULL if there is no current notifiable. */
    BarrierNotifiable *new_child()
    {
        if (done_)
        {
            return done_->new_child();
        }
        else
        {
            return nullptr;
        }
    }

    size_t size()
    {
        return size_;
    }

    /** Expand the buffer by allocating a buffer double the size, copying the
     * contents to the new buffer, and freeing the old buffer.  The "this"
     * pointer
     * of the caller will be used to free the buffer.
     * @return newly expanded buffer
     */
    BufferBase *expand();

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

    BarrierNotifiable *done_;

    /** Constructor.  Initializes count to 1 and done_ to NULL.
     * @param size size of buffer data
     * @param pool pool this buffer belong to
     */
    BufferBase(size_t size, Pool *pool)
        : QMember()
        , size_(size)
        , count_(1)
        , pool_(pool)
        , done_(NULL)
    {
    }

    /** Destructor.
     */
    ~BufferBase()
    {
        if (done_)
        {
            done_->notify();
        }
    }

    friend class nmranet::AsyncIfTest;

    /** Allow Pool access to our constructor */
    friend class Pool;

    /** Allow DynamicPool access to our constructor */
    friend class DynamicPool;

    /** Allow FixedPool access to our constructor */
    friend class FixedPool;

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
     * @return the referenced buffer pointer.
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
        : BufferBase(sizeof(Buffer<T>), pool)
        , data_()
    {
    }

    /** Destructor.
     */
    ~Buffer()
    {
    }

    DISALLOW_COPY_AND_ASSIGN(Buffer);

    /** Allow Pool access to our constructor */
    friend class Pool;

    /** user data */
    T data_;
};

/** This class will automatically unref a Buffer when going out of scope. */
template <class T> class AutoReleaseBuffer
{
public:
    AutoReleaseBuffer(Buffer<T> *b)
        : b_(b)
    {
    }

    ~AutoReleaseBuffer()
    {
        if (b_)
        {
            b_->unref();
        }
    }

    /** Returns the held buffer pointer and relinquishes ownership. The buffer
     * will not be unref-ed when *this goes out of scope.
     *
     * @returns the buffer pointer held. Transfers ownership to the caller. */
    Buffer<T> *release()
    {
        Buffer<T> *b = b_;
        b_ = nullptr;
        return b;
    }

private:
    Buffer<T> *b_;
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

/** This is a struct for storing info about a specific size item in the
 * DynamicPool.
 */
class Bucket : public Q
{
public:
    /** Allocate a Bucket array off of the heap initialized with sizes.
     * @param s size of first bucket
     * @param ... '0' terminated list of additional buckets
     * @todo (Stuart Baker) fix such that sizes do not need to be in strict
     * ascending order
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

        Bucket *bucket = (Bucket *)malloc(sizeof(Bucket) * count);
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
        : Q()
        , size_(size)
        , pending_()
    {
    }

    /** Destructor.
     */
    ~Bucket()
    {
    }

    size_t size_; /**< size of entry */
public:
    size_t allocCount_{0}; /**< total entries allocated */
private:
    /** list of anyone waiting for an item in the bucket */
    Q pending_;
};

/** A specialization of a pool which can allocate new elements dynamically
 * upon request.
 */
class DynamicPool : public Pool, private Atomic
{
public:
    /** Constructor.
     * @param sizes array of bucket sizes for the pool
     */
    DynamicPool(Bucket sizes[])
        : Pool()
        , totalSize(0)
        , buckets(sizes)
    {
    }

    /** default destructor */
    ~DynamicPool()
    {
        Bucket::destroy(buckets);
    }

    /** Number of free items in the pool.
     * @return number of free items in the pool
     */
    size_t free_items();

    /** Number of free items in the pool for a given allocation size.
     * @param size size of interest
     * @return number of free items in the pool for a given allocation size
     */
    size_t free_items(size_t size);

    /** Returns the total memory held by this pool. */
    size_t total_size()
    {
        return totalSize;
    }

protected:
    /** keep track of total allocated size of memory */
    size_t totalSize;

    /** Free buffer queue */
    Bucket *buckets;

private:
    /** Get a free item out of the pool.
     * @param result pointer to a pointer to the result
     * @param flow if !NULL, then the alloc call is considered async and will
     *        behave as if @ref alloc_async() was called.
     */
    BufferBase *alloc_untyped(size_t size, Executable *flow);

    /** Allocates a large memory block directly from the heap. */
    void* alloc_large(size_t size);
    /** Frees a large memory block allocated by alloc_large. */
    void free_large(void* block);

    /** Release an item back to the free pool.
     * @param item pointer to item to release
     * @param size size of buffer to free
     */
    void free(BufferBase *item);

    /** Default constructor.
     */
    DynamicPool();

    DISALLOW_COPY_AND_ASSIGN(DynamicPool);
};

/** Pool of fixed number of items which can be allocated up on request.
 */
class FixedPool : public Pool, public Atomic
{
public:
    /** Used in static pools to tell if this item is a member of the pool.
     * @param item to test validity on
     * @return true if the item is in the pool, else return false;
     */
    bool valid(QMember *item)
    {
        if ((char *)item >= mempool &&
            (char *)item < (mempool + (items * itemSize)))
        {
            return true;
        }
        return false;
    }

    /** Constructor for a fixed size pool.
     * @param item_size size of each item in the pool (including the Buffer
     * wrapper).
     * @param items number of items in the pool
     */
    FixedPool(size_t item_size, size_t items)
        : Pool()
        , totalSize(0)
        , mempool(new char[(item_size * items)])
        , itemSize(item_size)
        , items(items)
        , empty(false)
    {
        // HASSERT(item_size != 0 && items != 0);
        QMember *current = (QMember *)mempool;
        for (size_t i = 0; i < items; ++i)
        {
            current->init();
            queue.insert(current);
            current = (QMember *)((char *)current + item_size);
        }
    }

    /** default destructor */
    ~FixedPool()
    {
        delete[] mempool;
    }

    /** Number of free items in the pool.
     * @return number of free items in the pool
     */
    size_t free_items()
    {
        return items - (totalSize / itemSize);
    }

    /** Number of free items in the pool for a given allocation size.
     * @param size size of interest
     * @return number of free items in the pool for a given allocation size
     */
    size_t free_items(size_t size)
    {
        return size == itemSize ? free_items() : 0;
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
    /** Get a free item out of the pool.
     * @param result pointer to a pointer to the result
     * @param flow if !NULL, then the alloc call is considered async and will
     *        behave as if @ref alloc_async() was called.
     */
    BufferBase *alloc_untyped(size_t size, Executable *flow);

    /** Release an item back to the free pool.
     * @param item pointer to item to release
     * @param size size of buffer to free
     */
    void free(BufferBase *item);

    /** Default Constructor.
     */
    FixedPool();

    DISALLOW_COPY_AND_ASSIGN(FixedPool);
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

#endif /* _UTILS_BUFFER_HXX_ */
