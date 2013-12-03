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

#include <cstdint>
#include <cstdlib>

#include <os/OS.hxx>

class BufferPool;

/** main buffer pool instance */
extern BufferPool *mainBufferPool; 

/** Buffer structure that contains both metadata and user data */
class Buffer
{
public:
    /** Free this buffer to the BufferPool from whence it came.
     */
    inline void free();

    /** Advance the position of the buffer.
     * @param bytes number of bytes to advance.
     * @return pointer to the new position (next available byte)
     */
    void *advance(size_t bytes);
    
    /** reset the buffer position back to beginning.
     * @return pointer to the new position (next available byte)
     */
    void *zero()
    {
        left = _size;
        return data;
    }    
    
    /** Get a pointer to the first position (byte) of the buffer.
     * @return pointer to the first position (byte)
     */
    const void *start() const
    {
        return data;
    }

    /** Get a pointer to the first position (byte) of the buffer.
     * @return pointer to the first position (byte)
     */
    void *start()
    {
        return data;
    }

    /** Get a pointer to the current position of the buffer.
     * @return pointer to the current position (next available byte)
     */
    void *position()
    {
        return &data[_size - left];
    }

    /** Get the size of the buffer in bytes.
     * @return size of the buffer in bytes
     */
    size_t size() const
    {
        return _size;
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
        return _size - left;    
    }

    /** Expand the buffer size.
     * @param size size buffer after expansion.
     * @return newly expanded buffer with old buffer data moved
     */
    Buffer *expand(size_t size);

    /** Set the unique identifier for the buffer.
     * @param identifier 32-bit unique identifier
     */
    void id(uint32_t identifier)
    {
        _id = identifier;
    }

    /** Get the unique identifier for the buffer.
     * @return 32-bit unique identifier
     */
    uint32_t id()
    {
        return _id;
    }

    /** Add another reference to the buffer.
     * @return total number of references to this point
     */
    unsigned int reference();

private:
    /** Like a constructor, but in this case, we allocate extra space for the
     * user data.
     * @param pool BufferPool instance from which this buffer will come
     * @param size size of user data in bytes
     * @param items number of items to allocate
     * @return newly allocated buffer or addres of first item in an array of
     *         allocated buffers, HASSERT() on failure
     */
    static Buffer *alloc(BufferPool *pool, size_t size, size_t items = 1)
    {
        HASSERT(pool != NULL && items != 0);
        size_t align_size = sizeof_buffer(size);
        Buffer *buffer = (Buffer*)malloc(align_size * items);
        HASSERT(buffer != NULL);
        Buffer *result = buffer;
        for (size_t i = 0; i < items; ++i)
        {
            buffer->next = NULL;
            buffer->bufferPool = pool;
            buffer->_size = size;
            buffer->left = size;
            buffer->count = 1;
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
        HASSERT(buffer->bufferPool != NULL);
        HASSERT(buffer->_size == size);
        buffer->next = NULL;
        buffer->left = size;
        buffer->count = 1;
        return buffer;
    }

    /** Macro to position to beginning of structure from data member position*/
    #define BUFFER(_buffer) (Buffer*)((char *)(_buffer) - sizeof(Buffer));

    /* pointer to BufferPool instance that this buffer belongs to */
    BufferPool *bufferPool;

    /** next buffer in list */
    Buffer *next;
    
    /** size of data in bytes */
    size_t _size;

    /** amount for free space left in the buffer */
    size_t left;

    /** message ID for uniquely identifying this buffer in a queue */
    uint32_t _id;
    
    /** number of references in use */
    unsigned int count;
    
    /** user data */
    char data[];

    /** This class is a helper of BufferPool, so we know where to free to */
    friend class BufferPool;
    
    /** This class is a helper of BufferQueue */
    friend class BufferQueue;

    /** The total size of an array element of a Buffer for given payload.
     * @param size payload size
     */
    static size_t sizeof_buffer(size_t size)
    {
        return sizeof(Buffer) + (((size/sizeof(long)) + (size % sizeof(long) ? 1 : 0)) * sizeof(long));
    }
    
    /** Default constructor */
    Buffer();
    
    /** Default destructor */
    ~Buffer();

    DISALLOW_COPY_AND_ASSIGN(Buffer);
};

/** Pool of previously allocated, but currently unused, buffers. */
class BufferPool
{
public:
    /* Default Constructor */
    BufferPool()
        : totalSize(0),
          mutex(),
          pool {NULL, NULL, NULL, NULL},
          itemSize(0),
          items(0)
    {
    }

    /* Constructor for a fixed size pool.
     * @param item_size size of each item in the pool
     * @param items number of items in the pool
     */
    BufferPool(size_t item_size, size_t items)
        : totalSize(0),
          mutex(),
          pool {Buffer::alloc(this, item_size, items), NULL, NULL, NULL},
          itemSize(item_size),
          items(items)
    {
        Buffer *current = first;
        for (size_t i = 0; i < items; ++i)
        {
            current->next = pool[1];
            pool[1] = current;
            current = (Buffer*)((char*)current + Buffer::sizeof_buffer(item_size));
        }
        /* save the index just after last buffer in the bool */
        pool[2] = current;
    }

    /* default destructor */
    ~BufferPool()
    {
      /** @todo(stbaker): what is the required condition for a buffer pool to
       be deallocated?
       HASSERT(0);
      */
    }

    /** Used in static pools to tell if this buffer is a member of the pool.
     * @param buffer buffer to test validity on
     * @return true if the buffer is in the pool, or this is not a fixed pool,
     *         else return false;
     */
    bool valid(Buffer *buffer)
    {
        if (itemSize != 0)
        {
            if (buffer >= first && buffer <= pool[2])
            {
                return true;
            }
            return false;
        }
        return true;
    }
    
    /** Get a free buffer out of the pool.  A buffer may be
     * obtained without context (object reference) from the mainBufferPool
     * with the ::buffer_free method.
     *
     * @param size minimum size in bytes the buffer must hold
     * @return pointer to the newly allocated buffer
     */
    Buffer *buffer_alloc(size_t size);

    /** Release a buffer back to the free buffer pool.  A buffer may be
     * released without context (object reference) to the mainBufferPool
     * with the ::buffer_free method.
     *
     * @param buffer pointer to buffer to release
     */
    void buffer_free(Buffer *buffer);
    
private:
    /** keep track of total allocated size of memory */
    size_t totalSize;
    
    /** Mutual exclusion for buffer pool */
    OSMutex mutex;


    /** this union save overlapping memory */
    union
    {
        /** Free buffer pool */
        Buffer *pool[4];
        
        /** First buffer in a pre-allocated array pool */
        Buffer *first;
    };
    
    /** item Size for fixed pools */
    size_t itemSize;
    
    /** number of items for fixed pools */
    size_t items;

    /** This class is a helper of BufferQueue */
    friend class BufferQueue;

    /** This class is a helper of Buffer */
    friend class Buffer;

    DISALLOW_COPY_AND_ASSIGN(BufferPool);
};

/** This class implements a linked list "queue" of buffers.  It may be
 * instantiated to use the mainBufferPool for its memory pool, or optionally
 * another BufferPool instance may be specified for its memory pool.
 */
class BufferQueue
{
public:
    /** Default Constructor, use mainBufferPool for buffer allocation. */
    BufferQueue()
        : head(NULL),
          tail(NULL),
          count(0),
          mutex()
    {
    }

    /** Default destructor.
     */
    ~BufferQueue()
    {
    }

    /** Release a buffer back to the free buffer pool.
     * @param buffer pointer to buffer to release
     */
    void buffer_free(Buffer *buffer)
    {
        buffer->free();
    }

    /** Add a buffer to the back of the queue.
     * @param buffer buffer to add to queue
     */
    void insert(Buffer *buffer);

    /** Get a buffer from the front of the queue.
     * @return buffer buffer retrieved from queue
     */
    Buffer *next();

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
    /** head buffer in queue */
    Buffer *head;
    
    /** tail buffer in queue */
    Buffer *tail;
    
    /** number of buffers in queue */
    size_t count;

    /** @todo (Stuart Baker) For free RTOS, we may want to consider a different
     * (smaller) locking mechanism
     */
    /** Mutual exclusion for Queue */
    OSMutex mutex;

    DISALLOW_COPY_AND_ASSIGN(BufferQueue);
};

/** A BufferQueue that adds the ability to wait on the next buffer.
 * Yes this uses multiple inheritance.  Yes multiple inheritance is bad.  It
 * is okay in this case, so get over it.
 */
class BufferQueueWait : public BufferQueue, public OSSem
{
public:
    /** Default Constructor, use mainBufferPool for buffer allocation. */
    BufferQueueWait()
        : BufferQueue(),
          sem(0)
    {
    }

    /** Default destructor.
     */
    ~BufferQueueWait()
    {
    }

    /** Add a buffer to the back of the queue.
     * @param buffer buffer to add to queue
     */
    void insert(Buffer *buffer)
    {
        BufferQueue::insert(buffer);
        post();
    }

    /** Get a buffer from the front of the queue.
     * @return buffer buffer retrieved from queue
     */
    Buffer *next()
    {
        Buffer *result = BufferQueue::next();
        if (result != NULL)
        {
            /* decrement semaphore */
            OSSem::wait();
        }
        return result;
    }
    
    /** Wait for a buffer from the front of the queue.
     * @return buffer buffer retrieved from queue
     */
    Buffer *wait()
    {
        OSSem::wait();
        Buffer *result = BufferQueue::next();
        HASSERT(result != NULL);
        return result;
    }
    
    /** Wait for a buffer from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @return buffer buffer retrieved from queue, NULL on timeout or error
     */
    Buffer *timedwait(long long timeout)
    {
        if (OSSem::timedwait(timeout) != 0)
        {
            return NULL;
        }
        
        Buffer *result = BufferQueue::next();
        HASSERT(result != NULL);
        return result;
    }
    
private:
    /** Semaphore that we will wait on */
    OSSem sem;

    DISALLOW_COPY_AND_ASSIGN(BufferQueueWait);
};

/** Get a free buffer out of the mainBufferPool pool.
 * @param size minimum size in bytes the buffer must hold
 * @return pointer to the newly allocated buffer
 */
inline Buffer *buffer_alloc(size_t size)
{
    return mainBufferPool->buffer_alloc(size);
}

/** Release a buffer back to the mainBufferPool free buffer pool.
 * @param buffer pointer to buffer to release
 */
inline void buffer_free(Buffer *buffer)
{
    mainBufferPool->buffer_free(buffer);
}

/** Free this buffer to the BufferPool from whence it came.
 */
inline void Buffer::free()
{
    HASSERT(bufferPool != NULL);
    bufferPool->buffer_free(this);
}

/** Advance the position of the buffer.
 * @param bytes number of bytes to advance.
 * @return pointer to the new position (next available byte)
 */
inline void *Buffer::advance(size_t bytes)
{
    /** @todo (Stuart Baker) do we really need a mutex lock here? */
    bufferPool->mutex.lock();
    left -= bytes;    
    bufferPool->mutex.unlock();
    return &data[_size - left];
}

/** Add another reference to the buffer.
 * @return total number of references to this point
 */
inline unsigned int Buffer::reference()
{
    bufferPool->mutex.lock();
    ++count;
    bufferPool->mutex.unlock();
    return count;
}

#endif /* _BufferQueue_hxx_ */
