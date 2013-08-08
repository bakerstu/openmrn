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
    void *advance(size_t bytes)
    {
        left -= bytes;    
        return &data[_size - left];
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
    size_t size()
    {
        return _size;
    }

    /** Get the number of unused bytes in the buffer.
     * @return number of unused bytes
     */
    size_t available()
    {
        return left;    
    }

    /** Expand the buffer size.
     * @param size size buffer after expansion.
     * @return newly expanded buffer with old buffer data moved
     */
    Buffer *expand(size_t size);

private:
    /** Like a constructor, but in this case, we allocate extra space for the
     * user data.
     * @param pool BufferPool instance from which this buffer will come
     * @param size size of user data in bytes
     * @return newly allocated buffer, HASSERT() on failure
     */
    static Buffer *alloc(BufferPool *pool, size_t size)
    {
        HASSERT(pool != NULL);
        Buffer *buffer = (Buffer*)malloc(size + sizeof(Buffer));
        HASSERT(buffer != NULL);
        buffer->next = NULL;
        buffer->bufferPool = pool;
        buffer->_size = size;
        buffer->left = size;
        return buffer;
    }

    /** Like a constructor, but in this case, we re-purpose an existing buffer
     * with no new memory allocation.
     * @param pool BufferPool instance from which this buffer will come
     * @param size size of user data in bytes
     * @return newly allocated buffer, HASSERT() on failure
     */
    static Buffer *init(Buffer *buffer, size_t size)
    {
        HASSERT(buffer->bufferPool != NULL);
        HASSERT(buffer->_size != size);
        buffer->next = NULL;
        buffer->left = size;
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
    
    /** user data */
    char data[];

    /** This class is a helper of BufferPool, so we know where to free to */
    friend class BufferPool;
    
    /** This class is a helper of BufferQueue */
    friend class BufferQueue;

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
          pool({NULL, NULL, NULL, NULL})
    {
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

    /** Free buffer pool */
    Buffer *pool[4];

    /** This class is a helper of BufferQueue */
    friend class BufferQueue;

    /* default destructor */
    ~BufferPool();

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
          mutex(),
          bufferPool(mainBufferPool)
    {
    }
    
    /** Use provided buffer pool for buffer allocation.
     * @param bufferPool buffer pool to use for this buffer queue
     */
    BufferQueue(BufferPool *bufferPool)
        : head(NULL),
          tail(NULL),
          count(0),
          mutex(),
          bufferPool(bufferPool)
    {
        HASSERT(bufferPool != NULL);
    }

    /** Get a free buffer out of the pool.
     * @param size minimum size in bytes the buffer must hold
     * @return pointer to the newly allocated buffer
     */
    Buffer *buffer_alloc(size_t size)
    {
        return bufferPool->buffer_alloc(size);
    }

    /** Release a buffer back to the free buffer pool.
     * @param buffer pointer to buffer to release
     */
    void buffer_free(Buffer *buffer)
    {
        bufferPool->buffer_free(buffer);
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
    /** Default destructor.
     */
    ~BufferQueue();

private:
    /** head buffer in queue */
    Buffer *head;
    
    /** tail buffer in queue */
    Buffer *tail;
    
    /** number of buffers in queue */
    size_t count;

    /** Mutual exclusion for Queue */
    OSMutex mutex;

    /** memory pool */
    BufferPool *bufferPool;
    
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
    
    /** Use provided buffer pool for buffer allocation.
     * @param bufferPool buffer pool to use for this buffer queue
     */
    BufferQueueWait(BufferPool *bufferPool)
        : BufferQueue(bufferPool),
          sem(0)
    {
        HASSERT(bufferPool != NULL);
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
    /** Default destructor.
     */
    ~BufferQueueWait();

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
void Buffer::free()
{
    HASSERT(bufferPool != NULL);
    bufferPool->buffer_free(this);
}

#endif /* _BufferQueue_hxx_ */
