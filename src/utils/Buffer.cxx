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
 * \file Buffer.cxx
 * Implementation of non-templated functions of buffers and pools.
 *
 * @author Stuart W. Baker
 * @date 3 August 2013
 */

#include "utils/Buffer.hxx"

DynamicPool *mainBufferPool = nullptr;

Pool* init_main_buffer_pool()
{
    if (!mainBufferPool)
    {
        mainBufferPool = new DynamicPool(Bucket::init(16, 32, 48, 72, 0));
    }
    return mainBufferPool;
}

/** Expand the buffer by allocating a buffer double the size, copying the
 * contents to the new buffer, and freeing the old buffer.  The "this" pointer
 * of the caller will be used to free the buffer.
 * @return newly expanded buffer
 */
BufferBase *BufferBase::expand()
{
    /* create the new buffer */
    BufferBase *expanded_buffer = pool_->alloc_untyped(size_ * 2, NULL);
    HASSERT(expanded_buffer);

    /* copy uninitialized data over */
    memcpy(expanded_buffer + 1, this + 1, size_ - sizeof(BufferBase));
    expanded_buffer->count_ = count_;
    expanded_buffer->done_ = done_;
    
    /* free the old buffer */
    pool_->free(this);

    return expanded_buffer;
}

/** Number of free items in the pool.
 * @return number of free items in the pool
 */
size_t DynamicPool::free_items()
{
    size_t total = 0;
    for (Bucket *current = buckets; current->size() != 0; ++current)
    {
        total += current->pending();
    }
    return total;
}

/** Number of free items in the pool for a given allocation size.
 * @param size size of interest
 * @return number of free items in the pool for a given allocation size
 */
size_t DynamicPool::free_items(size_t size)
{
    for (Bucket *current = buckets; current->size() != 0; ++current)
    {
        if (current->size() >= size)
        {
            return current->pending();
        }
    }
    return 0;
}

#ifdef DEBUG_BUFFER_MEMORY
/* key: buffer pointer. Value: instruction pointer for allocation caller. */
std::map<BufferBase*, void*> g_alloc_source;
Atomic g_alloc_atomic;
void* g_current_alloc;
#endif

extern "C" {
extern void *buffer_malloc(size_t size);
}

/** Get a free item out of the pool.
 * @param result pointer to a pointer to the result
 * @param flow if !NULL, then the alloc call is considered async and will
 *        behave as if @ref alloc_async() was called.
 */
BufferBase *DynamicPool::alloc_untyped(size_t size, Executable *flow)
{
    BufferBase *result = NULL;

    for (Bucket *current = buckets; current->size() != 0; ++current)
    {
        if (size <= current->size())
        {
            result = static_cast<BufferBase*>(current->next().item);
            if (result == NULL)
            {
                result = (BufferBase*)buffer_malloc(current->size());
                {
                    AtomicHolder h(this);
                    if (0 && totalSize < 5000 && totalSize + current->size() >= 5000) {
                        HASSERT(0);
                    }
                    totalSize += current->size();
                }
            }
            new (result) BufferBase(size, this);
            break;
        }
    }

    if (!result)
    {
        /* big items are just malloc'd freely */
        result = (BufferBase*)alloc_large(size);
        new (result) BufferBase(size, this);
        {
            AtomicHolder h(this);
            totalSize += size;
        }
    }
#ifdef DEBUG_BUFFER_MEMORY
    {
        AtomicHolder h(&g_alloc_atomic);
        g_alloc_source[result] = g_current_alloc;
    }
#endif
    if (flow)
    {
        flow->alloc_result(result);
    }
    return result;
}

void* DynamicPool::alloc_large(size_t size) {
    return malloc(size);
}

void DynamicPool::free_large(void* buffer) {
    ::free(buffer);
}

/** Release an item back to the free pool.
 * @param item pointer to item to release
 * @param size size of buffer to free
 */
void DynamicPool::free(BufferBase *item)
{
#ifdef DEBUG_BUFFER_MEMORY
    {
        AtomicHolder h(&g_alloc_atomic);
        g_alloc_source.erase(item);
    }
#endif
    for (Bucket *current = buckets; current->size() != 0; ++current)
    {
        if (item->size() <= current->size())
        {
            current->insert(item);
            return;
        }
    }
    /* big items are just freed */
    {
        AtomicHolder h(this);
        totalSize -= item->size();
    }
    free_large(item);
}

/** Get a free item out of the pool.
 * @param result pointer to a pointer to the result
 * @param flow if !NULL, then the alloc call is considered async and will
 *        behave as if @ref alloc_async() was called.
 */
BufferBase *FixedPool::alloc_untyped(size_t size, Executable *flow)
{
    BufferBase *result = NULL;
    
    {
        AtomicHolder h(this);
        if (empty == false)
        {
            result = static_cast<BufferBase *>(queue.next(0));
            if (result)
            {
                new (result) BufferBase(size, this);
                totalSize += itemSize;
            }
            else
            {
                empty = true;
            }
        }
        if (flow && empty)
        {
            queue.insert(flow);
        }
    }
    if (result && flow)
    {
        flow->alloc_result(result);
    }
    return result;
}

/** Release an item back to the free pool.
 * @param item pointer to item to release
 * @param size size of buffer to free
 */
void FixedPool::free(BufferBase *item)
{
    HASSERT(valid(item));
    HASSERT(item->size() <= itemSize);
    
    Executable *waiting = NULL;
    {
        AtomicHolder h(this);
        if (empty == true)
        {
            waiting = static_cast<Executable *>(queue.next().item);
            if (queue.empty())
            {
                empty = false;
            }
            if (!waiting)
            {
                queue.insert(item);
                totalSize -= itemSize;
            }
        }
        else
        {
            queue.insert(item);
            totalSize -= itemSize;
        }
    }
    if (waiting)
    {
        waiting->alloc_result(item);
    }

}
