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

#include <cstdio>

#include "utils/BufferQueue.hxx"

#if defined (__linux__)
#define DEBUG_PRINTF printf
#else
#define DEBUG_PRINTF(_fmt...)
#endif

DynamicPool *mainBufferPool = new DynamicPool(Bucket::init(4, 8, 16, 32, 0));



/** Add an item to the back of the queue.
 * @param item to add to queue
 * @param index unused parameter
 */
void Q::insert(QMember *item, unsigned index)
{
    HASSERT(item->next == NULL);
    HASSERT(item != tail);
    AtomicHolder h(this);
    if (head == NULL)
    {
        head = tail = item;
    }
    else
    {
        tail->next = item;
        tail = item;
    }
    item->next = NULL;
    ++count;
}

/** Get an item from the front of the queue.
 * @return @ref Result structure with item retrieved from queue, NULL if
 *         no item available
 */
Q::Result Q::next()
{
    AtomicHolder h(this);
    if (head == NULL)
    {
        return Result();
    }
    --count;
    QMember *qm = head;
    if (head == tail)
    {
        tail = NULL;
    }
    head = (qm->next);
    qm->next = NULL;

    return Result(qm, 0);
}

/** Add an item to the back of the queue.
 * @param item to add to queue
 * @param index unused parameter
 */
void QAsync::insert(QMember *item, unsigned index)
{
    Executable *executable = NULL;
    {
        AtomicHolder h(this);
        if (waiting)
        {
            if (Q::pending() == 0)
            {
                Q::insert(item);
                waiting = false;
            }
            else
            {
                executable = static_cast<Executable *>(Q::next().item);
            }
        }
        else
        {
            Q::insert(item);
        }
    }
    if (executable)
    {
        executable->alloc_result(item);
    }
}

/** Get an item from the front of the queue.
 * @param flow Executable that will wait on the item
 * @return item retrieved from queue, NULL if no item available
 */
void QAsync::next_async(Executable *flow)
{
    QMember *qm = NULL;
    {
        AtomicHolder h(this);
        if (waiting)
        {
            Q::insert(flow);
        }
        else
        {
            qm = Q::next(0);
            if (qm == NULL)
            {
                Q::insert(flow);
                waiting = true;
            }
        }
    }
    if (qm)
    {
        flow->alloc_result(qm);
    }
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
                result = (BufferBase*)malloc(current->size());
                {
                    AtomicHolder h(this);
                    totalSize += current->size();
                }
            }
            new (result) BufferBase(size, this);
        }
    }

    if (!result)
    {
        /* big items are just malloc'd freely */
        result = (BufferBase*)malloc(size);
        new (result) BufferBase(size, this);
        {
            AtomicHolder h(this);
            totalSize += size;
        }
    }
    if (flow)
    {
        flow->alloc_result(result);
    }
    return result;
}

/** Release an item back to the free pool.
 * @param item pointer to item to release
 * @param size size of buffer to free
 */
void DynamicPool::free(BufferBase *item)
{
    for (Bucket *current = buckets; current->size() != 0; ++current)
    {
        if (item->size() <= current->size())
        {
            current->insert(item);
            {
                AtomicHolder h(this);
                totalSize -= current->size();
            }
            return;
        }
    }
    /* big items are just freed */
    {
        AtomicHolder h(this);
        totalSize -= item->size();
    }
    ::free(item);
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
            if (queue.pending() == 0)
            {
                empty = false;
            }
            if (!waiting)
            {
                queue.insert(item);
            }
        }
        else
        {
            queue.insert(item);
        }
    }
    if (waiting)
    {
        waiting->alloc_result(item);
    }

}

