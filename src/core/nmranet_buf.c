/** \copyright
 * Copyright (c) 2012, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file nmranet_buf.c
 * This file Implements a buffer pool for networking interfaces to use.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#include "os/os.h"
#include "core/nmranet_buf.h"

#include <stdio.h>

#if defined (__linux__)
#define DEBUG_PRINTF printf
#else
#define DEBUG_PRINTF(_fmt...)
#endif

/** Mutual exclusion for queues. */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** Buffer structure.
 */
typedef struct buffer
{
    struct buffer *next; /**< next buffer in list */
    size_t size; /**< size of data in bytes */
    size_t free; /**< amount for free space left in the buffer */
    char   data[]; /**< data */
} Buffer;

/** Que structure
 */
typedef struct queue
{
    Buffer *head; /**< head buffer in queue */
    Buffer *tail; /**< tail buffer in queue */
    size_t count; /**< number of buffers in the queue */
} Queue;

static size_t totalSize = 0;

/** Array of power of 2 buffer sizes
 */
static Buffer *pool[4] = {NULL, NULL, NULL, NULL};

/** Obtain a pointer to the buffer including metadata from its data pointer.
 * @param _buffer pointer to the data element, (nmranet_buf_t) of a buffer
 */
#define BUFFER(_buffer) (Buffer*)((char *)(_buffer) - sizeof(Buffer));

/** Get a free buffer out of the pool.
 * @param size minimum size in bytes the buffer must hold
 * @return pointer to the newly allocated buffer
 */
void *nmranet_buffer_alloc(size_t size)
{
    int index = 0;
    Buffer *buf;

    if (size <= 4)
    {
        index = 0;
        size = 4;
    }
    else if (size <= 8)
    {
        index = 1;
        size = 8;
    }
    else if (size <= 16)
    {
        index = 2;
        size = 16;
    }
    else if (size <= 32)
    {
        index = 3;
        size = 32;
    }
    else
    {
        /* big buffers are just malloc'd freely */
        buf = malloc(size + sizeof(Buffer));
        if (buf == NULL)
        {
            return NULL;
        }
        buf->next = NULL;
        buf->size = size;
        buf->free = size;
        totalSize += size + sizeof(Buffer);
        DEBUG_PRINTF("c buffer total size: %zu\n", totalSize);
        return buf->data;
    }

    os_mutex_lock(&mutex);
    if (pool[index] != NULL)
    {
        buf = pool[index];
        pool[index] = buf->next;
    }
    else
    {
        buf = malloc(size + sizeof(Buffer));
        if (buf == NULL)
        {
            return NULL;
        }
        totalSize += size + sizeof(Buffer);
        DEBUG_PRINTF("c buffer total size: %zu\n", totalSize);
    }
    os_mutex_unlock(&mutex);

    buf->next = NULL;
    buf->size = size;
    buf->free = size;
    
    return buf->data;
}

/** Release a buffer back to the free buffer pool.
 * @param buffer pointer to buffer to release
 */
void nmranet_buffer_free(const void *buffer)
{
    Buffer *buf = BUFFER(buffer);
    int index = 0;

    switch (buf->size)
    {
        default:
            /* big buffers are just freed */
            totalSize -= buf->size;
            totalSize -= sizeof(Buffer);
            free(buf);
            return;
        case 4:
            index = 0;
            break;
        case 8:
            index = 1;
            break;
        case 16:
            index = 2;
            break;
        case 32:
            index = 3;
            break;
    }

    os_mutex_lock(&mutex);
    buf->next = pool[index];
    pool[index] = buf;
    os_mutex_unlock(&mutex);
}

/** Advance the position of the buffer.
 * @param buffer pointer to buffer 
 * @param bytes number of bytes to advance.
 * @return pointer to the new position (next available byte)
 */
void *nmranet_buffer_advance(void *buffer, size_t bytes)
{
    Buffer *buf = BUFFER(buffer);
    buf->free -= bytes;    
    return &buf->data[buf->size - buf->free];
}

/** Get a pointer to the current position of the buffer.
 * @param buffer pointer to buffer 
 * @return pointer to the current position (next available byte)
 */
void *nmranet_buffer_position(const void *buffer)
{
    Buffer *buf = BUFFER(buffer);
    return &buf->data[buf->size - buf->free];
}

/** Get the size of the buffer in bytes.
 * @param buffer pointer to buffer 
 * @return size of the buffer in bytes
 */
size_t nmranet_buffer_size(const void *buffer)
{
    Buffer *buf = BUFFER(buffer);
    return buf->size;
}

/** Get the number of unused bytes in the buffer.
 * @param buffer pointer to buffer 
 * @return number of unused bytes
 */
size_t nmranet_buffer_available(const void *buffer)
{
    Buffer *buf = BUFFER(buffer);
    return buf->free;    
}

/** Expand the buffer size.
 * @param buffer pointer to buffer to expand
 * @param size size buffer after expansion.
 * @return newly expanded buffer with old buffer data moved
 */
void *nmranet_buffer_expand(void *buffer, size_t size)
{
    Buffer *buf     = BUFFER(buffer);
    Buffer *new_buf = BUFFER(nmranet_buffer_alloc(size));
    
    memcpy(new_buf->data, buf->data, buf->size - buf->free);
    new_buf->free = size + buf->free;
    nmranet_buffer_free(buf->data);
    return new_buf->data;
}

/** Create and empty buffer queue.
 * @return handle to the created queue
 */
nmranet_queue_t nmranet_queue_create(void)
{
    Queue *q = malloc(sizeof(Queue));

    q->head = NULL;
    q->tail = NULL;
    q->count = 0;

    return q;
}

/** Add a buffer to the back of the queue.
 * @param queue queue to add buffer to
 * @param buffer buffer to add to queue
 */
void nmranet_queue_insert(nmranet_queue_t queue, const void *buffer)
{
    Queue *q = (Queue*)queue;
    Buffer *buf = BUFFER(buffer);

    os_mutex_lock(&mutex);
    if (q->head == NULL)
    {
        q->head = q->tail = buf;
    }
    else
    {
        q->tail->next = buf;
        q->tail = buf;
    }
    buf->next = NULL;
    q->count++;
    os_mutex_unlock(&mutex);
}

/** Get a buffer from the front of the queue.
 * @param queue queue get buffer from
 * @return buffer buffer retrieved from queue
 */
void *nmranet_queue_next(nmranet_queue_t queue)
{
    Queue *q = (Queue*)queue;
    Buffer *buf;

    os_mutex_lock(&mutex);
    if (q->head == NULL)
    {
        os_mutex_unlock(&mutex);
        return NULL;
    }
    buf = q->head;
    q->head = buf->next;
    q->count--;
    os_mutex_unlock(&mutex);

    return buf->data;
}

/** Get the number of pending items in the queue.
 * @param queue queue to test
 * @return number of pending items in the queue
 */
size_t nmranet_queue_pending(nmranet_queue_t queue)
{
    Queue *q = (Queue*)queue;
    
    os_mutex_lock(&mutex);
    size_t result = q->count;
    os_mutex_unlock(&mutex);

    return result;
}

/** Test if the queue is empty.
 * @param queue queue to test
 * @return true if empty, else false
 */
int nmranet_queue_empty(nmranet_queue_t queue)
{
    Queue *q = (Queue*)queue;
    
    os_mutex_lock(&mutex);
    int result = (q->head == NULL);
    os_mutex_unlock(&mutex);

    return result;
}
