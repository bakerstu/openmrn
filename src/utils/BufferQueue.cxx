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

#include "BufferQueue.hxx"

#if defined(__linux__)
#define DEBUG_PRINTF printf
#else
#define DEBUG_PRINTF(_fmt...)
#endif

BufferPool *mainBufferPool = new BufferPool();

/** Get a free buffer out of the pool.
 * @param size minimum size in bytes the buffer must hold
 * @return pointer to the newly allocated buffer, NULL if there is no free
 * buffer
 */
Buffer *BufferPool::buffer_alloc(size_t size) {
  Buffer *buffer = NULL;

  if (itemSize != 0) {
    HASSERT(size <= itemSize);
    mutex.lock();
    if (pool[1] != NULL) {
      buffer = pool[1];
      pool[1] = buffer->next;
      (void)Buffer::init(buffer, size);
      totalSize++;
      DEBUG_PRINTF("static buffer total size: %zu\n", totalSize);
    }
    mutex.unlock();
    return buffer;
  }

  int index = 0;

  if (size <= 4) {
    index = 0;
    size = 4;
  } else if (size <= 8) {
    index = 1;
    size = 8;
  } else if (size <= 16) {
    index = 2;
    size = 16;
  } else if (size <= 32) {
    index = 3;
    size = 32;
  } else {
    /* big buffers are just malloc'd freely */
    buffer = Buffer::alloc(this, size);
    mutex.lock();
    totalSize += size + sizeof(Buffer);
    mutex.unlock();
    DEBUG_PRINTF("cxx buffer total size: %zu\n", totalSize);
    return buffer;
  }

  mutex.lock();
  if (pool[index] != NULL) {
    buffer = pool[index];
    pool[index] = buffer->next;
    (void)Buffer::init(buffer, size);
  } else {
    buffer = Buffer::alloc(this, size);

    totalSize += size + sizeof(Buffer);
    DEBUG_PRINTF("cxx buffer total size: %zu\n", totalSize);
  }
  mutex.unlock();

  return buffer;
}

/** Release a buffer back to the free buffer pool.
 * @param buffer pointer to buffer to release
 */
void BufferPool::buffer_free(Buffer *buffer) {
  HASSERT(this == buffer->bufferPool);

  int index = 1;

  mutex.lock();
  if (--(buffer->count) == 0) {
    /* this buffer is no longer in use by anyone */
    if (itemSize == 0) {
      /* we don't have a fixed pool */
      switch (buffer->_size) {
        default:
          /* big buffers are just freed */
          totalSize -= buffer->_size;
          totalSize -= sizeof(Buffer);
          DEBUG_PRINTF("buffer total size: %zu\n", totalSize);
          free(buffer);
          mutex.unlock();
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
    } else {
      totalSize--;
      DEBUG_PRINTF("static buffer total used: %zu\n", totalSize);
    }

    buffer->next = pool[index];
    pool[index] = buffer;
  }

  mutex.unlock();
}

/** Expand the buffer size.  Exercise caution when using this API.  If anyone
 * else is holding onto a reference of this, their reference will be corrupted.
 * @param size size buffer after expansion.
 * @return newly expanded buffer with old buffer data moved
 */
Buffer *Buffer::expand(size_t size) {
  Buffer *new_buffer = buffer_alloc(size);

  memcpy(new_buffer->data, data, _size - left);
  new_buffer->left = _size + left;
  buffer_free(this);
  return new_buffer;
}

/** Add a buffer to the back of the queue.
 * @param buffer buffer to add to queue
 */
void BufferQueue::insert(Buffer *buffer) {
  mutex.lock();
  if (head == NULL) {
    head = tail = buffer;
  } else {
    tail->next = buffer;
    tail = buffer;
  }
  buffer->next = NULL;
  count++;
  mutex.unlock();
}

/** Get a buffer from the front of the queue.
 * @return buffer buffer retrieved from queue
 */
Buffer *BufferQueue::next() {
  Buffer *buf;

  mutex.lock();
  if (head == NULL) {
    mutex.unlock();
    return NULL;
  }
  buf = head;
  head = buf->next;
  count--;
  mutex.unlock();

  return buf;
}
