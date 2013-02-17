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
 * \file nmranet_buf.h
 * This file Implements a buffer pool for networking interfaces to use.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#ifndef _nmranet_buf_h_
#define _nmranet_buf_h_

#include <stdlib.h>

/** NMRAnet queue handle type.
 */
typedef void *nmranet_queue_t;

/** Get a free buffer out of the pool.
 * @param size minimum size in bytes the buffer must hold
 * @return pointer to the newly allocated buffer
 */
void *nmranet_buffer_alloc(size_t size);
/** Release a buffer back to the free buffer pool.
 * @param buffer pointer to buffer to release
 */
void nmranet_buffer_free(const void *buffer);

/** Advance the position of the buffer.
 * @param buffer pointer to buffer 
 * @param bytes number of bytes to advance.
 * @return pointer to the new position (next available byte)
 */
void *nmranet_buffer_advance(void *buffer, size_t bytes);

/** Get a pointer to the current position of the buffer.
 * @param buffer pointer to buffer 
 * @return pointer to the current position (next available byte)
 */
void *nmranet_buffer_position(const void *buffer);

/** Get the size of the buffer in bytes.
 * @param buffer pointer to buffer 
 * @return size of the buffer in bytes
 */
size_t nmranet_buffer_size(const void *buffer);

/** Get the number of unused bytes in the buffer.
 * @param buffer pointer to buffer 
 * @return number of unused bytes
 */
size_t nmranet_buffer_available(const void *buffer);

/** Expand the buffer size.
 * @param buffer pointer to buffer to expand
 * @param size size buffer after expansion.
 * @return newly expanded buffer with old buffer data moved
 */
void *nmranet_buffer_expand(void *buffer, size_t size);

/** Create and empty buffer queue.
 * @return handle to the created queue
 */
nmranet_queue_t nmranet_queue_create(void);

/** Add a buffer to the back of the queue.
 * @param queue queue to add buffer to
 * @param buffer buffer to add to queue
 */
void nmranet_queue_insert(nmranet_queue_t queue, const void *buffer);

/** Get a buffer from the front of the queue.
 * @param queue queue get buffer from
 * @return buffer buffer retreived from queue
 */
void *nmranet_queue_next(nmranet_queue_t queue);

/** Test if the queue is empty.
 * @param queue queue to test
 * @return true if empty, else false
 */
int nmranet_queue_empty(nmranet_queue_t queue);

/** Get the number of pending items in the queue.
 * @param queue queue to test
 * @return number of pending items in the queue
 */
size_t nmranet_queue_pending(nmranet_queue_t queue);

#endif /* _nmranet_buf_h_ */

