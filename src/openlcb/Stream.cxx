/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file Stream.cxx
 * This file defines NMRAnet streams.
 *
 * @author Stuart W. Baker
 * @date 20 October 2013
 */

#include "openlcb/Stream.hxx"
#include "openlcb/NMRAnetNode.hxx"
#include "openlcb/NMRAnetMessageID.hxx"
#include "utils/macros.h"

namespace openlcb
{

/** Open a stream.
 * @param dst destination node for stream
 * @param timeout 0 = do not wait for success, deliver success asynchronously
 *                > 0, wait for success, or fail on timeout
 * @return NULL on fail, else stream handle on success or pending success
 */
Stream::StreamHandle Stream::sopen(NodeHandle dst, long long timeout)
{
    StreamHandle handle = NULL;

    Node::mutex.lock();
    if (count < CHANNELS_PER_NODE)
    {
        Metadata* metadata = new Metadata;
        metadata->data = RingBuffer<uint8_t>::create(MAX_BUFFER_SIZE * 2);
        metadata->size = MAX_BUFFER_SIZE;
        metadata->count = 0;
        metadata->srcID = 0;
        metadata->nodeHandle = dst;
        RBTree<uint8_t, Metadata*>::Node *last = outboundTree.last();
        
        if (last != NULL)
        {
            metadata->srcID = last->key + 1;
            while (outboundTree.find(metadata->srcID) != NULL)
            {
                metadata->srcID++;
            }
        }
        
        Buffer *buffer = buffer_alloc(6);
        uint8_t *data = (uint8_t*)buffer->start();
        
        data[0] = (MAX_BUFFER_SIZE >> 8) & 0xFF;
        data[1] = (MAX_BUFFER_SIZE >> 0) & 0xFF;
        data[2] = 0;
        data[3] = 0;
        data[4] = metadata->srcID;
        data[5] = 0;

        buffer->advance(6);
        
        RBTree<uint8_t, Metadata*>::Node *node =
            new RBTree<uint8_t, Metadata*>::Node(metadata->srcID, metadata);

        outboundTree.insert(node);
        
        write(Defs::MTI_STREAM_INITIATE_REQUEST, dst, buffer);
        
        if (timeout)
        {
            metadata->state = WAITING_O;

            Node::mutex.unlock();
            int result = sem.timedwait(timeout);
            Node::mutex.lock();

            if (result != 0 || metadata->state != CONNECTED_O)
            {
                outboundTree.remove(node);
                metadata->data->destroy();
                delete node;
                delete metadata;
            }
            else
            {
                count++;
                handle = (StreamHandle)metadata;
            }
            /** @todo (Stuart Baker) is there a race between connected and timeout */
        }
        else
        {
            metadata->state = PENDING_O;
            count++;
            handle = (StreamHandle)metadata;
        }
    }    
    Node::mutex.unlock();

    return handle;
}

/** Close a stream.
 * @param handle to stream to close
 */
void Stream::sclose(StreamHandle handle)
{
    Metadata *metadata = (Metadata*)handle;
    
    Node::mutex.lock();
    if (metadata->data->items() == 0)
    {
        /* no more data left to send.  Hooray! */
        Buffer *buffer = buffer_alloc(4);
        uint8_t *data = (uint8_t*)buffer->start();
        data[0] = metadata->srcID;
        data[1] = metadata->dstID;
        data[2] = 0;
        data[3] = 0;
        buffer->advance(4);
        
        write(Defs::MTI_STREAM_COMPLETE, metadata->nodeHandle, buffer);
    
        /* no more data left in the buffer */
        RBTree<uint8_t, Metadata*>::Node *node = outboundTree.remove(metadata->srcID);
        metadata->data->destroy();
        delete metadata;
        delete node;
    }
    else
    {
        /* we have some data left to send, but mark us as closed */
        metadata->state = CLOSED_O;
    }
    Node::mutex.unlock();
}

/** Write data to a stream.
 * @param handle handle to stream to write to
 * @param buf buffer to copy data from
 * @param size size in bytes to write to stream
 * @return number of bytes written, else -1 on error with errno set to indicate error
 */
ssize_t Stream::swrite(StreamHandle handle, const void *buf, size_t size)
{
    Metadata *metadata = (Metadata*)handle;
    
    ssize_t result = 0;
    
    Node::mutex.lock();
    if (metadata->data->items() || metadata->count >= metadata->size)
    {
        /* there is already some data in flight, so lets just add to the back
         * of the buffer.
         */
        result = metadata->data->put((uint8_t*)buf, size);
    }
    else
    {
        size_t buffer_size;
        if (size > (metadata->size - metadata->count))
        {
            buffer_size = metadata->size - metadata->count;
        }
        else
        {
            buffer_size = size;
        }

        Buffer *buffer = buffer_alloc(buffer_size + 2);
        uint8_t *data = (uint8_t*)buffer->start();
        memcpy(data, buf, buffer_size);
        data[buffer_size + 0] = metadata->srcID;
        data[buffer_size + 1] = metadata->dstID;
        buffer->advance(buffer_size + 2);
        write(Defs::MTI_STREAM_DATA, metadata->nodeHandle, buffer);
        
        result = buffer_size;
        size -= buffer_size;
        metadata->count += buffer_size;

        if (size)
        {
            result += metadata->data->put((uint8_t*)buf + buffer_size, size);
        }
    }
    Node::mutex.unlock();

    return result;
}

/** Read data from a stream.
 * @param handle handle to stream to read from
 * @param buf buffer to copy data to
 * @param size size in bytes to read from stream
 * @return number of bytes read, else -1 on error with errno set to indicate error
 */
ssize_t Stream::sread(StreamHandle handle, void *buf, size_t size)
{
    Metadata *metadata = (Metadata*)handle;
    
    ssize_t result = 0;
    
    Node::mutex.lock();
    size_t space = metadata->data->space();
    result = metadata->data->get((uint8_t*)buf, size);

    if (space < metadata->size && metadata->data->space() >= metadata->size)
    {
        /* proceed since we made room for the next data set */
        Buffer *buffer = buffer_alloc(4);
        uint8_t *data = (uint8_t*)buffer->start();
        data[0] = metadata->srcID;
        data[1] = metadata->dstID;
        data[2] = 0;
        data[3] = 0;
        buffer->advance(4);
        
        metadata->count = 0;
        
        write(Defs::MTI_STREAM_PROCEED, metadata->nodeHandle, buffer);
    }
    Node::mutex.unlock();

    return result;
}

/** Handle incoming stream initiate request messages.
 * @param src source Node ID
 * @param data datagram to post
 */
void Stream::initiate_request(NodeHandle src, Buffer *buffer)
{
    if (count < CHANNELS_PER_NODE && buffer->used() >= 6)
    {
        uint8_t *data = (uint8_t*)buffer->start();
        
        size_t buffer_size = (data[0] << 8) + data[1];
        
        if (MAX_BUFFER_SIZE < buffer_size)
        {
            buffer_size = MAX_BUFFER_SIZE;
        }

        Metadata* metadata = new Metadata;
        metadata->protocol = DISCOVER_1;
        metadata->data = RingBuffer<uint8_t>::create(MAX_BUFFER_SIZE * 2);
        metadata->nodeHandle = src;
        metadata->size = buffer_size;
        metadata->count = 0;
        metadata->state = CONNECTED_I;
        metadata->srcID = data[4];
        metadata->dstID = 0;

        RBTree<uint8_t, Metadata*>::Node *last = inboundTree.last();
        
        if (last != NULL)
        {
            metadata->dstID = last->key + 1;
            while (inboundTree.find(metadata->dstID) != NULL)
            {
                metadata->dstID++;
            }
        }
        
        /* reuse buffer to reply since we are otherwise done with it */
        buffer->zero();
        
        data[0] = (buffer_size >> 8) & 0xFF;
        data[1] = (buffer_size >> 0) & 0xFF;
        data[2] = ACCEPT;
        data[3] = 0;
        data[4] = metadata->srcID;
        data[5] = metadata->dstID;

        buffer->advance(6);
        
        write(Defs::MTI_STREAM_INITIATE_REPLY, src, buffer);
        
        RBTree<uint8_t, Metadata*>::Node *node =
            new RBTree<uint8_t, Metadata*>::Node(metadata->dstID, metadata);

        inboundTree.insert(node);
        
        buffer = buffer_alloc(sizeof(IdStreamType));
        IdStreamType* type = (IdStreamType*)buffer->start();
        type->stream = (StreamHandle)node->value;
        buffer->id(ID_STREAM_NEW_CONNECTION);
        buffer->advance(sizeof(IdStreamType));
        
        rx_queue()->insert(buffer);
    }
}

/** Handle incoming stream initiate reply messages.
 * @param src source Node ID
 * @param data datagram to post
 */
void Stream::initiate_reply(NodeHandle src, Buffer *buffer)
{
    uint8_t *data = (uint8_t*)buffer->start();
    
    size_t buffer_size = (data[0] << 8) + data[1];
    
    RBTree<uint8_t, Metadata*>::Node *node = outboundTree.find(data[4]);
    
    HASSERT(node);
    
    if (node->value->size > buffer_size)
    {
        node->value->size = buffer_size;
    }

    node->value->dstID = data[5];

    buffer = buffer_alloc(sizeof(IdStreamType));
    IdStreamType* type = (IdStreamType*)buffer->start();
    type->stream = (StreamHandle)node->value;
    buffer->id(ID_STREAM_COMPLETED_CONNECTION);
    buffer->advance(sizeof(IdStreamType));
    
    if (node->value->state == WAITING_O)
    {
        node->value->state = CONNECTED_O;
        sem.post();
    }
    else
    {
        node->value->state = CONNECTED_O;
        rx_queue()->insert(buffer);
    }
}

/** Handle incoming stream data messages.
 * @param src source Node ID
 * @param data datagram to post
 */
void Stream::handle_data(NodeHandle src, Buffer *buffer)
{
    bool wakeup;
    uint8_t *data = (uint8_t*)buffer->start();

    RBTree<uint8_t, Metadata*>::Node *node = inboundTree.find(data[buffer->used() - 1]);

    /* we only wakeup if starting from zero, otherwise, the node already knows
     * there is data available, so no need to re-notify it.
     */
    if (node->value->data->items() == 0)
    {
        wakeup = true;
    }
    else
    {
        wakeup = false;
    }
    
    /* last two bytes represent source and destination ID */
    size_t len = buffer->size() - buffer->available();

    /* some sanity tests that will help us debug this */
    HASSERT(node->value->state == CONNECTED_I);
    HASSERT(len > 2);
    HASSERT(node->value->data->space() >= (len - 2));

    while ((node->value->protocol & DISCOVER_MASK) != DISCOVERED && len > 2)
    {
        int shift = (node->value->protocol >> DISCOVER_SHIFT) - 1;
        node->value->protocol |= (*data) << (shift * 8);
        data++;
        len--;
        node->value->protocol-= DISCOVERED_DEC;
    }

    node->value->count += (len - 2);
    HASSERT(node->value->count <= node->value->size);

    node->value->data->put(data, len - 2);

    /* release buffer, we are done with it */
    buffer->free();

    /* this operation must come before proceed because proceed will release the
     * critical Node::mutex and cause a race condition.
     */
    if (wakeup && (node->value->protocol & DISCOVER_MASK) == DISCOVERED)
    {
        buffer = buffer_alloc(sizeof(IdStreamType));
        IdStreamType* type = (IdStreamType*)buffer->start();
        type->stream = (StreamHandle)node->value;
        buffer->id(ID_STREAM_DATA_POSTED);
        buffer->advance(sizeof(IdStreamType));

        rx_queue()->insert(buffer);
    }

    if (node->value->count == node->value->size &&
        node->value->data->space() >= MAX_BUFFER_SIZE)
    {
        /* proceed since we already have room for the next data set */
        buffer = buffer_alloc(4);
        data = (uint8_t*)buffer->start();
        data[0] = node->value->srcID;
        data[1] = node->value->dstID;
        data[2] = 0;
        data[3] = 0;
        buffer->advance(4);
        
        node->value->count = 0;
        
        write(Defs::MTI_STREAM_PROCEED, src, buffer);
    }
}

/** Handle incoming stream proceed messages.
 * @param src source Node ID
 * @param data datagram to post
 */
void Stream::proceed(NodeHandle src, Buffer *buffer)
{
    uint8_t *data = (uint8_t*)buffer->start();

    RBTree<uint8_t, Metadata*>::Node *node = outboundTree.find(data[1]);

    node->value->count = 0;
    
    buffer->free();

    if (node->value->data->items())
    {
        size_t segment_size;
        if (node->value->data->items() < node->value->size)
        {
            segment_size = node->value->data->items();
        }
        else
        {
            segment_size = node->value->size;
        }
        
        buffer = buffer_alloc(segment_size + 2);
        data = (uint8_t*)buffer->start();
        node->value->data->get(data, segment_size);
        data[segment_size + 0] = node->value->srcID;
        data[segment_size + 1] = node->value->dstID;
        buffer->advance(segment_size + 2);
        node->value->count = segment_size;

        write(Defs::MTI_STREAM_DATA, src, buffer);
    }
}

/** Handle incoming stream complete messages.
 * @param src source Node ID
 * @param data datagram to post
 */
void Stream::complete(NodeHandle src, Buffer *buffer)
{
    uint8_t *data = (uint8_t*)buffer->start();

    RBTree<uint8_t, Metadata*>::Node *node = inboundTree.find(data[2]);

    if (node->value->data->items() == 0)
    {
        /* no more data left in the buffer */
        outboundTree.remove(node);
        node->value->data->destroy();
        delete node->value;
        delete node;
    }
    else
    {
        /* we have some data left, give the application a chance to get it */
        node->value->state = CLOSED_I;
    }
}

/** Handle incoming stream messages.
 * @param mti Message Type Indicator
 * @param src source Node ID
 * @param data datagram to post
 */
void Stream::packet(Defs::MTI mti, NodeHandle src, Buffer *data)
{
    switch (mti)
    {
        default:
            HASSERT(0);
            break;
        case Defs::MTI_STREAM_INITIATE_REQUEST:
            initiate_request(src, data);
            break;
        case Defs::MTI_STREAM_INITIATE_REPLY:
            initiate_reply(src, data);
            break;
        case Defs::MTI_STREAM_DATA:
            handle_data(src, data);
            break;
        case Defs::MTI_STREAM_PROCEED:
            proceed(src, data);
            break;
        case Defs::MTI_STREAM_COMPLETE:
            complete(src, data);
            break;
    }
}

};

