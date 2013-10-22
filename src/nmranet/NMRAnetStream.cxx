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
 * \file NMRAnetStream.cxx
 * This file defines NMRAnet streams.
 *
 * @author Stuart W. Baker
 * @date 20 October 2013
 */

#include "nmranet/NMRAnetStream.hxx"
#include "nmranet/NMRAnetNode.hxx"
#include "nmranet/NMRAnetMessageID.hxx"

namespace NMRAnet
{

/** Open a stream.
 * @param dst destination node for stream
 * @param timeout 0 = do not wait for success, deliver success asynchronously
 *                > 0, wait for success, or fail on timeout
 * @return NULL on fail, else stream handle on success or pending success
 */
Stream::StreamHandle Stream::Open(NodeHandle dst, long long timeout)
{
    StreamHandle handle = NULL;

    Node::mutex.lock();
    if (count < CHANNELS_PER_NODE)
    {
        Metadata* metadata = new Metadata;
        metadata->ping = buffer_alloc(MAX_BUFFER_SIZE);
        metadata->pong = buffer_alloc(MAX_BUFFER_SIZE);
        metadata->srcID = 0;
        RBTree<uint8_t, Metadata*>::Node *last = outboundTree.last();
        
        if (last != NULL)
        {
            metadata->srcID = last->key + 1;
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
        
        write(If::MTI_STREAM_INITIATE_REQUEST, dst, buffer);
        
        RBTree<uint8_t, Metadata*>::Node *node =
            new RBTree<uint8_t, Metadata*>::Node(metadata->srcID, metadata);

        outboundTree.insert(node);
        
        if (timeout)
        {
            metadata->state = WAITING_O;

            Node::mutex.unlock();
            int result = sem.timedwait(timeout);
            Node::mutex.lock();

            if (result != 0 || metadata->state != CONNECTED_O)
            {
                outboundTree.remove(node);
                metadata->ping->free();
                metadata->pong->free();
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
        metadata->ping = buffer_alloc(MAX_BUFFER_SIZE);
        metadata->pong = buffer_alloc(MAX_BUFFER_SIZE);
        metadata->state = CONNECTED_I;
        metadata->srcID = data[4];
        metadata->dstID = 0;

        RBTree<uint8_t, Metadata*>::Node *last = inboundTree.last();
        
        if (last != NULL)
        {
            metadata->dstID = last->key + 1;
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
        
        write(If::MTI_STREAM_INITIATE_REPLY, src, buffer);
        
        RBTree<uint8_t, Metadata*>::Node *node =
            new RBTree<uint8_t, Metadata*>::Node(metadata->dstID, metadata);

        inboundTree.insert(node);
        
        buffer = buffer_alloc(sizeof(StreamHandle));
        StreamHandle* handle = (StreamHandle*)buffer->start();
        handle[0] = (StreamHandle)metadata;
        buffer->id(ID_STREAM_NEW_CONNECTION);
        buffer->advance(sizeof(StreamHandle));
        
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

    buffer = buffer_alloc(sizeof(StreamHandle));
    StreamHandle* handle = (StreamHandle*)buffer->start();
    handle[0] = (StreamHandle)node->value;
    buffer->id(ID_STREAM_COMPLETED_CONNECTION);
    buffer->advance(sizeof(StreamHandle));
    
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

/** Handle incoming stream messages.
 * @param mti Message Type Indicator
 * @param src source Node ID
 * @param data datagram to post
 */
void Stream::packet(If::MTI mti, NodeHandle src, Buffer *data)
{
    switch (mti)
    {
        default:
            HASSERT(0);
            break;
        case If::MTI_STREAM_INITIATE_REQUEST:
            initiate_request(src, data);
            break;
        case If::MTI_STREAM_INITIATE_REPLY:
            initiate_reply(src, data);
            break;
        case If::MTI_STREAM_DATA:
            break;
        case If::MTI_STREAM_PROCEED:
            break;
        case If::MTI_STREAM_COMPLETE:
            break;
    }
}

};

