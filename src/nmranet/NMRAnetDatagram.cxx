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
 * \file NMRAnetDatagram.cxx
 * This file provides the NMRAnet datagram implementation.
 *
 * @author Stuart W. Baker
 * @date 5 October 2013
 */

#include "nmranet/NMRAnetDatagram.hxx"

#include <unistd.h>

#include "nmranet/NMRAnetMessageID.hxx"
#include "nmranet/NMRAnetNode.hxx"

namespace NMRAnet
{

/** Timeout for datagram acknowledgement. */
#define DATAGRAM_TIMEOUT 3000000000LL

DatagramService *Datagram::service_ = NULL;

FixedPool<Buffer> Datagram::pool(sizeof(Datagram::Message), Datagram::POOL_SIZE);
BufferQueueWait Datagram::dq;
BufferQueueWait Datagram::pending;
OSThreadOnce Datagram::once(Datagram::one_time_init);

/** Constructor. */
Datagram::Datagram(Node *node)
    : txMessage(NULL),
      timer(timeout, this, NULL)
{
    node->protocols_ |= Node::Protocols::DATAGRAM;
    once.once();
}

/** Thread for handling platform datagrams.
 * @param arg unused
 * @return should never return
 */
void *Datagram::thread(void *arg)
{
    for ( ; /* forever */ ; )
    {
        Buffer *buffer = pending.wait();
        
        Datagram::Message *m = (Datagram::Message*)buffer->start();
        Node *node = Node::find(m->to);

        switch (protocol(buffer))
        {
            default:
                /* unhandled protocol */
                node->rejected(buffer, NOT_ACCEPTED);
                break;
            case CONFIGURATION:
                node->MemoryConfig::process(buffer);
                break;
        }
    }

    return NULL;
}

/** One time initialization.
 */
void Datagram::one_time_init()
{
    for (size_t i = 0; i < POOL_SIZE; ++i)
    {
        Buffer *buffer = pool.alloc();
        dq.insert(buffer);
    }
    
    new OSThread("Datagram", 0, THREAD_STACK_SIZE, thread, NULL);
}

/** Allocate and prepare a datagram buffer.
 * @param protocol datagram protocol to use
 * @param size max length of data in bytes
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
 * @return buffer handle upon success, else NULL on error with errno set
 */
Buffer *Datagram::buffer_get(uint64_t protocol, size_t size, long long timeout)
{
    HASSERT(size <= (MAX_SIZE - protocol_size(protocol)) && timeout >= 0);

    Buffer *buffer = dq.timedwait(timeout);
    if (buffer == NULL)
    {
        /* we must have timed out */
        return NULL;
    }

    Message *message = (Message*)buffer->start();
        
    message->size = size + protocol_size(protocol);
    message->from.id = id();


    /* copy over the protocol */
    switch (protocol_size(protocol))
    {
        default:
            /* fall through */
        case 1:
            message->data[0] = protocol;
            break;
        case 2:
            message->data[0] = ((protocol >> 0) & 0xFF);
            message->data[1] = ((protocol >> 8) & 0xFF);
            break;
        case 6:
            message->data[0] = ((protocol >>  0) & 0xFF);
            message->data[1] = ((protocol >>  8) & 0xFF);
            message->data[2] = ((protocol >> 16) & 0xFF);
            message->data[3] = ((protocol >> 24) & 0xFF);
            message->data[4] = ((protocol >> 32) & 0xFF);
            message->data[5] = ((protocol >> 40) & 0xFF);
            break;
    }

    return buffer;
}

/** Fill an already allocated datagram buffer.
 * @param buffer handle to buffer which is to hold datagram
 * @param protocol datagram protocol to use
 * @param data datagram to fill into datagram
 * @param offset offset within datagram payload to start filling
 * @param size length of data in bytes to fill
 */
void Datagram::buffer_fill(Buffer *buffer, uint64_t protocol, const void *data,
                           size_t offset, size_t size)
{
    Message *m = (Message*)buffer->start();
    
    /* copy over the data payload */
    switch (protocol_size(protocol))
    {
        default:
            /* fall through */
        case 1:
            offset += 1;
            break;
        case 2:
            offset += 2;
            break;
        case 6:
            offset += 6;
            break;
    }
    HASSERT((offset + size) <= MAX_SIZE);

    memcpy(m->data + offset, data, size);
}

/** Release a buffer back to the datagram buffer pool.
 * @param buffer buffer to release
 */
void Datagram::buffer_release(Buffer *buffer)
{
    HASSERT(buffer);

    /* put the buffer back in the free pool queue */
    dq.insert(buffer);
}

/** Timeout handler for datagram timeouts.
 * @param data1 instance of @ref Datagram cast to a void*
 * @param data2 unused
 * @return restart credentials for timer
 */
long long Datagram::timeout(void *data1, void *data2)
{
    Datagram *datagram = (Datagram*)data1;

    /** @todo (Stuart Baker) currently we timeout quietly, should we do
     * something more? */
    Node::mutex.lock();
    if (datagram->txMessage)
    {
        datagram->buffer_release(datagram->txMessage);
        datagram->txMessage = NULL;
    }
    Node::mutex.unlock();
    
    return OS_TIMER_NONE;
}

/** Produce a Datagram from a given node using a pre-allocated buffer.
 * @param dst destination node id or alias
 * @param datagram datagram to produce
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
 * @return 0 upon success, else -1 on error with errno set
 */
int Datagram::produce(NodeHandle dst, Buffer *buffer, long long timeout)
{
    /* make sure we are passed a buffer out of the datagram pool */
    HASSERT(pool.valid(buffer));

    /* timestamp the entry to this function */
    long long start = os_get_time_monotonic();

    Node::mutex.lock();
    while (txMessage)
    {
        if ((start + timeout) <= os_get_time_monotonic() && timeout != OS_WAIT_FOREVER)
        {
            Node::mutex.unlock();
            errno = EBUSY;
            return -1;
        }
        /** @todo (Stuart Baker) Use of a semaphore here would be more time
         *  efficient,however it would also be less memory efficient.  Maybe
         *  we should have a configurable option.
         */
        Node::mutex.unlock();
        usleep(MSEC_TO_USEC(200));
        Node::mutex.lock();
    }
    txMessage = buffer;

    write(Defs::MTI_DATAGRAM, dst, buffer);
    timer.start(DATAGRAM_TIMEOUT);
    Node::mutex.unlock();

    return 0;
}

/** Produce a Datagram from a given node.
 * @param dst destination node id or alias
 * @param protocol datagram protocol to use
 * @param data datagram to produce
 * @param size length of data in bytes
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
 * @return 0 upon success, else -1 on error with errno set
 */
int Datagram::produce(NodeHandle dst, uint64_t protocol, const void *data,
                      size_t size, long long timeout)
{
    HASSERT(!(dst.id == 0 && dst.alias == 0));

    Buffer *buffer = buffer_get(protocol, size, timeout);
    
    if (buffer == NULL)
    {
        /* could not allocate buffer, ERRNO already set appropriately */
        return -1;
    }
    
    buffer_fill(buffer, protocol, data, 0, size);

    if (produce(dst, buffer, timeout) != 0)
    {
        /* ERRNO already set appropriately */
        buffer_release(buffer);
        return -1;
    }

    return 0;
}

/** Determine the datagram protocol (could be 1, 2, or 6 bytes).
 * @param m @ref Message pointer to the beginning of the datagram
 * @return protocol type
 */
uint64_t Datagram::protocol(Buffer *data)
{
    Message *m = (Message*)data->start();
    switch (m->data[0] & 0xF0)
    {
        default:
            return m->data[0];
        case PROTOCOL_SIZE_2:
            return ((uint64_t)m->data[1] << 8) + ((uint64_t)m->data[2] << 0);
        case PROTOCOL_SIZE_6:
            return ((uint64_t)m->data[1] << 40) + ((uint64_t)m->data[2] << 32) +
                   ((uint64_t)m->data[3] << 24) + ((uint64_t)m->data[4] << 16) +
                   ((uint64_t)m->data[5] <<  8) + ((uint64_t)m->data[6] <<  0);
    }
}

/** Determine the datagram payload
 * @param data buffer pointer to the beginning of the datagram
 * @return pointer to payload as a uint8_t array, assume byte alignment
 */
uint8_t *Datagram::payload(Buffer *data)
{
    Message *m = (Message*)data->start();

    switch (m->data[0] & 0xF0)
    {
        default:
            return m->data + 1;
        case PROTOCOL_SIZE_2:
            return m->data + 3;
        case PROTOCOL_SIZE_6:
            return m->data + 7;
    }
}

/** Acknowledge the successful receipt of a Datagram.
 * @param buffer reference to the buffer containing the received datagram
 * @param flags @ref Flags associated with the reply
 * @param free buffer automatically freed upon return if true, else buffer preserved
 */
void Datagram::received_okay(Buffer *buffer, uint8_t flags, bool free)
{
    Message *m = (Message*)buffer->start();
    NodeHandle dst = m->from;
    Buffer *b = NULL;
    
    /* setup any flags */
    if (flags)
    {
        /* we will try and reuse existing buffer if we can */
        b = free ? buffer : buffer_alloc(sizeof(uint8_t));
        b->zero();
        uint8_t *data = (uint8_t*)b->start();
        data[0] = flags;
        b->advance(1);
    }
    
    /* transmit message */
    write_unlocked(Defs::MTI_DATAGRAM_OK, dst, b);
    
    /* free the buffer if we are done with it */
    if (free && buffer != b)
    {
        buffer->free();
    }
}

/** Reject the unsuccessful receipt of a Datagram.
 * @param buffer reference to the buffer containing the received datagram
 * @param flags @ref Flags associated with the reply
 * @param free buffer automatically freed upon return if true, else buffer preserved
 */
void Datagram::rejected(Buffer *buffer, uint16_t error, bool free)
{
    HASSERT(error != 0);

    Message *m = (Message*)buffer->start();
    NodeHandle dst = m->from;
    
    /* setup any error */
    /* we will try and reuse existing buffer if we can */
    Buffer *b = free ? buffer : buffer_alloc(sizeof(uint16_t));
    b->zero();
    uint8_t *data = (uint8_t*)b->start();
    error = htobe16(error);
    data[0] = (error >> 0) & 0xFF;
    data[1] = (error >> 8) & 0xFF;
    b->advance(2);
    
    /* transmit message */
    write_unlocked(Defs::MTI_DATAGRAM_REJECTED, dst, b);
}

/** Process the datagram automatically.
 * @param data buffer to process
 * @return 0 if we do not have an automated processing option, else return 1
 */
int Datagram::process(Buffer *data)
{
    switch (protocol(data))
    {
        default:
            /* unhandled protocol */
            //return 0;
        case CONFIGURATION:
            pending.insert(data);
            break;
    }
    return 1;
}

/** Post the reception of a datagram to given node.
 * @param mti Message Type Indicator
 * @param src source Node ID
 * @param data datagram to post
 */
void Datagram::packet(Defs::MTI mti, NodeHandle src, Buffer *data)
{
    switch (mti)
    {
        default:
            HASSERT(0);
            break;
        case Defs::MTI_DATAGRAM_REJECTED:
        {
            Message *m = (Message*)data->start();
            uint16_t error = m->data[1] + (m->data[0] << 8);

            if (resend_ok(error))
            {
                /* we can try again */
                timer.stop();

                if (txMessage)
                {
                    write(Defs::MTI_DATAGRAM, src, txMessage);
                    timer.start(DATAGRAM_TIMEOUT);
                }
            }
            else
            {
#ifndef __FreeRTOS__
                printf("datagram rejected\n");
#endif
                timer.stop();
                if (txMessage)
                {
                    buffer_release(txMessage);
                    txMessage = NULL;
                }
            }
            /* release buffer back to the pool from whence it came */
            data->free();
            break;
        }
        case Defs::MTI_DATAGRAM_OK:
            /* success! */
            HASSERT(data == NULL);
            timer.stop();
            if (txMessage)
            {
                buffer_release(txMessage);
                txMessage = NULL;
            }
            if (data)
            {
                data->free();
            }
            break;
        case Defs::MTI_DATAGRAM:
            if (process(data) == 0)
            {
                /* push this up for the application to process */
                data->id(ID_DATAGRAM_DELIVER);
                rx_queue()->insert(data);
            }
            break;
    }
}

StateFlowBase::Action DatagramService::Incoming::entry(Message *msg)
{
#if 0
    //If *nmranet_if = static_cast<If*>(me());
    If::InMessage *in_message = static_cast<If::InMessage*>(msg->start());
    NodeID dst = in_message->dst;
    NodeHandle src = in_message->src;
    Buffer *data = in_message->data;
    Defs::MTI mti = in_message->mti;

    switch (mti)
    {
        default:
            HASSERT(0);
            break;
#if 0
        case Defs::MTI_DATAGRAM_REJECTED:
        {
            Datagram::Message *m = (Datagram::Message*)data->start();
            uint16_t error = m->data[1] + (m->data[0] << 8);

            if (Datagram::resend_ok(error))
            {
                /* we can try again */
                timer.stop();

                if (txMessage)
                {
                    write(Defs::MTI_DATAGRAM, src, txMessage);
                    timer.start(DATAGRAM_TIMEOUT);
                }
            }
            else
            {
#ifndef __FreeRTOS__
                printf("datagram rejected\n");
#endif
                timer.stop();
                if (txMessage)
                {
                    buffer_release(txMessage);
                    txMessage = NULL;
                }
            }
            /* release buffer back to the pool from whence it came */
            data->free();
            break;
        }
        case Defs::MTI_DATAGRAM_OK:
            /* success! */
            HASSERT(data == NULL);
            timer.stop();
            if (txMessage)
            {
                buffer_release(txMessage);
                txMessage = NULL;
            }
            if (data)
            {
                data->free();
            }
            break;
        case Defs::MTI_DATAGRAM:
            if (process(data) == 0)
            {
                /* push this up for the application to process */
                data->id(ID_DATAGRAM_DELIVER);
                rx_queue()->insert(data);
            }
            break;
#endif
    }
#endif
    return release_and_exit(msg);
}

} /* namespace NMRAnet */

