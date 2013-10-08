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

#include <unistd.h>

#include "nmranet/NMRAnetMessageID.hxx"
#include "nmranet/NMRAnetDatagram.hxx"
#include "nmranet/NMRAnetNode.hxx"

namespace NMRAnet
{

/** Timeout for datagram acknowledgement. */
#define DATAGRAM_TIMEOUT 3000000000LL

BufferPool Datagram::pool(sizeof(Datagram::Message), Datagram::POOL_SIZE);
BufferQueueWait Datagram::dq;
BufferQueueWait Datagram::pending;
OSThreadOnce Datagram::once(Datagram::one_time_init);

/** Thread for handling platform datagrams.
 * @param arg unused
 * @return should never return
 */
void *Datagram::thread(void *arg)
{
    for ( ; /* forever */ ; )
    {
        Buffer *buffer = pending.wait();
        
        switch (protocol(buffer))
        {
            default:
                /* unhandled protocol */
                break;
            case CONFIGURATION:
                //datagram_memory_config(d->to, datagram);
                break;
        }
        buffer->free();
    }

    return NULL;
}

/** One time initialization */
void Datagram::one_time_init()
{
    for (size_t i = 0; i < POOL_SIZE; ++i)
    {
        Buffer *buffer = pool.buffer_alloc(sizeof(Datagram::Message));
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
        datagram->txMessage->free();
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
    Node::mutex.unlock();

    write(If::MTI_DATAGRAM, dst, buffer);
    timer.start(DATAGRAM_TIMEOUT);

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
            return 0;
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
void Datagram::packet(If::MTI mti, NodeHandle src, Buffer *data)
{
    switch (mti)
    {
        default:
            HASSERT(0);
            break;
        case If::MTI_DATAGRAM_REJECTED:
        {
            Message *m = (Message*)data->start();
            uint16_t error = m->data[0] + (m->data[1] << 8);
            error = htobe16(error);
            if (resend_ok(error))
            {
                /* we can try again */
                timer.stop();

                if (txMessage)
                {
                    write(If::MTI_DATAGRAM, src, txMessage);
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
            break;
        }
        case If::MTI_DATAGRAM_OK:
            /* success! */
            timer.stop();
            if (txMessage)
            {
                buffer_release(txMessage);
                txMessage = NULL;
            }
            break;
        case If::MTI_DATAGRAM:
            write(If::MTI_DATAGRAM_OK, src, NULL);
            if (process(data) == 0)
            {
                /* push this up for the application to process */
                data->id(ID_DATAGRAM_DELIVER);
                rx_queue()->insert(data);
            }
            break;
    }
}

};

