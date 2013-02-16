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
 * \file nmranet_datagram.c
 * This file defines the handling of NMRAnet datagrams.
 *
 * @author Stuart W. Baker
 * @date 4 November 2012
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/tree.h>
#include <endian.h>
#include "core/nmranet_datagram_private.h"
#include "core/nmranet_node_private.h"
#include "core/nmranet_buf.h"
#include "core/nmranet_train_control.h"
#include "os/os.h"
#include "nmranet_config.h"

/** Timeout for datagram acknowledgement. */
#define DATAGRAM_TIMEOUT 3000000000LL

/** Mutual exclusion for socket library */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** pool of unused datagrams */
static nmranet_queue_t pool = NULL;

/** Number of datagrams currently in flight */
static size_t datagramCount = 0;

/** One time initialization for the datagram pool.
 */
static os_thread_once_t datagramOnce;

/** Initialize the datagram pool.
 */
static void datagram_init(void)
{
    pool = nmranet_queue_create();
}

/** Allocate a datagram from the pool.
 * @return datagram allocated, or NULL if there are no more datagrams available.
 */
Datagram *nmranet_datagram_alloc(void)
{
    os_mutex_lock(&mutex);
    if (datagramCount >= DATAGRAM_POOL_SIZE && DATAGRAM_POOL_SIZE != 0)
    {
        /* we have met our quota for outstanding datagrams in flight */
        os_mutex_unlock(&mutex);
        return NULL;
    }

    datagramCount++;
    os_mutex_unlock(&mutex);

    os_thread_once(&datagramOnce, datagram_init);

    Datagram *datagram = nmranet_queue_next(pool);

    if (datagram == NULL)
    {
        datagram = nmranet_buffer_alloc(sizeof(Datagram));
    }

    return datagram;
}

/** Consumed datagrams must be released so that their memory can be freed.
 * The application may hold onto the datagram for an appropriate amount of time
 * to process it before releasing it.  After being released, the memory holding
 * the datagram is no longer available for use by the application.
 * @param datagram datagram to release, this pointer is stale upon return
 */
void nmranet_datagram_release(datagram_t datagram)
{
    nmranet_queue_insert(pool, datagram);
    os_mutex_lock(&mutex);
    datagramCount--;
    os_mutex_unlock(&mutex);
}

/** Determine the datagram protocol (could be 1, 2, or 6 bytes).
 * @param datagram pointer to the beginning of the datagram
 */
uint64_t nmranet_datagram_protocol(datagram_t datagram)
{
    Datagram *d = datagram;
    const unsigned char *bytes = datagram;

    /** @todo we need to double check the byte order here for correct endianness */
    switch (bytes[0] & 0xF0)
    {
        default:
            return d->data[0];
        case DATAGRAM_PROTOCOL_SIZE_2:
            return ((uint64_t)d->data[0] << 0) + ((uint64_t)d->data[1] << 16) +
                   ((uint64_t)d->data[2] << 8);
        case DATAGRAM_PROTOCOL_SIZE_6:
            return ((uint64_t)d->data[0] <<  0) + ((uint64_t)d->data[1] << 48) +
                   ((uint64_t)d->data[2] << 40) + ((uint64_t)d->data[3] << 32) +
                   ((uint64_t)d->data[4] << 24) + ((uint64_t)d->data[5] << 16) +
                   ((uint64_t)d->data[6] << 8);
    }
}

/** Datagram timeout handler.
 * @param data1 struct node_private pointer for node
 * @param data2 NULL
 * @return timer restart value
 */
long long datagram_timeout(void *data1, void* data2)
{
    struct id_node *n = data1;
    NodePriv *priv = n->priv;
    
    /** @todo currently we timeout quietly, should be do something more? */
    priv->txDatagram = NULL;
    
    return OS_TIMER_NONE;
}

/** Produce a Datagram from a given node.
 * @param node node to produce datagram from
 * @param dst destination node id or alias
 * @param protocol datagram protocol to use
 * @param data datagram to produce
 * @param size length of data in bytes
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait, LLONG_MAX = blocking
 * @return 0 upon success, else -1 on error with errno set
 */
int nmranet_datagram_produce(node_t node, node_handle_t dst, uint64_t protocol, const void *data, size_t size, long long timeout)
{
    struct id_node *n = node;
    NodePriv *priv = n->priv;

    if (size > (DATAGRAM_MAX_SIZE - DATAGRAM_PROTOCOL_SIZE(protocol)) ||
        (dst.id == 0 && dst.alias == 0) || timeout < 0)
    {
        /* invalid parameter */
        errno = EINVAL;
        return -1;
    }

    /* timestamp the entry to this function */
    long long start = os_get_time_monotonic();
    
    Datagram *datagram = nmranet_datagram_alloc();

    while (datagram == NULL)
    {
        if ((start + timeout) <= os_get_time_monotonic())
        {
            errno = ENOMEM;
            return -1;
        }
        usleep(MSEC_TO_USEC(200));
        datagram = nmranet_datagram_alloc();
    }
    
    datagram->from.id = nmranet_node_id(node);
    datagram->size = size + DATAGRAM_PROTOCOL_SIZE(protocol);

    /* copy over the protocol and data payload */
    switch (DATAGRAM_PROTOCOL_SIZE(protocol))
    {
        default:
            /* fall through */
        case 1:
            datagram->data[0] = protocol;
            memcpy(&datagram->data[1], data, size);
            break;
        case 2:
            datagram->data[0] = ((protocol >> 0) & 0xFF);
            datagram->data[1] = ((protocol >> 8) & 0xFF);
            memcpy(&datagram->data[2], data, size);
            break;
        case 6:
            datagram->data[0] = ((protocol >>  0) & 0xFF);
            datagram->data[1] = ((protocol >>  8) & 0xFF);
            datagram->data[2] = ((protocol >> 16) & 0xFF);
            datagram->data[3] = ((protocol >> 24) & 0xFF);
            datagram->data[4] = ((protocol >> 32) & 0xFF);
            datagram->data[5] = ((protocol >> 40) & 0xFF);
            memcpy(&datagram->data[6], data, size);
            break;
    }

    os_mutex_lock(&nodeMutex);
    while (priv->txDatagram)
    {
        if ((start + timeout) <= os_get_time_monotonic())
        {
            errno = EBUSY;
            return -1;
        }
        os_mutex_unlock(&nodeMutex);
        usleep(MSEC_TO_USEC(200));
        os_mutex_lock(&nodeMutex);
    }
    priv->txDatagram = datagram;
    os_mutex_unlock(&nodeMutex);

    nmranet_node_write(node, MTI_DATAGRAM, dst, datagram);
    os_timer_start(priv->datagramTimer, DATAGRAM_TIMEOUT);
    return 0;
}

#if 0
/** Process the datagram automatically.  @ref nmranet_datagram_release will be
 * called on the datagram prior to the return of this method.
 * @param node node handle the datagram was received from
 * @param datagram datagram to process, this pointer is stale upon return
 */
void nmranet_datagram_process(node_t node, Datagram *datagram)
{
    switch (nmranet_datagram_protocol(datagram->data))
    {
        default:
            /* unknown protocol */
            break;
        case DATAGRAM_TRAIN_CONTROL:
            nmranet_train_control_process(node, datagram);
            break;
    }
    nmranet_datagram_release(datagram);
}

#endif

/** Post the reception of a datagram with to given node.
 * @param node to post event to
 * @param mti Message Type Indeicator
 * @param src source Node ID
 * @param data datagram to post
 */
void rx_datagram(node_t node, uint16_t mti, node_handle_t src, const uint8_t *data)
{
    struct id_node *n = (struct id_node*)node;
    
    switch (mti)
    {
        case MTI_DATAGRAM_REJECTED:
        {
            uint16_t error = data[0] + (data[1] << 8);
            error = htobe16(error);
            if (IS_DATAGRAM_RESEND_OK(error))
            {
                /* we can try again */
                nmranet_node_write(node, MTI_DATAGRAM, src, n->priv->txDatagram);
                os_timer_start(n->priv->datagramTimer, DATAGRAM_TIMEOUT);
            }
            else
            {
                printf("datagram rejected\n");
                os_timer_stop(n->priv->datagramTimer);
                n->priv->txDatagram = NULL;
            }
            nmranet_buffer_free(data);
            break;
        }
        case MTI_DATAGRAM_OK:
            /* success! */
            os_timer_stop(n->priv->datagramTimer);
            break;
        case MTI_DATAGRAM:
            os_mutex_lock(&nodeMutex);

            nmranet_queue_insert(n->priv->eventQueue, data);

            if (n->priv->wait != NULL)
            {
                /* wakeup whoever is waiting */
                os_sem_post(n->priv->wait);
            }
            os_mutex_unlock(&nodeMutex);
            
            nmranet_node_write(node, MTI_DATAGRAM_OK, src, NULL);
            break;
    }
}

/** Grab a datagram from the datagram queue of the node.
 * @param node to grab datagram from
 * @return NULL if queue is empty, else pointer to the datagram
 */
datagram_t nmranet_datagram_consume(node_t node)
{
    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&nodeMutex);
    Datagram *datagram = nmranet_queue_next(n->priv->eventQueue);
    os_mutex_unlock(&nodeMutex);
    
    return datagram;
}

