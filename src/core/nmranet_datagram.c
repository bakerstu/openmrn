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

#ifndef TARGET_LPC11Cxx
#define NMRANET_SUPPORT_DATAGRAM
#endif

#ifdef NMRANET_SUPPORT_DATAGRAM

void datagram_memory_config(node_t node, datagram_t datagram);
void nmranet_memory_config_init(void);


/** Timeout for datagram acknowledgement. */
#define DATAGRAM_TIMEOUT 3000000000LL

/** Mutual exclusion for socket library */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** pool of unused datagrams */
static nmranet_queue_t pool = NULL;

/** pool of unused datagrams */
static nmranet_queue_t pending = NULL;

/** Number of datagrams currently in flight */
static size_t datagramCount = 0;

/** One time initialization for the datagram pool.
 */
static os_thread_once_t datagramOnce = OS_THREAD_ONCE_INIT;

/** Counter for number of datagrams to process in the library.
 */
static os_sem_t sem;

/** The thread for processing platform datagrams.
 * @param arg unused
 * @return should never return
 */
static void *datagram_thread(void* arg);

/** Initialize the datagram pool.
 */
static void datagram_init(void)
{
    os_thread_t thread_handle;

    pool = nmranet_queue_create();
    pending = nmranet_queue_create();
    os_sem_init(&sem, 0);
    nmranet_memory_config_init();

    os_thread_create( &thread_handle, "datagram", 0, DATAGRAM_THREAD_STACK_SIZE, datagram_thread, NULL);
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
 * @return protocol type
 */
uint64_t nmranet_datagram_protocol(datagram_t datagram)
{
    Datagram *d = datagram;

    /** @todo we need to double check the byte order here for correct endianness */
    switch (d->data[0] & 0xF0)
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

/** Determine the datagram payload
 * @param datagram pointer to the beginning of the datagram
 * @return pointer to payload an a uint8_t array, assume byte alignment
 */
uint8_t *nmranet_datagram_payload(datagram_t datagram)
{
    Datagram *d = datagram;

    switch (d->data[0] & 0xF0)
    {
        default:
            return d->data + 1;
        case DATAGRAM_PROTOCOL_SIZE_2:
            return d->data + 3;
        case DATAGRAM_PROTOCOL_SIZE_6:
            return d->data + 7;
    }
}

/** Datagram timeout handler.
 * @param data1 struct node_private pointer for node
 * @param data2 NULL
 * @return timer restart value
 */
long long nmranet_datagram_timeout(void *data1, void* data2)
{
    struct id_node *n = data1;
    NodePriv *priv = n->priv;
    
    /** @todo currently we timeout quietly, should we do something more? */
    os_mutex_lock(&nodeMutex);
    if (priv->txDatagram)
    {
        nmranet_datagram_release(priv->txDatagram);
        priv->txDatagram = NULL;
    }
    os_mutex_unlock(&nodeMutex);
    
    return OS_TIMER_NONE;
}

/** Allocate and prepare a datagram buffer.
 * @param protocol datagram protocol to use
 * @param size max length of data in bytes
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
 * @return datagram handle upon success, else NULL on error with errno set
 */
datagram_t nmranet_datagram_buffer_get(uint64_t protocol, size_t size, long long timeout)
{
    if (size > (DATAGRAM_MAX_SIZE - DATAGRAM_PROTOCOL_SIZE(protocol)) || timeout < 0)
    {
        /* invalid parameter */
        errno = EINVAL;
        return NULL;
    }

    /* timestamp the entry to this function */
    long long start = os_get_time_monotonic();
    
    Datagram *datagram = nmranet_datagram_alloc();

    while (datagram == NULL)
    {
        if ((start + timeout) <= os_get_time_monotonic() && timeout != OS_WAIT_FOREVER)
        {
            errno = ENOMEM;
            return NULL;
        }
        usleep(MSEC_TO_USEC(200));
        datagram = nmranet_datagram_alloc();
    }
    
    datagram->size = size + DATAGRAM_PROTOCOL_SIZE(protocol);

    /* copy over the protocol */
    switch (DATAGRAM_PROTOCOL_SIZE(protocol))
    {
        default:
            /* fall through */
        case 1:
            datagram->data[0] = protocol;
            break;
        case 2:
            datagram->data[0] = ((protocol >> 0) & 0xFF);
            datagram->data[1] = ((protocol >> 8) & 0xFF);
            break;
        case 6:
            datagram->data[0] = ((protocol >>  0) & 0xFF);
            datagram->data[1] = ((protocol >>  8) & 0xFF);
            datagram->data[2] = ((protocol >> 16) & 0xFF);
            datagram->data[3] = ((protocol >> 24) & 0xFF);
            datagram->data[4] = ((protocol >> 32) & 0xFF);
            datagram->data[5] = ((protocol >> 40) & 0xFF);
            break;
    }

    return datagram;
}

/** Fill an already allocated datagram buffer.
 * @param protocol datagram protocol to use
 * @param data datagram to fill into datagram
 * @param offset offset within datagram payload to start filling
 * @param size length of data in bytes to fill
 */
void nmranet_datagram_buffer_fill(datagram_t datagram, uint64_t protocol, const void *data, size_t offset, size_t size)
{
    Datagram *d = datagram;
    
    /* copy over the data payload */
    switch (DATAGRAM_PROTOCOL_SIZE(protocol))
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
    if ((offset + size) > DATAGRAM_MAX_SIZE)
    {
        /* we went too far */
        return;
    }
    memcpy(&d->data[offset], data, size);
}

/** Produce a Datagram from a given node.
 * @param node node to produce datagram from
 * @param dst destination node id or alias
 * @param datagram datagram to produce
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
 * @return 0 upon success, else -1 on error with errno set
 */
int nmranet_datagram_buffer_produce(node_t node, node_handle_t dst, datagram_t datagram, long long timeout)
{
    struct id_node *n = node;
    NodePriv *priv = n->priv;

    /* timestamp the entry to this function */
    long long start = os_get_time_monotonic();

    os_mutex_lock(&nodeMutex);
    while (priv->txDatagram)
    {
        if ((start + timeout) <= os_get_time_monotonic() && timeout != OS_WAIT_FOREVER)
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

/** Produce a Datagram from a given node.
 * @param node node to produce datagram from
 * @param dst destination node id or alias
 * @param protocol datagram protocol to use
 * @param data datagram to produce
 * @param size length of data in bytes
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
 * @return 0 upon success, else -1 on error with errno set
 */
int nmranet_datagram_produce(node_t node, node_handle_t dst, uint64_t protocol, const void *data, size_t size, long long timeout)
{
    if (dst.id == 0 && dst.alias == 0)
    {
        /* invalid parameter */
        errno = EINVAL;
        return -1;
    }

    /* timestamp the entry to this function */
    Datagram *datagram = nmranet_datagram_buffer_get(protocol, size, timeout);
    
    if (datagram == NULL)
    {
        /* could not allocate buffer, ERRNO already set appropriately */
        return -1;
    }
    
    datagram->from.id = nmranet_node_id(node);

    nmranet_datagram_buffer_fill(datagram, protocol, data, 0, size);

    if (nmranet_datagram_buffer_produce(node, dst, datagram, timeout) != 0)
    {
        /* ERRNO already set appropriately */
        nmranet_datagram_release(datagram);
        return -1;
    }
    return 0;
}

/** The thread for processing platform datagrams.
 * @param arg unused
 * @return should never return
 */
static void *datagram_thread(void* arg)
{
    for ( ; /* forever */ ; )
    {
        os_sem_wait(&sem);
        datagram_t datagram = nmranet_queue_next(pending);
        Datagram *d = datagram;
        switch (nmranet_datagram_protocol(datagram))
        {
            default:
                /* unhandled protocol */
                break;
            case DATAGRAM_CONFIGURATION:
                datagram_memory_config(d->to, datagram);
                break;
        }
        nmranet_datagram_release(datagram);
    }
    return NULL;
}

/** Process the datagram automatically.  @ref nmranet_datagram_release will be
 * called on the datagram prior to the return of this method.
 * @param node node the datagram is to
 * @param datagram datagram to process, this pointer is stale upon return
 * @return 0 if we do not have an automated processing option, else return 1
 */
static int datagram_process(node_t node, datagram_t datagram)
{
    Datagram *d = datagram;
    d->to = node;
    switch (nmranet_datagram_protocol(datagram))
    {
        default:
            /* unhandled protocol */
            return 0;
        case DATAGRAM_CONFIGURATION:
            nmranet_queue_insert(pending, datagram);
            os_sem_post(&sem);
            break;
    }
    return 1;
}

/** Post the reception of a datagram to given node.
 * @param node to post datagram to
 * @param mti Message Type Indicator
 * @param src source Node ID
 * @param data datagram to post
 */
void nmranet_datagram_packet(node_t node, uint16_t mti, node_handle_t src, const uint8_t *data)
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
                os_timer_stop(n->priv->datagramTimer);

                if (n->priv->txDatagram)
                {
                    nmranet_node_write(node, MTI_DATAGRAM, src, n->priv->txDatagram);
                    os_timer_start(n->priv->datagramTimer, DATAGRAM_TIMEOUT);
                }
            }
            else
            {
#ifndef __FreeRTOS__
                printf("datagram rejected\n");
#endif
                os_timer_stop(n->priv->datagramTimer);
                if (n->priv->txDatagram)
                {
                    nmranet_datagram_release(n->priv->txDatagram);
                    n->priv->txDatagram = NULL;
                }
            }
            nmranet_buffer_free(data);
            break;
        }
        case MTI_DATAGRAM_OK:
            /* success! */
            os_timer_stop(n->priv->datagramTimer);
            if (n->priv->txDatagram)
            {
                nmranet_datagram_release(n->priv->txDatagram);
                n->priv->txDatagram = NULL;
            }
            break;
        case MTI_DATAGRAM:
            nmranet_node_write(node, MTI_DATAGRAM_OK, src, NULL);
            if (datagram_process(node, (datagram_t*)data) == 0)
            {
                /* push this up for the application to process */
                nmranet_queue_insert(n->priv->datagramQueue, data);
                if (n->priv->wait != NULL)
                {
                    /* wakeup whoever is waiting */
                    os_sem_post(n->priv->wait);
                }
            }
            
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
    Datagram *datagram = nmranet_queue_next(n->priv->datagramQueue);
    os_mutex_unlock(&nodeMutex);
    
    return datagram;
}

/** Number of datagrams pending in the datagram queue of the node.
 * @param node node to query
 * @return number of datagrams pending
 */
size_t nmranet_datagram_pending(node_t node)
{
    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&nodeMutex);
    size_t pending = nmranet_queue_pending(n->priv->datagramQueue);
    os_mutex_unlock(&nodeMutex);

    return pending; 
}

#else  // no datagram support

size_t nmranet_datagram_pending(node_t node) {
  return 0;
}

datagram_t nmranet_datagram_consume(node_t node) {
  return NULL;
}

int nmranet_datagram_produce(node_t node, node_handle_t dst, uint64_t protocol, const void *data, size_t size, long long timeout) {
  abort();
}

Datagram *nmranet_datagram_alloc(void) {
  return NULL;
}

void nmranet_datagram_release(datagram_t datagram) {
  abort();
}

uint64_t nmranet_datagram_protocol(datagram_t datagram) {
  abort();
}

uint8_t *nmranet_datagram_payload(datagram_t datagram) {
  abort();
}

datagram_t nmranet_datagram_buffer_get(uint64_t protocol, size_t size, long long timeout) {
  abort();
}

void nmranet_datagram_buffer_fill(datagram_t datagram, uint64_t protocol, const void *data, size_t offset, size_t size) {
  abort();
}

int nmranet_datagram_buffer_produce(node_t node, node_handle_t dst, datagram_t datagram, long long timeout) {
  abort();
}

long long nmranet_datagram_timeout(void *data1, void* data2) {
  abort();
}

void nmranet_datagram_packet(node_t node, uint16_t mti, node_handle_t src, const uint8_t *data) {
  // Ignores datagram packets.
}

#endif
