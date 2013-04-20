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

    os_thread_create( &thread_handle, "dgram", 0, 512, datagram_thread, NULL);
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

/** Allocate and prepare a datagram buffer.
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

    os_thread_once(&datagramOnce, datagram_init);
    return 0;
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


const uint8_t cdi[] =
{
   60, 63, 120, 109, 108, 32, 118, 101, 114, 115, 105, 111, 110, 61, 34, 49, 46, 48, 34, 63, 62, 10, 60, 63, 120, 109, 108, 45, 115, 116, 121, 108, 101, 115, 104, 101, 101, 116, 32, 116, 121, 112, 101, 61, 34, 116, 101, 120, 116, 47, 120, 115, 108, 34, 32, 104, 114, 101, 102, 61, 34, 120, 115, 108, 116, 47, 99, 100, 105, 46, 120, 115, 108, 34, 63, 62, 10, 60, 99, 100, 105, 32, 120, 109, 108, 110, 115, 58, 120, 115, 105, 61, 34, 104, 116, 116, 112, 58, 47, 47, 119, 119, 119, 46, 119, 51, 46, 111, 114, 103, 47, 50, 48, 48, 49, 47, 88, 77, 76, 83, 99, 104, 101, 109, 97, 45, 105, 110,    // | <?xml version="1.0"?><?xml-stylesheet type="text/xsl" href="xslt/cdi.xsl"?><cdi xmlns:xsi="http://www.w3.org/2001/XMLSchema-in|
   115, 116, 97, 110, 99, 101, 34, 32, 120, 115, 105, 58, 110, 111, 78, 97, 109, 101, 115, 112, 97, 99, 101, 83, 99, 104, 101, 109, 97, 76, 111, 99, 97, 116, 105, 111, 110, 61, 34, 104, 116, 116, 112, 58, 47, 47, 111, 112, 101, 110, 108, 99, 98, 46, 111, 114, 103, 47, 116, 114, 117, 110, 107, 47, 112, 114, 111, 116, 111, 116, 121, 112, 101, 115, 47, 120, 109, 108, 47, 115, 99, 104, 101, 109, 97, 47, 99, 100, 105, 46, 120, 115, 100, 34, 62, 10, 10, 60, 105, 100, 101, 110, 116, 105, 102, 105, 99, 97, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 60, 109, 97, 110, 117, 102, 97, 99, 116, 117,    // | stance" xsi:noNamespaceSchemaLocation="http://openlcb.org/trunk/prototypes/xml/schema/cdi.xsd"><identification>    <manufactu|
   114, 101, 114, 62, 83, 112, 97, 99, 101, 108, 121, 32, 83, 112, 114, 111, 99, 107, 101, 116, 115, 60, 47, 109, 97, 110, 117, 102, 97, 99, 116, 117, 114, 101, 114, 62, 10, 32, 32, 32, 32, 60, 109, 111, 100, 101, 108, 62, 77, 111, 100, 101, 108, 32, 49, 50, 51, 32, 85, 110, 105, 98, 108, 97, 98, 60, 47, 109, 111, 100, 101, 108, 62, 10, 32, 32, 32, 32, 60, 104, 97, 114, 100, 119, 97, 114, 101, 86, 101, 114, 115, 105, 111, 110, 62, 69, 67, 32, 52, 49, 53, 60, 47, 104, 97, 114, 100, 119, 97, 114, 101, 86, 101, 114, 115, 105, 111, 110, 62, 10, 32, 32, 32, 32, 60, 115, 111, 102,    // | rer>Spacely Sprockets</manufacturer>    <model>Model 123 Uniblab</model>    <hardwareVersion>EC 415</hardwareVersion>    <sof|
   116, 119, 97, 114, 101, 86, 101, 114, 115, 105, 111, 110, 62, 49, 46, 50, 46, 51, 46, 52, 60, 47, 115, 111, 102, 116, 119, 97, 114, 101, 86, 101, 114, 115, 105, 111, 110, 62, 10, 32, 32, 32, 32, 60, 109, 97, 112, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 114, 101, 108, 97, 116, 105, 111, 110, 62, 60, 112, 114, 111, 112, 101, 114, 116, 121, 62, 83, 105, 122, 101, 60, 47, 112, 114, 111, 112, 101, 114, 116, 121, 62, 60, 118, 97, 108, 117, 101, 62, 56, 32, 99, 109, 32, 98, 121, 32, 49, 50, 32, 99, 109, 60, 47, 118, 97, 108, 117, 101, 62, 60, 47, 114, 101, 108, 97, 116, 105,    // | twareVersion>1.2.3.4</softwareVersion>    <map>        <relation><property>Size</property><value>8 cm by 12 cm</value></relati|
   111, 110, 62, 10, 32, 32, 32, 32, 60, 47, 109, 97, 112, 62, 10, 60, 47, 105, 100, 101, 110, 116, 105, 102, 105, 99, 97, 116, 105, 111, 110, 62, 10, 10, 60, 115, 101, 103, 109, 101, 110, 116, 32, 111, 114, 105, 103, 105, 110, 61, 34, 48, 34, 32, 115, 112, 97, 99, 101, 61, 34, 48, 34, 62, 10, 32, 32, 32, 32, 60, 103, 114, 111, 117, 112, 32, 111, 102, 102, 115, 101, 116, 61, 34, 49, 50, 56, 34, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 110, 97, 109, 101, 62, 85, 115, 101, 114, 32, 73, 100, 101, 110, 116, 105, 102, 105, 99, 97, 116, 105, 111, 110, 60, 47, 110, 97, 109,    // | on>    </map></identification><segment origin="0" space="0">    <group offset="128">        <name>User Identification</nam|
   101, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 76, 101, 116, 115, 32, 116, 104, 101, 32, 117, 115, 101, 114, 32, 97, 100, 100, 32, 104, 105, 115, 32, 111, 119, 110, 32, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 60, 47, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 115, 116, 114, 105, 110, 103, 32, 115, 105, 122, 101, 61, 34, 51, 50, 34, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 60, 110, 97, 109, 101, 62, 78, 111, 100, 101, 32, 78, 97,    // | e>        <description>Lets the user add his own description</description>        <string size="32">            <name>Node Na|
   109, 101, 60, 47, 110, 97, 109, 101, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 47, 115, 116, 114, 105, 110, 103, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 115, 116, 114, 105, 110, 103, 32, 115, 105, 122, 101, 61, 34, 57, 54, 34, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 60, 110, 97, 109, 101, 62, 78, 111, 100, 101, 32, 68, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 60, 47, 110, 97, 109, 101, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 47, 115, 116, 114, 105, 110, 103, 62, 10, 32, 32, 32, 32, 60, 47, 103, 114, 111, 117, 112, 62, 10,    // | me</name>        </string>        <string size="96">            <name>Node Description</name>        </string>    </group>|
   32, 32, 32, 32, 60, 103, 114, 111, 117, 112, 32, 111, 102, 102, 115, 101, 116, 61, 34, 49, 50, 56, 34, 32, 114, 101, 112, 108, 105, 99, 97, 116, 105, 111, 110, 61, 34, 50, 34, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 110, 97, 109, 101, 62, 80, 114, 111, 100, 117, 99, 101, 100, 32, 69, 118, 101, 110, 116, 115, 60, 47, 110, 97, 109, 101, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 84, 104, 101, 32, 69, 118, 101, 110, 116, 73, 68, 115, 32, 102, 111, 114, 32, 116, 104, 101, 32, 112, 114, 111, 100, 117, 99, 101, 114,    // |     <group offset="128" replication="2">        <name>Produced Events</name>        <description>The EventIDs for the producer|
   115, 60, 47, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 101, 118, 101, 110, 116, 105, 100, 47, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 101, 118, 101, 110, 116, 105, 100, 47, 62, 10, 32, 32, 32, 32, 60, 47, 103, 114, 111, 117, 112, 62, 10, 32, 32, 32, 32, 60, 103, 114, 111, 117, 112, 32, 114, 101, 112, 108, 105, 99, 97, 116, 105, 111, 110, 61, 34, 50, 34, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 110, 97, 109, 101, 62, 67, 111, 110, 115, 117, 109, 101, 100, 32, 69, 118, 101, 110, 116, 115, 60, 47, 110, 97,    // | s</description>        <eventid/>        <eventid/>    </group>    <group replication="2">        <name>Consumed Events</na|
   109, 101, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 84, 104, 101, 32, 69, 118, 101, 110, 116, 73, 68, 115, 32, 102, 111, 114, 32, 116, 104, 101, 32, 99, 111, 110, 115, 117, 109, 101, 114, 115, 60, 47, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 101, 118, 101, 110, 116, 105, 100, 47, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 101, 118, 101, 110, 116, 105, 100, 47, 62, 10, 32, 32, 32, 32, 60, 47, 103, 114, 111, 117, 112, 62, 10, 32, 32, 32, 32, 60, 98, 105,    // | me>        <description>The EventIDs for the consumers</description>        <eventid/>        <eventid/>    </group>    <bi|
   116, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 110, 97, 109, 101, 62, 83, 97, 109, 112, 108, 101, 32, 98, 105, 116, 32, 118, 97, 114, 105, 97, 98, 108, 101, 60, 47, 110, 97, 109, 101, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 68, 111, 101, 115, 110, 39, 116, 32, 100, 111, 32, 97, 110, 121, 116, 104, 105, 110, 103, 60, 47, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 60, 47, 98, 105, 116, 62, 10, 32, 32, 32, 32, 60, 105, 110, 116, 32, 115, 105, 122, 101, 61, 34, 50, 34, 62,    // | t>        <name>Sample bit variable</name>        <description>Doesn't do anything</description>    </bit>    <int size="2">|
   10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 110, 97, 109, 101, 62, 83, 97, 109, 112, 108, 101, 32, 105, 110, 116, 101, 103, 101, 114, 32, 118, 97, 114, 105, 97, 98, 108, 101, 60, 47, 110, 97, 109, 101, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 68, 111, 101, 115, 110, 39, 116, 32, 100, 111, 32, 97, 110, 121, 116, 104, 105, 110, 103, 60, 47, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 109, 105, 110, 62, 49, 60, 47, 109, 105, 110, 62, 10, 32, 32, 32, 32, 32, 32,    // |         <name>Sample integer variable</name>        <description>Doesn't do anything</description>        <min>1</min>      |
   32, 32, 60, 109, 97, 120, 62, 57, 57, 57, 60, 47, 109, 97, 120, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 100, 101, 102, 97, 117, 108, 116, 62, 49, 50, 60, 47, 100, 101, 102, 97, 117, 108, 116, 62, 10, 32, 32, 32, 32, 60, 47, 105, 110, 116, 62, 10, 60, 47, 115, 101, 103, 109, 101, 110, 116, 62, 10, 10, 60, 115, 101, 103, 109, 101, 110, 116, 32, 111, 114, 105, 103, 105, 110, 61, 34, 49, 50, 56, 34, 32, 115, 112, 97, 99, 101, 61, 34, 49, 34, 62, 10, 32, 32, 32, 32, 60, 105, 110, 116, 32, 115, 105, 122, 101, 61, 34, 49, 34, 62, 10, 32, 32, 32, 32, 32, 32,    // |   <max>999</max>        <default>12</default>    </int></segment><segment origin="128" space="1">    <int size="1">      |
   32, 32, 60, 110, 97, 109, 101, 62, 82, 101, 115, 101, 116, 60, 47, 110, 97, 109, 101, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 67, 111, 110, 116, 114, 111, 108, 115, 32, 114, 101, 108, 111, 97, 100, 105, 110, 103, 32, 97, 110, 100, 32, 99, 108, 101, 97, 114, 105, 110, 103, 32, 110, 111, 100, 101, 32, 109, 101, 109, 111, 114, 121, 46, 32, 66, 111, 97, 114, 100, 32, 109, 117, 115, 116, 32, 98, 101, 32, 114, 101, 115, 116, 97, 114, 116, 101, 100, 32, 102, 111, 114, 32, 116, 104, 105, 115, 32, 116, 111, 32, 116, 97, 107, 101, 32,    // |   <name>Reset</name>        <description>Controls reloading and clearing node memory. Board must be restarted for this to take |
   101, 102, 102, 101, 99, 116, 46, 60, 47, 100, 101, 115, 99, 114, 105, 112, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 109, 97, 112, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 60, 114, 101, 108, 97, 116, 105, 111, 110, 62, 60, 112, 114, 111, 112, 101, 114, 116, 121, 62, 56, 53, 60, 47, 112, 114, 111, 112, 101, 114, 116, 121, 62, 60, 118, 97, 108, 117, 101, 62, 40, 78, 111, 32, 114, 101, 115, 101, 116, 41, 60, 47, 118, 97, 108, 117, 101, 62, 60, 47, 114, 101, 108, 97, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,    // | effect.</description>        <map>            <relation><property>85</property><value>(No reset)</value></relation>          |
   32, 32, 60, 114, 101, 108, 97, 116, 105, 111, 110, 62, 60, 112, 114, 111, 112, 101, 114, 116, 121, 62, 48, 60, 47, 112, 114, 111, 112, 101, 114, 116, 121, 62, 60, 118, 97, 108, 117, 101, 62, 82, 101, 115, 101, 116, 32, 97, 108, 108, 32, 116, 111, 32, 100, 101, 102, 97, 117, 108, 116, 115, 60, 47, 118, 97, 108, 117, 101, 62, 60, 47, 114, 101, 108, 97, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 60, 114, 101, 108, 97, 116, 105, 111, 110, 62, 60, 112, 114, 111, 112, 101, 114, 116, 121, 62, 49, 55, 48, 60, 47, 112, 114, 111, 112, 101, 114, 116, 121, 62,    // |   <relation><property>0</property><value>Reset all to defaults</value></relation>            <relation><property>170</property>|
   60, 118, 97, 108, 117, 101, 62, 82, 101, 115, 101, 116, 32, 106, 117, 115, 116, 32, 69, 118, 101, 110, 116, 73, 68, 115, 32, 116, 111, 32, 100, 101, 102, 97, 117, 108, 116, 115, 60, 47, 118, 97, 108, 117, 101, 62, 60, 47, 114, 101, 108, 97, 116, 105, 111, 110, 62, 10, 32, 32, 32, 32, 32, 32, 32, 32, 60, 47, 109, 97, 112, 62, 10, 32, 32, 32, 32, 60, 47, 105, 110, 116, 62, 10, 60, 47, 115, 101, 103, 109, 101, 110, 116, 62, 10, 10, 60, 47, 99, 100, 105, 62, 10,    // | <value>Reset just EventIDs to defaults</value></relation>        </map>    </int></segment></cdi>|
   0
};

/** Process the process a memory configuration datagram.
 * @param node node the datagram is to
 * @param datagram datagram to process, this pointer is stale upon return
 */
static void datagram_process_configuration(node_t node, datagram_t datagram)
{
    Datagram *d = datagram;
    uint8_t *data = nmranet_datagram_payload(datagram);
    uint8_t space;
    uint32_t address;
    uint8_t space_offset = 0;
    uint8_t space_special = data[CONFIG_COMMAND] & CONFIG_COMMAND_FLAG_MASK;

    /* figure out what memory space we are operating on */
    switch (space_special)
    {
        default:
            /* This is not a special memory space */
            space = data[CONFIG_SPACE];
            space_offset = 1;
            break;
        case CONFIG_COMMAND_CDI:        /* fall through */
        case CONFIG_COMMAND_ALL_MEMORY: /* fall through */
        case CONFIG_COMMAND_CONFIG:
            space = CONFIG_SPACE_SPECIAL + space_special;
            break;
    }
    
    /* figure out the address */
    address = ((uint32_t)data[CONFIG_ADDRESS + 0] << 24) +
              ((uint32_t)data[CONFIG_ADDRESS + 1] << 16) +
              ((uint32_t)data[CONFIG_ADDRESS + 2] <<  8) +
              ((uint32_t)data[CONFIG_ADDRESS + 3] <<  0);
              
    /* figure out the command */
    switch (data[CONFIG_COMMAND] & CONFIG_COMMAND_MASK)
    {
        default:
            /* We don't know this command */
            break;
        case CONFIG_COMMAND_READ:
        {
            if (space == CONFIG_SPACE_CDI)
            {
                uint8_t count = data[CONFIG_COUNT + space_offset];
                uint8_t command = CONFIG_COMMAND_READ_REPLY | CONFIG_COMMAND_CDI;
                count = (address + count) < sizeof(cdi) ?
                        count : (sizeof(cdi) - address);
                datagram_t reply = nmranet_datagram_buffer_get(DATAGRAM_CONFIGURATION,
                                                               count + 5,
                                                               OS_WAIT_FOREVER);
                nmranet_datagram_buffer_fill(reply,
                                             DATAGRAM_CONFIGURATION,
                                             &command,
                                             CONFIG_COMMAND,
                                             1);
                nmranet_datagram_buffer_fill(reply,
                                             DATAGRAM_CONFIGURATION,
                                             &data[CONFIG_ADDRESS],
                                             CONFIG_ADDRESS,
                                             sizeof(uint32_t));
                nmranet_datagram_buffer_fill(reply,
                                             DATAGRAM_CONFIGURATION,
                                             cdi + address,
                                             CONFIG_DATA,
                                             count);
                nmranet_datagram_buffer_produce(node, d->from, reply, OS_WAIT_FOREVER);
            }
            else if (space == CONFIG_SPACE_CONFIG)
            {
                uint8_t count = data[CONFIG_COUNT + space_offset];
                uint8_t command = CONFIG_COMMAND_READ_REPLY | CONFIG_COMMAND_CONFIG;
                datagram_t reply = nmranet_datagram_buffer_get(DATAGRAM_CONFIGURATION,
                                                               count + 5,
                                                               OS_WAIT_FOREVER);
                nmranet_datagram_buffer_fill(reply,
                                             DATAGRAM_CONFIGURATION,
                                             &command,
                                             CONFIG_COMMAND,
                                             1);
                nmranet_datagram_buffer_fill(reply,
                                             DATAGRAM_CONFIGURATION,
                                             &data[CONFIG_ADDRESS],
                                             CONFIG_ADDRESS,
                                             sizeof(uint32_t));
#if 0
                nmranet_datagram_buffer_fill(reply,
                                             DATAGRAM_CONFIGURATION,
                                             cdi + address,
                                             CONFIG_DATA,
                                             count);
#endif
                nmranet_datagram_buffer_produce(node, d->from, reply, OS_WAIT_FOREVER);
            }
            break;
        }
        case CONFIG_COMMAND_OPTIONS:
        {
            uint16_t available = CONFIG_AVAIL_UR;
            uint8_t reply[6];
            reply[CONFIG_COMMAND] = CONFIG_COMMAND_OPTIONS_REPLY;
            reply[CONFIG_AVAILABLE] = available >> 8;
            reply[CONFIG_AVAILABLE + 1] = available & 0xFF;
            reply[CONFIG_LENGTHS] = 0;
            reply[CONFIG_HIGHEST] = CONFIG_SPACE_CDI;
            reply[CONFIG_LOWEST] = CONFIG_SPACE_CONFIG;
            nmranet_datagram_produce(node, d->from, DATAGRAM_CONFIGURATION, reply, 6, OS_WAIT_FOREVER);
            break;
        }
        case CONFIG_COMMAND_INFORMATION:
        {
            uint32_t address_highest = sizeof(cdi);
            uint8_t reply[7];
            reply[CONFIG_COMMAND] = CONFIG_COMMAND_INFORMATION_REPLY | CONFIG_COMMAND_PRESENT;
            reply[CONFIG_ADDRESS_SPACE] = data[CONFIG_ADDRESS_SPACE];
            reply[CONFIG_ADDRESS_HIGHEST + 0] = (address_highest >> 24) & 0xFF;
            reply[CONFIG_ADDRESS_HIGHEST + 1] = (address_highest >> 16) & 0xFF;
            reply[CONFIG_ADDRESS_HIGHEST + 2] = (address_highest >>  8) & 0xFF;
            reply[CONFIG_ADDRESS_HIGHEST + 3] = (address_highest >>  0) & 0xFF;
            reply[CONFIG_FLAGS] = CONFIG_FLAG_RO;
            nmranet_datagram_produce(node, d->from, DATAGRAM_CONFIGURATION, reply, 7, OS_WAIT_FOREVER);
            break;
        }
    }
    
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
                datagram_process_configuration(d->to, datagram);
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
                printf("datagram rejected\n");
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

