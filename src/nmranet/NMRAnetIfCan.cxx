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
 * \file NMRAnetIfCan.cxx
 * This file provides an NMRAnet interface specific to CAN.
 *
 * @author Stuart W. Baker
 * @date 18 September 2013
 */

#if defined(__linux__) || defined(__MACH__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#endif

#include <endian.h>
#include <unistd.h>

#include "nmranet/NMRAnetIfCan.hxx"
#include "nmranet/NMRAnetDatagram.hxx"

#include "core/nmranet_datagram_private.h"

namespace NMRAnet
{

#define DATAGRAM_MESSAGE_SIZE (sizeof(Datagram::Message) + sizeof(OSTimer))

/** This is how long we should wait before giving up on an incoming datagram.
 */
#define DATAGRAM_TIMEOUT 3000000000LL

/** Constructor.
 * @param node_id node ID of interface
 * @param device description for this instance
 * @param read read method for this interface
 * @param write write method for this interface
 */
IfCan::IfCan(NodeID node_id, const char* device,
             ssize_t (*read)(int, void*, size_t),
             ssize_t (*write)(int, const void*, size_t))
    : If(node_id),
      read(read),
      write(write),
      fd(open(device, O_RDWR)),
      writeBuffer(this),
      pool((Pool*)malloc(sizeof(Pool) * ALIAS_POOL_SIZE)),
      downstreamCache(0, DOWNSTREAM_ALIAS_CACHE_SIZE),
      upstreamCache(node_id, UPSTREAM_ALIAS_CACHE_SIZE, upstream_alias_removed,
                    this),
      mutex(),
      linkStatus(DOWN),
      datagramPool(DATAGRAM_MESSAGE_SIZE, Datagram::POOL_SIZE),
      datagramTree(Datagram::POOL_SIZE)

{
    {
        /* setup the timers for the datagram pool buffers */
        Buffer* buffer[Datagram::POOL_SIZE];
        for (unsigned int i = 0; i < Datagram::POOL_SIZE; ++i) {
            buffer[i] = datagramPool.buffer_alloc(DATAGRAM_MESSAGE_SIZE);
            Datagram::Message* m = (Datagram::Message*)buffer[i]->start();
            OSTimer* t = (OSTimer*)(m + 1);
            /* Placement new allows for runtime/link-time array size */
            new (t) OSTimer(datagram_timeout, this, buffer[i]);
        }

        for (unsigned int i = 0; i < Datagram::POOL_SIZE; ++i) {
            buffer[i]->free();
        }
    }
    for (unsigned int i = 0; i < ALIAS_POOL_SIZE; ++i) {
        /* Placement new allows for runtime/link-time array size */
        new (pool + i) Pool(this);
    }

    if (fd >= 0) {
        link_up();
    }
#if defined(__linux__) || defined(__MACH__)
    else if (!strncmp(device, "/tcp/", 5)) {
        /* we must be working with a socket here */
        int port = strtol(device + 5, NULL, 0);
        int yes = 1;

        HASSERT(!(port == 0 && errno == EINVAL));

        socklen_t addrlen;
        struct sockaddr_in addr;
        int listen_fd;

        listen_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

        HASSERT(listen_fd >= 0);

        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        printf("port: %d\n", port);

        int result = setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes,
                                sizeof(yes));

        HASSERT(result == 0);

        result = bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr));

        HASSERT(result == 0);

        addrlen = sizeof(addr);

        result = getsockname(listen_fd, (struct sockaddr*)&addr, &addrlen);

        HASSERT(result == 0);

        result = listen(listen_fd, 1);

        HASSERT(result == 0);

        fd = accept(listen_fd, (struct sockaddr*)&addr, &addrlen);

        printf("fd: %d, errno: %s\n", fd, strerror(errno));
        HASSERT(fd >= 0);

        /* turn off Nagel algorithm. */
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        link_up();
    }
#else
    else {
        HASSERT(0);
    }
#endif

    new OSThread(device, 0, CAN_IF_READ_THREAD_STACK_SIZE, read_thread_entry,
                 this);
}

/** Transition to link up state.
 */
void IfCan::link_up()
{
    mutex.lock();
    for (unsigned int i = 0; i < ALIAS_POOL_SIZE; ++i) {
        claim_alias(0, upstreamCache.generate(), pool + i);
    }
    mutex.unlock();
}

/** Transition to link down state.
 */
void IfCan::link_down()
{
}

/** Get the NMRAnet MTI from a can identifier.
 * @param can_id CAN identifider
 * @return NMRAnet MTI
 */
If::MTI IfCan::nmranet_mti(uint32_t can_id)
{
    switch (get_can_frame_type(can_id)) {
        default:
            return MTI_NONE;
        case GLOBAL_ADDRESSED:
            return (MTI)get_mti(can_id);
        case DATAGRAM_ONE_FRAME:
        /* fall through */
        case DATAGRAM_FIRST_FRAME:
        /* fall through */
        case DATAGRAM_MIDDLE_FRAME:
        /* fall through */
        case DATAGRAM_FINAL_FRAME:
            return MTI_DATAGRAM;
        case STREAM_DATA:
            return MTI_STREAM_DATA;
    }
}

/** Get the CAN identifier from an NMRAnet mti and source alias.
 * @param mti NMRAnet MTI
 * @param src Source node alias
 * @return CAN identifier
 */
uint32_t IfCan::can_identifier(MTI mti, NodeAlias src)
{
    return ((src << SRC_SHIFT) & SRC_MASK) + ((mti << MTI_SHIFT) & MTI_MASK)
           + ((1 << CAN_FRAME_TYPE_SHIFT)) + ((1 << FRAME_TYPE_SHIFT))
           + ((1 << PRIORITY_SHIFT));
}

/** Put out a claim on an alias.  This method must always be called with the
 * mutex locked.
 * @param node_id node id that is making the claim
 * @param alias alias that node is claiming
 * @param entry entry within the pool to use for claim.
 */
void IfCan::claim_alias(NodeID node_id, NodeAlias alias, Pool* entry)
{
    HASSERT(entry->status == FREE);
    entry->alias = alias;
    entry->status = UNDER_TEST;
    if (node_id == 0) {
        /* this is an anonymous request, use the interface Node ID */
        node_id = nodeID;
    }
    struct can_frame frame[4];

    control_init(frame[0], alias, (node_id >> 0) & 0xfff, 7);
    control_init(frame[1], alias, (node_id >> 12) & 0xfff, 6);
    control_init(frame[2], alias, (node_id >> 24) & 0xfff, 5);
    control_init(frame[3], alias, (node_id >> 36) & 0xfff, 4);
    SET_CAN_FRAME_EFF(frame[0]);
    SET_CAN_FRAME_EFF(frame[1]);
    SET_CAN_FRAME_EFF(frame[2]);
    SET_CAN_FRAME_EFF(frame[3]);
    CLR_CAN_FRAME_RTR(frame[0]);
    CLR_CAN_FRAME_RTR(frame[1]);
    CLR_CAN_FRAME_RTR(frame[2]);
    CLR_CAN_FRAME_RTR(frame[3]);
    CLR_CAN_FRAME_ERR(frame[0]);
    CLR_CAN_FRAME_ERR(frame[1]);
    CLR_CAN_FRAME_ERR(frame[2]);
    CLR_CAN_FRAME_ERR(frame[3]);

    int result = (*write)(fd, frame, sizeof(struct can_frame) * 4);
    HASSERT(result == (sizeof(struct can_frame) * 4));

    /* wait 200+ msec */
    entry->timer.start(MSEC_TO_NSEC(200));
}

/** Callback that is called when an upstream alias is kicked out of the cache.
 * @param id 48-bit NMRAnet Node ID
 * @param alias node alias
 * @param context pointer to an interface instance
 */
void IfCan::upstream_alias_removed(NodeID id, NodeAlias alias, void* context)
{
    IfCan* if_can = (IfCan*)context;

    struct can_frame frame;
    /* tell everyone to un-map our alias */
    control_init(frame, alias, AMR_FRAME, 0);
    SET_CAN_FRAME_EFF(frame);
    CLR_CAN_FRAME_RTR(frame);
    CLR_CAN_FRAME_ERR(frame);
    frame.can_dlc = 6;
    frame.data[0] = (id >> 40) & 0xff;
    frame.data[1] = (id >> 32) & 0xff;
    frame.data[2] = (id >> 24) & 0xff;
    frame.data[3] = (id >> 16) & 0xff;
    frame.data[4] = (id >> 8) & 0xff;
    frame.data[5] = (id >> 0) & 0xff;

    int result = (*if_can->write)(if_can->fd, &frame, sizeof(struct can_frame));
    HASSERT(result == sizeof(struct can_frame));
}

/** This is the timeout for claiming an alias.  At this point, the alias will
 * either be claimed as a downstream node, or we can start using it.
 * @param data1 a @ref NMRAnetCanIF typecast to a void*
 * @param data2 a @ref alias_node typecast to a void*
 * @return OS_TIMER_NONE
 */
long long IfCan::Pool::timeout(void* data1, void* data2)
{
    IfCan* if_can = (IfCan*)data1;
    Pool* entry = (Pool*)data2;

    if_can->mutex.lock();
    if (entry->status != UNDER_TEST) {
        /* try again with a new alias */
        /** @todo should we do this in a background thread? */
        entry->status = FREE;
        NodeAlias new_alias;
        do {
            new_alias = if_can->upstreamCache.generate();
        } while (if_can->downstreamCache.lookup(new_alias) != 0);

        /* note that this call will restart the timer.  Even though we don't
         * restart the timer on return from timeout, it will have already been
         * restarted, so we are good.
         */
        if_can->claim_alias(0, new_alias, entry);
    } else {
        entry->status = RESERVED;
        /* Reserve the ID */
        struct can_frame frame;
        control_init(frame, entry->alias, RID_FRAME, 0);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);

        int result
            = (*if_can->write)(if_can->fd, &frame, sizeof(struct can_frame));
        HASSERT(result == (sizeof(struct can_frame)));
    }
    if_can->mutex.unlock();

    /* do not restart the timer */
    return OS_TIMER_NONE;
}

/** Setup the relationship between an alias and a downstream node.  This method
 * must always be called with the mutex locked.
 * @param node_id Node ID
 * @return assigned alias
 */
NodeAlias IfCan::upstream_alias_setup(NodeID node_id)
{
    for (; /* forever */;) {
        for (unsigned int i = 0; i < ALIAS_POOL_SIZE; ++i) {
            if (pool[i].status == RESERVED) {
                NodeAlias alias = pool[i].alias;
                struct can_frame frame;

                /* add this mapping to the cache */
                upstreamCache.add(node_id, alias);

                /* Tell the segment who maps to this alias */
                control_init(frame, alias, AMD_FRAME, 0);
                SET_CAN_FRAME_EFF(frame);
                CLR_CAN_FRAME_RTR(frame);
                CLR_CAN_FRAME_ERR(frame);
                frame.can_dlc = 6;
                frame.data[0] = (node_id >> 40) & 0xff;
                frame.data[1] = (node_id >> 32) & 0xff;
                frame.data[2] = (node_id >> 24) & 0xff;
                frame.data[3] = (node_id >> 16) & 0xff;
                frame.data[4] = (node_id >> 8) & 0xff;
                frame.data[5] = (node_id >> 0) & 0xff;

                int result = (*write)(fd, &frame, sizeof(struct can_frame));
                HASSERT(result == sizeof(struct can_frame));

                /** @todo should we do this in a background thread? */
                pool[i].status = FREE;
                NodeAlias new_alias;
                do {
                    new_alias = upstreamCache.generate();
                } while (downstreamCache.lookup(new_alias) != 0);
                claim_alias(0, new_alias, pool + i);

                return alias;
            }
        }
        /* the pool is empty, wait and try again */
        mutex.unlock();
        sleep(1);
        mutex.lock();
        /* to prevent a race, make sure we didn't get a mapping slip in while
         * we were sleeping and unlocked.
         */
        NodeAlias test = upstreamCache.lookup(node_id);
        if (test) {
            return test;
        }
    }
}

/** Write a message onto the CAN bus.  The interface mutex should already
 * be locked.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int IfCan::if_write_locked(MTI mti, NodeID src, NodeHandle dst, Buffer* data)
{
    NodeAlias alias = upstreamCache.lookup(src);
    if (alias == 0) {
        /* we have never seen this node before, let's claim an alias for it */
        alias = upstream_alias_setup(src);
    }

    HASSERT(alias != 0);

    if (dst.alias != 0 || dst.id != 0) {
        /* we have an addressed message */
        if (dst.alias == 0) {
            /* look for a downstream match */
            dst.alias = downstreamCache.lookup(dst.id);
        }
        if (dst.alias) {
            int index = 0;
            size_t len;
            /* check to see if we have a stream or datagram */
            if (get_mti_datagram(mti)) {
                HASSERT(data != NULL);
                uint8_t* payload;

                /* datagrams and streams are special because the CAN ID
                 * also contains the destination alias.  These may also span
                 * multiple CAN frames.
                 */
                if (mti == MTI_STREAM_DATA) {
                    len = data->size() - data->available();
                    HASSERT(len > 2);
                    /* chop off the source and destination IDs */
                    len -= 2;
                    payload = (uint8_t*)data->start();
                } else {
                    Datagram::Message* m = (Datagram::Message*)data->start();
                    len = m->size;
                    payload = m->data;
                }
                for (int i = 0; len > 0; ++i) {
                    CanFrameType type;
                    if (mti == MTI_STREAM_DATA) {
                        type = STREAM_DATA;
                    } else if (i == 0 && len <= 8) {
                        type = DATAGRAM_ONE_FRAME;
                    } else if (i == 0) {
                        type = DATAGRAM_FIRST_FRAME;
                    } else if (len <= 8) {
                        type = DATAGRAM_FINAL_FRAME;
                    } else {
                        type = DATAGRAM_MIDDLE_FRAME;
                    }
                    struct can_frame frame;
                    set_fields(&frame.can_id, alias, (MTI)dst.alias, type,
                               NMRANET_MSG, NORMAL_PRIORITY);
                    SET_CAN_FRAME_EFF(frame);
                    CLR_CAN_FRAME_RTR(frame);
                    CLR_CAN_FRAME_ERR(frame);
                    size_t seg_size = len < 8 ? len : 8;
                    memcpy(frame.data, payload + index, seg_size);
                    frame.can_dlc = seg_size;

                    int result = (*write)(fd, &frame, sizeof(struct can_frame));
                    HASSERT(result == sizeof(struct can_frame));

                    len -= seg_size;
                    index += seg_size;
                }
            } else {
                if (data) {
                    len = data->size() - data->available();
                } else {
                    len = 0;
                }
                /* typically, only the simple node ident info reply will require
                 * the sending of more than one CAN frame, but who knows what
                 * the future might hold?
                 */
                do {
                    struct can_frame frame;
                    frame.can_id = can_identifier(mti, alias);
                    SET_CAN_FRAME_EFF(frame);
                    CLR_CAN_FRAME_RTR(frame);
                    CLR_CAN_FRAME_ERR(frame);
                    frame.data[0] = dst.alias >> 8;
                    frame.data[1] = dst.alias & 0xff;
                    size_t seg_size = len < 6 ? len : 6;
                    memcpy(&frame.data[2], (char*)data->start() + index,
                           seg_size);
                    frame.can_dlc = 2 + seg_size;
                    int result = (*write)(fd, &frame, sizeof(struct can_frame));
                    HASSERT(result == sizeof(struct can_frame));

                    len -= seg_size;
                    index += seg_size;

                } while (len > 0);

                /* release the buffer if we can,
                 * we don't release datagrams and streams
                 */
                if (data) {
                    data->free();
                }
            }
        } else {
            if (get_mti_address(mti)) {
                /* this is an addressed message, so we need to buffer while
                 * we determine what alias this node ID belongs to.  This
                 * should happen infrequently because we should have the most
                 * often addressed aliases cached.
                 */
                while (writeBuffer.in_use()) {
                    mutex.unlock();
                    usleep(300);
                    mutex.lock();
                }
                writeBuffer.setup(mti, src, dst, data);

                /* Verify Node ID Number Global */
                struct can_frame frame;
                frame.can_id
                    = can_identifier(If::MTI_VERIFY_NODE_ID_GLOBAL, alias);
                SET_CAN_FRAME_EFF(frame);
                CLR_CAN_FRAME_RTR(frame);
                CLR_CAN_FRAME_ERR(frame);
                frame.can_dlc = 6;
                frame.data[0] = (dst.id >> 40) & 0xff;
                frame.data[1] = (dst.id >> 32) & 0xff;
                frame.data[2] = (dst.id >> 24) & 0xff;
                frame.data[3] = (dst.id >> 16) & 0xff;
                frame.data[4] = (dst.id >> 8) & 0xff;
                frame.data[5] = (dst.id >> 0) & 0xff;
                int result = (*write)(fd, &frame, sizeof(struct can_frame));
                HASSERT(result == (sizeof(struct can_frame)));
            }
        }
    } else {
        /* we have an unaddressed message */
        struct can_frame frame;
        frame.can_id = can_identifier(mti, alias);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);
        if (data != NULL) {
            frame.can_dlc = data->size() - data->available();
            memcpy(frame.data, data->start(), frame.can_dlc);
            data->free();
        } else {
            frame.can_dlc = 0;
        }
        int result = (write)(fd, &frame, sizeof(struct can_frame));
        HASSERT(result == (sizeof(struct can_frame)));
    }

    return 0;
}

/** Decode global or addressed can frame.
 * @param can_id can identifier
 * @param dlc data length code
 * @param data pointer to up to 8 bytes of data
 */
void IfCan::global_addressed(uint32_t can_id, uint8_t dlc, uint8_t* data)
{
    NodeHandle src;
    src.alias = get_src(can_id);
    mutex.lock();
    src.id = downstreamCache.lookup(src.alias);

    if (get_mti_address(nmranet_mti(can_id))) {
        if (dlc < 2) {
            /** @todo should we do something else here? */
            /* soft error, we throw this one away */
            return;
        }
        /* addressed message */
        uint16_t addressed = (data[0] << 0) + (data[1] << 8);
        addressed = be16toh(addressed);
        NodeID dst = upstreamCache.lookup(get_addressed_destination(addressed));
        mutex.unlock();
        if (dst != 0) {
            Buffer* buffer;
            if (dlc > 2) {
                /* collect the data */
                buffer = buffer_alloc(dlc - 2);
                memcpy(buffer->start(), data, (dlc - 2));
                buffer->advance((dlc - 2));
            } else {
                buffer = NULL;
            }

            rx_data(nmranet_mti(can_id), src, dst, buffer);
        }
    } else {
        /* global message */
        if (nmranet_mti(can_id) == MTI_VERIFIED_NODE_ID_NUMBER) {
            int mapped = 0;
            NodeID node_id;
            node_id = data[5];
            node_id |= (node_id_t)data[4] << 8;
            node_id |= (node_id_t)data[3] << 16;
            node_id |= (node_id_t)data[2] << 24;
            node_id |= (node_id_t)data[1] << 32;
            node_id |= (node_id_t)data[0] << 40;

            if (src.id) {
                if (src.id != node_id) {
                    /* Looks like we have an existing mapping conflict.
                     * Lets remove existing mapping.
                     */
                    downstreamCache.remove(src.alias);
                } else {
                    mapped = 1;
                }
            }
            /* Normally, with a buffered write, we are looking for a CCR AMD
             * frame.  However, this message has what we need, so if we get it
             * first, let's go ahead and use it.
             */
            if (writeBuffer.in_use()) {
                /* we have buffered a write for this node */
                if (node_id == writeBuffer.dst.id) {
                    writeBuffer.dst.alias = src.alias;
                    if_write(writeBuffer.mti, writeBuffer.src, writeBuffer.dst,
                             writeBuffer.data);
                    if (mapped == 0) {
                        /* We obviously use this alias, so let's cache it
                         * for later use.
                         */
                        downstreamCache.add(node_id, src.alias);
                        writeBuffer.release();
                        mapped = 1;
                    }
                }
            } else if (src.id && src.id != node_id) {
                /* we already had this mapping, we need to replace it */
                downstreamCache.add(node_id, src.alias);
            }
#if 0
            if (can_if->lookup_id.alias == src.alias)
            {
                /* we are performing a Node ID lookup based on an alias 
                 * and we found a match
                 */
                can_if->lookup_id.id = node_id;
                can_if->lookup_id.alias = 0;
                os_timer_stop(&can_if->lookup_id.timer);
                os_sem_post(&can_if->lookup_id.sem);
                if (mapped == 0)
                {
                    /* We obviously use this alias, so let's cache it
                     * for later use.
                     */
                    nmranet_alias_add(can_if->aliasCache, node_id, src.alias);
                    write_buffer_release(can_if);
                    mapped = 1;
                }
            }
#endif
            /* update the source Node ID */
            src.id = node_id;
        }
        mutex.unlock();
        Buffer* buffer;
        if (dlc) {
            /* collect the data */
            buffer = buffer_alloc(dlc);
            memcpy(buffer->start(), data, dlc);
            buffer->advance(dlc);
        } else {
            buffer = NULL;
        }
        rx_data(nmranet_mti(can_id), src, 0, buffer);
    }
}

/** Send a datagram error from the receiver to the sender.
 * @param src source alias to send message from
 * @param dst destination alias to send message to
 * @param error_code error value to send
 */
void IfCan::datagram_rejected(NodeAlias src, NodeAlias dst, int error_code)
{
    /* no buffer available, let the sender know */
    struct can_frame frame;
    frame.can_id = can_identifier(If::MTI_DATAGRAM_REJECTED, src);
    SET_CAN_FRAME_EFF(frame);
    CLR_CAN_FRAME_RTR(frame);
    CLR_CAN_FRAME_ERR(frame);
    frame.data[0] = dst >> 8;
    frame.data[1] = dst & 0xff;
    frame.data[2] = (error_code >> 8) & 0xff;
    frame.data[3] = (error_code >> 0) & 0xff;
    frame.can_dlc = 4;

    int result = write(fd, &frame, sizeof(struct can_frame));
    HASSERT(result == sizeof(struct can_frame));
}

/** This is the timeout for giving up an incoming multi-frame datagram.
 * mapping request.
 * @param data1 a @ref IfCan typecast to a void*
 * @param data2 a @ref Buffer reference typecast to void*
 * @return OS_TIMER_NONE
 */
long long IfCan::datagram_timeout(void* data1, void* data2)
{
    IfCan* if_can = (IfCan*)data1;
    Buffer* buffer = (Buffer*)data2;
    Datagram::Message* m = (Datagram::Message*)buffer->start();

    if_can->mutex.lock();

    /** @todo currently we fail silently, should we throw an error? */
    /* we have to re-lookup the datagram buffer to prevent a race condition */
    if (if_can->datagramTree.find((m->to << 12) + m->from.alias)) {
        if_can->datagramTree.remove((m->to << 12) + m->from.alias);
        buffer->free();
    }

    if_can->mutex.unlock();

    /* delete our selves since we are no longer relevant */
    return OS_TIMER_DELETE;
}

/** Decode datagram can frame.
 * @param can_id can identifier
 * @param dlc data length code
 * @param data pointer to up to 8 bytes of data
 */
void IfCan::datagram(uint32_t can_id, uint8_t dlc, uint8_t* data)
{
    /* There is no need to use a mutex to access the datagram tree.  This is
     * because this function can only be called from a single thread, the
     * receive thread.  If this changes in the future, a mutex must be added
     * to protect this tree.
     */
    NodeHandle src;
    NodeAlias dst_alias = get_dst(can_id);

    mutex.lock();
    NodeID dst_id = upstreamCache.lookup(dst_alias);

    if (dst_id == 0) {
        /* nobody here by that ID, not for us */
        mutex.unlock();
        return;
    }

    src.alias = get_src(can_id);
    src.id = downstreamCache.lookup(src.alias);

    switch (get_can_frame_type(can_id)) {
        default:
            break;
        case DATAGRAM_ONE_FRAME: {
            Buffer* buffer = datagramPool.buffer_alloc(DATAGRAM_MESSAGE_SIZE);
            if (buffer == NULL) {
                /* no buffer available, let the sender know */
                datagram_rejected(dst_alias, src.alias,
                                  Datagram::BUFFER_UNAVAILABLE);
                break;
            }
            Datagram::Message* m = (Datagram::Message*)buffer->start();
            memcpy(m->data, data, dlc);
            m->to = dst_id;
            m->from.id = src.id;
            m->from.alias = src.alias;
            m->size = dlc;
            mutex.unlock();
            rx_data(nmranet_mti(can_id), src, dst_id, buffer);
            return;
        }
        case DATAGRAM_FIRST_FRAME: {
            RBTree<uint64_t, Buffer*>::Node* node
                = datagramTree.find((dst_id << 12) + src.alias);

            if (node) {
                /* we are already receiving a datagram, this is an error */
                datagram_rejected(dst_alias, src.alias, Datagram::OUT_OF_ORDER);
                datagramTree.remove(node);
                Datagram::Message* m = (Datagram::Message*)node->value->start();
                OSTimer* t = (OSTimer*)(m + 1);
                t->stop();
                node->value->free();
                break;
            }

            Buffer* buffer = datagramPool.buffer_alloc(DATAGRAM_MESSAGE_SIZE);
            if (buffer == NULL) {
                /* no buffer available, let the sender know */
                datagram_rejected(dst_alias, src.alias,
                                  Datagram::BUFFER_UNAVAILABLE);
                break;
            }

            Datagram::Message* m = (Datagram::Message*)buffer->start();
            OSTimer* t = (OSTimer*)(m + 1);

            memcpy(m->data, data, dlc);
            m->to = dst_id;
            m->from.id = src.id;
            m->from.alias = src.alias;
            m->size = dlc;

            HASSERT(datagramTree.insert((dst_id << 12) + src.alias, buffer)
                    != NULL);
            t->start(DATAGRAM_TIMEOUT);
            break;
        }
        case DATAGRAM_MIDDLE_FRAME: {
            RBTree<uint64_t, Buffer*>::Node* node
                = datagramTree.find((dst_id << 12) + src.alias);

            if (node == NULL) {
                /* we have no record of this datagram, this is an error */
                datagram_rejected(dst_alias, src.alias, Datagram::OUT_OF_ORDER);
                break;
            }

            Datagram::Message* m = (Datagram::Message*)node->value->start();
            OSTimer* t = (OSTimer*)(m + 1);

            if ((m->size + dlc) > Datagram::MAX_SIZE) {
                /* we have too much data for a datagram, this is an error */
                datagram_rejected(dst_alias, src.alias, Datagram::OUT_OF_ORDER);
                datagramTree.remove(node);
                Datagram::Message* m = (Datagram::Message*)node->value->start();
                OSTimer* t = (OSTimer*)(m + 1);
                t->stop();
                node->value->free();
                break;
            }
            memcpy(m->data + m->size, data, dlc);
            m->size += dlc;
            t->start(DATAGRAM_TIMEOUT);
            break;
        }
        case DATAGRAM_FINAL_FRAME: {
            RBTree<uint64_t, Buffer*>::Node* node
                = datagramTree.find((dst_id << 12) + src.alias);

            if (node == NULL) {
                /* we have no record of this datagram, this is an error */
                datagram_rejected(dst_alias, src.alias, Datagram::OUT_OF_ORDER);
                break;
            }

            Datagram::Message* m = (Datagram::Message*)node->value->start();
            OSTimer* t = (OSTimer*)(m + 1);
            t->stop();

            if ((m->size + dlc) > Datagram::MAX_SIZE) {
                /* we have too much data for a datagram, this is an error */
                datagram_rejected(dst_alias, src.alias, Datagram::OUT_OF_ORDER);
                datagramTree.remove(node);
                Datagram::Message* m = (Datagram::Message*)node->value->start();
                OSTimer* t = (OSTimer*)(m + 1);
                t->stop();
                node->value->free();
                break;
            }
            memcpy(m->data + m->size, data, dlc);
            m->size += dlc;
            datagramTree.remove(node);
            mutex.unlock();
            rx_data(nmranet_mti(can_id), src, dst_id, node->value);
            return;
        }
    }
    mutex.unlock();
}

/** Decode stream can frame.
 * @param can_id can identifier
 * @param dlc data length code
 * @param data pointer to up to 8 bytes of data
 */
void IfCan::stream(uint32_t can_id, uint8_t dlc, uint8_t* data)
{
    NodeHandle src;
    NodeAlias dst_alias = get_dst(can_id);

    mutex.lock();
    NodeID dst_id = upstreamCache.lookup(dst_alias);

    if (dst_id == 0) {
        /* nobody here by that ID, not for us */
        mutex.unlock();
        return;
    }

    src.alias = get_src(can_id);
    src.id = downstreamCache.lookup(src.alias);

    Buffer* buffer = buffer_alloc(dlc + 2);
    memcpy(buffer->start(), data, dlc);
    memset((char*)buffer->start() + dlc, 0, 2);
    buffer->advance(dlc + 2);

    rx_data(nmranet_mti(can_id), src, dst_id, buffer);
}

/** Test to see if the alias is in conflict with an alias we are using.
 * @param alias alias to look for conflict with
 * @param release we should release the alias if we have it reserved
 * @return 0 if no conflict found, else 1
 */
bool IfCan::alias_conflict(NodeAlias alias, bool release)
{
    bool conflict = false;

    mutex.lock();
    NodeID id = upstreamCache.lookup(alias);
    if (id) {
        /* we have this alias reserved, prevent a collision */
        if (release) {
            struct can_frame frame;
            /* tell everyone to un-map our alias */
            control_init(frame, alias, AMR_FRAME, 0);
            SET_CAN_FRAME_EFF(frame);
            CLR_CAN_FRAME_RTR(frame);
            CLR_CAN_FRAME_ERR(frame);
            frame.can_dlc = 6;
            frame.data[0] = (id >> 40) & 0xff;
            frame.data[1] = (id >> 32) & 0xff;
            frame.data[2] = (id >> 24) & 0xff;
            frame.data[3] = (id >> 16) & 0xff;
            frame.data[4] = (id >> 8) & 0xff;
            frame.data[5] = (id >> 0) & 0xff;

            int result = (*write)(fd, &frame, sizeof(struct can_frame));
            HASSERT(result == sizeof(struct can_frame));

            upstreamCache.remove(alias);
        }
        conflict = true;
    } else {
        for (unsigned int i = 0; i < ALIAS_POOL_SIZE; ++i) {
            switch (pool[i].status) {
                case FREE:
                /* fall through */
                case CONFLICT:
                    /* keep looking */
                    continue;
                case UNDER_TEST:
                    if (pool[i].alias == alias) {
                        pool[i].status = CONFLICT;
                    }
                    break;
                case RESERVED:
                    if (pool[i].alias == alias) {
                        if (release) {
                            pool[i].status = CONFLICT;
                            NodeAlias new_alias;
                            do {
                                new_alias = upstreamCache.generate();
                            } while (downstreamCache.lookup(new_alias) != 0);
                            claim_alias(0, new_alias, pool + i);
                        } else {
                            conflict = true;
                        }
                    }
                    break;
            }
            break;
        }
    }
    mutex.unlock();

    /* This assertion would mean that a recoverable alias conflict occured
     * because someone was behaving badly on the bus.
     */
    DASSERT(release == false || conflict == false);

    return conflict;
}

/** Decode Check ID CAN control frame.
 * @param ccr CAN control frame
 */
void IfCan::ccr_cid_frame(uint32_t ccr)
{
    NodeAlias alias = get_control_src(ccr);

    if (alias_conflict(alias, 0)) {
        /* remind everyone we own, or are trying to own, this alias with a
         * Reserve ID frame
         */
        struct can_frame frame;
        control_init(frame, alias, RID_FRAME, 0);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);

        int result = (*write)(fd, &frame, sizeof(struct can_frame));
        HASSERT(result == (sizeof(struct can_frame)));
    }
}

/** Decode Alias Map Definition CAN control frame.
 * @param ccr CAN control frame
 * @param data frame data representing the full 48-bit Node ID
 */
void IfCan::ccr_amd_frame(uint32_t ccr, uint8_t data[])
{
    NodeAlias alias = get_control_src(ccr);

    /* look for and resolve conflicts in Alias mappings */
    alias_conflict(alias, 1);

    NodeID node_id;

    node_id = data[5];
    node_id |= (node_id_t)data[4] << 8;
    node_id |= (node_id_t)data[3] << 16;
    node_id |= (node_id_t)data[2] << 24;
    node_id |= (node_id_t)data[1] << 32;
    node_id |= (node_id_t)data[0] << 40;

    mutex.lock();
    if (writeBuffer.in_use()) {
        if (node_id == writeBuffer.dst.id) {
            writeBuffer.dst.alias = alias;

            if_write_locked(writeBuffer.mti, writeBuffer.src, writeBuffer.dst,
                            writeBuffer.data);

            /* we obviously use this alias, so let's cache it for later use */
            downstreamCache.add(node_id, writeBuffer.dst.alias);
            writeBuffer.release();
        }
    }
    mutex.unlock();

    /* remove any in progress datagrams from this alias */
    // remove_datagram_in_progress(can_if, GET_CAN_CONTROL_FRAME_SOURCE(ccr));
}

/** Send an AMD frame for a given Node ID and Alias pair.
 * @param data context pointer
 * @param id Node ID
 * @param alias Node Alias
 */
void IfCan::send_amd_frame(void* data, NodeID id, NodeAlias alias)
{
    IfCan* if_can = (IfCan*)data;

    HASSERT(alias && id);

    /* Tell the segment who maps to this alias */
    struct can_frame frame;
    control_init(frame, alias, AMD_FRAME, 0);
    SET_CAN_FRAME_EFF(frame);
    CLR_CAN_FRAME_RTR(frame);
    CLR_CAN_FRAME_ERR(frame);
    frame.can_dlc = 6;
    frame.data[0] = (id >> 40) & 0xff;
    frame.data[1] = (id >> 32) & 0xff;
    frame.data[2] = (id >> 24) & 0xff;
    frame.data[3] = (id >> 16) & 0xff;
    frame.data[4] = (id >> 8) & 0xff;
    frame.data[5] = (id >> 0) & 0xff;

    int result = (*if_can->write)(if_can->fd, &frame, sizeof(struct can_frame));
    HASSERT(result == sizeof(struct can_frame));
}

/** Decode Alias Map Enquiry CAN control frame.
 * @param ccr CAN control frame
 * @param data frame data representing the full 48-bit Node ID
 */
void IfCan::ccr_ame_frame(uint32_t ccr, uint8_t data[])
{
    /* look for and resolve conflicts in Alias mappings */
    alias_conflict(get_control_src(ccr), 1);

    if (data) {
        NodeID node_id;

        node_id = data[5];
        node_id |= (node_id_t)data[4] << 8;
        node_id |= (node_id_t)data[3] << 16;
        node_id |= (node_id_t)data[2] << 24;
        node_id |= (node_id_t)data[1] << 32;
        node_id |= (node_id_t)data[0] << 40;

        mutex.lock();
        NodeAlias alias = upstreamCache.lookup(node_id);
        if (alias) {
            send_amd_frame(this, node_id, alias);
        }
    } else {
        mutex.lock();
        upstreamCache.for_each(send_amd_frame, this);
    }
    mutex.unlock();
}

/** Decode Alias Map Reset CAN control frame.
 * @param ccr CAN control frame
 * @param data frame data representing the full 48-bit Node ID
 */
void IfCan::ccr_amr_frame(uint32_t ccr, uint8_t data[])
{
    NodeAlias alias = get_control_src(ccr);
    NodeID node_id;

    node_id = data[5];
    node_id |= (node_id_t)data[4] << 8;
    node_id |= (node_id_t)data[3] << 16;
    node_id |= (node_id_t)data[2] << 24;
    node_id |= (node_id_t)data[1] << 32;
    node_id |= (node_id_t)data[0] << 40;

    /* look for and resolve conflicts in Alias mappings */
    alias_conflict(alias, 1);

    mutex.lock();
    downstreamCache.remove(alias);
    downstreamCache.remove(node_id);
    mutex.unlock();

    /* remove any in progress datagrams from this alias */
    // remove_datagram_in_progress(can_if, GET_CAN_CONTROL_FRAME_SOURCE(ccr));
}

/** Thread for reading the data from the interface.
 * @param data pointer to an IfCan instance
 * @return NULL, should never return
 */
void* IfCan::read_thread(void* data)
{
    for (; /* forever */;) {
        struct can_frame frame;
        CLR_CAN_FRAME_ERR(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_EFF(frame);

        int result = (*read)(fd, &frame, sizeof(struct can_frame));
        HASSERT(result == sizeof(struct can_frame));

        /* OpenLCB doesn't care about standard frames. */
        if (!IS_CAN_FRAME_EFF(frame)) {
            continue;
        }

        /* address any abnormalities */
        if (IS_CAN_FRAME_ERR(frame) || IS_CAN_FRAME_RTR(frame)) {
            /** @todo (Stuart Baker) do we need to dump any aliases under test
             * if we get an error frame?
             */
            continue;
        }

        if (get_frame_type(frame.can_id) == CONTROL_MSG) {
            switch (get_control_sequence(frame.can_id)) {
                default:
                    /* this is another protocol, let's grab the next frame */
                    continue;
                case 0x4:
                /* fall through */
                case 0x5:
                /* fall through */
                case 0x6:
                /* fall through */
                case 0x7:
                    ccr_cid_frame(frame.can_id);
                    /* we are done decoding, let's grab the next frame */
                    continue;
                case 0x0:
                    switch (get_control_field(frame.can_id)) {
                        default:
                            /* unknown field, let's grab the next frame */
                            continue;
                        case RID_FRAME:
                            ccr_rid_frame(frame.can_id);
                            break;
                        case AMD_FRAME:
                            ccr_amd_frame(frame.can_id, frame.data);
                            break;
                        case AME_FRAME:
                            ccr_ame_frame(frame.can_id, (frame.can_dlc == 0)
                                                            ? NULL
                                                            : frame.data);
                            break;
                        case AMR_FRAME:
                            ccr_amr_frame(frame.can_id, frame.data);
                            break;
                    } /* switch (GET_CAN_CONTROL_FRAME_FIELD(frame.can_id)) */
                    break;
            } /* switch (GET_CAN_CONTROL_FRAME_SEQUENCE(frame.can_id)) */
        }     /* if (GET_CAN_ID_FRAME_TYPE(frame.can_id) == 0) */
        else {
            if (alias_conflict(get_control_src(frame.can_id), 1)) {
                /* there was a conflict in the alias mappings */
                continue;
            }
            /** find the proper packet decoder */
            switch (get_can_frame_type(frame.can_id)) {
                default:
                    break;
                case GLOBAL_ADDRESSED:
                    global_addressed(frame.can_id, frame.can_dlc, frame.data);
                    break;
                case DATAGRAM_ONE_FRAME:
                /* fall through */
                case DATAGRAM_FIRST_FRAME:
                /* fall through */
                case DATAGRAM_MIDDLE_FRAME:
                /* fall through */
                case DATAGRAM_FINAL_FRAME:
                    datagram(frame.can_id, frame.can_dlc, frame.data);
                    break;
                case STREAM_DATA:
                    stream(frame.can_id, frame.can_dlc, frame.data);
                    break;
            } /* switch(GET_CAN_ID_CAN_FRAME_TYPE(frame.can_id) */
        }     /* if (GET_CAN_ID_FRAME_TYPE(frame.can_id) == 0), else */
    }         /* for ( ; forever ; ) */
    return NULL;
}

}; /* namespace NMRAnet */
