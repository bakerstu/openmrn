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
 * \file Stream.hxx
 * This file defines NMRAnet streams.
 *
 * @author Stuart W. Baker
 * @date 20 October 2013
 */

#ifndef _OPENLCB_STREAM_HXX_
#define _OPENLCB_STREAM_HXX_

#include "openlcb/NMRAnetIf.hxx"
#include "utils/RBTree.hxx"
#include "utils/RingBuffer.hxx"

namespace openlcb
{

/**
   Base service for OpenLCB streaming protocol.

   This code does not work yet.
 */
class Stream
{
public:
    /** number of simultaneous open streams per virtual node instance */
    static const size_t CHANNELS_PER_NODE;
    
    /** Max buffer size for stream transmission segments */
    static const uint16_t MAX_BUFFER_SIZE;

    /** Stream handle type */
    typedef void *StreamHandle;
    
    /** Open a stream.
     * @param dst destination node for stream
     * @param timeout 0 = do not wait for success, deliver success asynchronously
     *                > 0, wait for success, or fail on timeout
     * @return NULL on fail, else stream handle on success or pending success
     */
    StreamHandle sopen(NodeHandle dst, long long timeout);

    /** Close a stream.
     * @param handle handle to stream to close
     */
    void sclose(StreamHandle handle);

    /** Write data to a stream.
     * @param handle handle to stream to write to
     * @param buf buffer to copy data from
     * @param size size in bytes to write to stream
     * @return number of bytes written, else -1 on error with errno set to indicate error
     */
    ssize_t swrite(StreamHandle handle, const void *buf, size_t size);

    /** Read data from a stream.
     * @param handle handle to stream to read from
     * @param buf buffer to copy data to
     * @param size size in bytes to read from stream
     * @return number of bytes read, else -1 on error with errno set to indicate error
     */
    ssize_t sread(StreamHandle handle, void *buf, size_t size);

protected:
    /** Default constructor.
     */
    Stream()
        : outboundTree(),
          inboundTree(),
          count(0),
          sem()
    {
    }
    
    /** Default destructor.
     */
    ~Stream()
    {
    }

    /** Handle incoming stream messages.
     * @param mti Message Type Indicator
     * @param src source Node ID
     * @param data datagram to post
     */
    void packet(Defs::MTI mti, NodeHandle src, Buffer *data);

private:
    /** possible flag values.
     */
    enum Flag
    {
        PERMINATE_ERROR = 0x40, /**< a perminate error occured */
        ACCEPT          = 0x80  /**< initiate request accepted */
    };
    
    /** Possible stream states.
     */
    enum State
    {
        PENDING_O,   /**< connection pending, outbound */
        WAITING_O,   /**< waiting on a pending connection, outbound */
        CONNECTED_O, /**< connection made, outbound */
        CLOSED_O,    /**< connection closed, outbound */
        PENDING_I,   /**< connection pending, inbound */
        WAITING_I,   /**< waiting on a pending connection, inbound */
        CONNECTED_I, /**< connection made, inbound */
        CLOSED_I,    /**< connection closed, inbound */
        
    };
    
    enum Protocol
    {
        DISCOVER_1     = 0x0600000000000000,
        DISCOVER_2     = 0x0500000000000000,
        DISCOVER_3     = 0x0400000000000000,
        DISCOVER_4     = 0x0300000000000000,
        DISCOVER_5     = 0x0200000000000000,
        DISCOVER_6     = 0x0100000000000000,
        DISCOVERED     = 0x0000000000000000,
        DISCOVERED_DEC = 0x0100000000000000,
        DISCOVER_MASK  = 0xFF00000000000000,
        DISCOVER_SHIFT = 56,
    };
    
    /** Stream metadata, often cast to @ref StreamHandle.
     */
    struct Metadata
    {
        uint64_t protocol; /**< stream protocol */
        RingBuffer<uint8_t> *data; /**< stream data */
        NodeHandle  nodeHandle; /**< node we are communicating with */

        size_t  size;   /**< max buffer size */
        size_t  count;  /**< count received before proceed required */
        State   state;  /**< Stream state */
        uint8_t srcID;  /**< unique source ID */
        uint8_t dstID;  /**< unique destination ID */
    };

    /** Write a message from a node.  We should already have a mutex lock at this
     * at this point.
     * @param mti Message Type Indicator
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    virtual int write(Defs::MTI mti, NodeHandle dst, Buffer *data) = 0;

     /** Get handle to the receive queue for incoming NMRAnet messages.
     * @return handle to queue
     */
    virtual BufferQueueWait *rx_queue() = 0;

    /** Handle incoming stream initiate request messages.
     * @param src source Node ID
     * @param data datagram to post
     */
    void initiate_request(NodeHandle src, Buffer *buffer);

    /** Handle incoming stream initiate reply messages.
     * @param src source Node ID
     * @param data datagram to post
     */
    void initiate_reply(NodeHandle src, Buffer *buffer);

    /** Handle incoming stream data messages.
     * @param src source Node ID
     * @param data datagram to post
     */
    void handle_data(NodeHandle src, Buffer *buffer);

    /** Handle incoming stream proceed messages.
     * @param src source Node ID
     * @param data datagram to post
     */
    void proceed(NodeHandle src, Buffer *buffer);

    /** Handle incoming stream complete messages.
     * @param src source Node ID
     * @param data datagram to post
     */
    void complete(NodeHandle src, Buffer *buffer);
    
    /** tree for keeping track of outbound streams */
    RBTree <uint8_t, Metadata*> outboundTree;

    /** tree for keeping track of inbound streams */
    RBTree <uint8_t, Metadata*> inboundTree;

    /** number of active streams for this node */
    size_t count;

    /** wait for completion semaphore */
    OSSem sem;

    DISALLOW_COPY_AND_ASSIGN(Stream);
};

/// unused.
struct IdStreamType
{
    Stream::StreamHandle stream;
};


};

#endif // _OPENLCB_STREAM_HXX_
