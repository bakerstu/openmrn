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
 * \file NMRAnetStream.hxx
 * This file defines NMRAnet streams.
 *
 * @author Stuart W. Baker
 * @date 20 October 2013
 */

#ifndef _NMRAnetStream_hxx_
#define _NMRAnetStream_hxx_

#include "nmranet/NMRAnetIf.hxx"
#include "utils/RBTree.hxx"
#include "utils/RingBuffer.hxx"

namespace NMRAnet
{

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
    StreamHandle Open(NodeHandle dst, long long timeout);

protected:
    /** Default constructor. */
    Stream()
        : outboundTree(),
          inboundTree(),
          count(0),
          sem()
    {
    }
    
    /** Default destructor. */
    ~Stream()
    {
    }

    /** Handle incoming stream messages.
     * @param mti Message Type Indicator
     * @param src source Node ID
     * @param data datagram to post
     */
    void packet(If::MTI mti, NodeHandle src, Buffer *data);

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
    
    /** Stream metadata, often cast to @ref StreamHandle.
     */
    struct Metadata
    {
        Buffer *ping;  /**< ping buffer */
        Buffer *pong;  /**< pong buffer */
        size_t  size;  /**< max buffer size */
        State   state; /**< Stream state */
        uint8_t srcID; /**< unique source ID */
        uint8_t dstID; /**< unique destination ID */
    };

    /** Write a message from a node.  We should already have a mutex lock at this
     * at this point.
     * @param mti Message Type Indicator
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    virtual int write(If::MTI mti, NodeHandle dst, Buffer *data) = 0;

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

};

#endif /* _NMRAnetStream_hxx_ */
