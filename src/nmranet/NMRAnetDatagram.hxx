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
 * \file NMRAnetDatagram.hxx
 * This file provides the NMRAnet datagram implementation.
 *
 * @author Stuart W. Baker
 * @date 5 October 2013
 */

#ifndef _NMRAnetDatagram_hxx_
#define _NMRAnetDatagram_hxx_

#include "nmranet/NMRAnetIf.hxx"

namespace NMRAnet
{

class Node;

/** Implement the Datagram Service singleton */
class DatagramService : public Service
{
private:
    /** Message ID's that we can receive */
    enum MessageId
    {
        INCOMING = NMRANET_DATAGRAM_BASE,
        DATAGRAM_REJECTED,
        DATAGRAM_OK,
        DATAGRAM
    };
    
    /** Constructor.
     */
    DatagramService()
        : Service(nmranetExecutor),
          incoming(this)
    {
    }
    
    /** Destructor.
     */
    ~DatagramService()
    {
    }
    
    /** Translate an incoming Message ID into a StateFlow instance.
     * @param id itentifier to translate
     * @return StateFlow corresponding the given ID, NULL if not found
     */
    StateFlowBase *lookup(uint32_t id)
    {
        switch (id)
        {
            default:
                break;
            case INCOMING:
                return &incoming;
            case DATAGRAM_REJECTED:
            case DATAGRAM_OK:
            case DATAGRAM:
                break;
        }
        return NULL;
    }
    
    
    STATE_FLOW_START(Incoming, 4)
    STATE_FLOW_END()
    
    Incoming incoming;
    
    friend class Datagram;
};

/** Access to NMRAnet datagram logic.  Note!!!  @ref Datagram should only ever
 * be inherited by the @ref Node class.  In the @ref Datagram implementation
 * we make some assumptions that a Datagram* is equal to a Node*.
 */
class Datagram
{
public:
    /** Initialize the Datagram protocol.  This should not be called by user
     * code.  It should only be referenced through @ref nmranet_init.
     */
    static void init()
    {
        service_ = new DatagramService();
    }
    
    /** Get a reference to the datagram service.
     * @return reference to datagram service
     */
    static Service *service()
    {
        return service_;
    }
    
    /** total number of datagrams to allocate to the datagram memory pool */
    static const size_t POOL_SIZE;

    /** Thread stack size for handling platform level datagrams */
    static const size_t THREAD_STACK_SIZE;

    /** All known datagram protocols */
    enum Protocol
    {
        LOG_REQUEST   = 0x01, /**< request a placement into the log */
        LOG_REPLY     = 0x02, /**< reply to a @ref LOG_REQUEST */
        CONFIGURATION = 0x20, /**< configuration message */
        REMOTE_BUTTON = 0x21, /**< remote button input */
        DISPLAY       = 0x28, /**< place on display */
        TRAIN_CONTROL = 0x30, /**< operation of mobile nodes */
    };

    /** Allocate and prepare a datagram buffer.
     * @param protocol datagram protocol to use
     * @param size max length of data in bytes
     * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
     * @return Message handle upon success, else NULL on error with errno set
     */
    Buffer *buffer_get(uint64_t protocol, size_t size, long long timeout);

    /** Fill an already allocated datagram buffer.
     * @param buffer handle to buffer which is to hold datagram
     * @param protocol datagram protocol to use
     * @param data datagram to fill into datagram
     * @param offset offset within datagram payload to start filling
     * @param size length of data in bytes to fill
     */
    void buffer_fill(Buffer *buffer, uint64_t protocol, const void *data, size_t offset, size_t size);
    
    /** Produce a Datagram from a given node using a pre-allocated buffer.
     * @param dst destination node id or alias
     * @param datagram datagram to produce
     * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
     * @return 0 upon success, else -1 on error with errno set
     */
    int produce(NodeHandle dst, Buffer *buffer, long long timeout);

    /** Produce a Datagram from a given node.
     * @param dst destination node id or alias
     * @param protocol datagram protocol to use
     * @param data datagram to produce
     * @param size length of data in bytes
     * @param timeout time in nanoseconds to keep trying, 0 = do not wait, OS_WAIT_FOREVER = blocking
     * @return 0 upon success, else -1 on error with errno set
     */
    int produce(NodeHandle dst, uint64_t protocol, const void *data, size_t size, long long timeout);

    /** Determine the datagram protocol (could be 1, 2, or 6 bytes).
     * @param data buffer pointer to the beginning of the datagram
     * @return protocol type
     */
    static uint64_t protocol(Buffer *data);
    
    /** Determine the datagram payload
     * @param data buffer pointer to the beginning of the datagram
     * @return pointer to payload as a uint8_t array, assume byte alignment
     */
    static uint8_t *payload(Buffer *data);
    
    /** Acknowledge the successful receipt of a Datagram.
     * @param buffer reference to the buffer containing the received datagram
     * @param flags @ref Flags associated with the reply
     * @param free buffer automatically freed upon return if true, else buffer preserved
     */
    void received_okay(Buffer *buffer, uint8_t flags, bool free = true);

    /** Reject the unsuccessful receipt of a Datagram.
     * @param buffer reference to the buffer containing the received datagram
     * @param flags @ref Flags associated with the reply
     * @param free buffer automatically freed upon return if true, else buffer preserved
     */
    void rejected(Buffer *buffer, uint16_t error, bool free = true);

    /** Various public datagram constants. */
    enum
    {
        MAX_SIZE = 72, /**< maximum size in bytes of a datagram */
    };
    
    /** Possible flags for a successful receipt (received okay) of a Datagram.
     */
    enum Flag
    {
        FLAGS_NONE    = 0x00, /**< no flags set */
        REPLY_PENDING = 0x80, /**< A reply is pending */
        
        TIMEOUT_NONE  = 0x00, /**< no timeout */
        TIMEOUT_2     = 0x01, /**< 2 second timeout */
        TIMEOUT_4     = 0x02, /**< 4 second timeout */
        TIMEOUT_8     = 0x03, /**< 8 second timeout */
        TIMEOUT_16    = 0x04, /**< 16 second timeout */
        TIMEOUT_32    = 0x05, /**< 32 second timeout */
        TIMEOUT_64    = 0x06, /**< 64 second timeout */
        TIMEOUT_128   = 0x07, /**< 128 second timeout */
        TIMEOUT_256   = 0x08, /**< 256 second timeout */
        TIMEOUT_512   = 0x09, /**< 512 second timeout */
        TIMEOUT_1024  = 0x0A, /**< 1024 second timeout */
        TIMEOUT_2048  = 0x0B, /**< 2048 second timeout */
        TIMEOUT_4096  = 0x0C, /**< 4096 second timeout */
        TIMEOUT_8192  = 0x0D, /**< 8192 second timeout */
        TIMEOUT_16384 = 0x0E, /**< 16384 second timeout */
        TIMEOUT_32768 = 0x0F, /**< 32768 second timeout */
        
        TIMEOUT_MASK  = 0x0F, /**< Mask for reply timeout */
    };
    
    /** Possible error codes for a rejected datagram.
     */
    enum Error
    {
        RESEND_OK          = 0x2000, /**< We can try to resend the datagram. */
        TRANSPORT_ERROR    = 0x6000, /**< Transport error occurred. */
        BUFFER_UNAVAILABLE = 0x2020, /**< Buffer unavailable error occurred. */
        OUT_OF_ORDER       = 0x2040, /**< Out of order error occurred. */
        PERMANENT_ERROR    = 0x1000, /**< Permanent error occurred. */
        SRC_NOT_PERMITTED  = 0x1020, /**< Source not permitted error occurred. */
        NOT_ACCEPTED       = 0x1040, /**< Destination node does not accept datagrams of any kind. */
    };
    
    /** A datagram message structure. */
    struct Message
    {
        NodeID to; /**< the node that the datagram is to, only used in automatic processing */
        NodeHandle from; /**< node id or alias this datagram is from */
        size_t size; /**< size of datagram in bytes */
        union
        {
            uint8_t flags; /**< flags field for Received Okay response */
            uint16_t error; /**< error field for Rejected response */
        };
        uint8_t data[MAX_SIZE]; /**< datagram payload */
    };

protected:
    /** Constructor. */
    Datagram(Node *node);

    /** Default destructor */
    ~Datagram()
    {
    }

    /** Post the reception of a datagram to given node.
     * @param mti Message Type Indicator
     * @param src source Node ID
     * @param data datagram to post
     */
    void packet(Defs::MTI mti, NodeHandle src, Buffer *data);

private:
    /** Write a message from a node.  We should not have a mutex lock at
     * this at this point.
     * @param mti Message Type Indicator
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    virtual int write_unlocked(Defs::MTI mti, NodeHandle dst, Buffer *data) = 0;

    /** Write a message from a node.  We should already have a mutex lock at this
     * at this point.
     * @param mti Message Type Indicator
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    virtual int write(Defs::MTI mti, NodeHandle dst, Buffer *data) = 0;

    /** The parent class needs to provide a method for getting NMRAnet ID.
     */
    virtual NodeID id() = 0;

     /** Get handle to the receive queue for incoming NMRAnet messages.
     * @return handle to queue
     */
    virtual BufferQueueWait *rx_queue() = 0;

    /** Various datagram constants. */
    enum
    {
        PROTOCOL_SIZE_2    = 0xE0, /**< possible return value for @ref protocol_size */
        PROTOCOL_SIZE_6    = 0xF0, /**< possible return value for @ref protocol_size */
        PROTOCOL_SIZE_MASK = 0xF0, /**< mask used when determining protocol size */
    };

    /** We can try to resend the datagram.
     * @param error error number
     * @return true or false
     */
    static bool resend_ok(uint16_t error)
    {
        return error & RESEND_OK;
    }

    /** Transport error occurred.
     * @param error error number
     * @return true or false
     */
    bool transport_error(uint16_t error)
    {
        return error & TRANSPORT_ERROR;
    }

    /** Buffer unavailable error occurred.
     * @param error error number
     * @return true or false
     */
    bool buffer_unavailable(uint16_t error)
    {
        return error & BUFFER_UNAVAILABLE;
    }
#if 0
    /** Out of order error occurred.
     * @param _err error number
     * @return true or false
     */
    #define IS_DATAGRAM_OUT_OF_ORDER(_err)       (_err & 0x2040)

    /** Permanent error occurred.
     * @param _err error number
     * @return true or false
     */
    #define IS_DATAGRAM_PERMANENT_ERROR(_err)    (_err & 0x1000)

    /** Source not permitted error occurred.
     * @param _err error number
     * @return true or false
     */
    #define IS_DATAGRAM_SRC_NOT_PERMITTED(_err)  (_err & 0x1020)

    /** Destination node does not accept datagrams of any kind.
     * @param _err error number
     * @return true or false
     */
    #define IS_DATAGRAM_NOT_ACCEPTED(_err)        (_err & 0x1040)
#endif
    /** Determine if the protocol ID is represented by one, two, or six bytes.
     * @param _protocol protocol ID to interrogate
     * @return number of bytes representing the protocol
     */
    unsigned int protocol_size(uint64_t protocol)
    {
        return (((protocol & PROTOCOL_SIZE_MASK) == PROTOCOL_SIZE_6) ? 6 :
                ((protocol & PROTOCOL_SIZE_MASK) == PROTOCOL_SIZE_2) ? 2 : 1);
    }

    /** Release a buffer back to the datagram buffer pool.
     * @param buffer buffer to release
     */
    void buffer_release(Buffer *buffer);
    
    /** Process the datagram automatically.
     * @param data buffer to process
     * @return 0 if we do not have an automated processing option, else return 1
     */
    int process(Buffer *data);

    /** Timeout handler for datagram timeouts.
     * @param data1 instance of @ref Datagram cast to a void*
     * @param data2 unused
     * @return restart credentials for timer
     */
    static long long timeout(void *data1, void *data2);
    
    /** One time initialization.
     */
    static void one_time_init();
    
    /** Thread for handling platform datagrams.
     * @param arg unused
     * @return should never return
     */
    static void *thread(void *arg);

    /** Allocate a datagram from the pool.
     * @return datagram allocated, or NULL if there are no more datagrams available.
     */
    Message *alloc(void);
    
    /** the buffer pool used for all datagrams */
    static FixedPool<Buffer> pool;
    
    /** A queue of unused buffers */
    static BufferQueueWait dq;
    
    /** A queue of datagrams that the platform will handle */
    static BufferQueueWait pending;
    
    /** the currently in flight message for this instance */
    Buffer *txMessage;
    
    /** timer for in flight datagram timeouts */
    OSTimer timer;
    
    /** one time initialization structure */
    static OSThreadOnce once;

    /** Service instance for handling datagram messages */
    static DatagramService *service_;

    DISALLOW_COPY_AND_ASSIGN(Datagram);
    
    friend class DatagramService;
};

}

#endif /* _NMRAnetDatagram_hxx_ */

