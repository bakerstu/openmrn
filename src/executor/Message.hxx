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
 * \file Message.cxx
 *
 * Defines Message type used within Executor, Service, and ControlFlow.
 *
 * @author Stuart W Baker
 * @date 25 December 2013
 */

#ifndef _Message_hxx_
#define _Message_hxx_

#include "utils/BufferQueue.hxx"

/** Buffer with additional fields to represent a complete message.
 */
class Message : public BufferManager
{
public:
    /** Enumeration constants for manipulating 32-bit identifiers
     */
    enum ID
    {
        ID_VALUE_MSK   = 0x7FFFFFFF, /**< Mask off id value */
        IN_PROCESS_MSK = 0x80000000  /**< Mask off the in process bit */
    };
    
    /** Free this Message to the Pool from whence it came.
     */
    inline void free();

    /** Expand the buffer size.  Exercise caution when using this API.  If anyone
     * else is holding onto a reference of this, their reference will be corrupted.
     * @param size size buffer after expansion.
     * @return newly expanded buffer with old buffer data moved
     */
    inline Message *expand(size_t size);

    /** Set the directed to for the buffer.
     * @param object the object directed to.
     */
    void to(void *object)
    {
        to_ = object;
    }

    /** Get the directed to for the buffer.
     * @return the object directed to.
     */
    void *to()
    {
        return to_;
    }

    /** Set the directed to for the buffer.
     * @param object the object directed to.
     */
    void from(void *object)
    {
        from_ = object;
    }

    /** Get the directed to for the buffer.
     * @return the object directed to.
     */
    void *from()
    {
        return from_;
    }
    
    /** Set the unique identifier for the buffer.
     * @param identifier 32-bit unique identifier
     */
    void id(uint32_t identifier)
    {
        id_ = identifier;
    }

    /** Get the unique identifier for the buffer.
     * @return 32-bit unique identifier
     */
    uint32_t id()
    {
        return id_;
    }
    
    /** Get a pointer to the pool that this buffer belongs to.
     * @return pool that this buffer belongs to
     */
    Pool<Message> *pool()
    {
        return pool_;
    }
    
    /** The total size of an array element of a Message for given payload.
     * @param size payload size
     */
    static size_t sizeof_type(size_t size)
    {
        return sizeof(Message) + (((size/sizeof(long)) + (size % sizeof(long) ? 1 : 0)) * sizeof(long));
    }

    /** Like a constructor, but in this case, we allocate extra space for the
     * user data.
     * @param pool Pool instance from which this buffer will come
     * @param size size of user data in bytes
     * @param items number of items to allocate
     * @return newly allocated buffer or addres of first item in an array of
     *         allocated buffers, HASSERT() on failure
     */
    static Message *alloc(Pool<Message> *pool, size_t size, size_t items = 1)
    {
        HASSERT(pool != NULL);

        size_t align_size = sizeof_type(size);
        Message *msg = (Message*)malloc(align_size * items);
        Message *result = msg;
        for (size_t i = 0; i < items; ++i)
        {
            new (msg) Message(size, pool);
            msg = (Message*)((char*)msg + align_size);
        }
        return result;
    }

    /** Like a constructor, but in this case, we re-purpose an existing buffer
     * with no new memory allocation.
     * @param buffer instance of buffer to reinitialize
     * @param size size of user data in bytes
     * @return newly reinitialized buffer, HASSERT() on failure
     */
    static Message *init(Message *msg, size_t size)
    {
        HASSERT(msg->pool_ != NULL);
        HASSERT(msg->size_ == size);
        BufferManager::init(msg, size);
        msg->to_ = NULL;
        msg->from_ = NULL;
        msg->id_ = 0;
        return msg;
    }

private:
    /** get a pointer to the start of the data.
     */
    char *data()
    {
        return data_;
    }

    /** pointer to Pool instance that this buffer belongs to */
    Pool<Message> *pool_;

    /** who this message is directed to */
    void *to_;
    
    /** who this message is directed from */
    void *from_;
    
    /** message ID for uniquely identifying this message in a queue.  The lower
     * 31 bits identify the message, the upper 32'nd bit is set if the 
     * message is already being processed.
     */
    uint32_t id_;
    
    /** message payload */
    char data_[];
    
    /** This class is a helper of Pool */
    template <class T> friend class Pool;
        
    /** Constructor.
     * @param size size of Message data in bytes
     * @param pool pool that this buffer belongs to
     */
    Message(size_t size, Pool<Message> *pool)
        : BufferManager(size),
          pool_(pool),
          to_(NULL),
          from_(NULL),
          id_(0)
    {
    }
    
    /** Destructor.
     */
    ~Message()
    {
    }

    /** Default Constructor.
     */
    Message();

    DISALLOW_COPY_AND_ASSIGN(Message);
};

#if 0
/** Pool of previously allocated, but currently unused, Messages. */
class MessagePool : public Pool <Message>
{
public:
    /* Default Constructor */
    MessagePool()
        : Pool<Message>()
    {
    }

    /* Constructor for a fixed size pool.
     * @param item_size size of each item in the pool
     * @param items number of items in the pool
     */
    MessagePool(size_t item_size, size_t items)
        : Pool<Message>(item_size, items)
    {
    }

    /* default destructor */
    ~MessagePool()
    {
    }
    
private:
    DISALLOW_COPY_AND_ASSIGN(MessagePool);
};
#endif
/** Free this buffer to the BufferPool from whence it came.
 */
inline void Message::free()
{
    HASSERT(pool_ != NULL);
    pool_->free(this);
}

/** Expand the buffer size.  Exercise caution when using this API.  If anyone
 * else is holding onto a reference of this, their reference will be corrupted.
 * @param size size buffer after expansion.
 * @return newly expanded buffer with old buffer data moved
 */
inline Message *Message::expand(size_t size)
{
    Message *new_msg = pool_->alloc(size);
    
    memcpy(new_msg->data(), data(), size_ - left);
    new_msg->left = (size - size_) + left;
    pool_->free(this);
    return new_msg;
}

/** main message pool instance */
extern DynamicPool<Message> *mainMessagePool;

#endif /* _Message_hxx_ */
