/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * \file DataBuffer.hxx
 *
 * Specialization of the Buffer<> class for storing untyped data payloads;
 * allocation and Pool objects that match this behavior.
 *
 * @author Balazs Racz
 * @date 17 Feb 2020
 */

#ifndef _UTILS_DATABUFFER_HXX_
#define _UTILS_DATABUFFER_HXX_

#include "utils/Buffer.hxx"

class DataBufferPool;

/// Specialization of the Buffer class that is designed for storing untyped
/// data arrays. Adds the ability to treat the next pointers as links to
/// consecutive data bytes, ref'ing and unref'ing a sequence of buffers in one
/// go.
class DataBuffer : public Buffer<uint8_t[]>
{
public:
    /// Overrides the size of the data buffer. The semantic meaning of the
    /// size() of DataBuffer is the number of bytes that are filled in inside
    /// data().
    /// @param s new value of size.
    void set_size(uint16_t s)
    {
        size_ = s;
    }

    /// Sets the linking pointer of the DataBuffer to a target buffer.
    /// @param n next buffer to link to.
    void set_next(DataBuffer *n)
    {
        QMember::next = n;
    }

    /// @return the pointer to the next chunk of buffer.
    DataBuffer *next()
    {
        return static_cast<DataBuffer *>(QMember::next);
    }

    /// @return the payload pointer, cast to a convenient type.
    uint8_t *data()
    {
        return static_cast<uint8_t *>(&data_[0]);
    }

    /// @return a new reference to *this.
    DataBuffer *ref()
    {
        return static_cast<DataBuffer *>(Buffer<uint8_t[]>::ref());
    }

    /// Acquires one reference to all blocks of this buffer.
    /// @param total_size the number of bytes starting from the beginning of
    /// *this.
    /// @param tail if non-null, will save the pointer to the tail buffer there.
    /// @param tail_size if non-null, will save the number of bytes that are in
    /// the tail buffer.
    /// @return the first of the referenced buffers.
    DataBuffer *ref_all(unsigned total_size, DataBuffer **tail = nullptr,
        unsigned *tail_size = nullptr)
    {
        DataBuffer *curr = this;
        do
        {
            HASSERT(curr);
            curr->ref();
            if (total_size > curr->size())
            {
                total_size -= curr->size();
            }
            else
            {
                if (tail)
                {
                    *tail = curr;
                }
                if (tail_size)
                {
                    *tail_size = total_size;
                }
                total_size = 0;
            }
            curr = curr->next();
        } while (total_size > 0);
        return this;
    }

    /// Releases one reference to all blocks of this buffer.
    /// @param total_size the number of bytes starting from the beginning of
    /// *this.
    void unref_all(unsigned total_size)
    {
        DataBuffer *curr = this;
        while (true)
        {
            HASSERT(curr);
            if (total_size > curr->size())
            {
                DataBuffer *next = curr->next();
                total_size -= curr->size();
                curr->unref();
                curr = next;
            }
            else
            {
                curr->unref();
                break;
            }
        }
    }

    /// Helper function to read out data from a linked data buffer.
    /// @param skip this is how many bytes from the beginning of the buffer
    /// will be skipped.
    /// @param ptr will be set to the pointer to the first non-skipped byte.
    /// @param available will contain the number of available consecutive bytes
    /// to read from that point on.
    /// @return the data buffer where to continue reading (with skip = 0),
    /// might be nullptr.
    DataBuffer *get_read_pointer(
        unsigned skip, uint8_t **ptr, unsigned *available)
    {
        DataBuffer *curr = this;
        while (curr->size() <= skip)
        {
            skip -= curr->size();
            curr = curr->next();
            HASSERT(curr);
        }
        *ptr = curr->data() + skip;
        *available = curr->size() - skip;
        return curr->next();
    }

private:
    friend class DataBufferPool;

    DataBuffer(DataBufferPool *p)
        : Buffer<uint8_t[]>((Pool *)p)
    {
    }
}; // class DataBuffer

using DataBufferPtr = std::unique_ptr<DataBuffer, BufferDelete<uint8_t[]>>;

/// A class that keeps ownership of a chain of linked DataBuffer references.
class LinkedDataBufferPtr
{
public:
    LinkedDataBufferPtr()
    {
    }

    ~LinkedDataBufferPtr()
    {
        reset();
    }

    /// Move constructor. Takes the ownership that o has. Leaves o as empty.
    LinkedDataBufferPtr(LinkedDataBufferPtr &&o)
        : head_(o.head_)
        , tail_(o.tail_)
        , size_(o.size_)
        , skip_(o.skip_)
        , free_(o.free_)
    {
        o.clear();
    }

    /// Move assignment operator. Takes the ownership that o has. Leaves o as
    /// empty.
    void operator=(LinkedDataBufferPtr &&o)
    {
        reset();
        head_ = o.head_;
        tail_ = o.tail_;
        size_ = o.size_;
        skip_ = o.skip_;
        free_ = o.free_;
        o.clear();
    }

    /// We do not permit default copy operation. Use the reset() function for
    /// that.
    LinkedDataBufferPtr(const LinkedDataBufferPtr &) = delete;
    void operator=(const LinkedDataBufferPtr &) = delete;

    /// Takes a reference of o, taking a prefix of len size (or all the
    /// data). The current buffer becomes non-extensible.
    /// @param o an owned LinkedDataBufferPtr
    /// @param size is non-negative, this is how many bytes from the beginning
    /// of o will be copied. If default (negative), takes all bytes that are
    /// filled.
    void reset(const LinkedDataBufferPtr &o, ssize_t size = -1)
    {
        reset();
        if (size < 0)
        {
            size = o.size_;
        }
        skip_ = o.skip_;
        size_ = size;
        // Takes references, keeping the tail and tail size.
        unsigned tail_size;
        head_ = o.head_->ref_all(o.skip_ + size, &tail_, &tail_size);
        HASSERT(tail_size > 0);
        free_ = -tail_size;
    }

    /// Clears the current contents and replaces it with the empty buf.
    /// @param buf is a new, empty DataBuffer. Ownership will be taken. The
    /// size() value of it has to be denoting the amount of available bytes.
    void reset(DataBuffer *buf)
    {
        reset();
        head_ = tail_ = buf;
        skip_ = 0;
        free_ = buf->size();
        size_ = 0;
        buf->set_size(0);
    }

    /// Set to a single data buffer.
    /// @param buf is a filled-in data buffer. Takes ownership. Must be a
    /// single (non-chained) buffer.
    /// @param skip how many bytes to skip at the beginning
    /// @param size how many bytes to take after skip bytes.
    void reset(DataBuffer *buf, unsigned skip, unsigned size)
    {
        reset();
        head_ = buf;
        skip_ = skip;
        size_ = size;
        free_ = -int(skip+size);
        tail_ = buf;
    }

    /// Adds an empty buffer to the end of this buffer chain.
    /// @param buf is a new, empty DataBuffer. Ownership will be taken. The
    /// size() value of it has to be denoting the amount of available bytes.
    void append_empty_buffer(DataBuffer *buf)
    {
        if (!head_)
        {
            reset(buf);
            return;
        }
        HASSERT(free_ >= 0);
        HASSERT(tail_);
        free_ = buf->size();
        buf->set_size(0);
        HASSERT(!tail_->next());
        tail_->set_next(buf);
        tail_ = buf;
    }

    /// Deallocates the current content (by releasing the references).
    void reset()
    {
        if (head_)
        {
            head_->unref_all(size_ + skip_);
        }
        clear();
    }

    /// @return the pointer where data can be appended into the tail of this
    /// buffer chain.
    uint8_t *data_write_pointer()
    {
        if (!tail_)
        {
            return nullptr;
        }
        return tail_->data() + tail_->size();
    }

    /// Advances the tail pointer after a write occurred into the tail.
    /// @param len how many bytes were written into the space pointed to by
    /// data_write_pointer().
    void data_write_advance(size_t len)
    {
        HASSERT(free_ >= 0 && ((int)len <= free_));
        free_ -= len;
        tail_->set_size(tail_->size() + len);
        size_ += len;
    }

    /// Advances the head pointer. Typically used after a successful read
    /// happened.
    /// @param len how many bytes to advance the read pointer.
    void data_read_advance(size_t len)
    {
        HASSERT(len <= size());
        while (len > 0)
        {
            uint8_t *p;
            unsigned available;
            DataBuffer *next_head =
                head_->get_read_pointer(skip_, &p, &available);
            if ((len > available) || (len == available && len < size_))
            {
                head_->unref();
                head_ = next_head;
                skip_ = 0;
                size_ -= available;
                len -= available;
            }
            else
            {
                skip_ += len;
                size_ -= len;
                len = 0;
                break;
            }
        }
    }

    /// @return buffer that is at head.
    DataBuffer *head() const
    {
        return head_;
    }

    /// @return how many bytes to skip from the head buffer.
    unsigned skip() const
    {
        return skip_;
    }

    /// @return how many bytes are filled in the current buffer.
    unsigned size() const
    {
        return size_;
    }

    /// @return the number of bytes that can be written into the tail of this
    /// buffer chain.
    size_t free() const
    {
        if (free_ < 0)
        {
            return 0;
        }
        return free_;
    }

    /// Transfers the ownership of the prefix of this buffer. The tail will
    /// remain in the current buffer chain.
    /// @param len how many bytes at the beginning (starting at skip_) to
    /// transfer. Must reach into the tail buffer.
    /// @return a new (moveable) LinkedDataBufferPtr that will get the
    /// ownership of the head. It will be non-extendible.
    LinkedDataBufferPtr transfer_head(size_t len)
    {
        LinkedDataBufferPtr ret;
        ret.head_ = head_;
        ret.tail_ = tail_;
        ret.skip_ = skip_;
        ret.size_ = len;

        HASSERT(tail_);
        HASSERT(len <= size_);
        HASSERT(size_ - len < tail_->size());

        head_ = tail_->ref();
        size_ -= len;
        skip_ = tail_->size() - size_;
        HASSERT(skip_ > 0);
        ret.free_ = -skip_;
        // free_ remains as is.
        return ret;
    }

    /// Appends all content in this buffer to an std::string.
    /// @param recvd string to append data to.
    void append_to(std::string *recvd) const
    {
        DataBuffer *head = head_;
        unsigned skip = skip_;
        size_t len = size_;
        recvd->reserve(recvd->size() + len);
        while (len > 0)
        {
            uint8_t *ptr;
            unsigned available;
            head = head->get_read_pointer(skip, &ptr, &available);
            if (available > len)
            {
                available = len;
            }
            recvd->append((char *)ptr, available);
            len -= available;
            skip = 0;
        }
    }

    /// Attempt to combine *this with o into a single LinkedDataBufferPtr
    /// this. This tries to do `*this += o`. It will succeed if o.head() ==
    /// this->tail() and the bytes in these buffers are back to back.
    /// @param o a LinkedDataBuffer with data payload.
    /// @return true if append succeeded. If false, nothing was changed.
    bool try_append_from(const LinkedDataBufferPtr &o)
    {
        if (!o.size())
        {
            return true; // zero bytes, nothing to do.
        }
        if (free_ >= 0)
        {
            // writeable buffer, cannot append.
            return false;
        }
        HASSERT(o.head());
        if (o.head() != tail_)
        {
            // Buffer does not start in the same chain where we end.
            return false;
        }
        if (-free_ != (int)o.skip())
        {
            // Not back-to-back.
            return false;
        }
        // Now we're good, so take over the extra buffers.
        tail_ = o.tail_;
        free_ = o.free_;
        size_ += o.size_;
        // Acquire extra references
        o.head_->ref_all(o.skip() + o.size());
        // Release duplicate reference between the two chains.
        o.head_->unref();
        return true;
    }

private:
    /// Internal helper function of constructors and reset functions. Clears
    /// the current structure (references have to have been dealth with
    /// before).
    void clear()
    {
        head_ = tail_ = nullptr;
        skip_ = free_ = size_ = 0;
    }

    /// First buffer in the chain. This is the root of the ownership.
    DataBuffer *head_ {nullptr};
    /// Last buffer in the chain. This is where we can extend the owned bytes.
    DataBuffer *tail_ {nullptr};
    /// How many bytes we have filled in (counting starts at head.data() +
    /// skip_).
    size_t size_ {0};
    /// How many bytes to skip in the head buffer.
    uint16_t skip_ {0};
    /// If >= 0: How many free bytes are there in the tail buffer. If < 0:
    /// non-appendable buffer, the -offset of the first non-contained byte in
    /// the tail buffer. In other words, -1 * the skip() of the next linked
    /// buffer.
    int16_t free_ {0};
};

/// Proxy Pool that can allocate DataBuffer objects of a certain size. All
/// memory comes from the mainBufferPool.
class DataBufferPool : public Pool
{
public:
    DataBufferPool(unsigned payload_size)
        : payloadSize_(payload_size)
    {
        HASSERT(payload_size <= 65535u - sizeof(BufferBase));
    }

#ifdef GTEST
    /// Use this variable with a ScopedOverride to temporarily change how much
    /// data gets allocated.
    uint16_t *payload_size_override()
    {
        return &payloadSize_;
    }
#endif    
    
    /// Number of free items in the pool.
    size_t free_items() override
    {
        return base_pool()->free_items(alloc_size());
    }

    /// Number of free items in the pool for a given allocation size.
    /// @param size size of interest
    /// @return number of free items in the pool for a given allocation size
    size_t free_items(size_t size) override
    {
        return base_pool()->free_items(size);
    }

    /** Get a free item out of the pool with untyped data of the size specified
     * in the constructor.
     * @param result pointer to a pointer to the result
     */
    void alloc(DataBuffer **result)
    {
#ifdef DEBUG_BUFFER_MEMORY
        g_current_alloc = &&alloc;
    alloc:
#endif
        *result = static_cast<DataBuffer *>(
            base_pool()->alloc_untyped(alloc_size(), nullptr));
        if (*result)
        {
            new (*result) DataBuffer(this);
            (*result)->size_ = payload_size();
        }
    }

    /** Get a free item out of the pool with untyped data of the size specified
     * in the constructor.
     * @param result holder pointer
     */
    void alloc(DataBufferPtr *result)
    {
        DataBuffer *b;
        alloc(&b);
        result->reset(b);
    }

private:
    /// Internal helper funciton used by the default Buffer
    /// allocimplementation.
    BufferBase *alloc_untyped(size_t size, Executable *flow) override
    {
        DIE("DataBufferPool does not support this type of allocation.");
    }

    /// Function called when a buffer refcount reaches zero.
    void free(BufferBase *item) override
    {
        // Restores the correct size for assigning it to the right freelist
        // bucket.
        item->size_ = alloc_size();
        // Clears the next pointer as we are not using these for queues.
        item->next = nullptr;
        base_pool()->free(item);
    }

    /// @return the pool from which we should get the actual memory we have.
    Pool *base_pool()
    {
        return mainBufferPool;
    }

    /// @return size of the buffers to allocate.
    uint16_t alloc_size()
    {
        return sizeof(BufferBase) + payloadSize_;
    }

    /// @return maximum number of bytes that can be stored inside an allocated
    /// buffer.
    uint16_t payload_size()
    {
        return payloadSize_;
    }

    /// Number of bytes that need to be stored in each buffer.
    uint16_t payloadSize_;
};

#endif // _UTILS_DATABUFFER_HXX_
