/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file ByteBuffer.hxx
 *
 * Specialization of the Buffer / Pool infrastructure for untyped data
 * stream. See { \link doc/byte_buffer.md }
 *
 * @author Balazs Racz
 * @date 17 Apr 2022
 */

#ifndef _UTILS_BYTEBUFFER_HXX_
#define _UTILS_BYTEBUFFER_HXX_

#include "utils/Buffer.hxx"

/// This is how many bytes we have in each raw buffer allocation.
static constexpr unsigned RAWBUFFER_SIZE = 1024;

/// Use this BufferPool to allocate raw buffers.
extern Pool *rawBufferPool;

/// Container for holding an arbitrary untyped data stream.
struct RawData
{
    uint8_t payload[RAWBUFFER_SIZE];
    /// Maximum length that can be stored in a single RawBuffer.
    static constexpr size_t MAX_SIZE = RAWBUFFER_SIZE;
};

/// Buffers of this type will be allocated from the rawBufferPool to hold the
/// payloads of untyped data streams. These buffers are never enqueued into Q
/// or QList objects.
using RawBuffer = Buffer<RawData>;

/// Holds a raw buffer.
using RawBufferPtr = BufferPtr<RawData>;

/// Holds a reference to a raw buffer, with the start and size information.
struct ByteChunk
{
    /// Owns a ref for a RawData buffer. If this is nullptr, then the data
    /// references by this chunk is externally owned.
    RawBufferPtr ownedData_;

    /// Points to the first byte of the useful data.
    uint8_t *data_ {nullptr};

    /// How many bytes from data_ does this chunk represent.
    size_t size_ {0};

    /// @return number of bytes pointed to by this chunk.
    size_t size() const
    {
        return size_;
    }

    /// Moves forward the data beginning pointer by a given number of
    /// bytes. Represents that some number of bytes were consumed by the sink.
    /// @param num_bytes how much data was consumed. Must be <= size().
    void advance(size_t num_bytes)
    {
        HASSERT(num_bytes <= size_);
        data_ += num_bytes;
        size_ -= num_bytes;
    }

    /// Overwrites this chunk from a raw buffer.
    ///
    /// @param buf An owned share of a RawBuffer.
    /// @param len How many bytes to take from this buffer.
    /// @param ofs From which offset we should take these bytes (default 0, may
    /// be omitted).
    ///
    void set_from(RawBufferPtr buf, size_t len, size_t ofs = 0)
    {
        ownedData_ = std::move(buf);
        size_ = len;
        data_ = ownedData_->data()->payload + ofs;
    }

    /// Overwrites this chunk from a string. WARNING: the ownership of the
    /// string is not transferred; the caller must make sure the string remains
    /// alive as long as this Chunk is ever in use (including all copies).
    void set_from(const string *data)
    {
        ownedData_.reset(); // no need for this anymore
        size_ = data->size();
        data_ = (uint8_t *)data->data();
    }

    /// Overwrites this chunk from an externally owned memory area. The caller
    /// must make sure the memory area remains alive as long as this Chunk is
    /// ever in use (including all copies).
    /// @param data payload to set into this buffer. Must stay alive.
    /// @param len number of bytes to use from that source.
    void set_from(const void *data, size_t len)
    {
        ownedData_.reset(); // no need for this anymore
        data_ = (uint8_t *)data;
        size_ = len;
    }

    /// Adds more data to the end of the buffer. Requirement: this chunk must
    /// be a data source, and there has to be an ownedData_ set.
    /// @param data payload to copy
    /// @param len how many bytes to add
    /// @return number of bytes added; this is typically less than len when the
    /// RawData buffer gets full. Can be zero.
    size_t append(const void *data, size_t len)
    {
        HASSERT(ownedData_.get());
        uint8_t *end = data_ + size_;
        uint8_t *max_end = ownedData_->data()->payload + RawData::MAX_SIZE;
        size_t max_len = max_end - end;
        if (max_len < len)
        {
            len = max_len;
        }
        memcpy(end, data, len);
        size_ += len;
        return len;
    }

    /// @return the place where data can be written to append to this buffer.
    /// Requirement: this chunk must be a data source, and there has to be an
    /// ownedData_ set.
    uint8_t *append_ptr()
    {
        HASSERT(ownedData_.get());
        uint8_t *end = data_ + size_;
        return end;
    }

    /// Notifies that a certain number of bytes have been appended, i.e.,
    /// written into append_ptr().
    /// Requirement: this chunk must be a data source, and there has to be an
    /// ownedData_ set.
    void append_complete(size_t len)
    {
        size_ += len;
    }

    /// @return how many free bytes are there in the underlying raw
    /// buffer. This shall only be used by the source (who set ownedData_ to a
    /// real raw buffer), and assumes that all bytes beyond the end are
    /// free. Always zero if data is owned externally.
    size_t free_space()
    {
        if (!ownedData_)
        {
            return 0;
        }
        uint8_t *end = data_ + size_;
        uint8_t *max_end = ownedData_->data()->payload + RawData::MAX_SIZE;
        size_t max_len = max_end - end;
        return max_len;
    }
};

/// Buffer type of references. These are enqueued for byte sinks.
using ByteBuffer = Buffer<ByteChunk>;
/// Buffer pointer type for references.
using ByteBufferPtr = BufferPtr<ByteChunk>;

template <class T> class FlowInterface;

/// Interface for sending a stream of data from a source to a sink.
using ByteSink = FlowInterface<ByteBuffer>;

#endif // _UTILS_BYTEBUFFER_HXX_
