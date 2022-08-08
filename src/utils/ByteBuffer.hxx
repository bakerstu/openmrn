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
extern Pool* rawBufferPool;

/// Container for holding an arbitrary untyped data stream.
struct RawData
{
    uint8_t payload[RAWBUFFER_SIZE];
};

/// Buffers of this type will be allocated from the rawBufferPool to hold the
/// payloads of untyped data streams. These buffers are never enqueued into Q
/// or QList objects.
using RawBuffer = Buffer<RawData>;

/// Holds a reference to a raw buffer, with the start and size information.
struct ByteChunk
{
    /// Owns a ref for a RawData buffer. If this is nullptr, then the data
    /// references by this chunk is externally owned.
    BufferPtr<RawData> ownedData_;
    
    /// Points to the first byte of the useful data.
    uint8_t* data_ {nullptr};

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
};

/// Buffer type of references. These are enqueued for byte sinks.
using ByteBuffer = Buffer<ByteChunk>;

template<class T> class FlowInterface;

/// Interface for sending a stream of data from a source to a sink.
using ByteSink = FlowInterface<ByteBuffer>;

#endif // _UTILS_BYTEBUFFER_HXX_
