/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file SyncStream.hxx
 * Utility classes for processing a stream of bytes
 *
 * @author Balazs Racz
 * @date 17 May 2017
 */

#ifndef _UTILS_SYNCSTREAM_HXX_
#define _UTILS_SYNCSTREAM_HXX_

#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <memory>

class SyncStream
{
public:
    /** Main entry point to the data consumption.
     *
     * @param data is the pointer to a block of data to consume.
     * @param len is the number of bytes to consume.
     * @return 0 if the stream is completed/EOF (not consuming data anymore);
     * negative value if there is an error; or the number of bytes consumed
     * from the stream. */
    virtual ssize_t write(const void *data, size_t len) = 0;

    /** Called once after all data has been written to close the stream and
     * release resources. */
    virtual void finalize()
    {
    }

    /** Repeatedly writes until all data has been consumed or an error
     * occurs. Returns a short write only when an EOF occured. */
    ssize_t write_all(const void *data, size_t len)
    {
        auto *d = to_8(data);
        size_t written = 0;
        while (len > 0)
        {
            auto ret = write(d, len);
            if (ret < 0)
            {
                return ret;
            }
            if (ret == 0)
            {
                return written;
            }
            written += ret;
            d += ret;
            len -= ret;
        }
        return written;
    }

protected:
    /// Converts a void pointer to an equivalent byte pointer.
    static const uint8_t *to_8(const void *d)
    {
        return static_cast<const uint8_t *>(d);
    }

    /// Converts a void pointer to an equivalent byte pointer.
    static uint8_t *to_8( void *d)
    {
        return static_cast<uint8_t *>(d);
    }
};

/**
 * Stream implementation that takes a fixed number of bytes, filling in a
 * header structure, and then returns EOF.
 */
class HeaderStream : public SyncStream
{
public:
    HeaderStream(void *header, size_t header_size)
        : data_(to_8(header))
        , remaining_(header_size)
    {
    }

    virtual ssize_t write(const void *data, size_t len)
    {
        if (remaining_ == 0)
        {
            return 0;
        }
        if (len > remaining_)
        {
            len = remaining_;
        }
        memcpy(data_, data, len);
        remaining_ -= len;
        data_ += len;
        return len;
    }

private:
    uint8_t *data_;
    size_t remaining_;
};

class DelegateStream : public SyncStream
{
public:
    virtual ssize_t write(const void *data, size_t len)
    {
        if (len == 0) return 0;
        while (true)
        {
            if (!delegate_)
            {
                return 0;
            }
            ssize_t ret = delegate_->write(data, len);
            if (ret != 0)
            {
                // Both positive (data actually written) as well as negative
                // (error) will be returned directly.
                return ret;
            }
            on_eof();
        }
    }

protected:
    /// This function will be called when the delegate returns EOF. Usually
    /// used to process something and then set up a new delegate (or clear
    /// delegate which will start returning EOF).
    virtual void on_eof()
    {
        delegate_->finalize();
        delegate_.reset();
    }

    /// Stream implementation to delegate the logic to. Must be instantiated by
    /// the implementation of DelegateStream. If null, then the stream will
    /// return EOF to the caller.
    std::unique_ptr<SyncStream> delegate_;
};

#endif // _UTILS_SYNCSTREAM_HXX_
