/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file gc_pipe.cxx
 * Gridconnect parser/renderer pipe components.
 *
 * @author Balazs Racz
 * @date 20 May 2013
 */

#include "executor/StateFlow.hxx"
#include "nmranet_can.h"
#include "utils/BufferQueue.hxx"
#include "utils/PipeFlow.hxx"
#include "utils/gc_format.h"
#include "utils/gc_pipe.hxx"

class GCAdapter : public GCAdapterBase
{
public:
    GCAdapter(HubFlow *gc_side, CanHubFlow *can_side, bool double_bytes)
        : parser_(can_side->service(), can_side, &formatter_),
          formatter_(can_side->service(), gc_side, &parser_, double_bytes)
    {
        gc_side->register_port(&parser_);
        can_side->register_port(&formatter_);
    }

    GCAdapter(HubFlow *gc_side_read, HubFlow *gc_side_write,
              CanHubFlow *can_side, bool double_bytes)
        : parser_(can_side->service(), can_side, &formatter_),
          formatter_(can_side->service(), gc_side_write, &parser_, double_bytes)
    {
        gc_side_read->register_port(&parser_);
        can_side->register_port(&formatter_);
    }

    virtual ~GCAdapter()
    {
        parser_.destination()->unregister_port(&formatter_);
        /// @TODO(balazs.racz) This is incorrect if the 3-pipe constructor is
        /// used.
        formatter_.destination()->unregister_port(&parser_);
    }

private:
    class BinaryToGCMember : public CanHubPort
    {
    public:
        BinaryToGCMember(Service *service, HubFlow *destination,
                         HubPort *skip_member, int double_bytes)
            : CanHubPort(service),
              destination_(destination),
              skipMember_(skip_member),
              double_bytes_(double_bytes)
        {
        }

        HubFlow *destination()
        {
            return destination_;
        }

        Action entry()
        {
            char *end =
                gc_format_generate(message()->data(), dbuf_, double_bytes_);
            size_t size = (end - dbuf_);
            if (size)
            {
                Buffer<HubData> *target_buffer;
                /// @todo(balazs.racz) switch to asynchronous allocation here.
                mainBufferPool->alloc(&target_buffer);
                target_buffer->data()->skipMember_ =
                    reinterpret_cast<uintptr_t>(skipMember_);
                /// @todo(balazs.racz) try to use an assign function for better
                /// performance.
                target_buffer->data()->resize(size);
                memcpy(target_buffer->data(), dbuf_, size);
                destination_->send(target_buffer);
            }
            return release_and_exit();
        }

    private:
        //! Destination buffer (characters).
        char dbuf_[56];
        //! Pipe to send data to.
        HubFlow *destination_;
        //! The pipe member that should be sent as "source".
        HubPort *skipMember_;
        //! Non-zero if doubling was requested.
        int double_bytes_;
    };

    class GCToBinaryMember : public HubPort
    {
    public:
        GCToBinaryMember(Service *service, CanHubFlow *destination,
                         CanHubPort *skip_member)
            : HubPort(service),
              offset_(-1),
              destination_(destination),
              skipMember_(skip_member)
        {
        }

        CanHubFlow *destination()
        {
            return destination_;
        }

        /** Takes more characters from the pending incoming buffer. */
        Action entry()
        {
            inBuf_ = message()->data()->data();
            inBufSize_ = message()->data()->size();
            return call_immediately(STATE(parse_more_data));
        }

        Action parse_more_data()
        {
            while (inBufSize_--)
            {
                char c = *inBuf_++;
                if (consume_byte(c))
                {
                    // End of frame. Allocate an output buffer and parse the
                    // frame.
                    /// @todo(balazs.racz) use a configurable buffer pool
                    mainBufferPool->alloc(&outBuf_);
                    return call_immediately(STATE(parse_to_output_frame));
                }
            }
            // Will notify the caller.
            return release_and_exit();
        }

        /** Takes the completed frame in cbuf_, parses it into the allocation
         * result (a can pipe buffer) and sends off frame. Then comes back to
         * process buffer. */
        Action parse_to_output_frame()
        {
            // CanPipeBuffer *pbuf = GetTypedAllocationResult(&g_can_alloc);
            // pbuf->Reset();
            int ret = gc_format_parse(cbuf_, outBuf_->data());
            if (!ret)
            {
                outBuf_->data()->skipMember_ =
                    reinterpret_cast<uintptr_t>(skipMember_);
                destination_->send(outBuf_);
            }
            else
            {
                // Releases the buffer.
                outBuf_->unref();
            }
            return call_immediately(STATE(parse_more_data));
        }

        /** Adds the next character from the source stream. Returns true if
         * cbuf_ contains a complete frame. */
        bool consume_byte(char c)
        {
            if (c == ':')
            {
                // Frame is starting here.
                offset_ = 0;
                return false;
            }
            if (c == ';')
            {
                if (offset_ < 0)
                {
                    return false;
                }
                // Frame ends here.
                cbuf_[offset_] = 0;
                offset_ = -1;
                return true;
            }
            if (offset_ >= static_cast<int>(sizeof(cbuf_) - 1))
            {
                // We overran the buffer, so this can't be a valid frame.
                // Reset and look for sync byte again.
                offset_ = -1;
                return false;
            }
            if (offset_ >= 0)
            {
                cbuf_[offset_++] = c;
            }
            else
            {
                // Drop byte to the floor -- we're not in the middle of a
                // packet.
            }
            return false;
        }

    private:
        //! Collects data from a partial GC packet.
        char cbuf_[32];
        //! offset of next byte in cbuf to write.
        int offset_;

        //! The incoming characters.
        const char *inBuf_;
        //! The remaining number of characters in inBuf_.
        size_t inBufSize_;

        //! The buffer to send to the destination hub.
        Buffer<CanHubData> *outBuf_;

        // ==== static data ====

        //! Pipe to send data to.
        CanHubFlow *destination_;
        //! The pipe member that should be sent as "source".
        CanHubPort *skipMember_;
    };

    //! PipeMember doing the parsing.
    GCToBinaryMember parser_;
    //! PipeMember doing the formatting.
    BinaryToGCMember formatter_;
};

GCAdapterBase *GCAdapterBase::CreateGridConnectAdapter(HubFlow *gc_side,
                                                       CanHubFlow *can_side,
                                                       bool double_bytes)
{
    return new GCAdapter(gc_side, can_side, double_bytes);
}

GCAdapterBase *GCAdapterBase::CreateGridConnectAdapter(HubFlow *gc_side_read,
                                                       HubFlow *gc_side_write,
                                                       CanHubFlow *can_side,
                                                       bool double_bytes)
{
    return new GCAdapter(gc_side_read, gc_side_write, can_side, double_bytes);
}
