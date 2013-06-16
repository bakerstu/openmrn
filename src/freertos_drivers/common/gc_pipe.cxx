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

#include "pipe.hxx"
#include "gc_pipe.hxx"
#include "gc_format.h"
#include "nmranet_can.h"


class GCAdapter : public GCAdapterBase
{
public:
    GCAdapter(Pipe* gc_side, Pipe* can_side, bool double_bytes)
        : parser_(can_side, &formatter_),
          formatter_(gc_side, &parser_, double_bytes)
    {
        gc_side->RegisterMember(&parser_);
        can_side->RegisterMember(&formatter_);
    }

    virtual ~GCAdapter()
    {
        parser_.destination()->UnregisterMember(&formatter_);
        formatter_.destination()->UnregisterMember(&parser_);
    }    

private:
    class BinaryToGCMember : public PipeMember
    {
    public:
        BinaryToGCMember(Pipe* destination,
                         PipeMember* skip_member,
                         int double_bytes)
            : destination_(destination),
              skip_member_(skip_member),
              double_bytes_(double_bytes)
        {
        }

        Pipe* destination()
        {
            return destination_;
        }

        virtual void write(const void* buf, size_t count)
        {
            const struct can_frame* frame =
                static_cast<const struct can_frame*>(buf);
            char dbuf[56];
            while (count >= sizeof(*frame))
            {
                char* end = gc_format_generate(frame, dbuf, double_bytes_);
                int len = (end - dbuf);
                if (len) {
                    destination_->WriteToAll(skip_member_, dbuf, len);
                }
                count -= sizeof(*frame);
                frame++;
            }
        }

    private:
        //! Pipe to send data to.
        Pipe* destination_;
        //! The pipe member that should be sent as "source".
        PipeMember* skip_member_;
        //! Non-zero if doubling was requested.
        int double_bytes_;
    };


    class GCToBinaryMember : public PipeMember
    {
    public:
        GCToBinaryMember(Pipe* destination,
                         PipeMember* skip_member)
            : offset_(-1),
              destination_(destination),
              skip_member_(skip_member)
        {
        }

        Pipe* destination()
        {
            return destination_;
        }

        virtual void write(const void* buf, size_t count)
        {
            const char* cbuf = static_cast<const char*>(buf);
            for (size_t i = 0; i < count; ++i)
            {
                consume_byte(cbuf[i]);
            }
        }

        void consume_byte(char c)
        {
            if (c == ':')
            {
                // Frame is starting here.
                offset_ = 0;
                return;
            }
            if (c == ';')
            {
                // Frame ends here.
                cbuf_[offset_] = 0;
                struct can_frame frame;
                int ret = gc_format_parse(cbuf_, &frame);
                if (!ret)
                {
                    destination_->WriteToAll(skip_member_,
                                             &frame, sizeof(frame));
                }
                offset_ = -1;
                return;
            }
            if (offset_ >= static_cast<int>(sizeof(cbuf_) - 1))
            {
                // We overran the buffer, so this can't be a valid frame.
                // Reset and look for sync byte again.
                offset_ = -1;
                return;
            }
            cbuf_[offset_++] = c;
        }

    private:
        //! Collects data from a partial GC packet.
        char cbuf_[32];
        //! offset of next byte in cbuf to write.
        int offset_;
        //! Pipe to send data to.
        Pipe* destination_;
        //! The pipe member that should be sent as "source".
        PipeMember* skip_member_;
    };

    //! PipeMember doing the parsing.
    GCToBinaryMember parser_;
    //! PipeMember doing the formatting.
    BinaryToGCMember formatter_;
};
  

GCAdapterBase* GCAdapterBase::CreateGridConnectAdapter(Pipe* gc_side, Pipe* can_side, bool double_bytes)
{
    return new GCAdapter(gc_side, can_side, double_bytes);
}
