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
 * \file DirectHubGc.cxx
 *
 * GridConnect support for DirectHub.
 *
 * @author Balazs Racz
 * @date 1 Mar 2020
 */

#include "utils/DirectHub.hxx"

/// Message segmenter that chops incoming byte stream into gridconnect packets.
class DirectHubGcSegmenter : public MessageSegmenter
{
public:
    DirectHubGcSegmenter()
    {
        clear();
    }

    ssize_t segment_message(const void *d, size_t size) override
    {
        if (!size)
        {
            return 0; // nothing to do.
        }
        const char *data = static_cast<const char *>(d);
        if (packetLen_ == 0)
        {
            // beginning of packet.
            isGcPacket_ = data[0] == ':';
        }
        size_t ofs = 0;
        if (isGcPacket_)
        {
            // looking for terminating ;
            while ((ofs < size) && (data[ofs] != ';'))
            {
                ++ofs;
            }
            if (ofs < size)
            {
                // found the terminating ;
                ++ofs;
                // append any garbage we still have.
                while ((ofs < size) && (data[ofs] != ':'))
                {
                    ++ofs;
                }
                packetLen_ += ofs;
                return packetLen_;
            }
            else
            {
                // ran out of payload without finding terminating ;
                packetLen_ += size;
                return 0;
            }
        }
        else
        {
            // Looking for starting ':'
            while ((ofs < size) && (data[ofs] != ':'))
            {
                ++ofs;
            }
            packetLen_ += ofs;
            if (ofs < size)
            {
                // found it
                return packetLen_;
            }
            else
            {
                return 0;
            }
        }
    }

    /// Resets internal state machine. The next call to segment_message()
    /// assumes no previous data present.
    void clear() override
    {
        isGcPacket_ = false;
        packetLen_ = 0;
    }

private:
    /// True if the current packet is a gridconnect packet; false if it is
    /// garbage.
    uint32_t isGcPacket_ : 1;

    /// How many bytes long this packet is.
    uint32_t packetLen_ : 30;
};

MessageSegmenter *create_gc_message_segmenter()
{
    return new DirectHubGcSegmenter();
}

/// Message segmenter that keeps each packet as-is.
class DirectHubTrivialSegmenter : public MessageSegmenter
{
public:
    ssize_t segment_message(const void *d, size_t size) override
    {
        total_ += size;
        LOG(VERBOSE, "segment %zu total %zu", size, total_);
        return size;
    }

    void clear() override
    {
    }

    size_t total_ {0};
};

MessageSegmenter *create_trivial_message_segmenter()
{
    return new DirectHubTrivialSegmenter();
}
