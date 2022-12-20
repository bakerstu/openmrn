/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file MemoryConfigStream.hxx
 *
 * Implementation of the stream support for Memory Config Protocol server
 *
 * @author Balazs Racz
 * @date 20 Dec 2022
 */

#ifndef _OPENLCB_MEMORYCONFIGSTREAM_HXX_
#define _OPENLCB_MEMORYCONFIGSTREAM_HXX_

#include "openlcb/MemoryConfig.hxx"
#include "openlcb/If.hxx"
#include "openlcb/StreamTransport.hxx"
#include "openlcb/StreamSender.hxx"

namespace openlcb {

/// This is a self-owned flow which reads an memory space into a stream.
class MemorySpaceStreamReadFlow : public StateFlowBase
{
public:
    MemorySpaceStreamReadFlow(Node *node, MemorySpace *space, NodeHandle dst,
        uint8_t dst_stream_id, uint32_t ofs, uint32_t len)
        : StateFlowBase(node->iface())
        , dst_(dst)
        , space_(space)
        , dstStreamId_(dst_stream_id)
        , node_(node)
        , ofs_(ofs)
        , len_(len)
    {
        start_flow(STATE(initiate_stream));
    }


private:
    Action alloc_stream() {
        return exit();
    }

    Action initiate_stream() {
        return exit();

    }

    
    /// Address to which we are sending the stream.
    NodeHandle dst_;
    /// Memory space we are reading.
    MemorySpace* space_;
    /// Destination stream ID on the target node.
    uint8_t dstStreamId_;
    /// Node from which we are sending the stream.
    Node* node_;
    /// Next byte to read.
    uint32_t ofs_;
    /// How many bytes are left to read. 0xFFFFFFFF if all bytes until EOF need
    /// to be read.
    uint32_t len_;
    /// 
    /// 
    StreamSenderCan* senderCan_;
    StreamSender* sender_;
};

/// Handler for the stream read/write commands in the memory config protocol
/// (server side).
class MemoryConfigStreamHandler : public MemoryConfigHandlerBase
{
public:
    MemoryConfigStreamHandler(MemoryConfigHandler *parent)
        : MemoryConfigHandlerBase(parent->dg_service())
        , parent_(parent)
    {
        parent_->set_stream_handler(this);
    }

    Action entry() override
    {
        // The virification of the incoming data is already done by the calling
        // MemoryConfigHandler.
        response_.clear();
        const uint8_t *bytes = in_bytes();
        uint8_t cmd = bytes[1];

        switch (cmd & MemoryConfigDefs::COMMAND_MASK)
        {
            case MemoryConfigDefs::COMMAND_READ_STREAM:
            {
                return call_immediately(STATE(handle_read_stream));
            }
            /// @todo handle write stream
        }
        return exit();
    }

private:
    Action handle_read_stream()
    {
        size_t len = message()->data()->payload.size();
        const uint8_t *bytes = in_bytes();

        size_t stream_data_offset = 6;
        if (has_custom_space())
        {
            ++stream_data_offset;
        }
        if (len < stream_data_offset + 2)
        {
            return respond_reject(Defs::ERROR_INVALID_ARGS);
        }

        uint8_t dst_stream_id = bytes[stream_data_offset + 1];
        uint32_t num_bytes_to_read = 0xFFFFFFFFu;
        if (len >= stream_data_offset + 6)
        {
            memcpy(&num_bytes_to_read, bytes + stream_data_offset + 2, 4);
            num_bytes_to_read = be32toh(num_bytes_to_read);
        }
        new MemorySpaceStreamReadFlow(message()->data()->dst, get_space(),
            message()->data()->src, dst_stream_id, get_address(),
            num_bytes_to_read);
        /// @todo
        return exit();
    }

    /** Looks up the memory space for the current datagram. Returns NULL if no
     * space was registered (for neither the current node, nor global). */
    MemorySpace *get_space()
    {
        int space_number = get_space_number();
        if (space_number < 0)
            return nullptr;
        MemorySpace *space =
            registry()->lookup(message()->data()->dst, space_number);
        if (!space)
        {
            LOG(WARNING, "MemoryConfig: asked node 0x%012" PRIx64 " for unknown space "
                         "%d. Source {0x%012" PRIx64 ", %03x}",
                message()->data()->dst->node_id(), space_number,
                message()->data()->src.id, message()->data()->src.alias);
            return nullptr;
        }
        if (!space->set_node(message()->data()->dst))
        {
            LOG(WARNING, "MemoryConfig: Global space %d rejected node.",
                space_number);
            return nullptr;
        }
        return space;
    }
    
    Registry *registry()
    {
        return parent_->registry();
        ;
    }

    /// Parent object from which we are getting commands forwarded.
    MemoryConfigHandler *parent_;
};

} // namespace openlcb

#endif // _OPENLCB_MEMORYCONFIGSTREAM_HXX_
