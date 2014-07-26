/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file SimpleInfoProtocol.hxx
 *
 * Shared code among SNIP and similar protocols.
 *
 * @author Balazs Racz
 * @date 22 Jul 2013
 */

#ifndef _NMRANET_SIMPLEINFOPROTOCOL_HXX_
#define _NMRANET_SIMPLEINFOPROTOCOL_HXX_

#include "nmranet/If.hxx"
#include "executor/StateFlow.hxx"

namespace nmranet
{

struct SimpleInfoDescriptor;

/** Use a Buffer<SimpleInfoResponse> to send a SNIP response to a particular
 * destination host. */
struct SimpleInfoResponse
{
    SimpleInfoResponse()
        : src(nullptr)
        , dst({0, 0})
        , descriptor(nullptr)
    {
    }
    /** Initializes the fields of this message to be a response to an
     * NMRAnetMessage (i.e., flips destination and source from that
     * message). The descriptor array must end with an EOF entry and must stay
     * alive so long as the message is not sent. */
    void reset(const NMRAnetMessage *msg_to_respond,
               const SimpleInfoDescriptor *desc,
               Defs::MTI response_mti)
    {
        HASSERT(msg_to_respond->dstNode);
        src = msg_to_respond->dstNode;
        dst = msg_to_respond->src;
        descriptor = desc;
        mti = response_mti;
    }
    /** Source node to send the response from. */
    Node *src;
    /** MTI of the response to be sent. */
    Defs::MTI mti;
    /** Destination node to send the response to. */
    NodeHandle dst;
    /** Descriptor of payload to send. */
    const SimpleInfoDescriptor *descriptor;
};

/** This structure defines how to piece together a reply to a Simple Info
 * request, sich as SNIP. These structures will come in an array, where each
 * member of the array defines a variable number of bytes to append to the end
 * of the reply message. The last structure should be an EOF. */
struct SimpleInfoDescriptor
{
    enum Cmd
    {
        END_OF_DATA = 0,  //< End of the data sequence
        C_STRING = 1,     //< pointer points to a C-string with NULL term
        LITERAL_BYTE = 2, //< transfer argument as byte (e.g. version)
        CHAR_ARRAY = 3,   //< len = argument, pointer in data
    };

    uint8_t cmd; //< Command. See enum Cmd.
    uint8_t arg; //< Argument to the command.
    /** Points to a string if the command requires so. */
    const uint8_t *data;
};

typedef StateFlow<Buffer<SimpleInfoResponse>, QList<1>> SimpleInfoFlowBase;

class SimpleInfoFlow : public SimpleInfoFlowBase
{
public:
    /** Creates a simple ident flow handler.
     *
     * @param max_bytes_per_message tells how many bytes we should package in
     * one outgoing buffer. Responses longer than this will be sent as multiple
     * separate messages. Set this to 6 on CAN to completely avoid raw
     * pagination. Set to 255 to create one memory buffer that will then be
     * split into frames by the low-level interface. Maximum value is 255.
     * @param use_continue_bits should be true if we should instruct the
     * low-level interface to use the continuation-pending bits so long as we
     * have pending bytes. This will make the messages be pieced together at
     * the receiving end into one message. Setting this to false will send a
     * reply in multiple messages. */
    SimpleInfoFlow(Service *s, unsigned max_bytes_per_message = 255,
                   bool use_continue_bits = true)
        : SimpleInfoFlowBase(s)
        , maxBytesPerMessage_(
              max_bytes_per_message > 255 ? 255 : max_bytes_per_message)
        , useContinueBits_(use_continue_bits ? 1 : 0)
    {
    }

private:
    Action entry() OVERRIDE
    {
        HASSERT(message()->data()->src);
        HASSERT(message()->data()->descriptor);
        entryOffset_ = 0;
        byteOffset_ = 0;
        isFirstMessage_ = 1;
        update_for_next_entry();
        return call_immediately(STATE(continue_send));
    }

    const SimpleInfoDescriptor &current_descriptor()
    {
        return message()->data()->descriptor[entryOffset_];
    }

    /** @returns true if there are no more bytes to send. */
    bool is_eof()
    {
        return (current_descriptor().cmd == SimpleInfoDescriptor::END_OF_DATA);
    }

    /** Call this function after updating entryOffset_. */
    void update_for_next_entry()
    {
        const SimpleInfoDescriptor &d = current_descriptor();
        if (d.cmd == SimpleInfoDescriptor::C_STRING)
        {
            byteOffset_ = 0;
            currentLength_ = strlen((const char *)d.data);
        }
        else if (d.cmd == SimpleInfoDescriptor::CHAR_ARRAY)
        {
            byteOffset_ = 0;
            currentLength_ = d.arg;
        }
        else
        {
            currentLength_ = 0;
        }
    }

    /** Returns the current byte in the stream of data. */
    uint8_t current_byte()
    {
        const SimpleInfoDescriptor &d = current_descriptor();
        switch (d.cmd)
        {
            case SimpleInfoDescriptor::END_OF_DATA:
                return 0;
            case SimpleInfoDescriptor::LITERAL_BYTE:
                return d.arg;
            case SimpleInfoDescriptor::C_STRING:
            case SimpleInfoDescriptor::CHAR_ARRAY:
                if (byteOffset_ >= currentLength_)
                {
                    return 0;
                }
                else
                {
                    return d.data[byteOffset_];
                }
            default:
                DIE("Unexpected descriptor type.");
        }
    }

    /** Increments to the next byte in the stream of data. */
    void step_byte()
    {
        const SimpleInfoDescriptor &d = current_descriptor();
        switch (d.cmd)
        {
            case SimpleInfoDescriptor::END_OF_DATA:
                return;
            case SimpleInfoDescriptor::LITERAL_BYTE:
                break;
            case SimpleInfoDescriptor::C_STRING:
                // We include the terminating zero.
                if (++byteOffset_ > currentLength_)
                {
                    break;
                }
                else
                {
                    return;
                }
            case SimpleInfoDescriptor::CHAR_ARRAY:
                // There is no extra terminating zero here.
                if (++byteOffset_ >= currentLength_)
                {
                    break;
                }
                else
                {
                    return;
                }
            default:
                DIE("Unexpected descriptor type.");
        }
        ++entryOffset_;
        update_for_next_entry();
    }

    Action continue_send()
    {
        if (is_eof())
        {
            return release_and_exit();
        }
        return allocate_and_call(
            message()->data()->src->interface()->addressed_message_write_flow(),
            STATE(fill_buffer));
    }

    Action fill_buffer()
    {
        auto *b = get_allocation_result(message()
                                            ->data()
                                            ->src->interface()
                                            ->addressed_message_write_flow());
        const SimpleInfoResponse &r = *message()->data();
        b->data()->reset(r.mti, r.src->node_id(), r.dst, EMPTY_PAYLOAD);
        for (uint8_t offset = 0; offset < maxBytesPerMessage_ && !is_eof();
             ++offset, step_byte())
        {
            b->data()->payload.push_back(current_byte());
        }
        b->data()->set_flag_dst(NMRAnetMessage::WAIT_FOR_LOCAL_LOOPBACK);
        if (useContinueBits_)
        {
            if (!is_eof())
            {
                b->data()->set_flag_dst(
                    NMRAnetMessage::DSTFLAG_NOT_LAST_MESSAGE);
            }

            if (isFirstMessage_)
            {
                isFirstMessage_ = 0;
            }
            else
            {
                b->data()->set_flag_dst(
                    NMRAnetMessage::DSTFLAG_NOT_FIRST_MESSAGE);
            }
        }
        b->set_done(n_.reset(this));
        message()
            ->data()
            ->src->interface()
            ->addressed_message_write_flow()
            ->send(b);
        return wait_and_call(STATE(continue_send));
    }

    /** Configuration option. See constructor. */
    unsigned maxBytesPerMessage_ : 8;
    /** Configuration option. See constructor. */
    unsigned useContinueBits_ : 1;

    /** Whether this is the first reply message we are sending out. Used with
     * the continuation feature. */
    unsigned isFirstMessage_ : 1;
    /** Tells which descriptor entry we are processing. */
    unsigned entryOffset_ : 5;
    unsigned reserved_ : 1; // for alignment

    /** Byte offset within a descriptor entry. */
    unsigned byteOffset_ : 8;
    /** Total length of the current block. This is typically strlen() (not
     * including the zero). */
    unsigned currentLength_ : 8;

    BarrierNotifiable n_;
};

} // namespace nmranet

#endif // _NMRANET_SIMPLEINFOPROTOCOL_HXX_
