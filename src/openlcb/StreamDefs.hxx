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
 * \file StreamDefs.hxx
 * Constants and helper function useful for a stream implementation.
 *
 * @author Balazs Racz
 * @date 14 December 2014
 */

#ifndef _OPENLCB_STREAMDEFS_HXX_
#define _OPENLCB_STREAMDEFS_HXX_

#include "openlcb/Defs.hxx"

namespace openlcb
{

/// Static constants and helper functions for the OpenLCB streaming protocol.
struct StreamDefs
{
    /// Maximum window size for stream send.
    static constexpr uint16_t MAX_PAYLOAD = 0xffff;
    /// This value is invalid as a source or destination stream ID.
    static constexpr uint8_t INVALID_STREAM_ID = 0xff;
    /// Supply this value to the total byte count in stream close to mark it as
    /// invalid.
    static constexpr uint32_t INVALID_TOTAL_BYTE_COUNT = 0xffffffff;

    enum Flags
    {
        FLAG_CARRIES_ID = 0x01,
        FLAG_REJECT_OUT_OF_ORDER = 0x02,
        /// @todo synchronize the draft text in the standards repository with
        /// these definitions. These definitions match other OpenLCB protocols.
        FLAG_PERMANENT_ERROR = 0x10,
        FLAG_TEMPORARY_ERROR = 0x20,
        FLAG_ACCEPT = 0x80,
    };

    enum AdditionalFlags
    {
        REJECT_INFO_LOGGED = 0x01,
        REJECT_PERMANENT_INVALID_REQUEST = 0x20,
        REJECT_PERMANENT_SRC_NOT_PERMITTED = 0x40,
        REJECT_PERMANENT_STREAMS_NOT_ACCEPTED = 0x80,
        REJECT_TEMPORARY_BUFFER_FULL = 0x20,
        REJECT_TEMPORARY_OUT_OF_ORDER = 0x40,
    };

    /// This code is sent back in the error code field in the stream initiate
    /// reply if the stream is accepted.
    static constexpr uint16_t STREAM_ACCEPT = ((uint16_t)FLAG_ACCEPT) << 8;

    /// This code is sent back in the error code field in the stream initiate
    /// reply if the stream is rejected with invalid arguments.
    static constexpr uint16_t STREAM_ERROR_INVALID_ARGS =
        (((uint16_t)FLAG_PERMANENT_ERROR) << 8) |
        REJECT_PERMANENT_INVALID_REQUEST;

    /// Creates a Stream Initiate Request message payload.
    ///
    /// @param max_buffer_size value to propose as stream window size.
    /// @param has_ident if true, sets the flag for carrying a source stream
    /// ID.
    /// @param src_stream_id source stream ID value.
    ///
    /// @return a Payload object for a GenMessage.
    ///
    static Payload create_initiate_request(uint16_t max_buffer_size,
        bool has_ident, uint8_t src_stream_id,
        uint8_t dst_stream_id = INVALID_STREAM_ID)
    {
        Payload p(6, 0);
        p[0] = max_buffer_size >> 8;
        p[1] = max_buffer_size & 0xff;
        p[2] = has_ident ? FLAG_CARRIES_ID : 0;
        p[3] = 0; // flags
        p[4] = src_stream_id;
        p[5] = dst_stream_id;
        return p;
    }

    /// Creates a Stream Initiate Reply message payload.
    ///
    /// @param max_buffer_size the definite window size of the stream
    /// @param src_stream_id stream ID on the source side.
    /// @param dst_stream_id stream ID on the dst side.
    /// @param error_code error code if the stream is rejected, otherwise
    /// STREAM_ACCEPT if it is accepted.
    ///
    /// @return a Payload object for a GenMessage.
    ///
    static Payload create_initiate_response(uint16_t max_buffer_size,
        uint8_t src_stream_id, uint8_t dst_stream_id,
        uint16_t error_code = STREAM_ACCEPT)
    {
        Payload p(6, 0);
        p[0] = max_buffer_size >> 8;
        p[1] = max_buffer_size & 0xff;
        p[2] = error_code >> 8;
        p[3] = error_code & 0xff;
        p[4] = src_stream_id;
        p[5] = dst_stream_id;
        return p;
    }

    /// Creates a Stream Data Proceed message payload.
    ///
    /// @param src_stream_id stream ID on the source side
    /// @param dst_stream_id stream ID on the destination side
    ///
    /// @return Payload object for GenMessage
    ///
    static Payload create_data_proceed(
        uint8_t src_stream_id, uint8_t dst_stream_id)
    {
        Payload p(2, 0);
        p[0] = src_stream_id;
        p[1] = dst_stream_id;
        return p;
    }

    /// Creates the payload for a stream close message.
    ///
    /// @param src_stream_id 1-byte SID stream identifier at the source side
    /// @param dst_stream_id 1-byte SID stream identifier at the dst side
    /// @param total_bytes if nonzero, specifies how many bytes were
    /// transferred in the stream in total.
    ///
    /// @return a Payload object for GenMessage.
    ///
    static Payload create_close_request(uint8_t src_stream_id,
        uint8_t dst_stream_id, uint32_t total_bytes = INVALID_TOTAL_BYTE_COUNT)
    {
        Payload p(total_bytes != INVALID_TOTAL_BYTE_COUNT ? 6 : 2, 0);
        p[0] = src_stream_id;
        p[1] = dst_stream_id;
        if (total_bytes != INVALID_TOTAL_BYTE_COUNT)
        {
            p[2] = (total_bytes >> 24) & 0xff;
            p[3] = (total_bytes >> 16) & 0xff;
            p[4] = (total_bytes >> 8) & 0xff;
            p[5] = (total_bytes >> 0) & 0xff;
        }
        return p;
    }
};

} // namespace openlcb

#endif // _OPENLCB_STREAMDEFS_HXX_
