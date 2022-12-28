/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file MemoryConfigDefs.hxx
 *
 * Declarations related to the memory config protocol
 *
 * @author Balazs Racz
 * @date 17 Aug 2021
 */

#ifndef _OPENLCB_MEMORYCONFIGDEFS_HXX_
#define _OPENLCB_MEMORYCONFIGDEFS_HXX_

#include "openlcb/DatagramDefs.hxx"
#include "openlcb/Defs.hxx"
#include "utils/macros.h"

namespace openlcb
{

/// Static constants and helper functions related to the Memory Configuration
/// Protocol.
struct MemoryConfigDefs
{
    using DatagramPayload = string;

    /** Possible Commands for a configuration datagram.
     */
    enum commands
    {
        COMMAND_MASK              = 0xFC,
        COMMAND_FLAG_MASK         = 0x03, /**< mask for special memory space flags */
        COMMAND_PRESENT_MASK      = 0x01, /**< mask for address space present bit */
        COMMAND_REPLY_BIT_FOR_RW  = 0x10, /**< This bit is present in REPLY commands for read-write commands. */

        COMMAND_WRITE             = 0x00, /**< command to write data to address space */
        COMMAND_WRITE_UNDER_MASK  = 0x08, /**< command to write data under mask */
        COMMAND_WRITE_REPLY       = 0x10, /**< reply to write data to address space */
        COMMAND_WRITE_FAILED      = 0x18, /**< failed to write data to address space */
        COMMAND_WRITE_STREAM      = 0x20, /**< command to write data using a stream */
        COMMAND_WRITE_STREAM_REPLY= 0x30, /**< reply to write data using a stream */
        COMMAND_WRITE_STREAM_FAILED= 0x38, /**< failed to write data using a stream */
        COMMAND_READ              = 0x40, /**< command to read data from address space */
        COMMAND_READ_REPLY        = 0x50, /**< reply to read data from address space */
        COMMAND_READ_FAILED       = 0x58, /**< failed to read data from address space */
        COMMAND_READ_STREAM       = 0x60, /**< command to read data using a stream */
        COMMAND_READ_STREAM_REPLY = 0x70, /**< reply to read data using a stream */
        COMMAND_READ_STREAM_FAILED= 0x78, /**< failed to read data using a stream */
        COMMAND_MAX_FOR_RW        = 0x80, /**< command <= this value have fixed bit arrangement. */
        COMMAND_OPTIONS           = 0x80,
        COMMAND_OPTIONS_REPLY     = 0x82,
        COMMAND_INFORMATION       = 0x84,
        COMMAND_INFORMATION_REPLY = 0x86,
        COMMAND_LOCK              = 0x88, /**< lock the configuration space */
        COMMAND_LOCK_REPLY        = 0x8A, /**< unlock the configuration space */
        COMMAND_UNIQUE_ID         = 0x8C, /**< ask for a node unique id */
        COMMAND_UNIQUE_ID_REPLY   = 0x8D, /**< node unique id */
        COMMAND_UPDATE_COMPLETE   = 0xA8, /**< indicate that a sequence of commands is complete */
        COMMAND_RESET             = 0xA9, /**< reset node to its power on state */
        COMMAND_FACTORY_RESET     = 0xAA, /**< reset node to factory defaults */
        COMMAND_ENTER_BOOTLOADER  = 0xAB, /**< reset node in bootloader mode */
        COMMAND_FREEZE            = 0xA1, /**< freeze operation of node */
        COMMAND_UNFREEZE          = 0xA0, /**< unfreeze operation of node */

        COMMAND_PRESENT    = 0x01, /**< address space is present */

        COMMAND_CDI        = 0x03, /**< flags for a CDI space */
        COMMAND_ALL_MEMORY = 0x02, /**< flags for an all memory space */
        COMMAND_CONFIG     = 0x01, /**< flags for a config memory space */
    };

    /** Possible memory spaces.
     */
    enum spaces
    {
        SPACE_SPECIAL    = 0xFC, /**< offset for the special memory spaces */
        SPACE_CDI        = 0xFF, /**< CDI space */
        SPACE_ALL_MEMORY = 0xFE, /**< all memory space */
        SPACE_CONFIG     = 0xFD, /**< config memory space */
        SPACE_ACDI_SYS   = 0xFC, /**< read-only ACDI space */
        SPACE_ACDI_USR   = 0xFB, /**< read-write ACDI space */
        SPACE_FDI        = 0xFA, /**< read-only for function definition XML */
        SPACE_FUNCTION   = 0xF9, /**< read-write for function data */
        SPACE_DCC_CV     = 0xF8, /**< proxy space for DCC functions */
        SPACE_FIRMWARE   = 0xEF, /**< firmware upgrade space */
    };

    /** Possible available options.
     */
    enum available
    {
        AVAIL_WUM   = 0x8000, /**< write under mask supported */
        AVAIL_UR    = 0x4000, /**< unaligned reads supported */
        AVAIL_UW    = 0x2000, /**< unaligned writes supported */
        /// @todo This is a proposed value, see
        /// https://github.com/openlcb/documents/issues/57
        AVAIL_SR    = 0x1000, /**< stream reads supported */
        AVAIL_R0xFC = 0x0800, /**< read from adddress space 0xFC available */
        AVAIL_R0xFB = 0x0400, /**< read from adddress space 0xFB available */
        AVAIL_W0xFB = 0x0200, /**< write from adddress space 0xFB available */
    };

    /** Possible supported write lengths.
     */
    enum lengths
    {
        LENGTH_1         = 0x80, /**< write length of 1 supported */
        LENGTH_2         = 0x40, /**< write length of 2 supported */
        LENGTH_4         = 0x20, /**< write length of 4 supported */
        LENGTH_63        = 0x10, /**< write length of 64 supported */
        LENGTH_ARBITRARY = 0x02, /**< arbitrary write of any length supported */
        LENGTH_STREAM    = 0x01, /**< stream writes supported */
    };

    /** Possible address space information flags.
     */
    enum flags
    {
        FLAG_RO   = 0x01, /**< space is read only */
        FLAG_NZLA = 0x02, /**< space has a nonzero low address */
    };

    enum errors
    {
        ERROR_SPACE_NOT_KNOWN = Defs::ERROR_INVALID_ARGS | 0x0001,
        ERROR_OUT_OF_BOUNDS = Defs::ERROR_INVALID_ARGS | 0x0002,
        ERROR_WRITE_TO_RO = Defs::ERROR_INVALID_ARGS | 0x0003,
    };

    static constexpr unsigned MAX_DATAGRAM_RW_BYTES = 64;

    static bool is_special_space(uint8_t space)
    {
        return space > SPACE_SPECIAL;
    }

    static DatagramPayload write_datagram(
        uint8_t space, uint32_t offset, const string &data = "")
    {
        DatagramPayload p;
        p.reserve(7 + data.size());
        p.push_back(DatagramDefs::CONFIGURATION);
        p.push_back(COMMAND_WRITE);
        p.push_back(0xff & (offset >> 24));
        p.push_back(0xff & (offset >> 16));
        p.push_back(0xff & (offset >> 8));
        p.push_back(0xff & (offset));
        if (is_special_space(space))
        {
            p[1] |= space & ~SPACE_SPECIAL;
        }
        else
        {
            p.push_back(space);
        }
        p += data;
        return p;
    }

    static DatagramPayload read_datagram(
        uint8_t space, uint32_t offset, uint8_t length)
    {
        DatagramPayload p;
        p.reserve(7);
        p.push_back(DatagramDefs::CONFIGURATION);
        p.push_back(COMMAND_READ);
        p.push_back(0xff & (offset >> 24));
        p.push_back(0xff & (offset >> 16));
        p.push_back(0xff & (offset >> 8));
        p.push_back(0xff & (offset));
        if (is_special_space(space))
        {
            p[1] |= space & ~SPACE_SPECIAL;
        }
        else
        {
            p.push_back(space);
        }
        p.push_back(length);
        return p;
    }

    static DatagramPayload read_stream_datagram(uint8_t space, uint32_t offset,
        uint8_t dst_stream_id, uint32_t length = 0xFFFFFFFF)
    {
        DatagramPayload p;
        p.reserve(13);
        p.push_back(DatagramDefs::CONFIGURATION);
        p.push_back(COMMAND_READ_STREAM);
        p.push_back(0xff & (offset >> 24));
        p.push_back(0xff & (offset >> 16));
        p.push_back(0xff & (offset >> 8));
        p.push_back(0xff & (offset));
        if (is_special_space(space))
        {
            p[1] |= space & ~SPACE_SPECIAL;
        }
        else
        {
            p.push_back(space);
        }
        p.push_back(0xff); // src ID
        p.push_back(dst_stream_id); // dst ID
        p.push_back(0xff & (length >> 24));
        p.push_back(0xff & (length >> 16));
        p.push_back(0xff & (length >> 8));
        p.push_back(0xff & (length));
        return p;
    }
    
    /// @return true if the payload has minimum number of bytes you need in a
    /// read or write datagram message to cover for the necessary fields
    /// (command, offset, space).
    /// @param payload is a datagram (read or write, request or response)
    /// @param extra is the needed bytes after address and space, usually 0 for
    /// write and 1 for read.
    static bool payload_min_length_check(
        const DatagramPayload &payload, unsigned extra)
    {
        auto *bytes = payload_bytes(payload);
        size_t sz = payload.size();
        if (sz < 6 + extra)
        {
            return false;
        }
        if (((bytes[1] & COMMAND_FLAG_MASK) == 0) && (sz < 7 + extra))
        {
            return false;
        }
        return true;
    }

    /// @return addressed memory space number.
    /// @param payload is a read or write datagram request or response message
    static uint8_t get_space(const DatagramPayload &payload)
    {
        auto *bytes = payload_bytes(payload);
        if (bytes[1] & COMMAND_FLAG_MASK)
        {
            return COMMAND_MASK + (bytes[1] & COMMAND_FLAG_MASK);
        }
        return bytes[6];
    }

    static unsigned get_payload_offset(const DatagramPayload &payload)
    {
        auto *bytes = payload_bytes(payload);
        if (bytes[1] & COMMAND_FLAG_MASK)
        {
            return 6;
        }
        else
        {
            return 7;
        }
    }

    /// @param payload is a datagram read or write request or response
    /// @return the address field from the datagram
    static uint32_t get_address(const DatagramPayload &payload)
    {
        auto *bytes = payload_bytes(payload);
        uint32_t a = bytes[2];
        a <<= 8;
        a |= bytes[3];
        a <<= 8;
        a |= bytes[4];
        a <<= 8;
        a |= bytes[5];
        return a;
    }

    /// Type casts a DatagramPayload to an array of bytes.
    /// @param payload datagram
    /// @return byte array pointing to the same memory
    static const uint8_t *payload_bytes(const DatagramPayload &payload)
    {
        return (uint8_t *)payload.data();
    }

private:
    /** Do not instantiate this class. */
    MemoryConfigDefs();
};

} // namespace openlcb

#endif // _OPENLCB_MEMORYCONFIGDEFS_HXX_
