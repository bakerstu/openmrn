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
 * \file dcc/Defs.hxx
 *
 * Definitions for DCC concepts.
 *
 * @author Balazs Racz
 * @date 27 Feb 2016
 */

#ifndef _DCC_DEFS_HXX_
#define _DCC_DEFS_HXX_

#include <stdint.h>

namespace dcc
{

/// Which address type this legacy train node uses. These address types
/// translate to mutually independent packets on the track.
enum class TrainAddressType : uint8_t
{
    /// DCC packets with short address (1..127)
    DCC_SHORT_ADDRESS = 1,
    /// DCC packets with long address (128..~10000)
    DCC_LONG_ADDRESS,
    /// Marklin-motorola packets. Addresses 1..80 are supported.
    MM,
    /// Per-output addressing of DCC basic accessories (user facing is 1 to
    /// 2048, internal is 0 to 2047)
    DCC_ACCY_BASIC_OUTPUT,
    /// DCC 11-bit extended accessory decoder address (user facing is 1 to
    /// 2048, internal is 0 to 2047).
    DCC_ACCY_EXT,
    /// Unsupported address type (e.g. a protocol we don't have an
    /// implementation for).
    UNSUPPORTED = 255,
    /// Unspecified address type (default / match any).
    UNSPECIFIED = 254,
};

namespace Defs
{

enum
{
    MARKLIN_DEFAULT_CMD = 0b00100110,
    // Direction change bits for marklin-old format.
    MARKLIN_CHANGE_DIR_B2 = 0b11000000,

    DCC_DEFAULT_CMD = 0,
    DCC_LONG_PREAMBLE_CMD = 0b00001100,
    DCC_SERVICE_MODE_5X_WITH_ACK_CMD = 0b00111000,
    // standard dcc packet with 5x repeat.
    DCC_SERVICE_MODE_5X_CMD = 0b00101000,
    DCC_SERVICE_MODE_1X_CMD = 0b00001000,
    /// Prefix for DCC long addresses (first byte).
    DCC_LONG_ADDRESS_FIRST = 0b11000000,

    // Baseline packet: speed and direction.
    DCC_BASELINE_SPEED = 0b01000000,
    DCC_BASELINE_SPEED_FORWARD = 0b00100000,
    DCC_BASELINE_SPEED_LIGHT = 0b00010000,
    DCC_FUNCTION1 = 0b10000000,
    DCC_FUNCTION1_F0 = 0b00010000,
    DCC_FUNCTION2_F5 = 0b10110000,
    DCC_FUNCTION2_F9 = 0b10100000,
    DCC_FEATURE_EXP_F13 = 0b11011110,
    DCC_FEATURE_EXP_F21 = 0b11011111,
    DCC_FEATURE_EXP_FNHI = 0b11011000,
    DCC_BINARY_SHORT = 0b11011101,
    DCC_BINARY_LONG = 0b11000000,
    DCC_ANALOG_FN = 0b00111101,

    DCC_PROG_READ1 = 0b11100100,
    DCC_PROG_WRITE1 = 0b11101100,
    DCC_PROG_READ4 = 0b11100000,

    DCC_SVC_BIT_MANIPULATE = 0b01111000,
    DCC_SVC_WRITE = 0b01111100,
    DCC_SVC_VERIFY = 0b01110100,
    DCC_SVC_MASK = 0b11111100,

    DCC_SVC_BITVAL_WRITE = 0b11110000,
    DCC_SVC_BITVAL_VERIFY = 0b11100000,
    DCC_SVC_BITVAL_VALUE = 0b00001000,
    DCC_SVC_BITVAL_MASK = 0b11110000,

    DCC_SVC_PAGED_WRITE = 0b01111000,
    DCC_SVC_PAGED_VERIFY = 0b01110000,
    DCC_SVC_PAGED_MASK = 0b11111000,

    // Byte 1 of dcc accessory control packet
    DCC_BASIC_ACCESSORY_B1 = 0b10000000,
    // Mask to check on DCC_BASIC_ACCESSORY_B1
    DCC_BASIC_ACCESSORY_MASK1 = 0b11000000,
    // Byte 2 of dcc basic accessory control packet
    DCC_BASIC_ACCESSORY_B2 = 0b10000000,
    // Mask for dcc basic accessory control packet
    DCC_BASIC_ACCESSORY_MASK2 = 0b10000000,
    // Byte 2 of DCC extended accessory control packet
    DCC_EXT_ACCESSORY_B2 = 0b00000001,
    // Mask for DCC_EXT_ACCESSORY_B2
    DCC_EXT_ACCESSORY_MASK2 = 0b10001001,

    // Basic accy activate command (in byte 2)
    DCC_BASIC_ACCESSORY_B2_ACTIVATE = 0b00001000,
    // Basic accy deactivate command (in byte 2)
    DCC_BASIC_ACCESSORY_B2_DEACTIVATE = 0b00000000,
    // Mask for activate/deactivate
    DCC_BASIC_ACCESSORY_B2_ACTIVATE_MASK = DCC_BASIC_ACCESSORY_B2_ACTIVATE,
    // Basic accy thrown / reverse / red / off command (in byte 2)
    DCC_BASIC_ACCESSORY_B2_THROWN = 0b00000000,
    // Basic accy closed / normal / green / on command (in byte 2)
    DCC_BASIC_ACCESSORY_B2_CLOSED = 0b00000001,
    // Mask for thrown / closed
    DCC_BASIC_ACCESSORY_B2_DIR_MASK = DCC_BASIC_ACCESSORY_B2_CLOSED,

    // Extended packet: 128-step speed.
    DCC_EXT_SPEED = 0b00111111,
    DCC_EXT_SPEED_FORWARD = 0x80,

    /// Address partition used for logon features per S-9.2.1.1 and RCN-218
    ADDRESS_LOGON = 254,
    /// Address partition used for advanced extended packets per S-9.2.1.1
    ADDRESS_EXT = 253,

    // Logon commands
    /// Logon enable in the 254 address partition
    DCC_LOGON_ENABLE = 0b11111100,
    DCC_LOGON_ENABLE_MASK = 0b11111100,
    /// Select command in the 254 address partition
    DCC_SELECT = 0b11010000,
    DCC_SELECT_MASK = 0b11110000,
    /// Get Data Start command in the 254 address partition
    DCC_GET_DATA_START = 0,
    /// Get Data Continue command in the 254 address partition
    DCC_GET_DATA_CONT = 1,
    /// Logon Assign command the 254 address partition
    DCC_LOGON_ASSIGN = 0b11100000,
    DCC_LOGON_ASSIGN_MASK = 0b11110000,

    /// Minimum value of second byte for DID assigned packets.
    DCC_DID_MIN = DCC_SELECT,
    /// Maximum value of second byte for DID assigned packets.
    DCC_DID_MAX = DCC_LOGON_ASSIGN + 0xF,
    /// Mask used for the DCC_DID packets primary command byte.
    DCC_DID_MASK = 0xF0,

    // Commands in 254 Select and 253 addressed.
    CMD_READ_SHORT_INFO = 0b11111111,
    CMD_READ_BLOCK = 0b11111110,
    CMD_READ_BACKGROUND = 0b11111101,
    CMD_WRITE_BLOCK = 0b11111100,

    // Address partitions as defined by S-9.2.1.1. These are 6-bit values for
    // the first byte of the reported and assigned address.

    /// Mask for the address partition bits.
    ADR_MASK = 0b111111,

    /// 7-bit mobile decoders
    ADR_MOBILE_SHORT = 0b00111000,
    /// Mask for 7-bit mobile decoders
    ADR_MOBILE_SHORT_MASK = 0xFF,
    /// 14-bit mobile decoders
    ADR_MOBILE_LONG = 0,
    /// Maximum value of the first byte for a 14-bit mobile decoder.
    MAX_MOBILE_LONG = 0b00100111,
    /// 11-bit extended accessory decoder
    ADR_ACC_EXT = 0b00101000,
    /// Mask for 11-bit extended accessory decoder
    MASK_ACC_EXT = 0b00111000,
    /// 9-bit basic accessory decoder
    ADR_ACC_BASIC = 0b00110000,
    /// Mask for 9-bit basic accessory decoder
    MASK_ACC_BASIC = 0b00111000,

    /// This value, when given to a decoder, represents an invalid
    /// (unassignable) address. This is a 2-byte value that can go to the wire
    /// -- above we only have the constants for address partitions, which is
    /// the first byte.
    ADR_INVALID = (ADR_MOBILE_SHORT << 8),
};

/// Decodes a 14-bit address (according to S-9.2.1.1) into an address type and
/// a raw address.
/// @param addr14 a 14-bit address according to S-9.2.1.1
/// @param addr the decoded address value (8 to 14 bits)
/// @param partition the partition of the address type (the top byte, max 6 bits, e.g. ADR_MOBILE_SHORT)
/// @param atype address type enum
/// @return true if the decoding is successful, false if the value is not
/// representable.
bool decode_address_partition(uint16_t addr14, uint16_t* addr, uint8_t* partition, dcc::TrainAddressType* atype);

/// Convers a DCC basic or extended accessory decoder address from user address
/// (1-2048) to binary address (0..2047). This takes into account the 2023
/// draft of the DCC standard for address mapping. It uses per-output
/// addressing for basic accessory decoders. It uses 2044 for the address that
/// previously was the broadcast address for basic accessory decoders.
/// @param user user address for accy decoders. 1..2040 uses the legacy
/// addressing. 2041 .. 2048 uses the 2023 RCN-213 and DCC S-9.2.1 encoding.
/// @return binary address to be used in the DCC packet. The one's complement
/// of the highest address bits is not yet applied.
static inline unsigned accy_address_user_to_binary(unsigned user)
{
    if (user >= 2045)
    {
        return user - 2045;
    }
    else
    {
        return user + 3;
    }
}

/// Convers a DCC basic or extended accessory decoder address from a binary
/// address (0 - 2047) to user address (1 - 2048). This takes into account the
/// 2023 draft of the DCC standard for address mapping. It uses per-output
/// addressing for basic accessory decoders. It uses 2044 user address for the
/// binary address that previously was the broadcast for basic accessory
/// decoders.
/// @param binary the address in the DCC packet. The one's complement for the
/// highest bits should be undone already.
/// @return user user address for accy decoders. 1..2040 uses the legacy
/// addressing. 2041 .. 2048 uses the 2023 RCN-213 and DCC S-9.2.1 encoding.
static inline unsigned accy_address_binary_to_user(unsigned binary)
{
    if (binary < 4)
    {
        return binary + 2045;
    }
    else
    {
        return binary - 3;
    }
}

/// Parameters for the Logon Enable command.
enum class LogonEnableParam
{
    /// All decoders respond and ignore backoff.
    NOW = 0b11,
    /// Locomotive decoders only.
    LOCO = 0b01,
    /// Accessory decoders only.
    ACC = 0b10,
    /// All decoders respond.
    ALL = 0b00,
};

} // namespace dcc::Defs

} // namespace dcc

#endif
