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
 * \file RailCom.hxx
 *
 * Helper functions and tables for RailCom implementations.
 *
 * @author Balazs Racz
 * @date 12 Nov 2014
 */

#ifndef _DCC_RAILCOM_HXX_
#define _DCC_RAILCOM_HXX_

#include <cstdint>
#include <string>
#include <vector>

#include "dcc/railcom.h"

namespace dcc
{

/// Structure used for reading (railcom) feedback data from DCC / Railcom
///  device drivers.
struct Feedback : public DCCFeedback
{
    /// Clears the structure and sets the feedback key to a specific value.
    void reset(uint32_t feedback_key)
    {
        this->feedbackKey = feedback_key;
        ch1Size = 0;
        ch2Size = 0;
        channel = 0;
    }

    /// Appends a byte to the channel 1 payload.
    void add_ch1_data(uint8_t data)
    {
        if (ch1Size < sizeof(ch1Data))
        {
            ch1Data[ch1Size++] = data;
        }
    }

    /// Appends a byte to the channel 2 payload.
    void add_ch2_data(uint8_t data)
    {
        if (ch2Size < sizeof(ch2Data))
        {
            ch2Data[ch2Size++] = data;
        }
    }
};

/// Formats a dcc::Feedback message into a debug string.
std::string railcom_debug(const Feedback& fb);

/** Table for 8-to-6 decoding of railcom data. This table can be indexed by the
 * 8-bit value read from the railcom channel, and the return value will be
 * either a 6-bit number, or one of the constants in @ref RailcomDefs. If the
 * value is invalid, the INV constant is returned. */
extern const uint8_t railcom_decode[256];
/// Table for 6-to-8 encoding of railcom data. The table can be indexed by a
/// 6-bit value that is the semantic content of a railcom byte, and returns the
/// matching 8-bit value to put out on the UART. This table only contains the
/// standard codes, for the special codes like ACK use RailcomDefs::ACK.
extern const uint8_t railcom_encode[64];

/// Special constant values returned by the @ref railcom_decode[] array.
struct RailcomDefs
{
    // These values appear in the railcom_decode table to mean special symbols.
    enum
    {
        /// invalid value (not conforming to the 4bit weighting requirement)
        INV = 0xff,
        /// Railcom ACK; the decoder received the message ok. NOTE: There are
        /// two codepoints that map to this.
        ACK = 0xfe,
        /// The decoder rejected the packet.
        NACK = 0xfd,
        /// The decoder is busy; send the packet again. This is typically
        /// returned when a POM CV write is still pending; the caller must
        /// re-try sending the packet later.
        BUSY = 0xfc,

        /// Reserved for future expansion.
        RESVD1 = 0xfb,
        /// Reserved for future expansion.
        RESVD2 = 0xfa,
    };

    // These values need to be sent on the UART
    enum
    {
        /// Code point for ACK  (according to RCN-217)
        CODE_ACK = 0xf0,
        /// Another accepted code point for ACK (according to RCN-217)
        CODE_ACK2 = 0x0f,
        /// Code point for NACK  (according to RCN-217)
        CODE_NACK = 0x3c,
        /// Code point for BUSY  (according to NMRA S-9.3.2)
        CODE_BUSY = 0xE1,
    };

    /// Encodes 12 bits of useful payload into 16 bits of UART data to transmit.
    /// @param nibble top 4 bits of the payload to send
    /// @param data bottom 8 bits of payload to send.
    /// @return the uart bytes, first byte in the high 8 bits, second byte in
    /// the low 8 bits.
    static uint16_t encode12(uint8_t nibble, uint8_t data)
    {
        return (railcom_encode[((nibble << 2) | (data >> 6)) & 0x3F] << 8) |
            railcom_encode[data & 0x3f];
    }

    /// Encodes 12 bits of useful payload into 16 bits of UART data to transmit.
    /// @param nibble top 4 bits of the payload to send
    /// @param data bottom 8 bits of payload to send.
    /// @param dst this is where the payload will be stored.
    static void append12(uint8_t nibble, uint8_t data, uint8_t* dst)
    {
        *dst++ = railcom_encode[((nibble << 2) | (data >> 6)) & 0x3F]; 
        *dst++ = railcom_encode[data & 0x3f]; 
    }

    /// Encodes a 36-bit railcom datagram into UART bytes.
    /// @param nibble the railcom ID (top 4 bits)
    /// @param data the 32 bit payload. Will be transmitted MSbyte-first.
    /// @param dst this is where the payload will be stored.
    static void append36(uint8_t nibble, uint32_t data, uint8_t* dst)
    {
        *dst++ = railcom_encode[((nibble << 2) | (data >> 30)) & 0x3F]; 
        *dst++ = railcom_encode[(data >> 24) & 0x3F]; 
        *dst++ = railcom_encode[(data >> 18) & 0x3F]; 
        *dst++ = railcom_encode[(data >> 12) & 0x3F]; 
        *dst++ = railcom_encode[(data >> 6) & 0x3F]; 
        *dst++ = railcom_encode[data & 0x3F]; 
    }

    /// Creates a Logon Enable feedback with the decoder unique ID.
    /// @param decoder_id the 44-bit decoder ID (justified to MSb).
    /// @param fb the feedback packet to generate.
    static void add_did_feedback(uint64_t decoder_id, Feedback *fb);

    /// Creates a ShortInfo feedback.
    /// @param requested_address 14-bit encoding of the requested address.
    /// @param max_fn maximum supported function (0-255)
    /// @param psupp protocol support flags (capabilities[0])
    /// @param ssupp space support flags (capabilities[1])
    /// @param fb the feedback packet to generate.
    static void add_shortinfo_feedback(uint16_t requested_address,
        uint8_t max_fn, uint8_t psupp, uint8_t ssupp, Feedback *fb);

    /// Creates a Logon Assign feedback.
    /// @param changeflags 8 bits of change flags
    /// @param changecount 12 bits of changecount
    /// @param supp2 protocol support flags (capabilities[2])
    /// @param supp3 protocol support flags (capabilities[3])
    /// @param fb the feedback packet to generate.
    static void add_assign_feedback(uint8_t changeflags, uint16_t changecount,
        uint8_t supp2, uint8_t supp3, Feedback *fb);

private:
    /// This struct cannot be instantiated.
    RailcomDefs();
};


/// Packet identifiers from Mobile Decoders.
enum RailcomMobilePacketId
{
    RMOB_POM = 0,
    RMOB_ADRHIGH = 1,
    RMOB_ADRLOW = 2,
    RMOB_EXT = 3,
    RMOB_DYN = 7,
    RMOB_XPOM0 = 8,
    RMOB_XPOM1 = 9,
    RMOB_XPOM2 = 10,
    RMOB_XPOM3 = 11,
    RMOB_SUBID = 12,
    RMOB_LOGON_ASSIGN_FEEDBACK = 13,
    RMOB_LOGON_ENABLE_FEEDBACK = 15,
};

/// Represents a single Railcom datagram. There can be multiple railcom
/// datagrams in the cutout of one railcom packet: usually zero or one in
/// channel 1 and up to four datagrams in channel 2.
struct RailcomPacket
{
    enum
    {
        GARBAGE,
        ACK,
        NACK,
        BUSY,
        MOB_POM,
        MOB_ADRHIGH,
        MOB_ADRLOW,
        MOB_EXT,
        MOB_DYN,
        MOB_XPOM0,
        MOB_XPOM1,
        MOB_XPOM2,
        MOB_XPOM3,
        MOB_SUBID
    };
    /// which detector supplied this data
    uint8_t hw_channel;
    /// which part of the railcom cutout (1 or 2)
    uint8_t railcom_channel;
    /// packet type, see enum above
    uint8_t type;
    /// payload of the railcom packet, justified to LSB.
    uint32_t argument;
    /// Constructor.
    ///
    /// @param _hw_channel which detector supplied this data
    /// @param _railcom_channel which cutout (ch1 or ch2) this is coming from
    /// @param _type see enum
    /// @param _argument payload of the railcom packet, justified to LSB.
    RailcomPacket(uint8_t _hw_channel, uint8_t _railcom_channel, uint8_t _type,
        uint32_t _argument)
        : hw_channel(_hw_channel)
        , railcom_channel(_railcom_channel)
        , type(_type)
        , argument(_argument)
    {
    }
};

/** Interprets the data from a railcom feedback. If the railcom data contains
 * error, will add a packet of type "GARBAGE" into the output list. Clears the
 * output list before fillign with the railcom data. */
void parse_railcom_data(
    const dcc::Feedback &fb, std::vector<struct RailcomPacket> *output);

}  // namespace dcc

#endif // _DCC_RAILCOM_HXX_
