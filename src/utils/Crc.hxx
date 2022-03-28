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
 * \file Crc.hxx
 *
 * Helper functions for computing checksums.
 *
 * @author Balazs Racz
 * @date 16 Dec 2014
 */

#ifndef _UTILS_CRC_HXX_
#define _UTILS_CRC_HXX_

#include <stdint.h>
#include <stddef.h>

/** Computes the 16-bit CRC value over data using the CRC16-ANSI (aka
 * CRC16-IBM) settings. This involves zero init value, zero terminating value,
 * reversed polynomial 0xA001, reversing input bits and reversing output
 * bits. The example CRC value of "123456789" is 0xbb3d.
 * @param data what to compute the checksum over
 * @param length_bytes how long data is
 * @return the CRC-16-IBM value of the checksummed data.
 */
uint16_t crc_16_ibm(const void* data, size_t length_bytes);

/** Computes the triple-CRC value over a chunk of data. checksum is an array of
 * 3 halfwords. The first halfword will get the CRC of the data array, the
 * second halfword the CRC of all odd bytes (starting with the first byte), the
 * third halfword will get the CRC of all even bytes. 
 *
 * This routine is helpful, because a similar routine is part of the TIVA
 * microcontroller ROM, thus needs no implementation on an actual part. It is
 * important to note that the TIVA version takes the length in 4-byte words and
 * allows only using data length divisible by 4.
 * @param data what to compute the checksum over
 * @param length_bytes how long data is
 * @param checksum is the output buffer where to store the 48-bit checksum.
 */
void crc3_crc16_ibm(const void* data, size_t length_bytes, uint16_t* checksum);


/// Helper class for computing CRC-8 according to Dallas/Maxim specification
/// for 1-wire protocol. This specification is used for BiDiB, RCN-218 and
/// S-9.2.1.1.
///
/// This class can incrementally compute CRC byte by byte. There are three
/// implementations available, with different code space requirements.
class Crc8DallasMaxim {
public:
    Crc8DallasMaxim()
        : state_(0)
    {
    }

    /// Re-sets the state machine for checksumming a new message.
    void init()
    {
        state_ = 0;
    }

    /// @return the checksum of the currently consumed message.
    uint8_t get()
    {
        return state_;
    }

    /// Checks that the message has a correct CRC. This function assumes that
    /// the CRC byte has already been consumed.
    /// @return true if the message checksum is correct.
    bool check_ok()
    {
        return (state_ == 0);
    }

    /// Checks that the message has a correct CRC. This function assumes that
    /// the CRC byte was not part of the message.
    /// @param checksum_byte the CRC8 byte of the received message.
    /// @return true if the message checksum is correct.
    bool check_ok(uint8_t checksum_byte)
    {
        return (state_ == checksum_byte);
    }

    /// Processes one byte of the incoming message. No lookup table will be
    /// used.
    /// @param message_byte next byte in the message.
    void update0(uint8_t message_byte)
    {
        uint8_t data = state_ ^ message_byte;
        uint8_t crc = 0;
        if (data & 1)
        {
            crc ^= 0x5e;
        }
        if (data & 2)
        {
            crc ^= 0xbc;
        }
        if (data & 4)
        {
            crc ^= 0x61;
        }
        if (data & 8)
        {
            crc ^= 0xc2;
        }
        if (data & 0x10)
        {
            crc ^= 0x9d;
        }
        if (data & 0x20)
        {
            crc ^= 0x23;
        }
        if (data & 0x40)
        {
            crc ^= 0x46;
        }
        if (data & 0x80)
        {
            crc ^= 0x8c;
        }
        state_ = crc;
    }

    /// Processes one byte of the incoming message. A small lookup table will be
    /// used.
    /// @param message_byte next byte in the message.
    void update16(uint8_t message_byte)
    {
        uint8_t data = state_ ^ message_byte;
        state_ = tableLo16[data & 0xf] ^ tableHi16[data >> 4];
    }

    /// Processes one byte of the incoming message. A 256-byte lookup table
    /// will be used.
    /// @param message_byte next byte in the message.
    void update256(uint8_t message_byte)
    {
        state_ = table256[state_ ^ message_byte];
    }

#ifdef CRC8_TABLE_SIZE
    /// Processes one byte of the incoming message.
    /// @param message_byte next byte in the message.
    void update(uint8_t message_byte)
    {
        update##CRC8_TABLE_SIZE(message_byte);
    }
#endif

private:
    // Of the static tables here only those will be linked into a binary which
    // have been used there.
    
    /// 256-entry lookup table for the update256 function.
    static uint8_t table256[256];

    /// 16-entry lookup table for the update16 function.
    static uint8_t tableHi16[16];
    /// 16-entry lookup table for the update16 function.
    static uint8_t tableLo16[16];
    
    /// Current value of the state register for the CRC computation.
    uint8_t state_;
};

#endif // _UTILS_CRC_HXX_
