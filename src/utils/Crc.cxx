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
 * \file Crc.cxx
 *
 * Helper functions for computing checksums.
 *
 * @author Balazs Racz
 * @date 16 Dec 2014
 */

#include <stdint.h>

#include "utils/Crc.hxx"
#include "utils/macros.h"

/// Initialization value for the CRC-16-IBM calculator.
static const uint16_t crc_16_ibm_init_value = 0x0000; // TODO: check
/// Polynomial for the CRC-16-IBM calculator.
static const uint16_t crc_16_ibm_poly = 0xA001; // TODO: check

/// Reverses the bits of a byte.
///
/// @param data input byte
///
/// @return the input bits reversed (bit 0 exchanged with bit 7 et al.)
///
uint8_t reverse(uint8_t data) {
    uint8_t out = 0;
    for (int i = 0; i < 8; i++) {
        out = (out << 1) | (data & 1);
        data >>= 1;
    }
    return out;
}

/*

static const uint16_t crc_16_ibm_poly = 0x8005; // WORKING

inline void crc_16_ibm_add(uint16_t& state, uint8_t data) {
    //data = reverse(data);
    // This does LSB-first.
    for (int i = 0; i < 8; i++) {
        bool bit = data & 1;
        data >>= 1;
        if (state & 0x8000) {
            bit = !bit;
        }
        if (bit) {
            state = (state << 1) ^ crc_16_ibm_poly;
        } else {
            state = (state << 1);
        }
    }
}

inline uint16_t crc_16_ibm_finish(uint16_t state) {
    return (reverse(state & 0xff) << 8) | reverse(state >> 8);
    //crc_16_ibm_add(state, 0);
    //crc_16_ibm_add(state, 0);
    //return state;
    }*/

/// Translation table for crc16-ibm, high nibble.
static const uint16_t CRC16_IBM_HI[16] =
{
    0x0000, 0xcc01, 0xd801, 0x1400, 0xf001, 0x3c00, 0x2800, 0xe401, 0xa001, 0x6c00, 0x7800, 0xb401, 0x5000, 0x9c01, 0x8801, 0x4400, 
};
/// Translation table for crc16-ibm, low nibble.
static const uint16_t CRC16_IBM_LO[16] =
{
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440, 
};

/// Appends a byte to a CRC16 state machine.
///
/// @param state the state machine of the CRC computer.
/// @param data next byte to add.
///
inline void __attribute__((always_inline)) crc_16_ibm_add(uint16_t &state, uint8_t data)
{
    state ^= data;
    state = (state >> 8) ^ CRC16_IBM_LO[state & 0xf] ^
        CRC16_IBM_HI[(state >> 4) & 0xf];
}

/// Bitwise implementation of the crc16 ibm add.
void crc_16_ibm_add_basic(uint16_t &state, uint8_t data)
{
    state ^= data;
    for (int i = 0; i < 8; i++)
    {
        if (state & 1)
        {
            state = (state >> 1) ^ crc_16_ibm_poly;
        }
        else
        {
            state = (state >> 1);
        }
    }
}

/// Finalizes the state machine of a CRC16-IBM calculator.
///
/// @param state internal state of the machine.
///
/// @return the CRC-16-IBM value of the byte sequence added to the state
/// machine during its lifetime.
///
inline uint16_t crc_16_ibm_finish(uint16_t state) {
    //return (reverse(state & 0xff) << 8) | reverse(state >> 8);
    //crc_16_ibm_add(state, 0);
    //crc_16_ibm_add(state, 0);
    return state;
}


uint16_t crc_16_ibm(const void* data, size_t length) {
    const uint8_t *payload = static_cast<const uint8_t*>(data);
    uint16_t state = crc_16_ibm_init_value;
    for (size_t i = 0; i < length; ++i) {
        crc_16_ibm_add(state, payload[i]);
    }
    return crc_16_ibm_finish(state);
}

void crc3_crc16_ibm(const void* data, size_t length_bytes, uint16_t* checksum) {
  uint16_t state1 = crc_16_ibm_init_value;
  uint16_t state2 = crc_16_ibm_init_value;
  uint16_t state3 = crc_16_ibm_init_value;

#ifdef ESP_NONOS
  // Aligned reads only.
  const uint32_t* payload = static_cast<const uint32_t*>(data);
  HASSERT((((uint32_t)payload) & 3) == 0);
  uint32_t cword = 0;
  for (size_t i = 1; i <= length_bytes; ++i) {
      if ((i & 3) == 1) {
          cword = payload[(i - 1) >> 2];
      } else {
          cword >>= 8;
      }
      uint8_t cbyte = cword & 0xff;
      crc_16_ibm_add(state1, cbyte);
      if (i & 1) {
          // odd byte
          crc_16_ibm_add(state2, cbyte);
      } else {
          // even byte
          crc_16_ibm_add(state3, cbyte);
      }
  }
#else
  const uint8_t *payload = static_cast<const uint8_t*>(data);
  for (size_t i = 1; i <= length_bytes; ++i) {
    crc_16_ibm_add(state1, payload[i-1]);
    if (i & 1) {
      // odd byte
      crc_16_ibm_add(state2, payload[i-1]);
    } else {
      // even byte
      crc_16_ibm_add(state3, payload[i-1]);
    }
  }
#endif

  checksum[0] = crc_16_ibm_finish(state1);
  checksum[1] = crc_16_ibm_finish(state2);
  checksum[2] = crc_16_ibm_finish(state3);
}

// static
uint8_t Crc8DallasMaxim::table256[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

// static
uint8_t Crc8DallasMaxim::tableLo16[16] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
};

// static
uint8_t Crc8DallasMaxim::tableHi16[16] = {
    0x00, 0x9d, 0x23, 0xbe, 0x46, 0xdb, 0x65, 0xf8,
    0x8c, 0x11, 0xaf, 0x32, 0xca, 0x57, 0xe9, 0x74
};
