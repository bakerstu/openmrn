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

namespace dcc {

/** Structure used for reading (railcom) feedback data from DCC / Railcom
 *  device drivers. */
struct Feedback {
    void reset(uint32_t feedback_key) {
        this->feedbackKey = feedback_key;
        ch1Size = 0;
        ch2Size = 0;
        channel = 0;
    }
    void add_ch1_data(uint8_t data) {
        if (ch1Size < sizeof(ch1Data)) {
            ch1Data[ch1Size++] = data;
        }
    }
    void add_ch2_data(uint8_t data) {
        if (ch2Size < sizeof(ch2Data)) {
            ch2Data[ch2Size++] = data;
        }
    }
    uint8_t ch1Size;
    uint8_t ch1Data[2];
    uint8_t ch2Size;
    uint8_t ch2Data[6];
    uint8_t channel; //< Used by multi-channel railcom receiver drivers.
    uint32_t feedbackKey;
};

namespace RailcomDefs {
  static const uint8_t INV = 0xff;
  static const uint8_t ACK = 0xfe;
  static const uint8_t NACK = 0xfd;
  static const uint8_t BUSY = 0xfc;
  static const uint8_t RESVD1 = 0xfb;
  static const uint8_t RESVD2 = 0xfa;
  static const uint8_t RESVD3 = 0xf8;
}

/** Table for 8-to-6 decoding of railcom data. This table can be indexed by the
 * 8-bit value read from the railcom channel, and the return value will be
 * either a 6-bit number, or one of the constants above. If the value is
 * invalid, the INV constant is returned. */
extern const uint8_t railcom_decode[256];

// Packet identifiers from Mobile Decoders.
enum RailcomMobilePacketId {
    RMOB_POM = 0,
    RMOB_ADRHIGH = 1,
    RMOB_ADRLOW = 2,
    RMOB_EXT = 3,
    RMOB_DYN = 4,
    RMOB_SUBID = 12,
};

}  // namespace dcc

#endif // _DCC_RAILCOM_HXX_
