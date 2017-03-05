/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file CC32xxAesCcm.hxx
 *
 * Helper function to perform authentication/decryption using AES-CCM algorithm
 * with hardware crypto engine on the CC32xx.
 *
 * @author Balazs Racz
 * @date 4 Mar 2017
 */

#ifndef _CC32xxAESCCM_HXX_
#define _CC32xxAESCCM_HXX_

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_aes.h"
#include "inc/hw_dthe.h"
#include "driverlib/prcm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/aes.h"

class CCMHelper
{
public:
    static void decrypt(const std::string &aes_key, const std::string &nonce,
        const std::string &auth_data, const std::string &cipher,
        std::string *plain, std::string *tag)
    {
        CCMHelper h;
        h.decrypt_init(aes_key, nonce, auth_data, cipher.size(), tag->size());
        unsigned ofs = cipher.size() / 3;
        h.data_process(cipher.substr(0, ofs), plain);
        std::string part;
        h.data_process(cipher.substr(ofs), &part);
        *plain += part;
        h.data_finalize(&part, tag);
        *plain += part;
    }

    static void encrypt(const std::string &aes_key, const std::string &iv,
        const std::string &auth_data, const std::string &plain,
        std::string *cipher, std::string *tag)
    {
    }

    /// Initializes streaming decryption. Sets up the module for decryption and
    /// installs the keys and configuration. Processes the cleartext pre-auth
    /// data. After this call zero or more data_process calls need to be made,
    /// then a single data_finalize.
    ///
    /// @param aes_key is the secret key, has to be 16, 24 or 32 bytes long for
    /// AES-128, 192 and 256.
    /// @param nonce is the public part of the initialization vector that came
    /// with the message.
    /// @param auth_data is the cleartext header whose signature needs to be
    /// verified.
    /// @param data_len is the length of the ciphertext.
    /// @param tag_len is the length of the signature in bytes; even number
    /// from 4 to 16 (recommended 16).
    void decrypt_init(const std::string &aes_key, const std::string &nonce,
        const std::string &auth_data, unsigned data_len, uint8_t tag_len)
    {
        reset();
        tagLength_ = tag_len;

        // Sets configuration
        unsigned mode = AES_CFG_DIR_DECRYPT | AES_CFG_MODE_CCM |
            AES_CFG_CTR_WIDTH_32 |
            get_mode(aes_key.size(), nonce.size(), tag_len);
        MAP_AESConfigSet(AES_BASE, mode);

        // Computes the initialization vector form the nonce.
        string real_iv;
        real_iv.push_back(nonce_length_to_l(nonce.size()) - 1);
        real_iv += nonce;
        while (real_iv.size() < 16)
        {
            real_iv.push_back(0);
        }
        MAP_AESIVSet(AES_BASE, (uint8_t *)real_iv.data());

        // Sets key and length variables needed before starting the decryption.
        MAP_AESKey1Set(AES_BASE, (uint8_t *)aes_key.data(),
            interpret_key_size(aes_key.size()));
        MAP_AESDataLengthSet(AES_BASE, data_len);
        MAP_AESAuthDataLengthSet(AES_BASE, auth_data.size());

        // Writes the pre-authenticated data.
        for (unsigned ofs = 0; ofs < auth_data.size(); ofs += 16)
        {
            MAP_AESDataWrite(AES_BASE, (uint8_t *)&auth_data[ofs],
                std::min(16u, auth_data.size() - ofs));
        }

        // Sets up the temporary buffer.
        writeBufferLength_ = 0;
    }

    /// Process streaming encryption data.
    /// @param payload input data (ciphertext for decryption). Can be arbitrary
    /// length (will be buffered internally if needed).
    /// @param payload_out output data (cleartext for decryption). Length can
    /// be both shorter or longer than payload by a block size.
    void data_process(const std::string &payload, std::string *payload_out)
    {
        payload_out->resize(payload.size() + 16);
        const uint8_t *dstart = (uint8_t *)payload.data();
        const uint8_t *dend = (uint8_t *)payload.data() + payload.size();
        uint8_t *dout = (uint8_t *)payload_out->data();
        uint8_t *const douts = dout;
        unsigned clen;
        // First part matched with the temporary holder.
        if (writeBufferLength_ > 0)
        {
            clen = std::min(dend - dstart, 16 - writeBufferLength_);
            memcpy(writeBuffer_ + writeBufferLength_, dstart, clen);
            dstart += clen;
            writeBufferLength_ += clen;
            if (writeBufferLength_ >= 16)
            {
                process_block(writeBuffer_, dout);
                dout += 16;
                writeBufferLength_ = 0;
            }
        }
        // Middle blocks.
        while (dstart + 16 <= dend)
        {
            process_block(dstart, dout);
            dstart += 16;
            dout += 16;
        }
        // Last part goes to the holder.
        clen = std::min(dend - dstart, 16 - writeBufferLength_);
        memcpy(writeBuffer_ + writeBufferLength_, dstart, clen);
        dstart += clen;
        writeBufferLength_ += clen;

        // Finalize output size.
        payload_out->resize(dout - douts);
    }

    /// Completes the streaming operation.
    /// @param payload_out will be the remaining output data (that came from
    /// the buffer). Could be zero to 15 bytes.
    /// @param tag_out will be filled with the authentication tag.
    void data_finalize(std::string *payload_out, std::string *tag_out)
    {
        if (writeBufferLength_)
        {
            payload_out->resize(16);
            uint8_t *dout = (uint8_t *)payload_out->data();
            memset(
                writeBuffer_ + writeBufferLength_, 0, 16 - writeBufferLength_);
            process_block(writeBuffer_, dout);
        }
        payload_out->resize(writeBufferLength_);
        //
        // Wait for the context data regsiters to be ready.
        //
        while ((AES_CTRL_SVCTXTRDY & (HWREG(AES_BASE + AES_O_CTRL))) == 0)
        {
        }

        tag_out->resize(16);
        MAP_AESTagRead(AES_BASE, (uint8_t *)&(tag_out->at(0)));
        tag_out->resize(tagLength_);
    }

private:
    /// Resets / turns on the AES engine.
    static void reset()
    {
        MAP_PRCMPeripheralClkDisable(PRCM_DTHE, PRCM_RUN_MODE_CLK);
        usleep(10);
        MAP_PRCMPeripheralClkEnable(PRCM_DTHE, PRCM_RUN_MODE_CLK);
        usleep(10);
        while (!MAP_PRCMPeripheralStatusGet(PRCM_DTHE))
            ;
    }

    /// Computes the internal constant used for the configuration of the key
    /// size.
    /// @param key_len is the length of the key in bytes
    /// @return configuration constant.
    static uint32_t interpret_key_size(unsigned key_len)
    {
        switch (key_len)
        {
            case 16:
                return AES_CFG_KEY_SIZE_128BIT;
            case 24:
                return AES_CFG_KEY_SIZE_192BIT;
            case 32:
                return AES_CFG_KEY_SIZE_256BIT;
            default:
                DIE("Unknown key length");
        }
        return 0;
    }

    /// Compute the size of the L parameter in bytes from the nonce length.
    /// @param nonce_len is the length in bytes of the nonce value.
    /// @return L value (in bytes) for the encryption.
    static int nonce_length_to_l(unsigned nonce_len)
    {
        switch (nonce_len)
        {
            case 15 - 2:
                return 2;
            case 15 - 4:
                return 4;
            case 15 - 8:
                return 8;
        }
        return -1;
    }

    /// Computes mode bits for the AES engine.
    /// @param key_len length of the key in bytes
    /// @param nonce_len length of the nonce in bytes
    /// @param tag_len length of the authentication checksum in bytes
    static unsigned get_mode(
        unsigned key_len, unsigned nonce_len, unsigned tag_len)
    {
        unsigned ret = interpret_key_size(key_len);

        switch (nonce_len)
        {
            case 15 - 2:
                ret |= AES_CFG_CCM_L_2;
                break;
            case 15 - 4:
                ret |= AES_CFG_CCM_L_4;
                break;
            case 15 - 8:
                ret |= AES_CFG_CCM_L_8;
                break;
            default:
                DIE("unsupported nonce length");
        }

        switch (tag_len)
        {
            case 4:
                ret |= AES_CFG_CCM_M_4;
                break;
            case 6:
                ret |= AES_CFG_CCM_M_6;
                break;
            case 8:
                ret |= AES_CFG_CCM_M_8;
                break;
            case 10:
                ret |= AES_CFG_CCM_M_10;
                break;
            case 12:
                ret |= AES_CFG_CCM_M_12;
                break;
            case 14:
                ret |= AES_CFG_CCM_M_14;
                break;
            case 16:
                ret |= AES_CFG_CCM_M_16;
                break;
            default:
                DIE("Unsupported tag length");
        }
        return ret;
    }

    /// Performs encryption (or decryption) on a single block of
    /// data. (cleartext/ciphertext, not the authentication data).
    /// @param din unaligned pointer to 16 bytes of input.
    /// @param dout unaligned pointer to 16 bytes of output space.
    void process_block(const uint8_t *din, uint8_t *dout)
    {
        MAP_AESDataWrite(AES_BASE, (uint8_t*)din, 16);
        MAP_AESDataRead(AES_BASE, dout, 16);
    }

    uint8_t writeBuffer_[16];
    uint8_t writeBufferLength_;
    uint8_t tagLength_;
};

#endif // _CC32xxAESCCM_HXX_
