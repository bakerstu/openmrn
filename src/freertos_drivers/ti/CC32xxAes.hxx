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

#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_aes.h"
#include "inc/hw_dthe.h"
#include "driverlib/prcm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/aes.h"

#include "utils/SyncStream.hxx"

/// Helper class for doing CCM encryption-with-authentication using the
/// CC32xx's hardware AES engine.
class CCMHelper
{
public:
    /// Performs authenticated decryption using CCM in one call. Use this when
    /// all of the ciphertext is available.
    ///
    /// @param aes_key is the secret key. 16, 24 or 32 bytes long.
    /// @param nonce is the use-once initialization that came in cleartext with
    /// the encrypted data.
    /// @param auth_data is cleartext data (usually packet headers) that need
    /// to be checked for authenticity.
    /// @param cipher is the encrypted data.
    /// @param plain will hold the decrypted data.
    /// @param tag will hold the authentication checksum. Make sure to resize
    /// this to the appropriate number of bytes before calling (suggested 16
    /// bytes).
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

    /// Performs authenticated encryption using CCM in one call. Use this when
    /// all the plaintext is available and both cipher and plaintext fits into
    /// memory.
    ///
    /// @param aes_key is the secret key. 16, 24 or 32 bytes long.
    /// @param nonce is the use-once initialization. Must be 13, 11 or 7 bytes
    /// long.
    /// @param auth_data is cleartext data (usually packet headers) that need
    /// to be checked for authenticity.
    /// @param plain is the plain text data.
    /// @param tag_len is the desired length of the authentication
    /// tag. Recommend 16.
    /// @param cipher will hold the encrypted data.
    /// @param tag will hold the authentication checksum. Will be resized to
    /// tag_len.
    static void encrypt(const std::string &aes_key, const std::string &nonce,
        const std::string &auth_data, const std::string &plain,
        unsigned tag_len, std::string *cipher, std::string *tag)
    {
        static_encrypt_init(aes_key, nonce, auth_data, plain.size(), tag_len);
        cipher->resize(plain.size() + 15);
        int len = plain.size();
        unsigned ofs = 0;
        while (len >= 16)
        {
            process_block(
                (const uint8_t *)&plain[ofs], (uint8_t *)&(*cipher)[ofs]);
            ofs += 16;
            len -= 16;
        }
        if (len > 0)
        {
            uint8_t buf[16];
            memset(buf, 0, sizeof(buf));
            memcpy(buf, &plain[ofs], len);
            process_block(buf, (uint8_t *)&(*cipher)[ofs]);
        }
        cipher->resize(plain.size());
        tag->resize(tag_len);
        read_tag(tag);
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
        // will set the strings on each write.
        outputStream_ = new StringAppendStream(nullptr); 
        decryptorStream_.reset(create_decryptor_stream(aes_key, nonce, auth_data, data_len, tag_len, outputStream_, &tagOut_, nullptr));
    }

    static void static_decrypt_init(const std::string &aes_key,
        const std::string &nonce, const std::string &auth_data,
        unsigned data_len, uint8_t tag_len)
    {
        reset();
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
    }

    static void static_encrypt_init(const std::string &aes_key,
        const std::string &nonce, const std::string &auth_data,
        unsigned data_len, uint8_t tag_len)
    {
        reset();
        // Sets configuration
        unsigned mode = AES_CFG_DIR_ENCRYPT | AES_CFG_MODE_CCM |
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
    }

    /// Process streaming encryption data.
    /// @param payload input data (ciphertext for decryption). Can be arbitrary
    /// length (will be buffered internally if needed).
    /// @param payload_out output data (cleartext for decryption). Length can
    /// be both shorter or longer than payload by a block size.
    void data_process(const std::string &payload, std::string *payload_out)
    {
        payload_out->clear();
        payload_out->reserve(payload.size() + 16);
        outputStream_->set_output(payload_out);
        auto ret = decryptorStream_->write_all(payload.data(), payload.size());
        HASSERT(ret == (ssize_t)payload.size());
    }

    /// Completes the streaming operation.
    /// @param payload_out will be the remaining output data (that came from
    /// the buffer). Could be zero to 15 bytes.
    /// @param tag_out will be filled with the authentication tag.
    void data_finalize(std::string *payload_out, std::string *tag_out)
    {
        payload_out->clear();
        outputStream_->set_output(payload_out);
        auto ret = decryptorStream_->finalize(0);
        HASSERT(ret == 0);
        tag_out->swap(tagOut_);
    }

    /// Creates a stream for on-the-fly receiving encrypted data and passing on
    /// decrypted data to a consumer stream.
    /// @param aes_key is the secret key, has to be 16, 24 or 32 bytes long for
    /// AES-128, 192 and 256.
    /// @param nonce is the public part of the initialization vector that came
    /// with the message.
    /// @param auth_data is the cleartext header whose signature needs to be
    /// verified.
    /// @param data_len is the length of the ciphertext. You must send exactly
    /// this many bytes to the stream.
    /// @param tag_len is the length of the signature in bytes; even number
    /// from 4 to 16 (recommended 16).
    /// @param consumer is the stream that will receive the decrypted
    /// bytes. Takes ownership of it.
    /// @param tag_out will be set to the authentication tag after the stream
    /// is finalized.
    /// @param expected_tag if not null, will be compared to the actual tag in
    /// case of mismatch a finalization error will be generated.
    /// @return a stream to which the encrypted bytes need to be written. The
    /// caller owns this stream and is required to finalize() it in order to
    /// read the tag bytes.
    static SyncStream *create_decryptor_stream(const std::string &aes_key,
        const std::string &nonce, const std::string &auth_data,
        unsigned data_len, uint8_t tag_len, SyncStream *consumer,
                                               std::string *tag_out, const std::string* expected_tag)
    {
        static_decrypt_init(aes_key, nonce, auth_data, data_len, tag_len);
        tag_out->resize(tag_len);
        return new MinWriteStream(
            16, new DecryptorStream(
                tag_out, expected_tag, new MaxLengthStream(data_len, consumer)));
    }

    class DecryptorStream : public WrappedStream
    {
    public:
        DecryptorStream(std::string *tag_out, const std::string *expected_tag,
            SyncStream *consumer)
            : WrappedStream(consumer)
            , tagOut_(tag_out)
            , expectedTag_(expected_tag)
        {
        }

        ssize_t write(const void *data, size_t len) override
        {
            if (len < 16)
                return -1;
            unsigned ll = 0;
            auto rp = (const uint8_t *)(data);
            auto wp = (uint8_t *)(data);
            while (len >= 16)
            {
                process_block(rp, wp);
                rp += 16;
                wp += 16;
                ll += 16;
                len -= 16;
            }
            delegate_->write_all(data, ll);
            return ll;
        }

        int finalize(int status) override
        {
            read_tag(tagOut_);
            if (expectedTag_ && (*expectedTag_ != *tagOut_)) {
                status = -2;
            }
            return WrappedStream::finalize(status);
        }

    private:
        std::string *tagOut_;
        const std::string *expectedTag_;
    };

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
    static void process_block(const uint8_t *din, uint8_t *dout)
    {
        MAP_AESDataWrite(AES_BASE, (uint8_t *)din, 16);
        MAP_AESDataRead(AES_BASE, dout, 16);
    }

    /// Finalizes encyrption/decryption and reads out the checksum tag.
    /// @param tag_out needs to be resized to the desired number of bytes length
    /// before calling. It will be filled with the checksum bytes.
    static void read_tag(std::string* tag_out) {
        uint8_t tag[16];
        //
        // Wait for the context data registers to be ready.
        //
        while ((AES_CTRL_SVCTXTRDY & (HWREG(AES_BASE + AES_O_CTRL))) == 0)
        {
        }

        MAP_AESTagRead(AES_BASE, tag);
        tag_out->assign((const char*)tag, tag_out->size());
    }

    /// Implementation object.
    std::unique_ptr<SyncStream> decryptorStream_;
    /// Stream that catches the decrypted output in the user-provided
    /// strings. Owned by decryptorStream_.
    StringAppendStream* outputStream_;
    /// Variable that will receive the output tag.
    std::string tagOut_;
};

#endif // _CC32xxAESCCM_HXX_
