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
#include "driverlib/prcm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/aes.h"

class CCMHelper
{
public:
    static void Decrypt(const std::string &aes_key, const std::string &nonce,
        const std::string &auth_data, const std::string &cipher,
        std::string *plain, std::string *tag)
    {
        // HASSERT(32 == aes_key.size());
        // HASSERT(11 == iv.size());

        unsigned tag_len = tag->size();

        Reset();

        unsigned mode = AES_CFG_DIR_DECRYPT | AES_CFG_MODE_CCM |
            AES_CFG_CTR_WIDTH_32 |
            get_mode(aes_key.size(), nonce.size(), tag->size());
        MAP_AESConfigSet(AES_BASE, mode);

        string real_iv;
        real_iv.push_back(nonce_length_to_l(nonce.size()) - 1);
        real_iv += nonce;
        while (real_iv.size() < 16)
        {
            real_iv.push_back(0);
        }
        MAP_AESIVSet(AES_BASE, (uint8_t *)real_iv.data());

        string copy_a(auth_data);
        for(int i = 0; i < 16; ++i) copy_a.push_back(0);
        MAP_AESKey1Set(AES_BASE, (uint8_t *)aes_key.data(),
            interpret_key_size(aes_key.size()));
        plain->resize(cipher.size() + 16);
        tag->resize(16);
        MAP_AESDataProcessAE(AES_BASE, (uint8_t *)cipher.data(),
            (uint8_t *)(&plain->at(0)), cipher.size(),
            (uint8_t *)copy_a.data(), auth_data.size(),
            (uint8_t *)&tag->at(0));
        plain->resize(cipher.size());
        tag->resize(tag_len);
    }

    /// Compute the size of the L bytes
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

    /// Compute the size of the L bytes
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

    static void Encrypt(const std::string &aes_key, const std::string &iv,
        const std::string &auth_data, const std::string &plain,
        std::string *cipher, std::string *tag)
    {
    }

    static void Reset()
    {
        MAP_PRCMPeripheralClkDisable(PRCM_DTHE, PRCM_RUN_MODE_CLK);
        usleep(10);
        MAP_PRCMPeripheralClkEnable(PRCM_DTHE, PRCM_RUN_MODE_CLK);
        usleep(10);
        while (!MAP_PRCMPeripheralStatusGet(PRCM_DTHE))
            ;
    }
};

#endif // _CC32xxAESCCM_HXX_
