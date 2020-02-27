/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file CC32xxAesTest.hxx
 *
 * Test runner for the CC32xx AES-CCM and SHA tests.
 *
 * @author Balazs Racz
 * @date 29 July 2019
 */

#ifndef _FREERTOS_DRIVERS_TI_CC32XXAESTEST_HXX_
#define _FREERTOS_DRIVERS_TI_CC32XXAESTEST_HXX_

#define LOGLEVEL INFO

#include "freertos_drivers/ti/CC32xxAes.hxx"
#include "freertos_drivers/ti/CC32xxHelper.hxx"
#include "freertos_drivers/ti/CC32xxSha.hxx"
#include "fs.h"

namespace aes_test {

int nibble_to_int(char n)
{
    if ('0' <= n && n <= '9')
    {
        return n - '0';
    }
    if ('a' <= n && n <= 'f')
    {
        return n - 'a' + 10;
    }
    if ('A' <= n && n <= 'F')
    {
        return n - 'A' + 10;
    }
    DIE("Unknown nibble arrived.");
}

std::string hex2str(const char *hex)
{
    std::string ret;
    while (*hex && *(hex + 1))
    {
        if (*hex == ' ') {
            ++hex;
            continue;
        }
        ret.push_back((nibble_to_int(*hex) << 4) | (nibble_to_int(*(hex + 1))));
        hex += 2;
    }
    return ret;
}

const char HEXCHR[17] = "0123456789abcdef";

std::string str2hex(const string& s) {
    std::string ret;
    for (char c : s) {
        ret.push_back(HEXCHR[c>>4]);
        ret.push_back(HEXCHR[c & 0xf]);
    }
    return ret;
}

void get_example(int index, string &Key, string &Nonce, string &Adata,
    string &Payload, string &CT)
{
#include "utils/AesCcmTestVectors.hxx"
#include "utils/AesCcmTestVectorsEx.hxx"
}

void get_sha_example(int index, string &Key, string &Hash, string &Payload)
{
#include "utils/ShaTestVectors.hxx"
}

bool have_failure = false;

void printcomp(const string& exp, const string& act, const char* name) {
    if (exp == act) {
        LOG(INFO, "%s correct.", name);
        return;
    }
    have_failure = true;
    LOG(INFO, "%s failed.", name);
    LOG(INFO, "Expected=%s", str2hex(exp).c_str());
    LOG(INFO, "Actual  =%s", str2hex(act).c_str());
}

/// @return true if the tests passed, false if they failed.
bool run_all_tests()
{
    string key;
    string nonce;
    string auth_data;
    string plain;
    string cipher;
    string tag;

    for (int i : {40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 60, 61, 62, 63, 63, 64, 65, 66, 10, 11, 12, 13, 14, 15, 16, 17, 18})
    {
        get_example(i, key, nonce, auth_data, plain, cipher);
        // Move the last part of the cipher to the tag.
        tag = cipher.substr(plain.size());
        cipher.resize(plain.size());
        LOG(INFO, "\nExample %d keylen=%d nlen=%d alen=%d taglen=%d datalen=%d", i, (int)key.size(), (int)nonce.size(), (int)auth_data.size(), (int)tag.size(), (int)plain.size());
        usleep(5000);
        string o_plain;
        string o_tag;
        o_tag.resize(tag.size());
        HASSERT(cipher.size() == plain.size());
        CCMHelper::decrypt(
            key, nonce, auth_data, cipher, &o_plain, &o_tag);
        printcomp(tag, o_tag, "tag");
        printcomp(plain, o_plain, "plain");

        LOG(INFO, "\nEncrypting");

        string o_cipher;
        o_tag.clear();
        CCMHelper::encrypt(
            key, nonce, auth_data, plain, tag.size(), &o_cipher, &o_tag);
        printcomp(tag, o_tag, "tag");
        printcomp(cipher, o_cipher, "cipher");
    }

    for (int i = 0; i <= 67; i++)
    {
        string digest;
        get_sha_example(i, key, digest, plain);
        LOG(INFO, "SHA256 Example %d datalen=%d", i, (int)plain.size());
        string o_digest = SHAHelper::sha256(plain.data(), plain.size());
        printcomp(digest, o_digest, "hash");
    }

    return !have_failure;
}

} // namespace aes_test

#endif // _FREERTOS_DRIVERS_TI_CC32XXAESTEST_HXX_
