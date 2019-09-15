/** \copyright
 * Copyright (c) 2015, Stuart Baker
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
 * \file main.cxx
 *
 * An application tests the Tiva/FreeRTOS implementation of the UART driver.
 *
 * @author Stuart Baker
 * @date 1 March 2015
 */

#define LOGLEVEL INFO

#include <unistd.h>
#include <fcntl.h>
#include "stdio.h"
#include <os/OS.hxx>

#include "nmranet_config.h"
#include "utils/logging.h"
#include "utils/stdio_logging.h"

#undef DIE
#define DIE(msg)                                                               \
    do                                                                         \
    {                                                                          \
        printf(msg "\n");                                                      \
        resetblink(BLINK_DIE_ABORT);                                           \
        sleep(1);                                                              \
        abort();                                                               \
    } while (false)


#include "freertos_drivers/ti/CC32xxHelper.hxx"
#include "freertos_drivers/ti/CC32xxAes.hxx"
#include "fs.h"

OVERRIDE_CONST(main_thread_stack_size, 2500);


void CheckResult(int res, int expected = 0)
{
    if (res != expected)
    {
        printf("FAILED CHECK: expected %d actual %d\n", expected, res);
        usleep(1000000);
        DIE("expect");
    }
}

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

void run_all_tests()
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
        LOG(INFO, "\nExample %d keylen=%d nlen=%d alen=%d taglen=%d", i, (int)key.size(), (int)nonce.size(), (int)auth_data.size(), (int)tag.size());
        usleep(5000);
        string o_plain;
        string o_tag;
        o_tag.resize(tag.size());
        HASSERT(cipher.size() == plain.size());
        CCMHelper::decrypt(
            key, nonce, auth_data, cipher, &o_plain, &o_tag);
        printcomp(tag, o_tag, "tag");
        printcomp(plain, o_plain, "plain");
    }
}

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    run_all_tests();
    LOG(WARNING, "done. %s.", have_failure ? " FAILURES" : "All good");
    while (1)
        ;
    return 0;
}
