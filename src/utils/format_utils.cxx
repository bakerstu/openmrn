/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file format_utils.hxx
 *
 * Simple codefor some trivial formatting uses.
 *
 * @author Balazs Racz
 * @date 7 Feb 2016
 */

#include "utils/macros.h"
#include "utils/format_utils.hxx"

/// Translates a number 0..15 to a hex character.
/// @param nibble input number
/// @return character in 0-9a-f
static char nibble_to_hex(unsigned nibble)
{
    return nibble <= 9 ? '0' + nibble : 'a' + (nibble - 10);
}

char* unsigned_integer_to_buffer_hex(unsigned int value, char* buffer)
{
    int num_digits = 0;
    unsigned int tmp = value;
    do
    {
        num_digits++;
        tmp /= 16;
    } while (tmp > 0);
    char* ret = buffer + num_digits--;
    *ret = 0;
    tmp = value;
    do
    {
        HASSERT(num_digits >= 0);
        buffer[num_digits--] = nibble_to_hex(tmp & 0xf);
        tmp >>= 4;
    } while (tmp);
    HASSERT(num_digits == -1);
    return ret;
}

char* uint64_integer_to_buffer_hex(uint64_t value, char* buffer)
{
    int num_digits = 0;
    uint64_t tmp = value;
    do
    {
        num_digits++;
        tmp /= 16;
    } while (tmp > 0);
    char* ret = buffer + num_digits--;
    *ret = 0;
    tmp = value;
    do
    {
        HASSERT(num_digits >= 0);
        buffer[num_digits--] = nibble_to_hex(tmp & 0xf);
        tmp >>= 4;
    } while (tmp);
    HASSERT(num_digits == -1);
    return ret;
}

char* int64_integer_to_buffer_hex(int64_t value, char* buffer)
{
    if (value < 0)
    {
        *buffer = '-';
        ++buffer;
        value = -value;
    }
    return uint64_integer_to_buffer_hex(value, buffer);
}

char* unsigned_integer_to_buffer(int value, char* buffer)
{
    int num_digits = 0;
    int tmp = value;
    do
    {
        num_digits++;
        tmp /= 10;
    } while (tmp > 0);
    char* ret = buffer + num_digits--;
    *ret = 0;
    tmp = value;
    do
    {
        HASSERT(num_digits >= 0);
        buffer[num_digits--] = '0' + (tmp % 10);
        tmp /= 10;
    } while (tmp);
    HASSERT(num_digits == -1);
    return ret;
}

char* uint64_integer_to_buffer(uint64_t value, char* buffer)
{
    int num_digits = 0;
    uint64_t tmp = value;
    do
    {
        num_digits++;
        tmp /= 10;
    } while (tmp > 0);
    char* ret = buffer + num_digits--;
    *ret = 0;
    tmp = value;
    do
    {
        HASSERT(num_digits >= 0);
        buffer[num_digits--] = '0' + (tmp % 10);
        tmp /= 10;
    } while (tmp);
    HASSERT(num_digits == -1);
    return ret;
}

char* integer_to_buffer(int value, char* buffer)
{
    if (value < 0)
    {
        *buffer = '-';
        ++buffer;
        value = -value;
    }
    return unsigned_integer_to_buffer(value, buffer);
}

char* int64_integer_to_buffer(int64_t value, char* buffer)
{
    if (value < 0)
    {
        *buffer = '-';
        ++buffer;
        value = -value;
    }
    return uint64_integer_to_buffer(value, buffer);
}

string integer_to_string(int value, unsigned padding)
{
    string ret;
    char tmp[12];
    integer_to_buffer(value, tmp);
    ret.append(tmp);
    if (padding > ret.size())
    {
        ret.insert(0, padding - ret.size(), ' ');
    }
    return ret;
}

string uint64_to_string(uint64_t value, unsigned padding)
{
    string ret;
    char tmp[21];
    uint64_integer_to_buffer(value, tmp);
    ret.append(tmp);
    if (padding > ret.size())
    {
        ret.insert(0, padding - ret.size(), ' ');
    }
    return ret;
}

string int64_to_string(int64_t value, unsigned padding)
{
    string ret;
    char tmp[22];
    int64_integer_to_buffer(value, tmp);
    ret.append(tmp);
    if (padding > ret.size())
    {
        ret.insert(0, padding - ret.size(), ' ');
    }
    return ret;
}

string uint64_to_string_hex(uint64_t value, unsigned padding)
{
    string ret;
    char tmp[17];
    uint64_integer_to_buffer_hex(value, tmp);
    ret.append(tmp);
    if (padding > ret.size())
    {
        ret.insert(0, padding - ret.size(), ' ');
    }
    return ret;
}

string int64_to_string_hex(int64_t value, unsigned padding)
{
    string ret;
    char tmp[18];
    int64_integer_to_buffer_hex(value, tmp);
    ret.append(tmp);
    if (padding > ret.size())
    {
        ret.insert(0, padding - ret.size(), ' ');
    }
    return ret;
}

string string_to_hex(const string &arg)
{
    string ret;
    ret.reserve(arg.size() * 2);
    for (char c : arg)
    {
        uint8_t cc = static_cast<uint8_t>(c);
        ret.push_back(nibble_to_hex((cc >> 4) & 0xf));
        ret.push_back(nibble_to_hex(cc & 0xf));
    }
    return ret;
}

static uint8_t get_nibble(char b)
{
    if ('0' <= b && b <= '9')
    {
        return b - '0';
    }
    if ('a' <= b && b <= 'f')
    {
        return b - 'a' + 10;
    }
    if ('A' <= b && b <= 'F')
    {
        return b - 'A' + 10;
    }
    return 0xff;
}

size_t hex_to_string(
    const char *input, size_t len, string *output, bool ignore_nonhex)
{
    uint8_t b = 0;         // current byte
    bool next_high = true; // next nibble is high of the byte
    for (size_t ofs = 0; ofs < len; ++ofs)
    {
        uint8_t nib = get_nibble(input[ofs]);
        if (nib == 0xff)
        {
            if (!ignore_nonhex)
            {
                return ofs;
            }
            continue;
        }
        b |= nib & 0xf;
        if (next_high)
        {
            b <<= 4;
            next_high = false;
            continue;
        }
        else
        {
            output->push_back(b);
            b = 0;
            next_high = true;
            continue;
        }
    }
    return len;
}

string mac_to_string(uint8_t mac[6], char colon)
{
    string ret;
    ret.reserve(12+6);
    char tmp[10];
    for (int i = 0; i < 6; ++i)
    {
        unsigned_integer_to_buffer_hex(mac[i], tmp);
        if (!tmp[1])
        {
            ret.push_back('0');
        }
        ret += tmp;
        if (colon)
        {
            ret.push_back(colon);
        }
    }
    if (colon)
    {
        ret.pop_back();
    }
    return ret;
}

string ipv4_to_string(uint8_t ip[4])
{
    string ret;
    ret.reserve(12+4);
    char tmp[10];
    for (int i = 3; i >= 0; --i)
    {
        unsigned_integer_to_buffer(ip[i], tmp);
        ret += tmp;
        ret.push_back('.');
    }
    ret.pop_back();

    return ret;
}