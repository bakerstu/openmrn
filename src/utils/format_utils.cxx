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

char* unsigned_integer_to_buffer_hex(int value, char* buffer)
{
    int num_digits = 0;
    int tmp = value;
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
        int tmp2 = tmp % 16;
        if (tmp2 <= 9)
        {
            buffer[num_digits--] = '0' + tmp2;
        }
        else
        {
            buffer[num_digits--] = 'a' + (tmp2 - 10);
        }
        tmp /= 16;
    } while (tmp);
    HASSERT(num_digits == -1);
    return ret;
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
