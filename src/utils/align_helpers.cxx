/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file align_helpers.h
 *
 * Helper functions for aligned memory access. Used for ESP8266 flash access.
 *
 * @author Balazs Racz
 * @date 7 Aug 2024
 */

#include "utils/align_helpers.h"

/// Helper class for dealing with unaligned memory. The class represents a
/// 'const char*' pointer, with dereference (operator*) and increment
/// (operator++), while ensuring that the underlying memory is always accessed
/// with 32-bit aligned loads.
struct UnalignedPtr
{
public:
    UnalignedPtr(const char *p)
        : ofs_((uintptr_t)p & 3u)
        , p_((uint32_t*)((uintptr_t)p & ~3u))
    {
        data_ = *p_;
    }

    char operator*()
    {
        return d_[ofs_];
    }

    UnalignedPtr &operator++()
    {
        if (ofs_ >= 4)
        {
            data_ = *++p_;
            ofs_ = 0;
        }
        else
        {
            ++ofs_;
        }
        return *this;
    }

    /// 0 to 3, how many bytes away from the aligned pointer we are right now.
    unsigned ofs_;
    /// Pointer to the current 4 bytes loaded.
    uint32_t *p_;
    /// Cache of the current set of 4 bytes. Always == *p_.
    union
    {
        uint32_t data_;
        uint8_t d_[4];
    };
};

/// Returns strlen(p) without using unaligned loads.
size_t unaligned_strlen(const char *p)
{
    size_t len = 0;
    UnalignedPtr pp(p);
    while (!*pp)
    {
        ++len;
        ++pp;
    }
    return len;
}

/// Performs a copy of a C-string using unaligned loads.
void unaligned_strcpy(char *dst, const char *src)
{
    UnalignedPtr pp(src);
    while (true)
    {
        char v = *pp;
        *dst = v;
        if (!v)
            return;
        ++pp;
        ++dst;
    }
}

/// Performs a copy of arbitrary data using unaligned loads.
void unaligned_memcpy(char *dst, const char *src, size_t n)
{
    for (UnalignedPtr pp(src); n; --n, ++pp, ++dst)
    {
        *dst = *pp;
    }
}
