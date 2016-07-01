/** \copyright
 * Copyright (c) 2013, D.E. Goodman-Wilson
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
 * \file mach/endian.h
 * This file represents endian swapping for OS X (MACH)
 *
 * @author D.E. Goodman-Wilson
 * @date 21 January 2013
 */

#include <architecture/byte_order.h>

/// Converts a host endian 16-bit value to big endian.
#define htobe16(x) OSSwapHostToBigInt16(x)
/// Converts a host endian 16-bit value to little endian.
#define htole16(x) OSSwapHostToLittleInt16(x)
/// Converts a big endian 16-bit value to host endian.
#define be16toh(x) OSSwapBigToHostInt16(x)
/// Converts a little endian 16-bit value to host endian.
#define le16toh(x) OSSwapLittleToHostInt16(x)
 
/// Converts a host endian 32-bit value to big endian.
#define htobe32(x) OSSwapHostToBigInt32(x)
/// Converts a host endian 32-bit value to little endian.
#define htole32(x) OSSwapHostToLittleInt32(x)
/// Converts a big endian 32-bit value to host endian.
#define be32toh(x) OSSwapBigToHostInt32(x)
/// Converts a little endian 32-bit value to host endian.
#define le32toh(x) OSSwapLittleToHostInt32(x)
 
/// Converts a host endian 64-bit value to big endian.
#define htobe64(x) OSSwapHostToBigInt64(x)
/// Converts a host endian 64-bit value to little endian.
#define htole64(x) OSSwapHostToLittleInt64(x)
/// Converts a big endian 64-bit value to host endian.
#define be64toh(x) OSSwapBigToHostInt64(x)
/// Converts a little endian 64-bit value to host endian.
#define le64toh(x) OSSwapLittleToHostInt64(x)
