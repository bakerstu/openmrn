/** @copyright
 * Copyright (c) 2026, Balazs Racz
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
 * @file StringPool.hxx
 *
 * A dynamically adjusting data structure for holding C strings.
 *
 * @author Balazs Racz
 * @date 2026-01-11
 */

#ifndef _UTILS_STRINGPOOL_HXX_
#define _UTILS_STRINGPOOL_HXX_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "utils/macros.h"

/// A dynamically adjusting data structure for holding C strings.
/// The StringPool allocates storage in chunks of 1 kb or memory. It supports up
/// to 63 kb of contents. Each participating string can be at most 127 bytes
/// long.
class StringPool
{
public:
    typedef uint16_t key_t;

    static constexpr size_t BLOCK_SIZE = 1024;
    static constexpr size_t MAX_BLOCKS = 63;
    static constexpr key_t INVALID_KEY = 0xFFFF;
    static constexpr key_t EMPTY_KEY = 0xFFFE;
    static constexpr size_t MAX_STRING_LEN = 127;

    StringPool();
    ~StringPool();

    /// Allocates storage for a string and copies the string into storage.
    /// @param value the string to store. Must be null-terminated.
    /// @return a key handle to the stored string, or INVALID_KEY if allocation
    /// failed, or EMPTY_KEY if the string is empty.
    key_t alloc(const char *value);

    /// Frees the storage of a previously allocated string.
    /// @param key the key of the string to free.
    void free(key_t key);

    /// @return the string stored under a given key. The returned pointer
    /// remains valid so long as the respective key is not free'd.
    const char *lookup(key_t key) const;

private:
    DISALLOW_COPY_AND_ASSIGN(StringPool);

    /// Computes a key handle from a block index and offset within the block.
    static inline key_t compute_key(uint16_t block_idx, uint16_t offset)
    {
        return (static_cast<key_t>(block_idx) << 10) | (offset & 0x03FF);
    }

    /// Extracts the block index from a key handle.
    static inline uint16_t block_idx_from_key(key_t key)
    {
        return key >> 10;
    }

    /// Extracts the byte offset within a block from a key handle.
    static inline uint16_t block_offset_from_key(key_t key)
    {
        return key & 0x03FF;
    }

    /// Tries to find a hole of at least size `size` in the existing blocks.
    /// @param size the size in bytes to allocate (including null terminator).
    /// @return a key if a hole was found, or INVALID_KEY.
    key_t find_hole(size_t size);

    /// Storage blocks.
    std::vector<char *> blocks_;

    /// Current block index we are appending to.
    uint8_t currentBlockIdx_;

    /// Current offset in the current block where free space starts.
    uint16_t currentBlockOffset_;
};

#endif // _UTILS_STRINGPOOL_HXX_
