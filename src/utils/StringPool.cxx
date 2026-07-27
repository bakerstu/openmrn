/** @copyright
 * Copyright (c) 2024, Balazs Racz
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
 * @file StringPool.cxx
 *
 * Implementation of StringPool.
 *
 * @author Balazs Racz
 * @date 2026-01-11
 */

#include "utils/StringPool.hxx"

#include <cstring>
#include <algorithm>

constexpr size_t StringPool::BLOCK_SIZE;
constexpr size_t StringPool::MAX_BLOCKS;
constexpr StringPool::key_t StringPool::INVALID_KEY;
constexpr StringPool::key_t StringPool::EMPTY_KEY;
constexpr size_t StringPool::MAX_STRING_LEN;

StringPool::StringPool()
    : currentBlockIdx_(0)
    , currentBlockOffset_(BLOCK_SIZE) // Force alloc first block
{
    blocks_.reserve(MAX_BLOCKS);
}

StringPool::~StringPool() {
    for (char* block : blocks_) {
        delete[] block;
    }
}

StringPool::key_t StringPool::alloc(const char* value) {
    if (!value || value[0] == '\0') {
        return EMPTY_KEY;
    }

    size_t len = strlen(value) + 1; // Include null terminator
    if (len > MAX_STRING_LEN) {
        return INVALID_KEY;
    }

    // Try to find a hole in existing blocks.
    key_t hole_key = find_hole(len);
    if (hole_key != INVALID_KEY) {
        // Found a hole. Use it.
        uint16_t block_idx = block_idx_from_key(hole_key);
        uint16_t offset = block_offset_from_key(hole_key);
        char* ptr = blocks_[block_idx] + offset;
        memcpy(ptr, value, len);
        return hole_key;
    }

    // Try to append to the current block if we have blocks and space
    if (!blocks_.empty()) {
        if (currentBlockOffset_ + len <= BLOCK_SIZE) {
            // Fits in current block at current offset
            char* ptr = blocks_[currentBlockIdx_] + currentBlockOffset_;
            memcpy(ptr, value, len);
            key_t key = compute_key(currentBlockIdx_, currentBlockOffset_);
            currentBlockOffset_ += len;
            return key;
        }
    }

    // No hole found and didn't fit in current block. Need a new block.
    if (blocks_.size() >= MAX_BLOCKS) {
        return INVALID_KEY;
    }

    // Allocate new block
    char* new_block = new char[BLOCK_SIZE];
    memset(new_block, 0, BLOCK_SIZE); // Zero initialize for invariant
    blocks_.push_back(new_block);
    
    currentBlockIdx_ = blocks_.size() - 1;
    currentBlockOffset_ = 0;

    // We know it fits because len <= MAX_STRING_LEN (127) and BLOCK_SIZE is 1024.
    memcpy(new_block, value, len);
    key_t key = compute_key(currentBlockIdx_, currentBlockOffset_);
    currentBlockOffset_ += len;
    
    return key;
}

void StringPool::free(key_t key) {
    if (key == INVALID_KEY || key == EMPTY_KEY) {
        return;
    }

    uint16_t block_idx = block_idx_from_key(key);
    uint16_t offset = block_offset_from_key(key);

    if (block_idx >= blocks_.size()) {
        return; // Out of bounds
    }

    char* ptr = blocks_[block_idx] + offset;
    // Determine length from storage
    size_t len = strlen(ptr);
    if (len == 0) {
        // Already free or empty?
        return;
    }
    
    // Zero out the string (invariant: consecutive zeros are free space)
    // We must zero out len + 1 bytes (the string + null terminator)
    memset(ptr, 0, len + 1);
}

const char* StringPool::lookup(key_t key) const {
    if (key == EMPTY_KEY) {
        return "";
    }
    if (key == INVALID_KEY) {
        return nullptr;
    }

    uint16_t block_idx = block_idx_from_key(key);
    uint16_t offset = block_offset_from_key(key);

    if (block_idx >= blocks_.size()) {
        return nullptr;
    }
    
    return blocks_[block_idx] + offset;
}

StringPool::key_t StringPool::find_hole(size_t size) {
    // Iterate through all blocks
    for (size_t b = 0; b < blocks_.size(); ++b) {
        char* block = blocks_[b];
        size_t search_limit = BLOCK_SIZE;
        
        if (b == currentBlockIdx_) {
            search_limit = currentBlockOffset_;
        }
        
        size_t i = 0;
        while (i < search_limit) {
            if (block[i] == '\0') {
                // Potential hole
                size_t free_len = 0;
                size_t start = i;
                
                // Count consecutive zeros
                while (i < search_limit && block[i] == '\0') {
                    free_len++;
                    i++;
                    
                    if (free_len >= size) {
                        return compute_key(b, start);
                    }
                }
            } else {
                // Found a non-zero byte (part of a string)
                while (i < search_limit && block[i] != '\0') {
                    i++;
                }
                // Skip the null terminator too if we haven't reached search_limit
                if (i < search_limit && block[i] == '\0') {
                    i++; 
                }
            }
        }
    }
    return INVALID_KEY;
}
