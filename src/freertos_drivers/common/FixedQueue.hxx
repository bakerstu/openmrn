/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * \file FixedQueue.hxx
 * Deprecated queue class for device drivers.
 *
 * @author Stuart W. Baker
 * @date 6 May 2014
 */

#ifndef _FREERTOS_DRIVERS_COMMON_FIXEDQUEUE_HXX_
#define _FREERTOS_DRIVERS_COMMON_FIXEDQUEUE_HXX_

#include <algorithm>
#include <cstdint>

#include "Devtab.hxx"
#include "executor/Notifiable.hxx"

/// This structure is safe to use from an interrupt context and a regular
/// context at the same time, provided that
///
/// . one context uses only the front() and the other only the back() functions.
///
/// . ++ and -- are compiled into atomic operations on the processor (on the
///   count_ variable).
///
/// @deprecated, use @ref DeviceBuffer instead.
///
/// @todo(balazs.racz) replace uses of this class with DeviceBuffer (to enable
/// select support for example).
template<class T, uint8_t SIZE> class FixedQueue {
public:
    FixedQueue()
        : rdIndex_(0)
        , wrIndex_(0)
        , count_(0)
    {
    }

    /// @return true if there is no entry in the queue.
    bool empty() { return size() == 0; }
    /// @return true if the queue cannot accept more elements.
    bool full()
    {
        auto sz = size();
        if (sz >= SIZE)
        {
            return true;
        }
        if (sz != 0 && rdIndex_ == wrIndex_)
        {
            // noncommit members make the queue full.
            return true;
        }
        return false;
    }
    /// @return the current number of entries in the queue.
    size_t size() { return __atomic_load_n(&count_, __ATOMIC_SEQ_CST); }

    /// Returns the head of the FIFO (next element to read).
    T& front() {
        HASSERT(!empty());
        return storage_[rdIndex_];
    }

    /// Removes the head of the FIFO from the queue.
    void increment_front() {
        HASSERT(!empty());
        if (++rdIndex_ >= SIZE) rdIndex_ = 0;
        __atomic_fetch_add(&count_, -1, __ATOMIC_SEQ_CST);
    }

    /// Returns the space to write the next element to.
    T& back() {
        HASSERT(!full());
        return storage_[wrIndex_];
    }

    /// Commits the element at back() into the queue.
    void increment_back() {
        HASSERT(!full());
        if (++wrIndex_ >= SIZE) wrIndex_ = 0;
        __atomic_fetch_add(&count_, 1, __ATOMIC_SEQ_CST);
    }

    /** Increments the back pointer without committing the entry into the
     * queue.
     *
     * This essentially reserves an entry in the queue for filling in, without
     * making that entry available for reading. Must be followed by a
     * commit_back call when filling in the entry is finished. An arbitrary
     * number of such entries can be reserved (up to the number of free entries
     * in the queue). */
    void noncommit_back() {
        HASSERT(has_noncommit_space());
        if (++wrIndex_ >= SIZE) wrIndex_ = 0;
    }

    /** @returns true if we can do a noncommit back. */
    bool has_noncommit_space() {
        if (full()) return false;
        auto new_index = wrIndex_;
        if (++new_index >= SIZE) new_index = 0;
        return new_index != rdIndex_;
    }

    /** Commits the oldest entry reserved by noncommit_back. */
    void commit_back() {
        HASSERT(count_ <= SIZE);
        __atomic_fetch_add(&count_, 1, __ATOMIC_SEQ_CST);
    }

    /// Checks if there is space in the back. If yes, allocates one entry as
    /// noncommit space and returns the pointer. If full, returns nullptr.
    /// @return nullptr if the queue is full, otherwise a noncommit entry.
    T *noncommit_back_or_null()
    {
        auto sz = size();
        if (sz >= SIZE)
        {
            return nullptr;
        }
        if (sz != 0 && rdIndex_ == wrIndex_)
        {
            // noncommit members make the queue full.
            return nullptr;
        }
        auto *ret = &storage_[wrIndex_];
        if (++wrIndex_ >= SIZE)
            wrIndex_ = 0;
        return ret;
    }

private:
    /// Payload of elements stored.
    T storage_[SIZE];
    /// The index of the element to return next upon a read. This element is
    /// typically full (unless the queue is empty itself).
    uint8_t rdIndex_;
    /// The index of the element where to write the next input to.
    uint8_t wrIndex_;
    /// How many elements are there in the queue.
    volatile uint8_t count_;
};

#endif  // _FREERTOS_DRIVERS_COMMON_FIXEDQUEUE_HXX_
