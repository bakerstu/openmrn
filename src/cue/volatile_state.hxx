/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file volatile_state.hxx
 * Contains a utility class for keeping volatile state and communicating
 * updates about it between components.
 *
 * In this context "volatile state" means that these state bytes can be changed
 * by other threads doing I/O or other processing asynchronously.
 *
 * @author Balazs Racz
 * @date 19 July 2013
 */

#ifndef _CUE_VOLATILE_STATE_HXX_
#define _CUE_VOLATILE_STATE_HXX_

#include <stdint.h>
#include <unistd.h>

#include <vector>

#include "utils/macros.h"
#include "utils/singleton.hxx"

class VolatileState;
struct VolatileStateRef;
struct VolatileStatePtr;

class VolatileState : public Singleton<VolatileState>
{
public:
    VolatileState(size_t size) : values_(size, 0)
    {
    }

    VolatileStateRef GetRef(int offset);

    VolatileStatePtr GetPtr(int offset);

    size_t size()
    {
        return values_.size();
    }

    uint8_t Get(size_t offset)
    {
        HASSERT(offset < size());
        // @todo (balazs.racz) locking
        return values_[offset];
    }

    void Set(size_t offset, uint8_t value)
    {
        HASSERT(offset < size());
        // @todo (balazs.racz) locking
        // @todo (balazs.racz) update notification
        values_[offset] = value;
    }

private:
    friend class VolatileStateRef;

    vector<uint8_t> values_;

    DISALLOW_COPY_AND_ASSIGN(VolatileState);
};

struct VolatileStateRef
{
    VolatileStateRef() = delete;

    explicit VolatileStateRef(int b) : base_(b)
    {
        HASSERT(base_ >= 0);
        HASSERT(static_cast<size_t>(base_)
                < VolatileState::instance()->values_.size());
    }

    operator uint8_t() const
    {
        return VolatileState::instance()->Get(base_);
    }

    VolatileStateRef& operator=(uint8_t value)
    {
        VolatileState::instance()->Set(base_, value);
        return *this;
    }

    int base_;
};

struct VolatileStatePtr
{
    explicit VolatileStatePtr(int base) : base_(base)
    {
    }

    VolatileStateRef operator*()
    {
        return VolatileStateRef(base_);
    }

    VolatileStateRef operator[](int offset)
    {
        return VolatileStateRef(base_ + offset);
    }

    int base_;
};

inline VolatileStateRef VolatileState::GetRef(int offset)
{
    return VolatileStateRef(offset);
}

inline VolatileStatePtr VolatileState::GetPtr(int offset)
{
    return VolatileStatePtr(offset);
}

#endif //_CUE_VOLATILE_STATE_HXX_
