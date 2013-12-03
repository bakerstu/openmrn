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
 * \file ReadDispatch.hxx
 *
 * Class for dispatching incoming messages to handlers.
 *
 * @author Balazs Racz
 * @date 2 Dec 2013
 */

#include "nmranet/ReadDispatch.hxx"

namespace NMRAnet
{

template <typename ID>
void DispatchFlow<ID>::RegisterHandler(ID id, ID mask, HandlerBase* handler)
{
    OSMutexLock h(&lock_);
    int idx = pending_delete_index_;
    while (idx < handlers_.size() && handlers_[idx].handler) {
        ++idx;
    }
    if (idx >= handlers_.size()) {
        idx = handlers_.size();
        handlers_.resize(handlers_.size() + 1);
    }
    handlers_[idx].handler = handler;
    handlers_[idx].id = id;
    handlers_[idx].mask = mask;
}

template <typename ID>
void DispatchFlow<ID>::UnregisterHandler(ID id, ID mask, HandlerBase* handler)
{
    OSMutexLock h(&lock_);
    idx = 0;
    while (idx < handlers_.size()
           && !(handlers_[idx].handler == handler && handlers_[idx].id == id
                && handlers_[idx].mask == mask))
        ++idx;
    HASSERT(idx < handlers_.size());
    handlers_[idx].handler = nullptr;
    if (idx < pending_delete_index_) {
        pending_delete_index_ = idx;
    }
}

} // namespace NMRAnet
