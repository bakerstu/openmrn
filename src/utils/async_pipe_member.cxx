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
 * \file async_pipe_member.cxx
 *
 * Helper class for adding an asynchronous implementation to a pipe.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#include "utils/async_pipe_member.hxx"
// TODO(balazs.racz) maybe this dependency is not in the correct linking order.
#include "executor/notifiable.hxx"

AsyncPipeMember::AsyncPipeMember(Pipe* parent)
    : in_buf_(nullptr), in_count_(0), parent_(parent)
{
    parent_->RegisterMember(this);
}

AsyncPipeMember::~AsyncPipeMember()
{
    parent_->UnregisterMember(this);
}

void AsyncPipeMember::ReceiveData(void* buf, size_t count, Notifiable* done)
{
    // We don't need locking in this call, because there cannot be any other
    // pending writes, so if there is a write thread, it will be waiting on the
    // semaphore.
    HASSERT(!in_count_);  // Ensure we don't have any other pending receives.
    in_buf_ = static_cast<uint8_t*>(buf);
    in_done_ = done;
    in_count_ = count;
    receive_sem_.post();
}

void AsyncPipeMember::write(const void* buf, size_t count) {
    const uint8_t* bytes = static_cast<const uint8_t*>(buf);
    while(true) {
        if (!in_count_) {
            receive_sem_.wait();
            // We don't need locking, because by the time the semaphore
            // returns, we all class variables are filled and ReceiveData will
            // not modify them anymore.
            continue;
        }
        size_t to_copy = count;
        if (in_count_ < count) to_copy = in_count_;
        memcpy(in_buf_, bytes, to_copy);
        in_count_ -= to_copy;
        if (!in_count_) {
            // Input buffer is filled. deliver callback.
            in_buf_ = nullptr;
            Notifiable* done = in_done_;
            in_done_ = nullptr;
            done->notify();
            // Step input pointers.
            bytes += to_copy;
            count -= to_copy;
            // Wait for another ReceiveData call.
            continue;
        } else {
            // We ran out of data before filling the input buffer. Step the
            // pointers and wait for another write call.
            in_buf_ += to_copy;
            in_count_ -= to_copy;
            return;
        }
    }
}
