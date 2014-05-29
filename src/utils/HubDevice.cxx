/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file HubDevice.cxx
 * Components for Hubs to connect to physical devices.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#include "utils/HubDevice.hxx"


template <>
const int FdHubPort<CanHubFlow>::ReadThread::kUnit = sizeof(struct can_frame);
template <>
const int FdHubPort<CanHubFlow>::ReadThread::kBufSize =
    sizeof(struct can_frame);

template <>
void FdHubPort<CanHubFlow>::ReadThread::send_message(const void *buf, int size)
{
    HASSERT(size == sizeof(struct can_frame));
    auto *b = port()->hub_->alloc();
    b->data()->skipMember_ = &port()->writeFlow_;
    memcpy(b->data()->mutable_frame(), buf, size);
    port()->hub_->send(b);
}

template <> const int FdHubPort<HubFlow>::ReadThread::kUnit = 1;
template <> const int FdHubPort<HubFlow>::ReadThread::kBufSize = 64;

template <>
void FdHubPort<HubFlow>::ReadThread::send_message(const void *buf, int size)
{
    auto *b = port()->hub_->alloc();
    b->data()->skipMember_ = &port()->writeFlow_;
    b->data()->assign((const char*)buf, size);
    port()->hub_->send(b);
}
