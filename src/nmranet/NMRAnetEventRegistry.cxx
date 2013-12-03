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
 * \file NMRAnetEventRegistry.cxx
 * Static declarations for handling NMRAnet events.
 *
 * @author Balazs Racz
 * @date 19 October 2013
 */

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/NMRAnetWriteFlow.hxx"

namespace NMRAnet
{

NMRAnetEventRegistry* NMRAnetEventRegistry::instance_ = nullptr;

AllocatorMutex event_handler_mutex;
WriteHelper event_write_helper1(DefaultWriteFlowExecutor());
WriteHelper event_write_helper2(DefaultWriteFlowExecutor());
WriteHelper event_write_helper3(DefaultWriteFlowExecutor());
WriteHelper event_write_helper4(DefaultWriteFlowExecutor());
BarrierNotifiable event_barrier;

NMRAnetEventRegistry::NMRAnetEventRegistry()
{
    HASSERT(instance_ == nullptr);
    instance_ = this;
}

NMRAnetEventRegistry::~NMRAnetEventRegistry()
{
    HASSERT(instance_ == this);
    instance_ = nullptr;
}

}; /* namespace NMRAnet */
