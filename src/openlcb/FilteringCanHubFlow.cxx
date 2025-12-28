/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file FilteringCanHubFlow.cxx
 *
 * Can Frame Hub that performs addressed message filtering for openlcb.
 *
 * @author Balazs Racz
 * @date 28 Dec 2025
 */

#include "openlcb/FilteringCanHubFlow.hxx"

namespace openlcb
{

FilteringCanHubFlow::FilteringCanHubFlow(Service *service)
    : CanHubFlow(service)
    , isFiltering_(true)
{
}

StateFlowBase::Action FilteringCanHubFlow::entry()
{
    if (isFiltering_)
    {
        filter_.prepare_packet(message()->data());
    }
    return CanHubFlow::entry();
}

void FilteringCanHubFlow::unregister_port(CanHubFlow::port_type *port)
{
    filter_.remove_port(reinterpret_cast<uintptr_t>(port));
    CanHubFlow::unregister_port(port);
}

StateFlowBase::Action FilteringCanHubFlow::iterate()
{
    if (!isFiltering_)
    {
        return CanHubFlow::iterate();
    }

    // Filtering behavior
    {
        OSMutexLock l(&lock_);
        for (; currentIndex_ < handlers_.size(); ++currentIndex_)
        {
            auto &h = handlers_[currentIndex_];
            // Filtering check. Will also prevent loopback.
            if (!filter_.is_matching(reinterpret_cast<uintptr_t>(h.handler)))
            {
                continue;
            }

            // At this point: we have another handler.
            if (!lastHandlerToCall_)
            {
                // This was the first we found.
                lastHandlerToCall_ = handlers_[currentIndex_].handler;
                continue;
            }
            break;
        }
    }
    if (currentIndex_ >= handlers_.size())
    {
        return iteration_done();
    }
    // Now: we have at least two different handlers. We need to clone the
    // message. We use the pool of the last handler to call by default.
    return allocate_and_clone();
}

} // namespace openlcb
