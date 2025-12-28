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
 * \file FilteringCanHubFlow.hxx
 *
 * Can Frame Hub that performs addressed message filtering for openlcb.
 *
 * @author Balazs Racz
 * @date 28 Dec 2025
 */

#ifndef _OPENLCB_FILTERINGCANHUBFLOW_HXX_
#define _OPENLCB_FILTERINGCANHUBFLOW_HXX_

#include "openlcb/CanFilter.hxx"
#include "utils/Hub.hxx"

namespace openlcb
{

/// A CAN Hub Flow that uses CanFilter to route messages.
class FilteringCanHubFlow : public CanHubFlow
{
public:
    FilteringCanHubFlow(Service *service);

    /// Sets whether filtering is enabled.
    /// @param is_filtering true to enable filtering, false for legacy behavior.
    void set_filtering(bool is_filtering)
    {
        isFiltering_ = is_filtering;
    }

    Action entry() override;
    Action iterate() override;

    void unregister_port(CanHubFlow::port_type *port) override;

private:
    CanFilter filter_;
    bool isFiltering_;
};

} // namespace openlcb

#endif // _OPENLCB_FILTERINGCANHUBFLOW_HXX_
