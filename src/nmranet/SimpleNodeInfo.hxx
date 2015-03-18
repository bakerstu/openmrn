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
 * \file SimpleNodeInfo.hxx
 *
 * Handler for the Simple Node Ident Info protocol.
 *
 * @author Balazs Racz
 * @date 24 Jul 2013
 */

#include "nmranet/If.hxx"
#include "nmranet/SimpleInfoProtocol.hxx"

namespace nmranet
{

extern const uint8_t SNIP_MANUFACTURER[];
extern const uint8_t SNIP_MODEL[];
extern const uint8_t SNIP_HW_VERSION[];
extern const uint8_t SNIP_SW_VERSION[];

extern const uint8_t SNIP_USER_NODE_NAME[];
extern const uint8_t SNIP_USER_NODE_DESCRIPTION[];

extern const SimpleInfoDescriptor SNIP_RESPONSE[];

class SNIPHandler : public IncomingMessageStateFlow
{
public:
    SNIPHandler(If *interface, SimpleInfoFlow *response_flow)
        : IncomingMessageStateFlow(interface)
        , responseFlow_(response_flow)
    {
        interface->dispatcher()->register_handler(
            this, Defs::MTI_IDENT_INFO_REQUEST, Defs::MTI_EXACT);
    }

    ~SNIPHandler()
    {
        interface()->dispatcher()->unregister_handler(
            this, Defs::MTI_IDENT_INFO_REQUEST, Defs::MTI_EXACT);
    }

    Action entry() OVERRIDE
    {
        if (!nmsg()->dstNode)
            return release_and_exit();
        return allocate_and_call(responseFlow_, STATE(send_response_request));
    }

    Action send_response_request()
    {
        auto *b = get_allocation_result(responseFlow_);
        b->data()->reset(nmsg(), SNIP_RESPONSE, Defs::MTI_IDENT_INFO_REPLY);
        responseFlow_->send(b);
        return release_and_exit();
    }

private:
    SimpleInfoFlow *responseFlow_;
};

} // namespace nmranet
