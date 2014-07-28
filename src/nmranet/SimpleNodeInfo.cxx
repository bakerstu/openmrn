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

#include "nmranet/SimpleNodeInfo.hxx"

namespace nmranet
{

extern const uint8_t __attribute__((weak)) SNIP_MANUFACTURER[];
const uint8_t SNIP_MANUFACTURER[] = "OpenMRN";
extern const uint8_t __attribute__((weak)) SNIP_MODEL[];
const uint8_t SNIP_MODEL[] = "Undefined model";
extern const uint8_t __attribute__((weak)) SNIP_HW_VERSION[];
const uint8_t SNIP_HW_VERSION[] = "Undefined HW version";
extern const uint8_t __attribute__((weak)) SNIP_SW_VERSION[];
const uint8_t SNIP_SW_VERSION[] = "0.9";

extern const uint8_t __attribute__((weak)) SNIP_USER_NODE_NAME[];
const uint8_t SNIP_USER_NODE_NAME[] = "Undefined node name";
extern const uint8_t __attribute__((weak)) SNIP_USER_NODE_DESCRIPTION[];
const uint8_t SNIP_USER_NODE_DESCRIPTION[] = "Undefined node descr";

const SimpleInfoDescriptor SNIP_RESPONSE[] = {
    {SimpleInfoDescriptor::LITERAL_BYTE, 1, nullptr},
    {SimpleInfoDescriptor::C_STRING, 0, SNIP_MANUFACTURER},
    {SimpleInfoDescriptor::C_STRING, 0, SNIP_MODEL},
    {SimpleInfoDescriptor::C_STRING, 0, SNIP_HW_VERSION},
    {SimpleInfoDescriptor::C_STRING, 0, SNIP_SW_VERSION},
    {SimpleInfoDescriptor::LITERAL_BYTE, 1, nullptr},
    {SimpleInfoDescriptor::C_STRING, 0, SNIP_USER_NODE_NAME},
    {SimpleInfoDescriptor::C_STRING, 0, SNIP_USER_NODE_DESCRIPTION},
    {SimpleInfoDescriptor::END_OF_DATA, 0, 0}};

} // namespace nrmanet
