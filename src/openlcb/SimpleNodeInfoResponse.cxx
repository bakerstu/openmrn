/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file SimpleNodeInfoResponse.cxx
 *
 * Response descriptor for the Simple Node Ident Info protocol handler.
 *
 * @author Balazs Racz
 * @date 3 Jan 2022
 */

#include "openlcb/SimpleNodeInfo.hxx"

namespace openlcb
{

const SimpleInfoDescriptor SNIPHandler::SNIP_RESPONSE[] =
{
    {SimpleInfoDescriptor::LITERAL_BYTE, 4, 0, nullptr},
    {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.manufacturer_name},
    {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.model_name},
    {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.hardware_version},
    {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.software_version},
#if OPENMRN_HAVE_POSIX_FD
    {SimpleInfoDescriptor::FILE_LITERAL_BYTE, 2, 0, SNIP_DYNAMIC_FILENAME},
    {SimpleInfoDescriptor::FILE_C_STRING, 63, 1, SNIP_DYNAMIC_FILENAME},
    {SimpleInfoDescriptor::FILE_C_STRING, 64, 64, SNIP_DYNAMIC_FILENAME},
#else
    /// @todo(balazs.racz) Add eeprom support to arduino.
    {SimpleInfoDescriptor::LITERAL_BYTE, 2, 0, nullptr},
    {SimpleInfoDescriptor::LITERAL_BYTE, 0, 0, nullptr},
    {SimpleInfoDescriptor::LITERAL_BYTE, 0, 0, nullptr},
#endif
    {SimpleInfoDescriptor::END_OF_DATA, 0, 0, 0}
};

const SimpleInfoDescriptor SNIPHandler::SNIP_STATIC_RESPONSE[] =
{
    {SimpleInfoDescriptor::LITERAL_BYTE, 4, 0, nullptr},
    {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.manufacturer_name},
    {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.model_name},
    {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.hardware_version},
    {SimpleInfoDescriptor::C_STRING, 0, 0, SNIP_STATIC_DATA.software_version},
    {SimpleInfoDescriptor::LITERAL_BYTE, 2, 0, nullptr},
    {SimpleInfoDescriptor::LITERAL_BYTE, 0, 0, nullptr},
    {SimpleInfoDescriptor::LITERAL_BYTE, 0, 0, nullptr},
    {SimpleInfoDescriptor::END_OF_DATA, 0, 0, 0}
};

}

