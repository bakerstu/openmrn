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

#include "openlcb/SimpleNodeInfo.hxx"

#include "openmrn_features.h"
#include "utils/format_utils.hxx"

namespace openlcb
{

extern const SimpleNodeStaticValues __attribute__((weak)) SNIP_STATIC_DATA = {
    4, "OpenMRN", "Undefined model", "Undefined HW version", "0.9"};

void init_snip_user_file(int fd, const char *user_name,
                         const char *user_description)
{
    ::lseek(fd, 0, SEEK_SET);
    SimpleNodeDynamicValues data;
    memset(&data, 0, sizeof(data));
    data.version = 2;
    str_populate(data.user_name, user_name);
    str_populate(data.user_description, user_description);
    int ofs = 0;
    auto *p = (const uint8_t *)&data;
    const int len = sizeof(data);
    while (ofs < len)
    {
        int ret = ::write(fd, p, len - ofs);
        if (ret < 0)
        {
            LOG(FATAL, "Init SNIP file: Could not write to fd %d: %s", fd,
                strerror(errno));
        }
        ofs += ret;
    }
}

static size_t find_string_at(const openlcb::Payload& payload, size_t start_pos, string* output) {
    if (start_pos >= payload.size()) {
        output->clear();
        return start_pos;
    }
    size_t epos = payload.find('\0', start_pos);
    output->assign(payload.substr(start_pos, epos - start_pos));
    if (epos == string::npos) {
        return epos;
    } else {
        return epos + 1;
    }
}

void decode_snip_response(
    const openlcb::Payload &payload, SnipDecodedData *output)
{
    output->clear();
    if (payload.empty())
    {
        return;
    }
    char sys_ver = payload[0];
    size_t pos = 1;
    pos = find_string_at(payload, pos, &output->manufacturer_name);
    pos = find_string_at(payload, pos, &output->model_name);
    pos = find_string_at(payload, pos, &output->hardware_version);
    pos = find_string_at(payload, pos, &output->software_version);
    // Future-proof with the version handling.
    for (int i = 4; i < sys_ver; ++i)
    {
        string discard;
        pos = find_string_at(payload, pos, &discard);
    }
    if (pos == string::npos)
    {
        return;
    }
    // char usr_ver = payload[pos];
    pos++;
    pos = find_string_at(payload, pos, &output->user_name);
    pos = find_string_at(payload, pos, &output->user_description);
}

} // namespace nrmanet
