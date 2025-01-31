/** \copyright
 * Copyright (c) 2023, Balazs Racz
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
 * \file ClientConnection.cxx
 *
 * Utilities for managing can-hub connections as a client application.
 *
 * @author Balazs Racz
 * @date 27 Dec 2023
 */

#include "utils/ClientConnection.hxx"

#include "nmranet_config.h"

#include "utils/FdUtils.hxx"

/// Callback from try_connect to donate the file descriptor.
/// @param fd is the file destriptor of the connection freshly opened.
void GCFdConnectionClient::connection_complete(int fd)
{
    const bool use_select =
        (config_gridconnect_tcp_use_select() == CONSTANT_TRUE);

    // Applies kernel parameters like socket options.
    FdUtils::optimize_fd(fd);
    
    fd_ = fd;

    if (hub_) {
        create_gc_port_for_can_hub(hub_, fd, &closedNotify_, use_select);
    } else if (directHub_) {
        create_port_for_fd(directHub_, fd,
            std::unique_ptr<MessageSegmenter>(create_gc_message_segmenter()),
            &closedNotify_);
    } else {
        DIE("Neither hub, nor directhub given to connection client");
    }
}
