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
 * \file LocalTrackIf.hxx
 *
 * Control flow that acts as a trackInterface and sends all packets to a local
 * fd that represents the DCC mainline, such as TivaDCC.
 *
 * @author Balazs Racz
 * @date 23 Aug 2014
 */

#ifndef _DCC_LOCALTRACKIF_HXX_
#define _DCC_LOCALTRACKIF_HXX_

#include "executor/Executor.hxx"
#include "executor/StateFlow.hxx"
#include "dcc/Packet.hxx"

namespace dcc
{

class LocalTrackIf : public StateFlow<Buffer<dcc::Packet>, QList<1>>
{
public:
    /** Constructs a TrackInterface from an fd to the mainline.
     *
     * This class currently does synchronous writes to the device. In order not
     * to block the executor, you have to create a new threadexecutor.
     *
     * @param service THE EXECUTOR OF THIS SERVICE WILL BE BLOCKED.
     * @param pool_size will determine how many packets the current flow's
     * alloc() will have.
     */
    LocalTrackIf(Service *service, int pool_size);

    FixedPool *pool() OVERRIDE
    {
        return &pool_;
    }

    /** You must call this function before sending any packets. @param fd is
     * the file descriptor to /dev/mainline. */
    void set_fd(int fd)
    {
        fd_ = fd;
    }

private:
    Action entry() OVERRIDE;

    Action finish()
    {
        return release_and_exit();
    }

    int fd_;
    FixedPool pool_;
};

} // namespace dcc

#endif // _DCC_LOCALTRACKIF_HXX_
