/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file async_pipe_member.hxx
 *
 * Helper class for adding an asynchronous implementation to a pipe.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _UTILS_ASYNC_PIPE_MEMBER_HXX_
#define _UTILS_ASYNC_PIPE_MEMBER_HXX_

#include <stdint.h>

#include "utils/pipe.hxx"

class Notifiable;

/** @todo(balazs.racz) I think this class should be deleted. */
class AsyncPipeMember : public PipeMember
{
public:
    AsyncPipeMember(Pipe* parent);
    virtual ~AsyncPipeMember();

    /** Fills a buffer with incoming data bytes. Calls 'done' when all the
        expected bytes have arrived.

        @param buf is the buffer to fill
        @param count is the number of bytes to receive
        @param done will be notified when the buffer is full.

        Note: this function will never to a partial read.

        At any point in time there may be only one such call pending.
     */
    void ReceiveData(void* buf, size_t count, Notifiable* done);

    Pipe* parent() { return parent_; }

private:
    /// Callback from the pipe on data received from the pipe.
    virtual void write(const void* buf, size_t count);

    uint8_t* in_buf_;     //< Buffer to receive input into, or NULL.
    size_t in_count_;     //< Remaining bytes to receive into in_buf.
    Notifiable* in_done_; //< Input notifiable.

    OSSem receive_sem_; //< Semaphore controlling the receive thread.

    Pipe* parent_; //< Parent pipe (kept for unregistering at destruction).
};

#endif //_UTILS_ASYNC_PIPE_MEMBER_HXX_
