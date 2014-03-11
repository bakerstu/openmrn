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
 * \file PipeFlow.hxx
 * Implementation of the pipe dispatcher flow.
 *
 * @author Balazs Racz
 * @date 8 Dec 2013
 */

#ifndef _utils_PipeFlow_hxx_
#define _utils_PipeFlow_hxx_

#include "utils/pipe.hxx"
#include "executor/Dispatcher.hxx"

class PipeBuffer;
class PipeMember;

class HubData;
// Abstract member of the hub. The Dispatcher keeps account by this type.
typedef FlowInterface<Buffer<HubData>> HubMember;

/** This class can be sent via a Buffer to a hub.
 *
 * Access the data content via members char* data() and size_t size().
 *
 * Set skipMember_ to non-NULL to skip a particular entry flow of the output.
 */
class HubData : public string
{
    HubData() : skipMember_(nullptr)
    {
    }
    typedef void *id_type;
    void *skipMember_;
    id_type id()
    {
        return skipMember_;
    }
};

/** A generic hub that proxies packets of untyped data. */
class HubFlow : public DispatchFlow<Buffer<HubData>, 1>
{
public:
    HubFlow(Service *s) : DispatchFlow<Buffer<HubData>, 1>> (s)
    {
        negateMatch_ = true;
    }
};

/** All ports interfacing via a hub will have to derive from this flow. */
typedef StateFlow<Buffer<HubData>, 1> HubPort;


#if 0
/// @TODO(balazs.racz) consider adding an api like this to HubFlow.
/**
   An interface class for channels where we forward data from a pipe.

   Various different pipe receivers will implement this interface.
*/
class PipeMember : public HandlerBase
{
public:
    virtual ~PipeMember()
    {
    }
    /**
       Writes bytes to the device.

       @param buf is the source buffer from whch to write bytes.

       @param count is the number of bytes to write. count is a multiple of the
    parent pipe's unit, otherwise implementations are allowed to drop data or
    die.

       Blocks until the write is complete (that is, all data is enqueued in a
    buffer which will drain as the output device's speed
    allows). Implementations may want to use a lock inside to avoid writes from
    multiple sources being interleaved.
    */
    virtual void write(const void *buf, size_t count) = 0;

    virtual void async_write(const void *buf, size_t count, Notifiable *done)
    {
        write(buf, count);
        done->notify();
    }
};
#endif // if 0

#endif // _utils_PipeFlow_hxx_
