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
 * \file AsyncIfCan.hxx
 *
 * Asynchronous NMRAnet interface implementation for CANbus.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _NMRAnetAsyncIfCan_hxx_
#define _NMRAnetAsyncIfCan_hxx_

#include "nmranet/ReadDispatch.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "utils/BufferQueue.hxx"
#include "nmranet/AsyncIf.hxx"
#include "nmranet_can.h"

class Pipe;

namespace NMRAnet
{

typedef ParamHandler<struct can_frame> IncomingFrameHandler;

class AsyncIfCan : public AsyncIf
{
 public:
  typedef TypedDispatchFlow<uint32_t, struct can_frame> FrameDispatchFlow;

  /**
     Creates a CAN interface.

     @param executor will be used to process incoming (and outgoing) messages.

     @param device is a Pipe. The interface will add a member to this pipe to
     handle incoming and outgoing traffic. The caller should add the necessary
     hardware device, GridConnect bridge or mock interface to this pipe (before
     this call or else outgoing packets might be lost).
   */
  AsyncIfCan(Executor* executor, Pipe* device);

  //! @returns the dispatcher of incoming CAN frames.
  FrameDispatchFlow* frame_dispatcher() { return &frame_dispatcher_; }

 private:
  //! Flow responsible for routing incoming messages to handlers.
  FrameDispatchFlow frame_dispatcher_;

  DISALLOW_COPY_AND_ASSIGN(AsyncIfCan);
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIfCan_hxx_
