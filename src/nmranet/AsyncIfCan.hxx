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

#include <memory>

#include "nmranet/ReadDispatch.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "utils/BufferQueue.hxx"
#include "utils/async_pipe_member.hxx"
#include "nmranet/AsyncIf.hxx"
#include "nmranet_can.h"

class Pipe;

namespace NMRAnet
{

typedef ParamHandler<struct can_frame> IncomingFrameHandler;

/** Interface class for the asynchronous frame write flow. This flow allows you
    to write frames to the CAN bus.

    Usage:
    . allocate a flow instance through the write_allocator.
    . fill in flow->mutable_frame() [*]
    . call flow->Send()

    [*] When a write flow arrives from an allocator, it is guaranteed that it
    is a regular (non-err, non-remote) extended data frame.

    The flow will return itself to the allocator when done.
*/
class CanFrameWriteFlow : public ControlFlow {
 public:
  CanFrameWriteFlow(Executor* e, Notifiable* done) : ControlFlow(e, done) {
    ResetFrameEff();
  }

  /// @returns the frame buffer to be filled.
  struct can_frame* mutable_frame() { return &frame_; }

  /** Requests the frame buffer to be sent to the bus. Takes ownership of
   *  *this and returns it to the allocator.
   *
   *  @param done will be notified, when the frame was successfully
   *  enqueued. In most cases it can be set to NULL.
   */
  virtual void Send(Notifiable* done) = 0;

  /** Releases this frame buffer without sending of anything to the bus. Takes
   *  ownership of *this and returns it to the allocator.
   */
  virtual void Cancel() = 0;

  /** Resets the frame buffer to regular (non-err, non-remote) extended data
   * frame */
  void ResetFrameEff() {
    CLR_CAN_FRAME_ERR(frame_);
    CLR_CAN_FRAME_RTR(frame_);
    SET_CAN_FRAME_EFF(frame_);
  }

 protected:
  struct can_frame frame_;
};

class AsyncIfCan : public AsyncIf
{
 public:
  typedef TypedDispatchFlow<uint32_t, struct can_frame> FrameDispatchFlow;

  /**
   * Creates a CAN interface.
   *
   * @param executor will be used to process incoming (and outgoing) messages.
   *
   * @param device is a Pipe. The interface will add a member to this pipe to
   * handle incoming and outgoing traffic. The caller should add the necessary
   * hardware device, GridConnect bridge or mock interface to this pipe (before
   * this call or else outgoing packets might be lost).
   */
  AsyncIfCan(Executor* executor, Pipe* device);

  ~AsyncIfCan();

  /** Initializes the write flow allocators with a number of new instances.
   *
   *  @param num_addressed number of new addressed write flows to create.
   *  @param num_global number of new global write flows to create.
   */
  void AddWriteFlows(int num_addressed, int num_global);

  //! @returns the dispatcher of incoming CAN frames.
  FrameDispatchFlow* frame_dispatcher() { return &frame_dispatcher_; }

  //! @returns the allocator for the write flow.
  TypedAllocator<CanFrameWriteFlow>* write_allocator() { return &write_allocator_; } 

  //! @returns the asynchronous read/write object.
  AsyncPipeMember* pipe_member() { return &pipe_member_; }

 private:
  //! Flow responsible for routing incoming messages to handlers.
  FrameDispatchFlow frame_dispatcher_;

  //! Handles asynchronous reading and writing from the device.
  AsyncPipeMember pipe_member_;

  class CanReadFlow;
  class CanWriteFlow;
  
  //! Various implementation control flows that this interface owns.
  std::vector<std::unique_ptr<ControlFlow> > owned_flows_;

  /** Allocator that holds (and mutex-controls) the frame write flow.

      It is important that this allocator be destructed before the
      owned_flows_. */
  TypedAllocator<CanFrameWriteFlow> write_allocator_;

  DISALLOW_COPY_AND_ASSIGN(AsyncIfCan);
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIfCan_hxx_
