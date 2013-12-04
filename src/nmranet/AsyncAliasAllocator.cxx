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
 * \file AsyncAliasAllocator.cxx
 *
 * Asynchronous implementation of NMRAnet alias reservation algorithms.
 *
 * @author Balazs Racz
 * @date 4 Dec 2013
 */

#define LOGLEVEL VERBOSE

#include "nmranet/AsyncAliasAllocator.hxx"
#include "nmranet/NMRAnetIfCan.hxx"

namespace NMRAnet
{

AsyncAliasAllocator::AsyncAliasAllocator(NodeID if_id, AsyncIfCan* if_can)
    : ControlFlow(if_can->frame_dispatcher()->executor(), // ensures we run in
                                                          // the same thread as
                                                          // the incoming
                                                          // messages
                  nullptr),
      if_id_(if_id),
      if_can_(if_can),
      pending_alias_(nullptr),
      cid_frame_sequence_(0)
{
    seed_ = if_id >> 30;
    seed_ ^= if_id >> 18;
    seed_ ^= if_id >> 6;
    seed_ ^= uint16_t(if_id >> 42) | uint16_t(if_id << 6);
    StartFlowAt(ST(HandleGetMoreWork));
}

AsyncAliasAllocator::~AsyncAliasAllocator()
{
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleGetMoreWork()
{
    return Allocate(&empty_alias_allocator_, ST(HandleInitAliasCheck));
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleInitAliasCheck()
{
    GetAllocationResult(&pending_alias_);
    cid_frame_sequence_ = 7;
    HASSERT(pending_alias_->state == AliasInfo::STATE_EMPTY);
    while (!pending_alias_->alias)
    {
        pending_alias_->alias = seed_;

        uint16_t offset;
        offset = if_id_ >> 36;
        offset ^= if_id_ >> 24;
        offset ^= if_id_ >> 12;
        offset ^= if_id_;
        offset <<= 1;
        offset |= 1; // ensures offset is odd.
        // This offset will be guaranteed different for any two node IDs that
        // are within 1024 of each other. Therefore it is guaranteed that they
        // will generate a different alias after a conflict. It is also
        // guaranteed that we will generate ewvery single possible alias before
        // generating a duplicate (the cycle length is always 2^12).

        seed_ += offset;

        // TODO(balazs.racz): check if the alias is already known about.
    }
    // Grab an outgoing frame buffer.
    return Allocate(if_can_->write_allocator(), ST(HandleSendCidFrames));
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleSendCidFrames()
{
    LOG(VERBOSE, "Sending CID frame %d for alias %03x", cid_frame_sequence_,
        pending_alias_->alias);
    CanFrameWriteFlow* write_flow;
    GetAllocationResult(&write_flow);
    IfCan::control_init(*write_flow->mutable_frame(), pending_alias_->alias,
                        (if_id_ >> (12 * (cid_frame_sequence_ - 4))) & 0xfff,
                        cid_frame_sequence_);
    write_flow->Send(nullptr);
    --cid_frame_sequence_;
    if (cid_frame_sequence_ >= 4)
    {
        return Allocate(if_can_->write_allocator(), ST(HandleSendCidFrames));
    }
    else
    {
        // All CID frames are sent, let's wait.
        return Sleep(&sleep_helper_, MSEC_TO_NSEC(200), ST(HandleWaitDone));
    }
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleWaitDone()
{
    // grab a frame buffer for the RID frame.
    return Allocate(if_can_->write_allocator(), ST(HandleSendRidFrame));
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleSendRidFrame()
{
    LOG(VERBOSE, "Sending RID frame for alias %03x", pending_alias_->alias);
    CanFrameWriteFlow* write_flow;
    GetAllocationResult(&write_flow);
    IfCan::control_init(*write_flow->mutable_frame(), pending_alias_->alias,
                        IfCan::RID_FRAME, 0);
    write_flow->Send(nullptr);
    // The alias is reserved, put it into the freelist.
    pending_alias_->state = AliasInfo::STATE_RESERVED;
    reserved_alias_allocator_.ReleaseBack(pending_alias_);
    pending_alias_ = nullptr;
    // We go back to the infinite loop.
    return CallImmediately(ST(HandleGetMoreWork));
}

TypedAllocator<ParamHandler<struct can_frame>>*
AsyncAliasAllocator::HandleMessage(struct can_frame* message, Notifiable* done)
{
    // @TODO(balazs.racz): kill the current alias allocation flow.
    done->Notify();
    return nullptr;
}

} // namespace NMRAnet
