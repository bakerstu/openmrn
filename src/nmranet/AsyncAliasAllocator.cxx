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
      cid_frame_sequence_(0),
      conflict_detected_(0)
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
    return Allocate(&empty_alias_allocator_, ST(HandleWorkArrived));
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleWorkArrived()
{
    GetAllocationResult(&pending_alias_);
    return CallImmediately(ST(HandleInitAliasCheck));
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleInitAliasCheck()
{
    cid_frame_sequence_ = 7;
    conflict_detected_ = 0;
    HASSERT(pending_alias_->state == AliasInfo::STATE_EMPTY);
    while (!pending_alias_->alias)
    {
        pending_alias_->alias = seed_;
        NextSeed();
        // TODO(balazs.racz): check if the alias is already known about.
    }
    // Registers ourselves as a handler for incoming CAN frames to detect
    // conflicts.
    if_can_->frame_dispatcher()->RegisterHandler(pending_alias_->alias,
                                                 ~0x1FFFF000U, this);

    // Grabs an outgoing frame buffer.
    return Allocate(if_can_->write_allocator(), ST(HandleSendCidFrames));
}

void AsyncAliasAllocator::NextSeed() {
    uint16_t offset;
    offset = if_id_ >> 36;
    offset ^= if_id_ >> 24;
    offset ^= if_id_ >> 12;
    offset ^= if_id_;
    offset <<= 1;
    offset |= 1; // ensures offset is odd.
    // This offset will be guaranteed different for any two node IDs that
    // are within 2048 of each other. Therefore it is guaranteed that they
    // will generate a different alias after a conflict. It is also
    // guaranteed that we will generate every single possible alias before
    // generating a duplicate (the cycle length is always 2^12).

    seed_ += offset;
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleSendCidFrames()
{
    LOG(VERBOSE, "Sending CID frame %d for alias %03x", cid_frame_sequence_,
        pending_alias_->alias);
    CanFrameWriteFlow* write_flow;
    GetAllocationResult(&write_flow);
    if (conflict_detected_) {
        write_flow->Cancel();
        return CallImmediately(ST(HandleAliasConflict));
    }
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

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleAliasConflict()
{
    // Marks that we are no longer interested in frames from this alias.
    if_can_->frame_dispatcher()->UnregisterHandler(pending_alias_->alias,
                                                   ~0x1FFFF000U, this);
    
    // Burns up the alias.
    pending_alias_->alias = 0;
    pending_alias_->state = AliasInfo::STATE_EMPTY;
    // Restarts the lookup.
    return CallImmediately(ST(HandleInitAliasCheck));
}

ControlFlow::ControlFlowAction AsyncAliasAllocator::HandleWaitDone()
{
    if (conflict_detected_) {
        return CallImmediately(ST(HandleAliasConflict));
    }
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
    if_can_->frame_dispatcher()->UnregisterHandler(pending_alias_->alias,
                                                   ~0x1FFFF000U, this);
    reserved_alias_allocator_.ReleaseBack(pending_alias_);
    pending_alias_ = nullptr;
    // We go back to the infinite loop.
    return CallImmediately(ST(HandleGetMoreWork));
}

TypedAllocator<ParamHandler<struct can_frame>>*
AsyncAliasAllocator::HandleMessage(struct can_frame* message, Notifiable* done)
{
    conflict_detected_ = 1;
    // @TODO(balazs.racz): wake up the actual flow to not have to wait all the
    // 200 ms of sleep. It's somewhat difficult to ensure there is no race
    // condition there; there are no documented guarantees on the timer
    // deletion call vs timer callbacks being delivered.
    done->Notify();
    return nullptr;
}

} // namespace NMRAnet
