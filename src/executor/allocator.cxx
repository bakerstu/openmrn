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
 * \file allocator.cxx
 *
 * Implementation for the allocator class.
 *
 * @author Balazs Racz
 * @date 20 October 2013
 */

#include "executor/allocator.hxx"

AllocatorBase::AllocatorBase()
    : has_free_entries_(1), hasEverSeenFreeEntries_(0) {}

void AllocatorBase::Release(QueueMember* entry) {
  hasEverSeenFreeEntries_ = 1;
  AllocationResult* caller = nullptr;
  {
    LockHolder l(this);
    if (has_free_entries_) {
      waiting_list_.PushFront(entry);
      return;
    } else {
      caller = static_cast<AllocationResult*>(waiting_list_.Pop());
      if (waiting_list_.empty()) has_free_entries_ = 1;
    }
  }
  HASSERT(caller != nullptr);
  caller->AllocationCallback(entry);
}

void AllocatorBase::ReleaseBack(QueueMember* entry) {
  hasEverSeenFreeEntries_ = 1;
  AllocationResult* caller = nullptr;
  {
    LockHolder l(this);
    if (has_free_entries_) {
      waiting_list_.Push(entry);
      return;
    } else {
      caller = static_cast<AllocationResult*>(waiting_list_.Pop());
      if (waiting_list_.empty()) has_free_entries_ = 1;
    }
  }
  HASSERT(caller != nullptr);
  caller->AllocationCallback(entry);
}

void AllocatorBase::AllocateEntry(AllocationResult* caller) {
  QueueMember* entry = nullptr;
  HASSERT(caller);
  {
    LockHolder l(this);
    if ((!has_free_entries_) || waiting_list_.empty()) {
      waiting_list_.Push(caller);
      has_free_entries_ = 0;
      return;
    }
    // Now: has_free_entries_ == true and !empty.
    entry = waiting_list_.Pop();
  }
  caller->AllocationCallback(entry);
}

QueueMember* AllocatorBase::AllocateOrNull() {
  QueueMember* entry = nullptr;
  {
    LockHolder l(this);
    if ((!has_free_entries_) || waiting_list_.empty()) {
      return nullptr;
    }
    // Now: has_free_entries_ == true and !empty.
    entry = waiting_list_.Pop();
  }
  return entry;
}

bool AllocatorBase::Peek() {
  LockHolder l(this);
  return has_free_entries_ && !waiting_list_.empty();
}
