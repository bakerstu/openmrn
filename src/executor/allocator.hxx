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
 * \file allocator.hxx
 *
 * Class for maintaining a bounded list of structures that can be allocated.
 *
 * @author Balazs Racz
 * @date 19 October 2013
 */

#ifndef _EXECUTOR_ALLOCATOR_HXX_
#define _EXECUTOR_ALLOCATOR_HXX_

#include "executor/executor.hxx"

class AllocationResult : public Executable {
 public:
  // Callback from the allocator when an item was successfully allocated.
  virtual void AllocationCallback(QueueMember* entry) = 0;
};

class AllocatorBase : public Lockable {
 public:
  // Returns an entry to the pool of free entries.
  void Release(QueueMember* entry);

  // Puts the caller into a waiting list for allocated objects. When the next
  // object to allocate is available, the allocation callback will be called.
  void AllocateEntry(AllocationResult* caller);

 private:
  // List of users waiting for entries OR list of entries available to be
  // allocated.
  Queue waiting_list_;
  // This is true if there are free entries on the waiting_list or the list is
  // empty.
  bool has_free_entries_;
};

#endif // _EXECUTOR_ALLOCATOR_HXX_

