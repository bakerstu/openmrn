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
#include "executor/notifiable.hxx"

class AllocationResult : public Executable {
 public:
  // Callback from the allocator when an item was successfully allocated.
  virtual void AllocationCallback(QueueMember* entry) = 0;
};

class AllocatorBase : public Lockable {
 public:
  AllocatorBase();

  //! Returns an entry to the pool of free entries. no type checking is
  //! performed by this function.
  //
  //! @param entry is the entry to release. It should be compatible with the
  //! entries expected from this Allocator.
  void Release(QueueMember* entry);

  //! Returns an entry to the end (back) of the pool of free entries. no type
  //! checking is performed by this function.
  //
  //! @param entry is the entry to release. It should be compatible with the
  //! entries expected from this Allocator.
  void ReleaseBack(QueueMember* entry);

  //! Allocates an object, possibly by putting the caller into a waiting list
  //! for allocated objects.
  //
  //! @param caller is a callback. When the next object to allocate is
  //! available, the allocation callback will be called.
  //
  //! The callback may be called inline, but it may also be called from a
  //! different thread. No locking is performed while calling the callback.
  void AllocateEntry(AllocationResult* caller);

  //! @returns true if there is an entry waiting to be allocated.
  //
  //! NOTE: using this method is likely result in race conditions.
  bool Peek();

  //! Synchronously tries to allocate an entry.
  //
  //! @returns the first allocatable entry, if there is any free. Otherwise
  //! returns NULL.
  QueueMember* AllocateOrNull();

 private:
  // List of users waiting for entries OR list of entries available to be
  // allocated.
  Queue waiting_list_;
  // This is true if there are free entries on the waiting_list or the list is
  // empty.
  bool has_free_entries_;
};

//! An allocator class that can be used for type-safe operations.
template <class T>
class TypedAllocator : public AllocatorBase {
 public:
  //! Returns an entry to the allocator.
  //
  //! @param entry is the object to return.
  void TypedRelease(T* entry) { Release(entry); }

  //! Returns an entry to the allocator at the back of the freelist..
  //
  //! @param entry is the object to return.
  void TypedReleaseBack(T* entry) { ReleaseBack(entry); }

  T* TypedAllocateOrNull() { return static_cast<T*>(AllocateOrNull()); }
};

//! Helper class that encapsulates a call to an allocator, which blocks the
//! current thread until a free entry shows up.
//
//! Usage:
//
//! SyncAllocation a(allocator);  // may block the thread
//! DoSomething(a.untyped_result())
class SyncAllocation : private AllocationResult {
 public:
  //! Synchronously allocates an entry from an allocator. Blocks the current
  //! thread until an entry is available.
  //
  //! @param allocator is the allocator to allocate an entry from.
  SyncAllocation(AllocatorBase* allocator) : result_(nullptr) {
    allocator->AllocateEntry(this);
    notify_.WaitForNotification();
  }

  //! @returns the entry allocated.
  QueueMember* untyped_result() { return result_; }

 private:
  //! Callback from the allocator.
  //
  //! @param entry is the entry allocated to this request.
  virtual void AllocationCallback(QueueMember* entry) {
    result_ = entry;
    notify_.Notify();
  }

  //! A callback that should never be called, since *this is never used as an
  //! Executable.
  virtual void Run() {
    extern int unexpected_call_to_syncallocation_run();
    HASSERT(false && unexpected_call_to_syncallocation_run());
  }

 protected:
  //! Stores the result of the allocation callback.
  QueueMember* result_;
  //! Helper for blocking the current thread until the allocation is
  //! successful.
  SyncNotifiable notify_;
};

//! Helper class for type-safe synchronous allocation from an Allocator.
template <class T>
class TypedSyncAllocation : public SyncAllocation {
 public:
  //! Allocates an entry from a (typed) allocator.
  //
  //! @param allocator is the allocator to allocate from.
  TypedSyncAllocation(TypedAllocator<T>* allocator)
      : SyncAllocation(allocator) {}

  //! @returns the result of the allocation.
  T* result() { return static_cast<T*>(untyped_result()); }
};

//! Helper class that simulates a (non-reentrant) mutex using the Allocator
//! queue and a single QueueMember token.
//
//! The mutex is defined as unlocked if there is an entry on the allocator
//! queue. Locking the mutex will take the entry off of the allocator
//! queue. Any other acquisition attempts will block so long as the allocator's
//! queue is empty.
//
//! Unlocking the mutex will release the token back to the allocator, waking up
//! the first caller in the queue.
//
//! To lock the mutex, use any allocation mechanism (e.g. control
//! flow::Allocate, or SyncAllocator). To Unlock the mutex, call the Unlock
//! method.
class AllocatorMutex : public AllocatorBase {
 public:
  //! Creates an allocator mutex.
  AllocatorMutex() { Unlock(); }

  //! Crashes if the the particular value is not the token associated with this
  //! mutex.
  //
  //! @param token is the value to check.
  void CheckToken(QueueMember* token) { HASSERT(token == &token_); }

  //! Crashes if the mutex is not locked.
  void AssertLocked() { HASSERT(!Peek()); }

  //! Crashes if the mutex is locked.
  void AssertUnlocked() { HASSERT(Peek()); }

  //! Unlocks the mutex. Crashes if the mutex is unlocked.
  void Unlock() { Release(&token_); }

  //! Synchronously locks the mutex. Might block the current thread.
  void Lock() { SyncAllocation a(this); }

 private:
  QueueMember token_;
};

#endif  // _EXECUTOR_ALLOCATOR_HXX_
