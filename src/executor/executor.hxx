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
 * \file executor.hxx
 *
 * Class to control execution of control flows, switching between them as their
 * inputs will be available.
 *
 * @author Balazs Racz
 * @date 5 Aug 2013
 */

#ifndef _EXECUTOR_EXECUTOR_HXX_
#define _EXECUTOR_EXECUTOR_HXX_

#include <stdint.h>
#include <stdlib.h>

#include "os/OS.hxx"
#include "executor/queue.hxx"
#include "executor/lock.hxx"
#include "executor/notifiable.hxx"

class ControlFlow;

namespace deprecated {

//! An object that can be scheduled on an executor to run.
class Executable : public QueueMember {
public:
  virtual void Run() = 0;
  virtual ~Executable() {}
};


class Executor : public Lockable {
public:
  Executor();
  ~Executor();

  // Runs the main loop of the executor in the current thread. Never returns.
  void ThreadBody() __attribute__((__noreturn__));

  // Adds entry to the back of the executor queue. `entry' must not be enqueued
  // here or in any other executor or queue.
  void Add(Executable* entry) {
    {
      LockHolder h(this);
      pending_flows_.Push(entry);
    }
    notify_.post();
  }

#if defined (__FreeRTOS__)
  void AddFromIsr(Executable* entry) {
      if (IsMaybePending(entry)) {
          return;
      }
      pending_flows_.Push(entry);
      notify_.post_from_isr();
  }
#endif

  /** Checks if `entry' is pending in the *current* executor. NOTE: This call
   * may or may not detect if `entry' is pending in a different executor or
   * enqueued on a different queue.
   */
  bool IsMaybePending(Executable* entry) {
    return pending_flows_.IsMaybePending(entry);
  }

  /** Returns true if the given entry is either pending in the queue of the
   * current executor, or is currently running in the executor. This call is
   * safe to be called from the any thread. NOTE: This call may or may not
   * detect if `entry' is pending in a different executor or enqueued on a
   * different queue.
   */
  bool IsPendingOrRunning(Executable* entry);

  /** Returns true if the given entry is on the executor right now.
   */
  bool IsRunning(Executable* entry) {
      return current_ == entry;
  }


  //! Returns true if this executor has run out of work.
  bool empty() {
    LockHolder h(this);
    return waiting_ && pending_flows_.empty();
  }

  //! Returns true if there are no flows waiting for running. There might be a
  //! flow currently being run.
  bool no_pending_flows() {
    return pending_flows_.empty();
  }

  void WaitUntilEmpty();

private:
  //! This semaphore is used for blocking the executor thread, and will be
  //! posted for each Add to wake up.
  OSSem notify_;
  //! Queue of work to do.
  Queue pending_flows_;
  //! true if the executor is waiting in the semaphore for work to do.
  bool waiting_;
  //! Work currently scheduled on the thread.
  Executable* current_;
};

//! An executor that automatically starts up a new thread to run its loop.
class ThreadExecutor : public Executor {
public:
  ThreadExecutor(const char* thread_name,
                 int priority,
                 size_t stack_size);
  ~ThreadExecutor() {}

private:
  DISALLOW_COPY_AND_ASSIGN(ThreadExecutor);

  OSThread thread_;
};

}  // namespace deprecated

#endif // _EXECUTOR_EXECUTOR_HXX_
