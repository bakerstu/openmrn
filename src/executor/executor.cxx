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

#include "utils/logging.h"
#include "executor/executor.hxx"

static void* start_executor_thread(void* arg) {
  static_cast<Executor*>(arg)->ThreadBody();
  return NULL;
}

Executor::Executor() : notify_(0), waiting_(true), current_(nullptr) {}

Executor::~Executor() {}

void Executor::ThreadBody() {
  while (1) {
    waiting_ = true;
    notify_.wait();
    waiting_ = false;
    {
      LockHolder h(this);
      current_ = static_cast<Executable*>(pending_flows_.Pop());
    }
    if (current_) {
      current_->Run();
    }
    {
      LockHolder h(this);
      current_ = nullptr;
    }
  }
}

bool Executor::IsPendingOrRunning(Executable* entry) {
  LockHolder h(this);
  if (current_ == entry) return true;
  if (pending_flows_.IsMaybePending(entry)) return true;
  return false;
}

ThreadExecutor::ThreadExecutor(const char* thread_name, int priority,
                               size_t stack_size)
    : thread_(thread_name, priority, stack_size, &start_executor_thread, this) {
}
