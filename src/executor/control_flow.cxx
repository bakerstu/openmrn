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
 * \file control_flow.cxx
 *
 * Class running processes with asynchronous I/O.
 *
 * @author Balazs Racz
 * @date 5 Aug 2013
 */


#include "executor/control_flow.hxx"

void ControlFlow::Run() {
  LOG(VERBOSE, "ControlFlow::Run %p", this);
  HASSERT(state_);
  do {
    ControlFlowAction action = (this->*state_)();
    // We got a WaitForNotification.
    if (!action.next_state_) return;
    state_ = action.next_state_;
  } while (1);
}

ControlFlow::ControlFlowAction ControlFlow::Exit() {
  if (done_) {
    state_ = &ControlFlow::Terminated;
    done_->Notify();
    done_ = nullptr;
  } else {
    delete this;
  }
  // If we return waitfornotification, the run method will not access *this
  // anymore, which is perfect if we just deleted ourselves.
  return WaitForNotification();
}

ControlFlow::ControlFlowAction ControlFlow::NotStarted() {
  return WaitForNotification();
}

ControlFlow::ControlFlowAction ControlFlow::Terminated() {
  return WaitForNotification();
}

ControlFlow::SleepData::~SleepData() {
  if (timer_handle != NULL) {
    os_timer_delete(timer_handle);
    timer_handle = NULL;
  }
}

void ControlFlow::NotifyControlFlowTimer(SleepData* entry) {
  LockHolder h(executor());
  entry->callback_count++;
  // here we use that executor's mutex is reentrant.
  Notify();
}
                                   

long long ControlFlow::control_flow_single_timer(void* arg_flow, void* arg_entry) {
  ControlFlow::SleepData* entry =
    static_cast<ControlFlow::SleepData*>(arg_entry);
  ControlFlow* flow = static_cast<ControlFlow*>(arg_flow);
  flow->NotifyControlFlowTimer(entry);
  return OS_TIMER_NONE;  // no restart.
}

long long ControlFlow::control_flow_repeated_timer(void* arg_flow, void* arg_entry) {
  ControlFlow::SleepData* entry =
    static_cast<ControlFlow::SleepData*>(arg_entry);
  ControlFlow* flow = static_cast<ControlFlow*>(arg_flow);
  flow->NotifyControlFlowTimer(entry);
  return OS_TIMER_RESTART;
}

ControlFlow::ControlFlowAction ControlFlow::Sleep(SleepData* data,
                                                  long long delay_nsec,
                                                  MemberFunction next_state) {
  data->callback_count = 0;
  if (data->timer_handle == NULL) {
    data->timer_handle =
      os_timer_create(&control_flow_single_timer, this, data);
  }
  os_timer_start(data->timer_handle, delay_nsec);

  return WaitForTimerWakeUpAndCall(data, next_state);
}

void ControlFlow::WakeUpRepeatedly(SleepData* data, long long period_nsec) {
  if (data->timer_handle != NULL) {
    os_timer_delete(data->timer_handle);
  }
  data->timer_handle =
    os_timer_create(&control_flow_repeated_timer, this, data);
  os_timer_start(data->timer_handle, period_nsec);
}

void ControlFlow::StopTimer(SleepData* data) {
  HASSERT(data->timer_handle != NULL);
  os_timer_stop(data->timer_handle);
}


ControlFlow::ControlFlowAction ControlFlow::WaitForTimerWakeUpAndCall(
    SleepData* data,
    MemberFunction next_state) {
  next_state_ = next_state;
  sub_flow_.sleep = data;
  // We use CallImmediately here in case the timer has expired before (or
  // immediately). Useful especially for repeated calls.
  return CallImmediately(&ControlFlow::WaitForTimer);
}

ControlFlow::ControlFlowAction ControlFlow::WaitForTimer() {
  {
    LockHolder h(executor_);
    if (sub_flow_.sleep->callback_count == 0) return WaitForNotification();
    --sub_flow_.sleep->callback_count;
  }
  return CallImmediately(next_state_);
}

//! Implementation state that is waiting for another flow to finish.
ControlFlow::ControlFlowAction ControlFlow::WaitForControlFlow() {
  LockHolder h(executor_);
  if (sub_flow_.called_flow->IsDone()) {
    return CallImmediately(next_state_);
  } else {
    return WaitForNotification();
  }
}
