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
 * \file WriteFlow.cxx
 *
 * Class that allows enqueing an outgoing message.
 *
 * @author Balazs Racz
 * @date 3 Nov 2013
 */

#include "nmranet/NMRAnetWriteFlow.hxx"
#include "nmranet_config.h"
#include "endian.h"

namespace NMRAnet {

Executor* DefaultWriteFlowExecutor() __attribute__((__weak__));

Executor* DefaultWriteFlowExecutor() {
  static ThreadExecutor e("write_flow", 0, WRITE_FLOW_THREAD_STACK_SIZE);
  return &e;
}

/** Callback in the executor thread.
 */
void WriteHelper::Run() {
  node_->write(mti_, dst_, buffer_);
  Notifiable* d = done_;
  if (d) {
    done_ = nullptr;
    d->Notify();
  }
}

Buffer* EventIdToBuffer(uint64_t eventid) {
  Buffer* buffer = buffer_alloc(sizeof(eventid));
  uint64_t* b = static_cast<uint64_t*>(buffer->start());
  *b = htobe64(eventid);
  buffer->advance(sizeof(eventid));
  return buffer;
}

}; /* namespace NMRAnet */
