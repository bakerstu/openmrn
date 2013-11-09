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
 * \file WriteFlow.hxx
 *
 * Class that allows enqueing an outgoing message.
 *
 * @author Balazs Racz
 * @date 3 Nov 2013
 */

#ifndef _NMRANET_WRITE_FLOW_H_
#define _NMRANET_WRITE_FLOW_H_

#include "nmranet_config.h"
#include "executor/executor.hxx"
#include "executor/notifiable.hxx"

#ifdef EVENT_NODE_CPP
#include "utils/BufferQueue.hxx"
#include "nmranet/NMRAnetNode.hxx"
#else
#include "core/nmranet_node_private.h"
#include "core/nmranet_buf.h"
#endif

Executor* DefaultWriteFlowExecutor();

class WriteHelper : private Executable {
 public:
#ifdef EVENT_NODE_CPP
  typedef If::MTI mti_type;
  typedef Node* node_type;
  typedef NodeHandle dst_type;

  typedef Buffer* buffer_type;
  static dst_type Global() {
    return {0, 0};
  }
  static buffer_type BufferAlloc(size_t len) { return buffer_alloc(len); }
  static void* BufferDeref(buffer_type b) { return b->start(); }
  static void BufferStep(buffer_type b, size_t len) { b->advance(len); }
#else
  typedef uint16_t mti_type;
  typedef node_t node_type;
  typedef node_handle_t dst_type;
  typedef void* buffer_type;
  static buffer_type BufferAlloc(size_t len) {
    return nmranet_buffer_alloc(len);
  }
  static void* BufferDeref(buffer_type b) { return b; }
  static void BufferStep(buffer_type b, size_t len) {
    nmranet_buffer_advance(b, len);
  }
  static dst_type Global() {
    return {0, 0};
  }
#endif

  WriteHelper(Executor* executor) : done_(nullptr), executor_(executor) {}

  /** Originales an NMRAnet message from a particular node.
   *
   * @param node is the originating node.
   * @param mti is the message to send
   * @param dst is the destination node to send to (may be Global())
   * @param buffer is the message payload.
   * @param done will be notified when the packet has been enqueued to the
   * physical layer.
   */
  void WriteAsync(node_type node, mti_type mti, dst_type dst,
                  buffer_type buffer, Notifiable* done) {
    HASSERT(!done_);
    node_ = node;
    mti_ = mti;
    dst_ = dst;
    buffer_ = buffer;
    done_ = done;
    executor_->Add(this);
  }

 private:
  //! Callback in the executor thread.
  virtual void Run();

  node_type node_;
  mti_type mti_;
  dst_type dst_;
  buffer_type buffer_;
  Notifiable* done_;
  Executor* executor_;
};

WriteHelper::buffer_type EventIdToBuffer(uint64_t eventid);

#endif  // _NMRANET_WRITE_FLOW_H_
