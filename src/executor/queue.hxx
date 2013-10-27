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
 * \file queue.hxx
 *
 * A class for a single-linked list with queue semantics.
 *
 * @author Balazs Racz
 * @date 5 Aug 2013
 */

#ifndef _EXECUTOR_QUEUE_HXX_
#define _EXECUTOR_QUEUE_HXX_

#include "utils/macros.h"

class Queue;

class QueueMember {
public:
  QueueMember()
    : next_(NULL) {}

private:
  DISALLOW_COPY_AND_ASSIGN(QueueMember);
  friend class Queue;
  QueueMember* next_;
};

class Queue : private QueueMember {
public:
  Queue()
    : tail_(this) {}

  //! Returns true if the queue has no elements.
  bool empty() {
    return next_ == NULL;
  }

  //! Adds an entry to the end of the queue. Not thread-safe (caller has to
  //! lock).
  void Push(QueueMember* entry) {
    HASSERT(entry);
    HASSERT(entry->next_ == NULL);
    HASSERT(entry != tail_);
    HASSERT(tail_->next_ == NULL);
    tail_->next_ = entry;
    tail_ = entry;
  }

  //! Adds an entry to the front of the queue. Not thread-safe (caller has to
  //! lock).
  void PushFront(QueueMember* entry) {
    HASSERT(entry);
    HASSERT(entry->next_ == NULL);
    HASSERT(entry != tail_);
    HASSERT(tail_->next_ == NULL);
    entry->next_ = next_;
    next_ = entry;
  }

  bool IsMaybePending(QueueMember* entry) {
    return ((entry->next_ != NULL) ||
            (entry == tail_));
  }

  void PushIfNotMember(QueueMember* entry) {
    HASSERT(entry);
    HASSERT(tail_->next_ == NULL);
    if (IsMaybePending(entry)) return;
    tail_->next_ = entry;
    tail_ = entry;
  }

  QueueMember* Pop() {
    QueueMember* ret = next_;
    if (ret) {
      next_ = ret->next_;
      if (!next_) tail_ = this;
      ret->next_ = NULL;
    }
    return ret;
  }

private:
  DISALLOW_COPY_AND_ASSIGN(Queue);

  QueueMember* tail_;
};

#endif // _EXECUTOR_QUEUE_HXX_
