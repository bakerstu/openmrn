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
 * \file lock.hxx
 *
 * Defines a lock base class for protecting small critical sections. The
 * implementation of the lock can be with recursive mutexes (on a large memory
 * system) or with critical sections on a small system.
 *
 * @author Balazs Racz
 * @date 5 Aug 2013
 */

#ifndef _UTILS_LOCK_HXX_
#define _UTILS_LOCK_HXX_

#ifdef __FreeRTOS__

#include "portmacro.h"

class Lockable {
 public:
  void lock() { portENTER_CRITICAL(); }
  void unlock() { portEXIT_CRITICAL(); }
};

#else

#include "os/OS.hxx"

class Lockable : public OSMutex {
 public:
  Lockable() : OSMutex(true) {}
};

#endif

//! See @OSMutexLock in os/OS.hxx
class LockHolder {
 public:
  LockHolder(Lockable* parent) : parent_(parent) { parent_->lock(); }
  ~LockHolder() { parent_->unlock(); }

 private:
  Lockable* parent_;
};

//! See @OSMutexLock in os/OS.hxx
#define LockHolder(l) int error_omitted_lock_holder_variable[-1]

#endif  // _UTILS_LOCK_HXX_
