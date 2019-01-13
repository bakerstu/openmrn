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
 * \file Atomic.hxx
 *
 * Defines a lock base class for protecting small critical sections. The
 * implementation of the lock can be with recursive mutexes (on a large memory
 * system) or with critical sections on a small system.
 *
 * @author Balazs Racz
 * @date 5 Aug 2013
 */

#ifndef _UTILS_ATOMIC_HXX_
#define _UTILS_ATOMIC_HXX_

#ifdef __FreeRTOS__
#include <stdint.h>
#if defined(ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#else
#include "FreeRTOS.h"
#include "portmacro.h"
#endif

#if defined(ESP32)
class Atomic {
public:
  void lock() {
    portENTER_CRITICAL(&_mux);
  }
  void unlock() {
    portEXIT_CRITICAL(&_mux);
  }
private:
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
};
#else
class Atomic {
public:
  void lock() {
    portENTER_CRITICAL();
  }
  void unlock() {
    portEXIT_CRITICAL();
  }
};
#endif

#else

#include "os/OS.hxx"
/// @todo make this a single global mutex
class Atomic : public OSMutex {
public:
  Atomic() : OSMutex(true) {}
};

#endif

/// See @ref OSMutexLock in os/OS.hxx
class AtomicHolder {
public:
    /// Constructor. Grabs the mutex as a side effect.
    ///
    /// @param parent the mutex (atomic) to hold.
    ///
  AtomicHolder(Atomic* parent)
    : parent_(parent) {
    parent_->lock();
  }
    /// Destructor. Releases the mutex as a side effect.
  ~AtomicHolder() {
    parent_->unlock();
  }
private:
    /// Parent mutex we are holding.
  Atomic* parent_;
};

/// See @ref OSMutexLock in os/OS.hxx. This stanza catches a common bug when
/// someone allocates an AtomicHolder without specifying the variable name:
///
/// BAD:
/// AtomicHolder(&lock_);
///
/// GOOD:
/// AtomicHolder h(&lock_);
///
/// The problem with the first instance is that is creates a temporary obejct
/// that gets immediately destructed. Thus while the code seems like correct,
/// it does not actually keep the lock.
#define AtomicHolder(l) int error_omitted_lock_holder_variable[-1]

#endif // _UTILS_LOCK_HXX_
