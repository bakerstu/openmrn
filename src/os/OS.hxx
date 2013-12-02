/** \copyright
 * Copyright (c) 2012, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file OS.hxx
 * This file represents a C++ language abstraction of common operating
 * system calls.
 *
 * @author Stuart W. Baker
 * @date 28 May 2012
 */

#ifndef _os_hxx_
#define _os_hxx_

#include <endian.h>

#include "utils/macros.h"
#include "os/os.h"

/** This class provides a threading API.
 */
class OSThread {
 public:
  /** Create a thread.
   * @param name name of thread, NULL for an auto generated name
   * @param priority priority of created thread
   * @param stack_size size in bytes of the created thread's stack
   * @param start_routine entry point of the thread
   * @param arg entry parameter to the thread
   */
  OSThread(const char *name, int priority, size_t stack_size,
           void *(*start_routine)(void *), void *arg) {
    os_thread_create(&handle, name, priority, stack_size, start_routine, arg);
  }

  /** Create a thread.  This constructor can be used when OSThread is inherited.
    * @param name name of thread, NULL for an auto generated name
    * @param priority priority of created thread
    * @param stack_size size in bytes of the created thread's stack
    */
  OSThread(const char *name, int priority, size_t stack_size) {
    os_thread_create(&handle, name, priority, stack_size, start, this);
  }

  /** Default destructor. */
  ~OSThread() {}

 protected:
  /** User entry point for the created thread.
   * @return exit status
   */
  virtual void *entry() { return NULL; }

 private:
  /** Starting point for a new thread.
   * @param arg pointer to an OSThread instance
   * @return exit status
   */
  static void *start(void *arg) {
    OSThread *thread = (OSThread *)arg;
    return thread->entry();
  }

  /** Default Constructor */
  OSThread();

  DISALLOW_COPY_AND_ASSIGN(OSThread);

  /** Private thread handle. */
  os_thread_t handle;
};

/** This class provides support for one time initialization.
 */
class OSThreadOnce {
 public:
  /** One time intialization constructor.
   * @param routine method to call once
   */
  OSThreadOnce(void (*routine)(void))
      : handle(OS_THREAD_ONCE_INIT), routine(routine) {}

  /** call one time intialization routine
   * @return 0 upon success
   */
  int once(void) { return os_thread_once(&handle, routine); }

  /** Default Destructor */
  ~OSThreadOnce() {}

 private:
  DISALLOW_COPY_AND_ASSIGN(OSThreadOnce);

  /** Private once handle. */
  os_thread_once_t handle;

  /** One time initialization routine */
  void (*routine)(void);
};

/** This class provides a timer API.
 * The return value of the callback determines the behavior of how the timer is
 * rearmed.  @ref OS_TIMER_NONE indicates that the timer is not restarted,
 * OS_TIMER_RESTART restarts the timer with the period of the last timeout,
 * OS_TIMER_DELETE deletes the timer, and all other values indicate the restart
 * period in nanoseconds.
 */
class OSTimer {
 public:
  /** Create a new timer.
   * @param callback callback associated with timer
   * @param data1 data to pass along with callback
   * @param data2 data to pass along with callback
   */
  OSTimer(long long (*callback)(void *, void *), void *data1, void *data2) {
    handle = os_timer_create(callback, data1, data2);
  }

  /** Delete a timer.
   */
  ~OSTimer() { os_timer_delete(handle); }

  /** Start a timer.
   * @param period period in nanoseconds before expiration
   */
  void start(long long period) { os_timer_start(handle, period); }

  /** Delete a timer.
   */
  void stop() { os_timer_stop(handle); }

 private:
  /** Default constructor.
   */
  OSTimer();

  DISALLOW_COPY_AND_ASSIGN(OSTimer);

  /** Private timer handle. */
  os_timer_t handle;
};

/** This class provides a counting semaphore API.
 */
class OSSem {
 public:
  /** Initialize a Semaphore.
   * @param value initial count
   */
  OSSem(unsigned int value = 0) { os_sem_init(&handle, value); }

  ~OSSem() { os_sem_destroy(&handle); }

  /** Post (increment) a semaphore.
   */
  void post() { os_sem_post(&handle); }

  /** Wait on (decrement) a semaphore.
   */
  void wait() { os_sem_wait(&handle); }

  /** Wait on (decrement) a semaphore with timeout condition.
   * @param timeout timeout in nanoseconds, else OS_WAIT_FOREVER to wait forever
   * @return 0 upon success, else -1 with errno set to indicate error
   */
  int timedwait(long long timeout) {
    return os_sem_timedwait(&handle, timeout);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(OSSem);

  /** Private semaphore handle. */
  os_sem_t handle;
};

/** This class provides a mutex API.
 */
class OSMutex {
 public:
  /** Initialize a mutex.
   * @param recursive false creates a normal mutex, true creates a recursive
   * mutex
   */
  OSMutex(bool recursive = false) {
    if (recursive) {
      os_recursive_mutex_init(&handle);
    } else {
      os_mutex_init(&handle);
    }
  }

  /** Lock a mutex.
   */
  void lock() { os_mutex_lock(&handle); }

  /** Unlock a mutex.
   */
  void unlock() { os_mutex_unlock(&handle); }

  /** Destructor */
  ~OSMutex() { os_mutex_destroy(&handle); }

 private:
  DISALLOW_COPY_AND_ASSIGN(OSMutex);

  friend class OSMutexLock;
  /** Private mutex handle. */
  os_mutex_t handle;
};

/**
 * Class to allow convenient locking and unlocking of mutexes in a C context.
 * The mutex will be automatically unlocked when the context is left, even if
 * there are multiple return, break or continue statements.
 *
 * Usage:
 *
 * void foo()
 * {
 *   // ...non-critical-section code here...
 *   {
 *     OSMutexLock locker(&mutex_);
 *     // ...critical section here...
 *     if (error) return;
 *     // ...more critical code here...
 *   }
 *   // ...at this point the mutex is unlocked...
 *   // ...more non-critical-section code here...
 * }
 *
 */
class OSMutexLock {
 public:
  OSMutexLock(OSMutex *mutex) : mutex_(&mutex->handle) {
    os_mutex_lock(mutex_);
  }

  OSMutexLock(os_mutex_t *mutex) : mutex_(mutex) { os_mutex_lock(mutex_); }

  ~OSMutexLock() { os_mutex_unlock(mutex_); }

 private:
  DISALLOW_COPY_AND_ASSIGN(OSMutexLock);

  os_mutex_t *mutex_;
};

/** This catches programming errors where you declare a mutex locker object
 *  without a name like this:
 *
 *  void foo()
 *  {
 *    OSMutexLock(&mutex_);
 *    // ...critical section here...
 *  }

 *  The above is valid C++, which creates a temporary OSMutexLock object, locks
 *  it, then immediately destructs the object, unlocking the mutex in the same
 *  line it was created. Thus the critical sections goes unprotected
 *  unintentionally.
 *
 *  The correct code is
 *
 *  void foo()
 *  {
 *    OSMutexLock locker(&mutex_);
 *    // ...critical section here...
 *  }
 *
 */
#define OSMutexLock(l) int error_omitted_mutex_lock_variable[-1]

class OSTime {
 public:
  /** Get the monotonic time since the system started.
   * @return time in nanoseconds since system start
   */
  static long long get_monotonic(void) { return os_get_time_monotonic(); }

 private:
  DISALLOW_COPY_AND_ASSIGN(OSTime);

  /* Private default constructor prevents instantiating this class. */
  OSTime();

  /** Default destructor. */
  ~OSTime();
};

#endif /* _os_hxx_ */
