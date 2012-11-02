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

#include "os/os.h"

/** This class provides a threading API.
 */
class OSThread
{
public:
    /** Create a thread.
     * @param priority priority of created thread
     * @param stack_size size in bytes of the created thread's stack
     * @param start_routine entry point of the thread
     * @param arg entry parameter to the thread
     */
    OSThread(int priority, size_t stack_size,
             void *(*start_routine)(void*), void *arg)
    {
        os_thread_create(&handle, priority, stack_size, start_routine, arg);
    }
private:
    /** Private thread handle. */
    os_thread_t handle;
};

/** This class provides a timer API.
 */
class OSTimer
{
public:
    /** Create a new timer.
     * @param callback callback associated with timer
     * @param data1 data to pass along with callback
     * @param data2 data to pass along with callback
     */
    OSTimer(long long (*callback)(void*, void*), void *data1, void *data2)
    {
        handle = os_timer_create(callback, data1, data2);
    }

    /** Delete a timer.
     */
    ~OSTimer()
    {
        os_timer_delete(handle);
    }

    /** Start a timer.
     * @param period period in nanoseconds before expiration
     */
    void start(long long period)
    {
        os_timer_start(handle, period);
    }

    /** Delete a timer.
     */
    void stop()
    {
        os_timer_stop(handle);
    }
private:
    /** Private timer handle. */
    os_timer_t handle;
};

/** This class provides a mutex API.
 */
class OSMutex
{
    /** Initialize a mutex.
     * @param recursive false creates a normal mutex, true creates a recursive mutex
     */
    OSMutex(bool recursive = false)
    {
        if (recursive)
        {
            os_recursive_mutex_init(&handle);
        }
        else
        {
            os_mutex_init(&handle);
        }
    }

    /** Lock a mutex.
     */
    void lock()
    {
        os_mutex_lock(&handle);
    }

    /** Unlock a mutex.
     */
    void unlock()
    {
        os_mutex_unlock(&handle);
    }

private:
    /** Private mutex handle. */
    os_mutex_t handle;
};

#endif /* _os_hxx_ */
