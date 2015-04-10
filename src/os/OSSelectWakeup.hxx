/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file OSSelectWakeup.hxx
 * Helper class for portable wakeup of a thread blocked in a select call.
 *
 * @author Balazs Racz
 * @date 10 Apr 2015
 */

#ifndef _OS_OSSELECTWAKEUP_HXX_
#define _OS_OSSELECTWAKEUP_HXX_

#include "utils/Atomic.hxx"
#include "os/os.h"

#ifndef __FreeRTOS__
#include <signal.h>
#endif

void empty_signal_handler(int);

/** Helper class that allows a select to be asynchronously woken up. */
class OSSelectWakeup : private Atomic
{
public:
    OSSelectWakeup()
        : pendingWakeup_(false)
        , inSelect_(false)
    {
    }

    /** Prepares the current thread for asynchronous wakeups. Can be called
     * only once. */
    void lock_to_thread()
    {
#ifdef __FreeRTOS__
        Device::select_insert(&selectInfo_);
#else
        // Gets the current thread.
        thread_ = os_thread_self();
        // Blocks SIGUSR1 in the signal mask of the current thread.
        sigset_t usrmask;
        HASSERT(!sigemptyset(&usrmask));
        HASSERT(!sigaddset(&usrmask, WAKEUP_SIG));
        HASSERT(!sigprocmask(SIG_BLOCK, &usrmask, &origMask_));
        HASSERT(!sigdelset(&origMask_, WAKEUP_SIG));
        struct sigaction action;
        action.sa_handler = &empty_signal_handler;
        HASSERT(!sigemptyset(&action.sa_mask));
        action.sa_flags = 0;
        HASSERT(!sigaction(WAKEUP_SIG, &action, nullptr));
#endif
    }

    /** Wakes up the select in the locked thread. */
    void wakeup()
    {
        bool need_wakeup = false;
        {
            AtomicHolder l(this);
            pendingWakeup_ = true;
            if (inSelect_)
            {
                need_wakeup = true;
            }
        }
        if (need_wakeup)
        {
#ifdef __FreeRTOS__
            Device::select_wakeup(&selectInfo_);
#else
            pthread_kill(thread_, WAKEUP_SIG);
#endif
        }
    }

    void clear_wakeup()
    {
        pendingWakeup_ = false;
    }

#ifdef __FreeRTOS__
    void wakeup_from_isr()
    {
        pendingWakeup_ = true;
        if (inSelect_)
        {
            int woken;
            Device::select_wakeup_from_isr(&selectInfo_, &woken);
        }
    }
#endif

    /** Portable call to a select that can be woken up asynchronously from a
     * different thread or an ISR context.
     *
     * @param nfds, readfds, writefds, exceptfds is as a regular ::select call.
     * @param deadline_nsec is the maximum time to sleep if no fd activity and
     * no wakeup happens. -1 to sleep indefinitely, 0 to return immediately.
     *
     * return what select would return (number of live FDs, 0 in case of
     * timeout), or -1 and errno==EINTR if the select was woken up
     * asynchronously
     */
    int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
        long long deadline_nsec)
    {
        {
            AtomicHolder l(this);
            inSelect_ = true;
            if (pendingWakeup_)
            {
                deadline_nsec = 0;
            }
            else
            {
#ifdef __FreeRTOS__
                Device::select_clear();
#endif
            }
        }
#ifdef __FreeRTOS__
        int ret =
            Device::select(nfds, readfds, writefds, exceptfds, deadline_nsec);
        if (!ret && pendingWakeup_)
        {
            ret = -1;
            errno = EINTR;
        }
#else
        struct timespec timeout;
        timeout.tv_sec = deadline_nsec / 1000000000;
        timeout.tv_nsec = deadline_nsec % 1000000000;
        int ret =
            ::pselect(nfds, readfds, writefds, exceptfds, &timeout, &origMask_);
#endif
        {
            AtomicHolder l(this);
            pendingWakeup_ = false;
            inSelect_ = false;
        }
        return ret;
    }

private:
    /** This signal is used for the wakeup kill in a pthreads OS. */
    static const int WAKEUP_SIG = SIGUSR1;
    /** True if there was a wakeup call since the previous select finished. */
    bool pendingWakeup_;
    /** True during the duration of a select operation. */
    bool inSelect_;
#ifdef __FreeRTOS__
    Device::SelectInfo selectInfo_;
#else
    os_thread_t thread_;
    sigset_t origMask_;
#endif
};

#endif // _OS_OSSELECTWAKEUP_HXX_
