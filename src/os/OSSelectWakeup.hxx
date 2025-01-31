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

#include <unistd.h>

#include "openmrn_features.h"
#include "utils/Atomic.hxx"
#include "os/os.h"

#if OPENMRN_FEATURE_DEVICE_SELECT
#include "Devtab.hxx"
#endif

#if OPENMRN_HAVE_PSELECT
#include <signal.h>
#endif

#ifdef __WINNT__
#include <winsock2.h>
#elif OPENMRN_HAVE_SELECT
#include <sys/select.h>
#endif

#ifdef ESP_PLATFORM
#include "sdkconfig.h"

#ifdef CONFIG_VFS_SUPPORT_TERMIOS
// remove defines added by arduino-esp32 core/esp32/binary.h which are
// duplicated in sys/termios.h which may be included by esp_vfs.h
#undef B110
#undef B1000000
#endif // CONFIG_VFS_SUPPORT_TERMIOS

#include <esp_vfs.h>

#endif // ESP_PLATFORM

/// Signal handler that does nothing. @param sig ignored.
void empty_signal_handler(int sig);

/** Helper class that allows a select to be asynchronously woken up. */
class OSSelectWakeup : private Atomic
{
public:
    OSSelectWakeup()
        : pendingWakeup_(false)
        , inSelect_(false)
    {
    }

    ~OSSelectWakeup()
    {
#ifdef ESP_PLATFORM
        esp_deallocate_vfs_fd();
#endif
    }

    /// @return the thread ID that we are engaged upon.
    os_thread_t main_thread()
    {
        return thread_;
    }

    /** Prepares the current thread for asynchronous wakeups. Can be called
     * only once. */
    void lock_to_thread()
    {
        // Gets the current thread.
        thread_ = os_thread_self();
#ifdef ESP_PLATFORM
        esp_allocate_vfs_fd();
#endif
#if OPENMRN_FEATURE_DEVICE_SELECT
        Device::select_insert(&selectInfo_);
#elif OPENMRN_HAVE_PSELECT
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
#if OPENMRN_FEATURE_DEVICE_SELECT
            HASSERT(selectInfo_.event);
            // We cannot destroy the thread ID in the local object.
            Device::SelectInfo copy(selectInfo_);
            Device::select_wakeup(&copy);
#elif OPENMRN_HAVE_PSELECT
            pthread_kill(thread_, WAKEUP_SIG);
#elif defined(ESP_PLATFORM)
            esp_wakeup();
#elif !defined(OPENMRN_FEATURE_SINGLE_THREADED)
            DIE("need wakeup code");
#endif
        }
    }

    /// Called from the main thread after being woken up. Enables further
    /// wakeup signals to be collected.
    void clear_wakeup()
    {
        pendingWakeup_ = false;
    }

#if OPENMRN_FEATURE_RTOS_FROM_ISR
    void wakeup_from_isr()
    {
#if defined(ESP_PLATFORM)
        // On multi-core ESP32s we need to lock objects even in ISRs.
        AtomicHolder h(this);
#endif
        pendingWakeup_ = true;
        if (inSelect_)
        {
#if OPENMRN_FEATURE_DEVICE_SELECT
            HASSERT(selectInfo_.event);
            Device::SelectInfo copy(selectInfo_);
            int woken;
            Device::select_wakeup_from_isr(&copy, &woken);
            os_isr_exit_yield_test(woken);
// TODO: confirm if pthread_kill is ISR safe
//#elif OPENMRN_HAVE_PSELECT
//            pthread_kill(thread_, WAKEUP_SIG);
#elif defined(ESP_PLATFORM)
            esp_wakeup_from_isr();
#else
            DIE("need wakeup code");
#endif
        }
    }
#endif // OPENMRN_FEATURE_RTOS_FROM_ISR

    /** Portable call to a select that can be woken up asynchronously from a
     * different thread or an ISR context.
     *
     * @param nfds is as a regular ::select call.
     * @param readfds is as a regular ::select call.
     * @param writefds is as a regular ::select call.
     * @param exceptfds is as a regular ::select call.
     * @param deadline_nsec is the maximum time to sleep if no fd activity and
     * no wakeup happens. -1 to sleep indefinitely, 0 to return immediately.
     *
     * @return what select would return (number of live FDs, 0 in case of
     * timeout), or -1 and errno==EINTR if the select was woken up
     * asynchronously
     */
    int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
               long long deadline_nsec);

private:
#ifdef ESP_PLATFORM
    void esp_allocate_vfs_fd();
    void esp_deallocate_vfs_fd();
    void esp_wakeup();
    void esp_wakeup_from_isr();
public:
    void esp_start_select(fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
                          esp_vfs_select_sem_t signal_sem);
    void esp_end_select();

private:
    /// FD for waking up select in ESP32 VFS implementation.
    int vfsFd_{-1};

    /// Semaphore provided by the ESP32 VFS layer to use for waking up the
    /// ESP32 early from the select() call.
    esp_vfs_select_sem_t espSem_;

    /// FD set provided by the ESP32 VFS layer to use when waking up early from
    /// select, this tracks which FDs have an error (or exception).
    fd_set *exceptFds_;

    /// Copy of the initial state of the except FD set provided by the ESP32 VFS
    /// layer. This is used for checking if we need to set the bit for the FD
    /// when waking up early from select().
    fd_set exceptFdsOrig_;

#endif // ESP_PLATFORM

#if OPENMRN_HAVE_PSELECT
    /** This signal is used for the wakeup kill in a pthreads OS. */
    static const int WAKEUP_SIG = SIGUSR1;
#endif
    /** True if there was a wakeup call since the previous select finished. */
    bool pendingWakeup_;
    /** True during the duration of a select operation. */
    bool inSelect_;
    /// ID of the main thread we are engaged upon.
    os_thread_t thread_;
#if OPENMRN_FEATURE_DEVICE_SELECT
    Device::SelectInfo selectInfo_;
#endif
#if OPENMRN_HAVE_PSELECT
    /// Original signal mask. Used for pselect to reenable the signal we'll be
    /// using to wake up.
    sigset_t origMask_;
#endif
};

#endif // _OS_OSSELECTWAKEUP_HXX_
