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
* \file OSSelectWakeup.cxx
* Helper class for portable wakeup of a thread blocked in a select call.
*
* @author Balazs Racz
* @date 10 Apr 2015
*/

#include "os/OSSelectWakeup.hxx"
#include "utils/logging.h"
#if defined(__MACH__)
#define _DARWIN_C_SOURCE // pselect
#endif

void empty_signal_handler(int)
{
}

int OSSelectWakeup::select(int nfds, fd_set *readfds,
                           fd_set *writefds, fd_set *exceptfds,
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
#if OPENMRN_FEATURE_DEVICE_SELECT
            Device::select_clear();
#endif
        }
    }
#if OPENMRN_FEATURE_DEVICE_SELECT
    int ret =
        Device::select(nfds, readfds, writefds, exceptfds, deadline_nsec);
    if (!ret && pendingWakeup_)
    {
        ret = -1;
        errno = EINTR;
    }
#elif OPENMRN_HAVE_PSELECT
    struct timespec timeout;
    timeout.tv_sec = deadline_nsec / 1000000000;
    timeout.tv_nsec = deadline_nsec % 1000000000;
    int ret =
        ::pselect(nfds, readfds, writefds, exceptfds, &timeout, &origMask_);
#elif OPENMRN_HAVE_SELECT
#ifdef ESP32
    fd_set newexcept;
    if (!exceptfds)
    {
        FD_ZERO(&newexcept);
        exceptfds = &newexcept;
    }
    FD_SET(vfsFd_, exceptfds);
    if (vfsFd_ >= nfds) {
        nfds = vfsFd_ + 1;
    }
#endif //ESP32
    struct timeval timeout;
    timeout.tv_sec = deadline_nsec / 1000000000;
    timeout.tv_usec = (deadline_nsec / 1000) % 1000000;
    int ret =
        ::select(nfds, readfds, writefds, exceptfds, &timeout);
#elif !defined(OPENMRN_FEATURE_SINGLE_THREADED)
    #error no select implementation in multi threaded OS.
#else
    // Single threaded OS: nothing to wake up.
    int ret = 0;
#endif
    {
        AtomicHolder l(this);
        pendingWakeup_ = false;
        inSelect_ = false;
    }
    return ret;
}

#ifdef ESP32
#include "freertos_includes.h"

#include <esp_system.h>
#include <esp_vfs.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

/// Protects the initialization of vfs_id.
static pthread_once_t vfs_init_once = PTHREAD_ONCE_INIT;

/// This per-thread key will store the OSSelectWakeup object that has been
/// locked to any given calling thread.
static pthread_key_t select_wakeup_key;

/// This is the VFS FD that will be returned for ::open().
///
/// NOTE: The ESP VFS layer will ensure uniqueness and pass this FD back into
/// all VFS APIs we implement.
static constexpr int WAKEUP_VFS_FD = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,0,0)
extern "C"
{
    void *sys_thread_sem_get();
    void sys_sem_signal(void *);
    void sys_sem_signal_isr(void *);
}
#endif // NOT IDF v4.0+

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
/// This function is called by the ESP32's select implementation. It is passed
/// in as a function pointer to the VFS API.
/// @param nfds see standard select API
/// @param readfds see standard select API
/// @param writefds see standard select API
/// @param exceptfds see standard select API
/// @param signal_sem is the semaphore object to trigger when the select has
/// completed.
/// @param end_select_args are the arguments to pass to end_select upon wakeup.
static esp_err_t esp_start_select(int nfds, fd_set *readfds, fd_set *writefds,
    fd_set *exceptfds, esp_vfs_select_sem_t signal_sem, void **end_select_args)
#else // NOT IDF v4.0+
/// This function is called by the ESP32's select implementation. It is passed
/// in as a function pointer to the VFS API.
/// @param nfds see standard select API
/// @param readfds see standard select API
/// @param writefds see standard select API
/// @param exceptfds see standard select API
/// @param signal_sem if non-NULL, the select can be woken up by notifying this
/// semaphore. If NULL, the select can be woken up by notifying the LWIP
/// semaphore. By the API contract this pointer needs to be passed into
/// esp_vfs_select_triggered.
static esp_err_t esp_start_select(int nfds, fd_set *readfds, fd_set *writefds,
    fd_set *exceptfds, esp_vfs_select_sem_t signal_sem)
#endif // IDF v4.0+
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    HASSERT(parent);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
    LOG(VERBOSE, "esp start select %p (thr %p parent %p)", signal_sem.sem
      , os_thread_self(), parent);
#else // NOT IDF v4.0+
    LOG(VERBOSE, "esp start select %p  (thr %p parent %p)", signal_sem
      , os_thread_self(), parent);
#endif // IDF v4.0+
    // Check if there is at least one FD and our VFS FD is included in one of
    // the sets before calling esp_start_select.
    if (nfds >= 1 &&
        (FD_ISSET(WAKEUP_VFS_FD, readfds) ||
         FD_ISSET(WAKEUP_VFS_FD, writefds) ||
         FD_ISSET(WAKEUP_VFS_FD, exceptfds)))
    {
        parent->esp_start_select(readfds, writefds, exceptfds, signal_sem);
    }
    return ESP_OK;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
/// This function is called inline from the ESP32's select implementation. It is
/// passed in as a function pointer to the VFS API.
///
/// @param arg is the value that was provided as part of esp_start_select, this
/// is not used today.
/// @return ESP_OK for success (there are no error conditions currently).
static esp_err_t esp_end_select(void *arg)
#else // NOT IDF v4.0+
/// This function is called inline from the ESP32's select implementation. It is
/// passed in as a function pointer to the VFS API.
static void esp_end_select()
#endif // IDF v4.0+
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    HASSERT(parent);
    LOG(VERBOSE, "esp end select (thr %p parent %p)", os_thread_self()
      , parent);
    parent->esp_end_select();
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
    return ESP_OK;
#endif // IDF v4.0+
}

/// This function is called by the ESP32's select implementation.
/// @param signal_sem is the semaphore container provided by the VFS layer that
/// can be used to wake up the select() call early.
void OSSelectWakeup::esp_start_select(fd_set *readfds, fd_set *writefds,
    fd_set *exceptfds, esp_vfs_select_sem_t signal_sem)
{
    AtomicHolder h(this);
    espSem_ = signal_sem;
    readFds_ = readfds;
    origReadFds_ = *readfds;
    writeFds_ = writefds;
    origWriteFds_ = *writefds;
    exceptFds_ = exceptfds;
    origExceptFds_ = *exceptfds;
    semValid_ = true;
}

/// This function marks the stored semaphore as invalid which indicates no
/// active select() call we are interested in.
void OSSelectWakeup::esp_end_select()
{
    AtomicHolder h(this);
    semValid_ = false;
}

/// This function will trigger the ESP32 to wake up from any pending select()
/// call.
void OSSelectWakeup::esp_wakeup()
{
    AtomicHolder h(this);
    if (semValid_)
    {
        if (FD_ISSET(vfsFd_, &origReadFds_))
        {
            FD_SET(vfsFd_, readFds_);
        }
        if (FD_ISSET(vfsFd_, &origWriteFds_))
        {
            FD_SET(vfsFd_, writeFds_);
        }
        if (FD_ISSET(vfsFd_, &origExceptFds_))
        {
            FD_SET(vfsFd_, exceptFds_);
        }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
        LOG(VERBOSE, "wakeup es %p %u", espSem_.sem, *(unsigned*)espSem_.sem);
#else
        LOG(VERBOSE, "wakeup es %p %u", espSem_, *(unsigned*)espSem_);
        if (espSem_)
        {
#endif // IDF v4+
        esp_vfs_select_triggered(espSem_);
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,0,0)
        }
        else
        {
            // Works around a bug in the implementation of
            // esp_vfs_select_triggered, which internally calls
            // sys_sem_signal(sys_thread_sem_get()); This is buggy because
            // sys_thread_sem_get() will get the semaphore that belongs to the
            // calling thread, not the target thread to wake up.
            sys_sem_signal(lwipSem_);
        }
#endif // IDF < v4.0
    }
}

/// This function will trigger the ESP32 to wake up from any pending select()
/// call from within an ISR context.
void OSSelectWakeup::esp_wakeup_from_isr()
{
    AtomicHolder h(this);
    if (semValid_)
    {
        BaseType_t woken = pdFALSE;
        if (FD_ISSET(vfsFd_, &origReadFds_))
        {
            FD_SET(vfsFd_, readFds_);
        }
        if (FD_ISSET(vfsFd_, &origWriteFds_))
        {
            FD_SET(vfsFd_, writeFds_);
        }
        if (FD_ISSET(vfsFd_, &origExceptFds_))
        {
            FD_SET(vfsFd_, exceptFds_);
        }
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,0,0)
        if (espSem_)
        {
#endif // IDF < v4.0
        esp_vfs_select_triggered_isr(espSem_, &woken);
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,0,0)
        }
        else
        {
            // Works around a bug in the implementation of
            // esp_vfs_select_triggered, which internally calls
            // sys_sem_signal(sys_thread_sem_get()); This is buggy because
            // sys_thread_sem_get() will get the semaphore that belongs to the
            // calling thread, not the target thread to wake up.
            sys_sem_signal_isr(lwipSem_);
        }
#endif // IDF < v4.0

// In IDF v4.3 the portYIELD_FROM_ISR macro was extended to take the value of
// the wakeup flag and if true will yield as expected otherwise it is mostly a
// no-op.
#if ESP_IDF_VERSION <= ESP_IDF_VERSION_VAL(4,2,0)
        if (wakeup == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
#else
        portYIELD_FROM_ISR(wakeup);
#endif // ESP_IDF_VERSION > 4.2.0
    }
}

static int esp_wakeup_open(const char * path, int flags, int mode) {
    // This virtual FS has only one fd, 0.
    return WAKEUP_VFS_FD;
}

static void esp_vfs_init()
{
    esp_vfs_t vfs;
    memset(&vfs, 0, sizeof(vfs));
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    vfs.start_select = esp_start_select;
    vfs.end_select = esp_end_select;
    vfs.open = esp_wakeup_open;
    ESP_ERROR_CHECK(esp_vfs_register("/dev/wakeup", &vfs, nullptr));
    HASSERT(0 == pthread_key_create(&select_wakeup_key, nullptr));
}

void OSSelectWakeup::esp_allocate_vfs_fd()
{
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,0,0)
    lwipSem_ = sys_thread_sem_get();
#endif // IDF < v4.0

    HASSERT(0 == pthread_once(&vfs_init_once, &esp_vfs_init));
    vfsFd_ = ::open("/dev/wakeup/0", 0, 0);
    HASSERT(vfsFd_ >= 0);
    HASSERT(0 == pthread_setspecific(select_wakeup_key, this));
    LOG(VERBOSE, "VFSALLOC wakeup fd %d (thr %p test %p)", vfsFd_,
        os_thread_self(), pthread_getspecific(select_wakeup_key));
}

void OSSelectWakeup::esp_deallocate_vfs_fd()
{
    if (vfsFd_ >= 0)
    {
        ::close(vfsFd_);
    }
    vfsFd_ = -1;
}

#endif // ESP32
