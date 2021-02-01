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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <esp_vfs.h>

/// Protects the initialization of vfs_id.
static pthread_once_t vfs_init_once = PTHREAD_ONCE_INIT;
/// This per-thread key will store the OSSelectWakeup object that has been
/// locked to any given calling thread.
static pthread_key_t select_wakeup_key;
static int wakeup_fd;

extern "C"
{
    void *sys_thread_sem_get();
    void sys_sem_signal(void *);
    void sys_sem_signal_isr(void *);
}

static int esp_wakeup_open(const char * path, int flags, int mode) {
    // This virtual FS has only one fd, 0.
    return 0;
}

static void esp_end_select()
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    HASSERT(parent);
    parent->esp_end_select();
}

/// This function is called inline from the ESP32's select implementation. It is
/// passed in as a function pointer to the VFS API.
/// @param nfds see standard select API
/// @param readfds see standard select API
/// @param writefds see standard select API
/// @param exceptfds see standard select API
/// @param signal_sem if non-NULL, the select can be woken up by notifying this
/// semaphore. If NULL, the select can be woken up by notifying the LWIP
/// semaphore. By the API contract this pointer needs to be passed into
/// esp_vfs_select_triggered.
static esp_err_t esp_start_select(int nfds, fd_set *readfds, fd_set *writefds,
    fd_set *exceptfds, SemaphoreHandle_t *signal_sem)
{
    OSSelectWakeup *parent =
        (OSSelectWakeup *)pthread_getspecific(select_wakeup_key);
    LOG(VERBOSE, "esp start select %p  (thr %p parent %p)", signal_sem, os_thread_self(), parent);
    HASSERT(parent);
    parent->esp_start_select(signal_sem);
    return ESP_OK;
}

void OSSelectWakeup::esp_start_select(void *signal_sem)
{
    AtomicHolder h(this);
    espSem_ = signal_sem;
    woken_ = false;
}

void OSSelectWakeup::esp_end_select()
{
    AtomicHolder h(this);
    woken_ = true;
}

void OSSelectWakeup::esp_wakeup()
{
    if (woken_)
    {
        return;
    }
    AtomicHolder h(this);
    if (woken_)
    {
        return;
    }
    woken_ = true;
#if 0
    esp_vfs_select_triggered((SemaphoreHandle_t *)espSem_);
#else
    LOG(VERBOSE, "wakeup es %p %u lws %p", espSem_, *(unsigned*)espSem_, lwipSem_);
    if (espSem_)
    {
        esp_vfs_select_triggered((SemaphoreHandle_t *)espSem_);
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
#endif
}

void OSSelectWakeup::esp_wakeup_from_isr()
{
    if (woken_)
    {
        return;
    }
    AtomicHolder h(this);
    if (woken_)
    {
        return;
    }
    woken_ = true;
    BaseType_t woken = pdFALSE;
#if 0
    esp_vfs_select_triggered_isr((SemaphoreHandle_t *)espSem_, &woken);
    if (woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
#else
    if (espSem_)
    {
        esp_vfs_select_triggered_isr((SemaphoreHandle_t *)espSem_, &woken);
        if (woken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
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
#endif
}

static void esp_vfs_init()
{
    esp_vfs_t vfs;
    memset(&vfs, 0, sizeof(vfs));
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    vfs.start_select = &esp_start_select;
    vfs.end_select = &esp_end_select;
    vfs.open = &esp_wakeup_open;
    ESP_ERROR_CHECK(esp_vfs_register("/dev/wakeup", &vfs, nullptr));
    HASSERT(0 == pthread_key_create(&select_wakeup_key, nullptr));
    wakeup_fd = ::open("/dev/wakeup/0", 0, 0);
    HASSERT(wakeup_fd >= 0);
    LOG(VERBOSE, "VFSINIT wakeup fd %d", wakeup_fd);
}

void OSSelectWakeup::esp_allocate_vfs_fd()
{
    lwipSem_ = sys_thread_sem_get();
    pthread_once(&vfs_init_once, &esp_vfs_init);
    vfsFd_ = wakeup_fd;
    pthread_setspecific(select_wakeup_key, this);
    LOG(VERBOSE, "VFSALLOC wakeup fd %d (thr %p test %p)", vfsFd_, os_thread_self(), pthread_getspecific(select_wakeup_key));
}

void OSSelectWakeup::esp_deallocate_vfs_fd()
{
}

#endif // ESP32
