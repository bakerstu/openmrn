/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file Select.cxx
 * This file imlements POSIX select().
 *
 * @author Stuart W. Baker
 * @date 26 January 2015
 */

#include <fcntl.h>
#include <sys/select.h>

#include "Devtab.hxx"

/** event used to wakeup select calls */
static OSEvent wakeup;

/** Returns an event bit unique to the current thread.
 * @return event bit unique to the current thread
 */
static OSEventType get_event()
{
    static int thread_count = 0;
    ThreadPriv *priv = (ThreadPriv*)xTaskGetApplicationTaskTag(NULL);

    if (priv->selectEventBit == 0)
    {
        if(thread_count >= OSEvent::number_of_bits())
        {
            thread_count = 0;
        }
        priv->selectEventBit = 0x1 << thread_count;
        ++thread_count;
    }

    return priv->selectEventBit;
}

void Device::select_clear()
{
    portENTER_CRITICAL();
    OSEventType event = get_event();
    portEXIT_CRITICAL();
    wakeup.clear(event);
}

/** POSIX select().
 * @param nfds highest numbered file descriptor in any of the three, sets plus 1
 * @param readfds fd_set of file descritpors to pend on read active
 * @param writefds fd_set of file descritpors to pend on write active
 * @param exceptfds fd_set of file descritpors to pend on error active
 * @param timeout timeout in nsec to wait, if 0, return immediately, if < 0
 *                wait forever
 * @return on success, number of file descriptors in the three sets that are
 *         active, 0 on timeout, -1 with errno set appropriately upon error.
 */
int Device::select(int nfds, fd_set *readfds, fd_set *writefds,
                   fd_set *exceptfds, long long timeout)
{
    long long until = 0;
    if (timeout >= 0)
    {
        until = OSTime::get_monotonic() + timeout;
    }
    fd_set rd_result;
    fd_set wr_result;
    fd_set ex_result;

    int mode_type[] = {FREAD, FWRITE, 0};
    fd_set *in_set[3] = {readfds, writefds, exceptfds};
    fd_set *out_set[3] = {&rd_result, &wr_result, &ex_result};
    int number = 0;

    FD_ZERO(&rd_result);
    FD_ZERO(&wr_result);
    FD_ZERO(&ex_result);

    portENTER_CRITICAL();
    OSEventType event = get_event();
    portEXIT_CRITICAL();

    bool first = true;

    for ( ; /* forever */ ; )
    {
        /* cycle through all the FD Sets */
        for (int mode = 0; mode < 3; ++mode)
        {
            if (in_set[mode])
            {
                /* cycle through all the file descriptors */
                for (int i = 0; i < nfds; ++i)
                {
                    if (FD_ISSET(i, in_set[mode]))
                    {
                        File *file = file_lookup(i);
                        if (!file)
                        {
                            /* errno should already be set appropriately */
                            return -1;
                        }
                        if (file->dev->select(file, mode_type[mode]))
                        {
                            /* active */
                            FD_SET(i, out_set[mode]);
                            ++number;
                        }
                    }
                }
            }
        }

        if (number)
        {
            /* We have a read */
            if (readfds)
            {
                *readfds = rd_result;
            }
            if (writefds)
            {
                *writefds = wr_result;
            }
            if (exceptfds)
            {
                *exceptfds = ex_result;
            }
            return number;
        }
        // We only run two rounds of this loop; if the event bit was signaled,
        // our thread was likely woken up by a signal.
        if (!first) {
            errno = EINTR;
            return -1;
        }
        first = false;

        if (until)
        {
            long long now = OSTime::get_monotonic();
            if (now >= until)
            {
                return 0;
            }
            wakeup.timedwait(event, NULL, true, OSEvent::WAIT_ANY, until - now);
        }
        else
        {
            /* block forever until fd active */
            wakeup.wait(event, NULL, true, OSEvent::WAIT_ANY);
        }
    }
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool Device::select(File* file, int mode)
{
    return true;
}

/** Add client to list of clients needing woken,
 * @param info wakeup event instance
 */
void Device::select_insert(SelectInfo *info)
{
    /** @todo do we need the critical lock here, or are we always already
     * locked?
     */
    portENTER_CRITICAL();
    info->event |= get_event();
    portEXIT_CRITICAL();
}

/** Wakeup the list of clients needing woken,
 * @param info wakeup event instance
 */
void Device::select_wakeup(SelectInfo *info)
{
    portENTER_CRITICAL();
    if (info->event != 0)
    {
        wakeup.set(info->event);
        info->event = 0;
    }
    portEXIT_CRITICAL();
}

/** Wakeup the list of clients needing woken,
 * @param info wakeup event instance
 */
void Device::select_wakeup_from_isr(SelectInfo *info, int *woken)
{
    if (info->event != 0)
    {
        wakeup.set_from_isr(info->event, woken);
        info->event = 0;
    }
    if (woken)
    {
#ifdef GCC_CM3
        portYIELD_FROM_ISR(*woken);
#elif defined(THUMB_INTERWORK) // armv4t
        if (*woken)
        {
            portYIELD_FROM_ISR();
        }
#else
        os_isr_exit_yield_test(*woken);
#endif
    }
}
