/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file BlockOrWakeUp.hxx
 * This file implements a synchronization primitive for device drivers.
 *
 * @author Balazs Racz
 * @date 26 August 2014
 */

#ifndef _FREERTOS_DRIVERS_COMMON_BLOCKORWAKEUP_HXX_
#define _FREERTOS_DRIVERS_COMMON_BLOCKORWAKEUP_HXX_

#include "os/OS.hxx"
#include "executor/Notifiable.hxx"

/** A synchronization primitive for device drivers, Where execution has to
 * happen in an interrupt context and on regular contexts. It supports the
 * following features:
 *
 * - Execution on a single regular context when multiple threads might be
 *   contending for this lock.
 *
 * - Execution on a single regular context with interrupts disabled.
 *
 * - Blocking a regular context until data is available.
 *
 * - Waking up a regular context from an interrupt.
 *
 * - Registering a Notifiable to be called when a regular context would be
 *   woken up.
 *
 * Critical should be an object (possibly with no member variables for zero
 * size) that has the following functions:
 * - void lock() to prevent execution of the given interrupt
 * - void unlock() to allow execution of the given interrupt
 * - a one-argument constructor or copy constructor.
 *
 * In some cases, Atomic is a good candidate for Critical.
 */
template <class Critical> class BlockOrWakeUp : protected Critical
{
public:
    template <class T>
    BlockOrWakeUp(const T &t)
        : Critical(t)
        , notifiable_(nullptr)
    {
    }

    class LockHolder;
    class CriticalHolder;
    friend class LockHolder;
    friend class CriticalHolder;

    struct CriticalHolder
    {
        ~CriticalHolder()
        {
            if (parent_)
                parent_->unlock();
        }

        CriticalHolder(CriticalHolder &&o)
            : parent_(o.parent_)
        {
            o.parent_ = nullptr;
        }

    private:
        friend class LockHolder;
        // This is private so that we can only get a new holder from a
        // LockHolder.
        CriticalHolder(BlockOrWakeUp<Critical> *parent)
            : parent_(parent)
        {
            parent_->lock();
        }

        BlockOrWakeUp<Critical> *parent_;
    };

    struct LockHolder
    {
        LockHolder(BlockOrWakeUp<Critical> *parent)
            : parent_(parent)
        {
            parent_->single_lock();
        }

        LockHolder(LockHolder &&o)
            : parent_(o.parent_)
        {
            o.parent_ = nullptr;
        }

        ~LockHolder()
        {
            if (parent_)
            {
                parent_->single_unlock();
            }
        }

        CriticalHolder critical() __attribute__((warn_unused_result))
        {
            auto h = CriticalHolder(parent_);
            return h;
        }

        void wait_for_notification()
        {
            parent_->single_unlock();
            parent_->s_.wait();
            parent_->single_lock();
        }

        void notify_next()
        {
            parent_->s_.post();
            Notifiable* n = nullptr;
            if (parent_->notifiable_) {
                n = parent_->notifiable_;
                parent_->notifiable_ = nullptr;
            }
            parent_->single_unlock();
            parent_ = nullptr;
            if (n) n->notify();
        }

    private:
        DISALLOW_COPY_AND_ASSIGN(LockHolder);

        BlockOrWakeUp<Critical> *parent_;
    };

    LockHolder holder() __attribute__((warn_unused_result)) {
        return LockHolder(this);
    }

    void notify_from_isr()
    {
        s_.post_from_isr();
        if (notifiable_) {
            notifiable_->notify_from_isr();
            notifiable_ = nullptr;
        }
    }

    Notifiable* register_notifiable(Notifiable *n)
    {
        Notifiable *r = notifiable_;
        notifiable_ = n;
        return r;
    }

private:
    void single_lock()
    {
        m_.lock();
    }
    void single_unlock()
    {
        m_.unlock();
    }

    OSMutex m_;
    OSSem s_;
    Notifiable *notifiable_;
};

#endif // _FREERTOS_DRIVERS_COMMON_BLOCKORWAKEUP_HXX_
