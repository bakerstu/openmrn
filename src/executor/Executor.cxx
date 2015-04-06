/** \copyright
 * Copyright (c) 2013, Stuart W Baker and Balazs Racz
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
 * \file Executor.cxx
 *
 * Class to control execution of tasks that get pulled of an input queue.  This
 * is based off of work started by Balazs on 5 August 2013.
 *
 * @author Stuart W Baker and Balazs Racz
 * @date 26 October 2013
 */

#include "executor/Executor.hxx"

#include <unistd.h>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "executor/Service.hxx"
#include "nmranet_config.h"

ExecutorBase *ExecutorBase::list = NULL;

/** Constructor.
 * @param name name of executor
 * @param priority thread priority
 * @param stack_size thread stack size
 */
ExecutorBase::ExecutorBase()
    : name_(NULL) /** @todo (Stuart Baker) is "name" still in use? */
    , next_(NULL)
    , activeTimers_(this)
    , done_(0)
    , started_(0)
    , selectPrescaler_(0)
{
    /** @todo (Stuart Baker) we need a locking mechanism here to protect
     *  the list.
     */
    if (list == NULL)
    {
        list = this;
        next_ = NULL;
    }
    else
    {
        next_ = list;
        list = this;
    }
    FD_ZERO(&selectRead_);
    FD_ZERO(&selectWrite_);
    FD_ZERO(&selectExcept_);
    selectNFds_ = 0;
}

/** Lookup an executor by its name.
 * @param name name of executor to lookup
 * @return pointer to executor upon success, else NULL if not found
 */
ExecutorBase *ExecutorBase::by_name(const char *name, bool wait)
{
    /** @todo (Stuart Baker) we need a locking mechanism here to protect
     *  the list.
     */
    for (; /* forever */;)
    {
        ExecutorBase *current = list;
        while (current)
        {
            if (!strcmp(name, current->name_))
            {
                return current;
            }
            current = current->next_;
        }
        if (wait)
        {
            sleep(1);
        }
        else
        {
            return NULL;
        }
    }
}

bool ExecutorBase::loop_once()
{
    unsigned priority;
    activeTimers_.get_next_timeout();
    Executable* msg = next(&priority);
    if (!msg)
    {
        return false;
    }
    if (msg == this)
    {
        // exit closure
        done_ = 1;
        return false;
    }
    current_ = msg;
    msg->run();
    current_ = nullptr;
    return true;
}

#ifdef __EMSCRIPTEN__

void executor_loop_some(void* arg)
{
  ExecutorBase* b = static_cast<ExecutorBase*>(arg);
  while (b->loop_once());
}

void *ExecutorBase::entry()
{
    started_ = 1;
    ExecutorBase* b = this;
    emscripten_set_main_loop_arg(&executor_loop_some, b, 100, true);
    return nullptr;
}

#else
/** Thread entry point.
 * @return Should never return
 */
void *ExecutorBase::entry()
{
    started_ = 1;
    /* wait for messages to process */
    for (; /* forever */;)
    {
        Executable *msg = nullptr;
        unsigned priority = UINT_MAX;
        long long wait_length = activeTimers_.get_next_timeout();
        if (!selectPrescaler_ || empty()) {
            wait_with_select(wait_length);
            selectPrescaler_ = config_executor_select_prescaler();
            msg = next(&priority);
        } else {
            --selectPrescaler_;
            msg = next(&priority);
        }
        if (msg == this)
        {
            // exit closure
            done_ = 1;
            return NULL;
        }
        if (msg != NULL)
        {
            current_ = msg;
            msg->run();
            current_ = nullptr;
        }
    }

    return NULL;
}

void ExecutorBase::select(Selectable *job)
{
    fd_set *s = get_select_set(job->type());
    int fd = job->fd_;
    if (FD_ISSET(fd, s))
    {
        LOG(FATAL,
            "Multiple Selectables are waiting for the same fd %d type %u", fd,
            job->selectType_);
    }
    FD_SET(fd, s);
    if (fd >= selectNFds_)
    {
        selectNFds_ = fd + 1;
    }
    HASSERT(!job->next);
    // Inserts the job into the select queue.
    selectables_.push_front(job);
}

void ExecutorBase::unselect(Selectable *job)
{
    fd_set *s = get_select_set(job->type());
    int fd = job->fd_;
    if (!FD_ISSET(fd, s))
    {
        LOG(FATAL, "Tried to remove a non-active selectable: fd %d type %u", fd,
            job->selectType_);
    }
    FD_CLR(fd, s);
    auto it = selectables_.begin();
    unsigned max_fd = 0;
    while (it != selectables_.end())
    {
        if (&*it == job)
        {
            selectables_.erase(it);
            continue;
        }
        max_fd = std::max(max_fd, it->fd_ + 1U);
        ++it;
    }
    selectNFds_ = max_fd;
}

void ExecutorBase::wait_with_select(long long wait_length)
{
    fd_set fd_r(selectRead_);
    fd_set fd_w(selectWrite_);
    fd_set fd_x(selectExcept_);
    if (!empty()) {
        wait_length = 0;
    }
    long long max_sleep = MSEC_TO_NSEC(config_executor_max_sleep_msec());
    if (wait_length > max_sleep)
    {
        wait_length = max_sleep;
    }
    struct timeval select_time;
    select_time.tv_sec = wait_length / 1000000000;
    select_time.tv_usec = (wait_length % 1000000000) / 1000;
    int ret = ::select(selectNFds_, &fd_r, &fd_w, &fd_x, &select_time);
    if (ret <= 0) {
        return; // nothing to do
    }
    unsigned max_fd = 0;
    for (auto it = selectables_.begin(); it != selectables_.end();) {
        fd_set* s = nullptr;
        switch(it->type()) {
        case Selectable::READ: s = &fd_r; break;
        case Selectable::WRITE: s = &fd_w; break;
        case Selectable::EXCEPT: s = &fd_x; break;
        }
        if (FD_ISSET(it->fd_, s)) {
            add(it->wakeup_, it->priority_);
            selectables_.erase(it);
            continue;
        }
        max_fd = std::max(max_fd, it->fd_ + 1U);
        ++it;
    }
    selectNFds_ = max_fd;
}

#endif

void ExecutorBase::shutdown()
{
    if (!started_) return;
    add(this);
    while (!done_)
    {
        usleep(100);
    }
}

ExecutorBase::~ExecutorBase()
{
    if (!done_)
    {
        shutdown();
    }
}
