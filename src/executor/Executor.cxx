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
    Executable *msg;

    /* wait for messages to process */
    for (; /* forever */;)
    {
        unsigned priority;
        long long wait_length = activeTimers_.get_next_timeout();
        msg = timedwait(wait_length, &priority);
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
