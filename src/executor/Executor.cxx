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

#include "executor/Service.hxx"

Executor *Executor::list = NULL;

#if defined (__FreeRTOS__)
OSMQ Executor::isrMQ(16, sizeof(Message*));

/* we need to run at the highest priority in the system to minimize the risk
 * of getting backlogged.
 */
OSThread Executor::isrThread("Executor ISR", configMAX_PRIORITIES - 1, 512,
                             Executor::isr_thread_entry, NULL);

void *Executor::isr_thread_entry(void *arg)
{
    for ( ; /* forever */ ; )
    {
        Message *msg;
        isrMQ.receive(&msg);
        static_cast<Service*>(msg->to())->send(msg);
    }

    return NULL;
}
#endif

/** Constructor.
 * @param name name of executor
 * @param priority thread priority
 * @param stack_size thread stack size
 */
Executor::Executor(const char *name, int priority, size_t stack_size)
    : OSThread(name, priority, stack_size),
      queue(),
      name(name),
      next(NULL)
{
    /** @todo (Stuart Baker) we need a locking mechanism here to protect
     *  the list.
     */
    if (list == NULL)
    {
        list = this;
        next = NULL;
    }
    else
    {
        next = list;
        list = this;
    }
}

/** Lookup an executor by its name.
 * @param name name of executor to lookup
 * @return pointer to executor upon success, else NULL if not found
 */
Executor *Executor::by_name(const char *name, bool wait)
{
    /** @todo (Stuart Baker) we need a locking mechanism here to protect
     *  the list.
     */
    for ( ; /* forever */ ; )
    {
        Executor *current = list;
        while (current)
        {
            if (!strcmp(name, current->name))
            {
                return current;
            }
            current = current->next;
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

/** Catch timer callback.
 * @param data1 pointer to an Executor instance
 * @param data2 pointer to an application callback
 * @return Always returns OS_TIMER_NONE
 */
long long Executor::timer_callback(void *data1, void *data2)
{
#if 0
    Executor *executor = (Executor*)data1;
    TimerCallback callback = (TimerCallback)data2;

    Buffer *buffer = buffer_alloc(sizeof(IdTimer));
    buffer->id(ID_TIMER);
    IdTimer *message = (IdTimer*)buffer->start();
    message->callback = callback;
    executor->queue.insert(buffer);
#endif
    return OS_TIMER_NONE;
}

/** Thread entry point.
 * @return Should never return
 */
void *Executor::entry()
{
    Message *msg;
    
    /* wait for messages to process */
    for ( ; /* forever */ ; )
    {
        msg = queue.wait();
        Service *service = (Service*)msg->to();
        HASSERT(service);
        service->process(msg);
    }

    return NULL;
}

