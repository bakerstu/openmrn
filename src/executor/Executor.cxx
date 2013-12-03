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

#include <unistd.h>

#include "executor/Executor.hxx"

Executor* Executor::list = NULL;

/** Constructor.
 * @param name name of executor
 * @param priority thread priority
 * @param stack_size thread stack size
 */
Executor::Executor(const char* name, int priority, size_t stack_size)
    : OSThread(name, priority, stack_size), queue(), name(name), next(NULL)
{
    /** @todo (Stuart Baker) we need a locking mechanism here to protect
     *  the list.
     */
    if (list == NULL) {
        list = this;
        next = NULL;
    } else {
        next = list;
        list = this;
    }
}

/** Get a connection handle to the given Executor name
 * @param name name of Executor to connect to
 * @param wait wait forever for a connection to come online
 * @return connection handle upon success, NULL upon failure
 */
Executor::Connection Executor::connection(const char* name, bool wait)
{
    /** @todo (Stuart Baker) we need a locking mechanism here to protect
     *  the list.
     */
    for (; /* forever */;) {
        Executor* current = list;
        while (current) {
            if (!strcmp(name, current->name)) {
                return current;
            }
            current = current->next;
        }
        if (wait) {
            sleep(1);
        } else {
            return NULL;
        }
    }
}

/** Catch timer callback.
 * @param data1 pointer to an Executor instance
 * @param data2 pointer to an application callback
 * @return Always returns OS_TIMER_NONE
 */
long long Executor::timer_callback(void* data1, void* data2)
{
    Executor* executor = (Executor*)data1;
    TimerCallback callback = (TimerCallback)data2;

    Buffer* buffer = buffer_alloc(sizeof(IdTimer));
    buffer->id(ID_TIMER);
    IdTimer* message = (IdTimer*)buffer->start();
    message->callback = callback;
    executor->queue.insert(buffer);
    return OS_TIMER_NONE;
}

/** Thread entry point.
 * @return Should never return
 */
void* Executor::entry()
{
    Buffer* buffer;

    /* Wait for Executor to be started */
    for (; /* forever */;) {
        buffer = queue.wait();
        if (buffer->id() == ID_EXECUTOR_START) {
            break;
        }
        buffer->free();
    }

    thread_initialize();

    /* Executor has been started, wait for messages to process */
    for (; /* forever */;) {
        buffer->free();
        buffer = queue.wait();
        if (buffer->id() == ID_TIMER) {
            IdTimer* timer = (IdTimer*)buffer->start();
            (timer->callback)();
        } else {
            process(buffer->id(), buffer->start(), buffer->used());
        }
    }

    return NULL;
}
