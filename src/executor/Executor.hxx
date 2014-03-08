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
 * \file Executor.hxx
 *
 * Class to control execution of tasks that get pulled of an input queue.  This
 * is based off of work started by Balazs on 5 August 2013.
 *
 * @author Stuart W Baker and Balazs Racz
 * @date 26 October 2013
 */

#ifndef _Executor_hxx_
#define _Executor_hxx_

#include "executor/Message.hxx"

/** This class implements an execution of tasks pulled off an input queue.
 */
class ExecutorBase : public OSThread
{
public:
    /** Constructor.
     * @param name name of executor
     * @param priority thread priority
     * @param stack_size thread stack size
     */
    ExecutorBase(const char *name, int priority, size_t stack_size);

    /** Destructor.
     */
    ~ExecutorBase()
    {
    }

    /** Lookup an executor by its name.
     * @param name name of executor to lookup
     * @param wait wait forever for an executor to show up
     * @return pointer to executor upon success, else NULL if not found
     */
    static ExecutorBase *by_name(const char *name, bool wait);

protected:    
    /** Send a message to this Executor's queue.
     * @param msg Message instance to insert into the input queue
     * @param priority priority of message
     */
    virtual void send(Message *msg, unsigned priority = UINT_MAX) = 0;

private:
    /** Wait for an item from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @param priority pass back the priority of the queue pulled from
     * @return item retrieved from queue, else NULL with errno set:
     *         ETIMEDOUT - timeout occured, EINTR - woken up asynchronously
     */
    virtual Message *timedwait(long long timeout, unsigned *priority) = 0;

    /** Wait for an item from the front of the queue.
     * @param priority pass back the priority of the queue pulled from
     * @return item retrieved from queue, else NULL with errno set:
     *         EINTR - woken up asynchronously
     */
    virtual Message *wait(unsigned *priority) = 0;

    /** Typedef for Timer.  This is a cast of @ref ServiceTimer* */
    typedef void *Timer;

    /** Thread entry point.
     * @return Should never return
     */
    void *entry();
    
    /** name of this Executor */
    const char *name;
    
    /** next executor in the lookup list */
    ExecutorBase *next;
    
    /** First next timer in the list */
    Timer active;
    
    /** executor list for lookup purposes */
    static ExecutorBase *list;

    /** Default Constructor.
     */
    ExecutorBase();
    
    /** provide access to Executor::send method. */
    friend class Service;

    DISALLOW_COPY_AND_ASSIGN(ExecutorBase);
};

template <unsigned priorities> class Executor : public ExecutorBase,
                                                public QueueListProtectedWait<Message, priorities>
{
public:
    /** Constructor.
     * @param name name of executor
     * @param priority thread priority
     * @param stack_size thread stack size
     */
    Executor(const char *name, int priority, size_t stack_size)
        : ExecutorBase(name, priority, stack_size),
          QueueListProtectedWait<Message, priorities>()
    {
    }
    
    /** Destructor.
     */
    ~Executor()
    {
    }

protected:
    /** Send a message to this Executor's queue.
     * @param msg Message instance to insert into the input queue
     * @param priority priority of message
     */
    void send(Message *msg, unsigned priority = UINT_MAX)
    {
        QueueListProtectedWait<Message, priorities>::insert(msg, priority >= priorities ? priorities - 1 : priority);
    }

private:
    /** Wait for an item from the front of the queue.
     * @param timeout time to wait in nanoseconds
     * @param priority pass back the priority of the queue pulled from
     * @return item retrieved from queue, else NULL with errno set:
     *         ETIMEDOUT - timeout occured, EINTR - woken up asynchronously
     */
    Message *timedwait(long long timeout, unsigned *priority)
    {
        typename QueueListProtectedWait<Message, priorities>::Result result =
            QueueListProtectedWait<Message, priorities>::timedwait(timeout);
        *priority = result.index;
        return result.item;
    }

    /** Wait for an item from the front of the queue.
     * @param priority pass back the priority of the queue pulled from
     * @return item retrieved from queue, else NULL with errno set:
     *         EINTR - woken up asynchronously
     */
    Message *wait(unsigned *priority)
    {
        typename QueueListProtectedWait<Message, priorities>::Result result =
            QueueListProtectedWait<Message, priorities>::wait();
        *priority = result.index;
        return result.item;
    }

    /** Default Constructor.
     */
    Executor();
    
    DISALLOW_COPY_AND_ASSIGN(Executor);
};

#endif /* _Executor_hxx_ */
