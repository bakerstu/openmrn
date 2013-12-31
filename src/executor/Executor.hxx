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
class Executor : public OSThread
{
public:
    /** Constructor.
     * @param name name of executor
     * @param priority thread priority
     * @param stack_size thread stack size
     */
    Executor(const char *name, int priority, size_t stack_size);

    /** Default destructor.
     */
    ~Executor()
    {
    }

    /** Callback type for timers */
    typedef void (*TimerCallback)();
    
    /** Catch timer callback.
     * @param data1 pointer to an Executor instance
     * @param data2 pointer to an application callback
     * @return Always returns OS_TIMER_NONE
     */
    static long long timer_callback(void *data1, void *data2);

    /** Register a timer callback.  The callback will be called from within
     * the same thread context as the Executor thread.
     * @param callback method to call upon expiration
     * @return pointer to created timer
     */
    OSTimer *timer_create(TimerCallback callback)
    {
        return new OSTimer(timer_callback, (void*)this, (void*)callback);
    }
    
    /** Lookup an executor by its name.
     * @param name name of executor to lookup
     * @param wait wait forever for an executor to show up
     * @return pointer to executor upon success, else NULL if not found
     */
    static Executor *by_name(const char *name, bool wait);

protected:    
    /** Send a message to this Executor's queue.
     * @param buffer buffer instance to insert into the input queue
     */
    void send(Message *msg)
    {
        queue.insert(msg);
    }
#if defined (__FreeRTOS__)
    /** Send a message to this Executor's queue.
     * @param buffer buffer instance to insert into the input queue
     * @param woken is the task woken up
     */
    void send_from_isr(Message *msg, int *woken)
    {
        int result = isrMQ.send_from_isr(&msg, woken);
        /* crash if the queue is full */
        HASSERT(result == OS_MQ_NONE);
    }
#endif
private:
    typedef void *Timer;

    /** Thread entry point.
     * @return Should never return
     */
    void *entry();

    /** queue to wait for incoming messages on */
    QueueProtectedWait<Message> queue;
    
    /** name of this Executor */
    const char *name;
    
    /** next executor in the lookup list */
    Executor *next;
    
    /** First next timer in the list */
    Timer active;
    
    /** executor list for lookup purposes */
    static Executor *list;

#if defined (__FreeRTOS__)
    /** message queue for connection between ISR and thread context */
    static OSMQ isrMQ;
    
    /** thread for catching incoming messages from thread context */
    static OSThread isrThread;
    
    /** Entry point for isrThread
     * @param arg unused
     * @return should never return
     */
    static void *isr_thread_entry(void *arg);
#endif

    /** Default Constructor.
     */
    Executor();
    
    /** provide access to Executor::send method. */
    friend class Service;

    DISALLOW_COPY_AND_ASSIGN(Executor);
};

#endif /* _Executor_hxx_ */
