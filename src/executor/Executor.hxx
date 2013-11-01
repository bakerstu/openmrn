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

#include "utils/BufferQueue.hxx"

enum
{
    /** Start the Executor */
    ID_EXECUTOR_START = 0,
    
    /** This is a timer */
    ID_TIMER,
};

/** This class implements an execution of tasks pulled off an input queue.
 */
class Executor : public OSThread
{
public:
    /** Callback type for timers */
    typedef void (*TimerCallback)();
    
    /** connection handle to an executor */
    typedef void *Connection;

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

    /** Send a message to this Executor's queue.
     * @param c Connection to an executor
     * @param buffer buffer instance to insert into the input queue
     */
    static void send(Connection *c, Buffer* buffer)
    {
        Executor *executor = (Executor*)c;
        executor->send(buffer);
    }
    
    /** Get a connection handle to the given Executor name.
     * @param name name of Executor to connect to
     * @param wait wait forever for a connection to come online
     * @return connection handle upon success, NULL upon failure
     */
    static Connection connection(const char *name, bool wait = false);
    
    /** Get a connection handle to the Executor caller instance.
     * @return connection handle
     */
    Connection connection()
    {
        return (Connection*)this;
    }
    
protected:
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

    /** Initialization hook that occurs at start outside the Executor thread context.
     */
    virtual void initialize()
    {
    }
    
    /** Initialization hook that occurs under the Executor thread context.
     */
    virtual void thread_initialize()
    {
    }
    
    /** Process an incoming message.
     * @param id message ID
     * @param data message payload
     * @param size message size
     */
    virtual void process(uint32_t id, void *data, size_t size) = 0;

    /** Send a message to this Executor's queue.
     * @param buffer buffer instance to insert into the input queue
     */
    void send(Buffer *buffer)
    {
        initialize();
        queue.insert(buffer);
    }

    /** Start the Executor processing.
     */
    void start()
    {
        /** wakeup the buffer Queue */
        Buffer *buffer = buffer_alloc(0);
        buffer->id(0);
        queue.insert(buffer);
    }

private:
    /** Structure for timer messages.
     */
    struct IdTimer
    {
        TimerCallback callback; /**< callback for executing timer */
    };

    /** Thread entry point.
     * @return Should never return
     */
    void *entry();

    /** queue to wait for incoming messages on */
    BufferQueueWait queue;
    
    /** name of this Executor */
    const char *name;
    
    /** next executor in the lookup list */
    Executor *next;
    
    /** executor list for lookup purposes */
    static Executor *list;
        
    /** Default Constructor.
     */
    Executor();

    DISALLOW_COPY_AND_ASSIGN(Executor);
};

#endif /* _Executor_hxx_ */
