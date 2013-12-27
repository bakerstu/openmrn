/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file Service.cxx
 *
 * Class to control execution of state machines pulled out of a queue.
 *
 * @author Stuart W Baker
 * @date 20 December 2013
 */

#include "executor/Service.hxx"

/** main message pool instance */
//MessagePool *mainMessagePool = new MessagePool();

/** Process an incoming message.
 * @param buffer message to process
 */
void Service::process(Message *msg)
{
    ControlFlow *cf = lookup(msg->id() & Message::ID_VALUE_MSK);
    
    HASSERT(cf);
    
    cf->process(msg);
}


/** Process an incoming message.
 * @param msg message to process
 */
void ControlFlow::process(Message *msg)
{
    if (msg->id() & Message::IN_PROCESS_MSK)
    {
        /* continue existing in process control flow */
        HASSERT(state != STATE(terminated));
    }
    else
    {
        if (state == STATE(terminated))
        {
            /* start the control flow */
            state = STATE(entry);
        }
        else
        {
            /* we have a new incoming flow that we must queue until the
             * previous flow is done precessing.
             */
            insert(msg);
            return;
        }
    }

    for ( ; /* forever */ ; )
    {
        ControlFlowAction result = (this->*state)(msg);
        state = result.next_state();

        if (state == STATE(terminated))
        {
            /* free up the current message it the pool from whence it came and
             * check to see if we have anything in our pending queue.
             */
            msg->free();
            msg = next();

            if (msg == NULL)
            {
                /* nothing left to process */
                return;
            }
            /* pulled something out of the queue, get it started */
            state = STATE(entry);
        }
        else
        {
            if (msg->id() & Message::IN_PROCESS_MSK)
            {
                /* Wait for next state transistion */
                return;
            }
        }
    }
}

/** Imediately queue up the next callback for this flow through the executor.
 * Similar to @ref call_immediately, except we place this flow on the back
 * of the Executor queue.
 * @param c Callback "state" to move to
 * @param msg Message instance we are acting upon
 */
ControlFlow::ControlFlowAction ControlFlow::yeild_and_call(Callback c, Message *msg)
{
    /* marks this as an in-process message and queue us up for the next go
     * around.
     */
    msg->id(msg->id() | Message::IN_PROCESS_MSK);
    service->send(msg);
    return ControlFlowAction(c);
}

/** Terminate current ControlFlow activity.  This method only exists for the
 * purpose of providing a unique address pointer.
 * @param msg unused
 */
ControlFlow::ControlFlowAction ControlFlow::terminated(Message *msg)
{
    HASSERT(0);
    return terminated(msg);
}


