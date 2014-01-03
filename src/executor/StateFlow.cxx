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
 * \file StateFlow.cxx
 *
 * Defines a type of state machine flow used within class Service.
 *
 * @author Stuart W Baker
 * @date 1 January 2013
 */

#include "executor/StateFlow.hxx"

/** Process an incoming message.
 * @param msg message to process
 * @param priority priority of message
 */
void StateFlowBase::process(Message *msg, unsigned priority)
{
    if (msg->id() & Message::IN_PROCESS_MSK)
    {
       HASSERT(state != STATE(terminated));

       /* Clear the IN_PROCESS_MASK and continue in process state flow */
       msg->id(msg->id() & ~Message::IN_PROCESS_MSK);
    }
    else
    {
        if (state == STATE(terminated))
        {
            /* start the state flow */
            state = STATE(entry);
        }
        else
        {
            /* we have a new incoming flow that we must queue until the
             * previous flow is done precessing.
             */
            insert(msg, priority);
            return;
        }
    }

    for ( ; /* forever */ ; )
    {
        Action result = (this->*state)(msg);
        state = result.next_state();

        if (state == STATE(terminated))
        {
            /* check to see if we have anything in our pending queue */
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
 * @return function pointer to passed in callback
 */
StateFlowBase::Action StateFlowBase::yeild_and_call(Callback c, Message *msg)
{
    /* marks this as an in-process message and queue us up for the next go
     * around.
     */
    msg->id(msg->id() | Message::IN_PROCESS_MSK);
    service->send(msg, msg->id());
    return Action(c);
}

/** Terminate current StateFlow activity.  This method only exists for the
 * purpose of providing a unique address pointer.
 * @param msg unused
 * @return should never return
 */
StateFlowBase::Action StateFlowBase::terminated(Message *msg)
{
    HASSERT(0);
    return terminated(msg);
}

