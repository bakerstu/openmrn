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

#include <climits>

#include "executor/StateFlow.hxx"

/** Process an incoming message.
 * @param msg message to process
 * @param priority priority of message
 */
void StateFlowBase::run()
{
    HASSERT(state_);
    do
    {
        Action action = (this->*state_)();
        if (!action.next_state())
        {
            // This action is == wait(). This means we are blocked or asked to
            // yield.
            return;
        }
        state_ = action.next_state();
    } while (1);
}

StateFlowBase::Action StateFlowWithQueue::wait_for_message()
{
    LockHolder h(this);
    unsigned priority;
    currentMessage_ = static_cast<Message *>(queue_next(&priority));
    if (currentMessage_)
    {
        isWaiting_ = 0;
        currentPriority_ = priority;
        // Yielding here will ensure that we are processing the next message on
        // the current executor according to its priority.
        return yield_and_call(STATE(entry));
    }
    else
    {
        isWaiting_ = 1;
        currentPriority_ = MAX_PRIORITY;
        return wait();
    }
}

void StateFlowBase::notify()
{
    service()->executor()->add(this);
}

void StateFlowWithQueue::notify()
{
    service()->executor()->add(this, currentPriority_);
}

/** Terminates the current StateFlow activity.  This is a sink state, and there
 * has to be an external call to do anything useful after this state has been
 * reached.
 * @returns delay.
 */
StateFlowBase::Action StateFlowBase::terminated()
{
    return wait();
}
