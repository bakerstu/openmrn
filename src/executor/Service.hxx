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
 * \file Service.hxx
 *
 * Class to control execution of state machines pulled out of a queue.
 *
 * @author Stuart W Baker
 * @date 20 December 2013
 */

#ifndef _Service_hxx_
#define _Service_hxx_

#include "executor/Executor.hxx"
#include "executor/Message.hxx"

class ControlFlow;

/** Collection of related state machines that pend on incoming messages.
 */
class Service
{
public:
    /** Constructor.
     * @param e Executor to run this service from.
     */
    Service(Executor *e)
        : executor(e)
    {
    }
    
    /** Destructor. */
    ~Service()
    {
    }

    /** Send a message anonymously, or to assert something other than a
     * service pointer into the Message::from field 
     * @param to destination of this message
     * @param buffer message to send
     */
    static void static_send(Service *to, Message *msg)
    {
        msg->to(to);
        to->executor->send(msg);        
    }

    /** Send a message to another service.
     * @param to destination of this message
     * @param msg message to send
     * @param id unique 31-bit identifier for the message
     */
    void send(Service *to, Message *msg, uint32_t id)
    {
        HASSERT((id & Message::IN_PROCESS_MSK) == 0);
        msg->id(id);
        send(to, msg);
    }

    /** Send a message to another service.
     * @param to destination of this message
     * @param msg message to send
     */
    void send(Service *to, Message *msg)
    {
        HASSERT(msg->id() != 0);
        msg->to(to);
        msg->from(this);
        to->executor->send(msg);
    }

    /** Send a message to self.
     * @param msg message to send
     */
    void send(Message *msg)
    {
        send(this, msg);
    }

    /** Send a message to self.
     * @param msg message to send
     * @param id unique 31-bit identifier for the message
     */
    void send(Message *msg, uint32_t id)
    {
        send(this, msg, id);
    }

#if defined (__FreeRTOS__)
    /** Send a message to another service from ISR.
     * @param to destination of this message
     * @param msg message to send
     * @param woken is the task woken up
     */
    void send_from_isr(Service *to, Message *msg, int *woken)
    {
        HASSERT(msg->id() != 0);
        msg->to(to);
        msg->from(this);
        to->executor->send_from_isr(msg, woken);
    }

    /** Send a message to self from ISR.
     * @param msg message to send
     * @param woken is the task woken up
     */
    void send_from_isr(Message *msg, int *woken)
    {
        send_from_isr(this, msg, woken);
    }
#endif
protected:
    /** Translate an incoming Message ID into a ControlFlow instance.
     */
    virtual ControlFlow *lookup(uint32_t id) = 0;

private:
    /** Process an incoming message.
     * @param buffer message to process
     */
    void process(Message *msg);

    /** access to the Service::process method */
    friend class Executor;

    /** Executor to use */
    Executor *executor;
};

#endif /* _Service_hxx_ */
