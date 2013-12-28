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
 * \file Allocator.hxx
 *
 * Defines an allocation pool type that can be used for notification of
 * pending resources.
 *
 * @author Stuart W Baker
 * @date 26 December 2013
 */

#ifndef _Allocator_hxx_
#define _Allocator_hxx_

#include "executor/Message.hxx"
#include "executor/Service.hxx"

/** Allocation pool of resources that a ControlFlow can pend on.
 */
template <class T> class Allocator : public Q<Message>, public FixedPool<T>
{
public:
    /** Construct pool of objects.
     * @param items number of objects to seed the pool with
     */
    Allocator(size_t items)
        : Q<Message>(),
          FixedPool<T>(sizeof(T), items)
    {
    }
    
    /** Construct pool of objects.
     * @param size size of variable sized objects
     * @param items number of objects to seed the pool with
     */
    Allocator(size_t size, size_t items)
        : Q<Message>(),
          FixedPool<T>(size, items)
    {
    }
    
    ~Allocator()
    {
    }

    /** Allocate an item from the free pool.  If an item is available, return
     * the message with from containing a pointer to it.  If an item is not
     * available, queue up the Message to be sent when it does become available.
     * @param msg Message to send when item becomes available.
     *        from field of message will point to the item allocated
     * @return true if allocated, false if not available
     */
    bool allocate_immediate(Message *msg)
    {
        FixedPool<T>::mutex.lock();
        T *item = FixedPool<T>::alloc();
        if (item)
        {
            /* item allocated */
            T::init(item, 0);
            msg->from(item);
        }
        else
        {
            /* could not allocate an item on the spot */
            msg->id(msg->id() | Message::IN_PROCESS_MSK);
            insert(msg);
        }
        FixedPool<T>::mutex.unlock();
        return item;
    }

    /** Allocate an item from the free pool.
     * @param msg Message to send when item becomes available.
     *        from field of message will point to the item allocated
     */
    void allocate(Message *msg)
    {
        /* flag this flow as in process */
        msg->id(msg->id() | Message::IN_PROCESS_MSK);

        FixedPool<T>::mutex.lock();
        T *item = FixedPool<T>::alloc();
        if (item)
        {
            /* item allocated */
            T::init(item, 0);
            msg->from(item);
            Service::static_send(static_cast<Service*>(msg->to()), msg);
        }
        else
        {
            /* could not allocate an item on the spot */
            insert(msg);
        }
        FixedPool<T>::mutex.unlock();
    }
    
    /** Release the item to the free pool.
     * @param item item to release
     */
    void free(T *item)
    {
        FixedPool<T>::mutex.lock();
        Message *msg = next();
        if (msg)
        {
            /* someone is waiting for item, let's repurpose this one */
            T::init(item, 0);
            msg->from(item);
            Service::static_send(static_cast<Service*>(msg->to()), msg);
        }
        else
        {
            /* nobody is waiting on the item */
            FixedPool<T>::free(item);
        }
        FixedPool<T>::mutex.unlock();
    }

    /** Obtain the result of an allocation.
     */
    static T *allocation_result(Message *msg)
    {
        return static_cast<T*>(msg->from());
    }
    
private:
    /** Default Constructor */
    Allocator();

    DISALLOW_COPY_AND_ASSIGN(Allocator);
};

#endif /* _Allocator_hxx_ */
