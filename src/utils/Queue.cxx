/** \copyright
 * Copyright (c) 2013, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file Queue.cxx
 * This file provides an implementation buffers and pools for buffers.
 *
 * @author Stuart W. Baker
 * @date 3 August 2013
 */

#include "utils/Queue.hxx"

/** Add an item to the back of the queue.
 * @param item to add to queue
 * @param index unused parameter
 */
void Q::insert(QMember *item, unsigned index)
{
    HASSERT(item->next == nullptr);
    HASSERT(item != tail);
    AtomicHolder h(this);
    if (head == NULL)
    {
        head = tail = item;
    }
    else
    {
        tail->next = item;
        tail = item;
    }
    item->next = NULL;
    ++count;
}

/** Get an item from the front of the queue.
 * @return @ref Result structure with item retrieved from queue, NULL if
 *         no item available
 */
Q::Result Q::next()
{
    AtomicHolder h(this);
    if (head == NULL)
    {
        return Result();
    }
    --count;
    QMember *qm = head;
    if (head == tail)
    {
        tail = NULL;
    }
    head = (qm->next);
    qm->next = NULL;

    return Result(qm, 0);
}

/** Add an item to the back of the queue.
 * @param item to add to queue
 * @param index unused parameter
 */
void QAsync::insert(QMember *item, unsigned index)
{
    Executable *executable = NULL;
    {
        AtomicHolder h(this);
        if (waiting)
        {
            if (Q::empty())
            {
                waiting = false;
                Q::insert(item);
            }
            else
            {
                executable = static_cast<Executable *>(Q::next().item);
            }
        }
        else
        {
            Q::insert(item);
        }
    }
    if (executable)
    {
        executable->alloc_result(item);
    }
}

/** Get an item from the front of the queue.
 * @param flow Executable that will wait on the item
 * @return item retrieved from queue, NULL if no item available
 */
void QAsync::next_async(Executable *flow)
{
    QMember *qm = NULL;
    {
        AtomicHolder h(this);
        if (waiting)
        {
            Q::insert(flow);
        }
        else
        {
            qm = Q::next(0);
            if (qm == NULL)
            {
                Q::insert(flow);
                waiting = true;
            }
        }
    }
    if (qm)
    {
        flow->alloc_result(qm);
    }
}
