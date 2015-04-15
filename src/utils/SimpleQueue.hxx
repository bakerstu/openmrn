/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file SimpleQueue.hxx
 * A simple fast single-linked queue class with non-virtual methods.
 *
 * @author Balazs Racz
 * @date 6 Apr 2015
 */

#ifndef _UTILS_SIMPLEQUEUE_HXX_
#define _UTILS_SIMPLEQUEUE_HXX_

#include "utils/QMember.hxx"

class SimpleQueue {
public:
    SimpleQueue() : head_(nullptr) {}

    bool empty() {
        return !head_;
    }

    void push_front(QMember* member) {
        HASSERT(!member->next);
        member->next = head_;
        head_ = member;
    }

    QMember* pop_front() {
        HASSERT(head_);
        QMember* f = head_;
        head_ = f->next;
        f->next = nullptr;
        return f;
    }

    QMember* front() const {
        return head_;
    }

    class end_iterator {};

    class iterator {
    public:
        iterator(QMember** link): link_(link) {
            HASSERT(link_);
        }

        bool operator==(const iterator& o) const {
            return *link_ == *o.link_;
        }

        bool operator==(const end_iterator& o) const {
            return *link_ == nullptr;
        }

        bool operator!=(const iterator& o) const {
            return *link_ != *o.link_;
        }

        bool operator!=(const end_iterator& o) const {
            return *link_ != nullptr;
        }

        iterator& operator++() {
            HASSERT(*link_);
            link_ = &(*link_)->next;
            return *this;
        }

        QMember* operator->() {
            return *link_;
        }

        QMember& operator*() {
            return **link_;
        }

    private:
        friend class SimpleQueue;

        QMember** link_;
    };

    template<class T> class typed_iterator : public iterator {
    public:
        typed_iterator(QMember** link): iterator(link) {}
        T* operator->() {
            return static_cast<T*>(*link_);
        }
        T& operator*() {
            return static_cast<T&>(**link_);
        }
    };

    iterator begin() {
        return iterator(&head_);
    }

    /** Returns a sentinel to compare against for determining when an iteration
     * is done. This sentinel cannot be used to insert entries at the end of
     * the queue. */
    end_iterator end() {
        return end_iterator{};
    }

    /** Inserts the element entry before the position.  The iterator will point
     * to the new member.
     */
    void insert(const iterator& position, QMember* entry) {
        HASSERT(!entry->next);
        entry->next = *position.link_;
        *position.link_ = entry;
    }

    /** Removes the entry pointed to by the iterator. */
    void erase(const iterator& position) {
        QMember* m = *position.link_;
        HASSERT(m);
        *position.link_ = m->next;
        m->next = nullptr;
    }

protected:
    /** Used as a guard for comparing against for the end of the queue. */
    static QMember* const PTR_END;

    QMember* head_;
};


template<class T>
class TypedQueue : public SimpleQueue {
public:
    void push_front(T* entry) {
        SimpleQueue::push_front(entry);
    }

    T* pop_front() {
        return static_cast<T*>(SimpleQueue::pop_front());
    }

    T* front() const {
        return static_cast<T*>(SimpleQueue::front());
    }

    typedef typed_iterator<T> iterator;

    iterator begin() {
        return iterator(&head_);
    }
};

#endif // _UTILS_SIMPLEQUEUE_HXX_
