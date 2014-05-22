/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file EventManager.cxx
 *
 * Implementation advanced event managers.
 *
 * @author Balazs Racz
 * @date 3 May 2014
 */

#include "nmranet/EventManager.hxx"

namespace nmranet
{

void TreeEventHandlers::register_handlerr(NMRAnetEventHandler *handler,
                                          EventId event, unsigned mask)
{
    AtomicHolder h(this);
    handlers_[mask].insert(make_pair(event, handler));
}

void TreeEventHandlers::unregister_handlerr(NMRAnetEventHandler *handler,
                                            EventId event, unsigned mask)
{
    AtomicHolder h(this);
    auto r = handlers_[mask].equal_range(event);
    for (auto it = r.first; it != r.second; ++it)
    {
        if (it->second == handler)
        {
            handlers_[mask].erase(it);
            return;
        }
    }
    DIE("tried to unregister a handler that was not registered");
}

class TreeEventHandlers::Iterator : public EventIterator
{
public:
    Iterator(TreeEventHandlers *parent)
        : parent_(parent)
    {
        AtomicHolder h(parent_);
        parent->handlers_[0];
        clear_iteration();
    }

    NMRAnetEventHandler *next_entry() OVERRIDE
    {
        AtomicHolder h(parent_);
        while (maskIterator_ != parent_->handlers_.end())
        {
            if (it_ == end_)
            {
                maskIterator_++;
                if (maskIterator_ != parent_->handlers_.end())
                {
                    setup_current_mask();
                }
                continue;
            }
            else
            {
                NMRAnetEventHandler *h = it_->second;
                it_++;
                return h;
            }
        }
        return nullptr;
    }

    void clear_iteration() OVERRIDE
    {
        AtomicHolder h(parent_);
        maskIterator_ = parent_->handlers_.end();
    }
    void init_iteration(EventReport *r) OVERRIDE
    {
        AtomicHolder h(parent_);
        currentReport_ = r;
        maskIterator_ = parent_->handlers_.begin();
        setup_current_mask();
    }

private:
    void setup_current_mask()
    {
        if (maskIterator_->first == 64)
        {
            // 64 bits -> all events go to everyone.
            it_ = maskIterator_->second.begin();
            end_ = maskIterator_->second.end();
            return;
        }
        unsigned mask_log = maskIterator_->first;
        uint64_t current_mask = (1ULL << mask_log) - 1;
        uint64_t eventid_key = currentReport_->event & (~current_mask);
        it_ = maskIterator_->second.lower_bound(eventid_key);
        eventid_key = currentReport_->event + currentReport_->mask;
        end_ = maskIterator_->second.upper_bound(eventid_key);
    }
    TreeEventHandlers *parent_;
    EventReport *currentReport_;
    MaskLookupMap::iterator maskIterator_;
    OneMaskMap::iterator it_;
    OneMaskMap::iterator end_;
};

EventIterator *TreeEventHandlers::create_iterator()
{
    return new Iterator(this);
}

TreeEventHandlers::TreeEventHandlers()
{
}

} // namespace nmranet
