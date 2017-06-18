/** @copyright
 * Copyright (c) 2017, Stuart W Baker
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
 * @file NonAuthoritativeEventProducer.cxx
 *
 * Defines implementations of Event Producers that are uathoratative, meaning
 * that they do not represent the actual state of an event, but can still
 * query that state (presumably from a consumer) and produce a new state.
 *
 * @author Stuart Baker
 * @date 17 June 2017
 */

#include "openlcb/NonAuthoritativeEventProducer.hxx"
#include "openlcb/EventService.hxx"

namespace openlcb
{

//
// BitRangeNonauthoritativeEventP::send_query_consumer()
//
void BitRangeNonAuthoritativeEventP::send_query_consumer(unsigned bit,
                                                        WriteHelper *writer,
                                                        BarrierNotifiable *done)
{
    HASSERT(bit < size_);
    uint64_t event = eventBaseOff_ == 0 ? eventBase_ + (bit * 2) :
                                          eventBaseOn_ + bit;
    writer->WriteAsync(node_, Defs::MTI_CONSUMER_IDENTIFY,
                       WriteHelper::global(),
                       eventid_to_buffer(event), done);
}

//
// BitRangeNonauthoritativeEventP::handle_event_report()
//
void BitRangeNonAuthoritativeEventP::handle_event_report(
                                                const EventRegistryEntry& entry,
                                                EventReport *event,
                                                BarrierNotifiable *done)
{
    done->notify();
    if (!stateCallback_)
    {
        // there is nobody to notify
        return;
    }

    if (eventBaseOff_ == 0)
    {
        if (event->event >= eventBase_ &&
            event->event < (eventBase_ + (size_ * 2)))
        {
            bool value = (event->event % 2) == (eventBase_ % 2);
            stateCallback_((event->event - eventBase_) / 2, value);
        }
    }
    else
    {
        if (event->event >= eventBaseOn_ &&
            event->event < (eventBaseOn_ + size_))
        {
            stateCallback_((event->event - eventBase_), true);
        }
        else if (event->event >= eventBaseOff_ &&
                 event->event < (eventBaseOff_ + size_))
        {
            stateCallback_((event->event - eventBase_), false);
        }
    }
}

//
// BitRangeNonauthoritativeEventP::handle_consumer_identified()
//
void BitRangeNonAuthoritativeEventP::handle_consumer_identified(
                                                const EventRegistryEntry& entry,
                                                EventReport *event,
                                                BarrierNotifiable *done)
{
    done->notify();
    if (!stateCallback_)
    {
        // there is nobody to notify
        return;
    }

    bool value;
    if (event->state == EventState::VALID)
    {
        value = true;
    }
    else if (event->state == EventState::INVALID)
    {
        value = false;
    }
    else
    {
        return; // nothing to learn from this message.
    }

    if (eventBaseOff_ == 0)
    {
        if (event->event >= eventBase_ &&
            event->event < (eventBase_ + (size_ * 2)))
        {
            if ((event->event % 2) == (eventBase_ % 2))
            {
                stateCallback_((event->event - eventBase_) / 2, value);
            }
            else
            {
                stateCallback_((event->event - eventBase_) / 2, !value);
            }
        }
    }
    else
    {
        if (event->event >= eventBaseOn_ &&
            event->event < (eventBaseOn_ + size_))
        {
            stateCallback_((event->event - eventBase_), value);
        }
        else if (event->event >= eventBaseOff_ &&
                 event->event < (eventBaseOff_ + size_))
        {
            stateCallback_((event->event - eventBase_), !value);
        }
    }
}

//
// BitRangeNonauthoritativeEventP::set()
//
void BitRangeNonAuthoritativeEventP::set(unsigned bit, bool new_value,
                                         WriteHelper *writer,
                                         BarrierNotifiable *done)
{
    HASSERT(bit < size_);

    uint64_t event;
    if (new_value)
    {
        event = eventBaseOff_ == 0 ? eventBase_ + (bit * 2) :
                                     eventBaseOn_ + bit;
    }
    else
    {
        event = eventBaseOff_ == 0 ? eventBase_ + (bit * 2) + 1 :
                                     eventBaseOff_ + bit;
    }

    writer->WriteAsync(node_, Defs::MTI_EVENT_REPORT, WriteHelper::global(),
                       eventid_to_buffer(event), done);
}

} // namespace openlcb
