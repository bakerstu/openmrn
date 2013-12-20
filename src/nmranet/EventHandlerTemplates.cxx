/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file EventHandlerTemplates.cxx
 *
 * Implementations of common event handlers.
 *
 * @author Balazs Racz
 * @date 6 November 2013
 */

#include <unistd.h>

#include "utils/logging.h"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/GlobalEventHandler.hxx"

#ifdef __linux__
#define DESCRIBE_VAR
#endif

#ifdef DESCRIBE_VAR
#include <string>
namespace NMRAnet
{
extern const string& GetNameForOffset(int);

__attribute__ ((weak))
const string& GetNameForOffset(int) { static string empty; return empty; }
}
#endif

namespace NMRAnet
{

BitRangeEventPC::BitRangeEventPC(AsyncNode *node,
                                 uint64_t event_base, uint32_t* backing_store,
                                 unsigned size)
    : event_base_(event_base), node_(node), data_(backing_store), size_(size) {
  NMRAnetEventRegistry::instance()->RegisterHandler(this, 0, 0);
}

BitRangeEventPC::~BitRangeEventPC() {
  NMRAnetEventRegistry::instance()->UnregisterHandler(this, 0, 0);
}

void BitRangeEventPC::GetBitAndMask(unsigned bit, uint32_t** data,
                                    uint32_t* mask) const {
  *data = nullptr;
  if ((bit >= size_) || (bit < 0))
    return;
  *data = data_ + (bit >> 5);
  *mask = 1 << (bit & 31);
}

bool BitRangeEventPC::Get(unsigned bit) const {
  HASSERT(bit < size_);
  uint32_t* ofs;
  uint32_t mask;
  GetBitAndMask(bit, &ofs, &mask);
  if (!ofs)
    return false;
  return (*ofs) & mask;
}

void BitRangeEventPC::Set(unsigned bit, bool new_value, WriteHelper* writer,
                          Notifiable* done) {
  HASSERT(bit < size_);
#ifdef DESCRIBE_VAR
  fprintf(stderr, "BitRange: OUT bit %x (%s) to %d\n", bit,
          GetNameForOffset(bit).c_str(), new_value);
#else
  LOG(VERBOSE, "BitRange: set bit %x to %d", bit, new_value);
#endif
  uint32_t* ofs;
  uint32_t mask;
  GetBitAndMask(bit, &ofs, &mask);
  bool old_value = new_value;
  HASSERT(ofs);
  if (ofs)
    old_value = (*ofs) & mask;
  if (old_value != new_value) {
    if (new_value) {
      *ofs |= mask;
    } else {
      *ofs &= ~mask;
    }
    uint64_t event = event_base_ + bit * 2;
    if (!new_value)
      event++;
    writer->WriteAsync(node_, If::MTI_EVENT_REPORT, WriteHelper::global(),
                       EventIdToBuffer(event), done);
    if (!done) {
      // We wait for the sent-out event to come back. Otherwise there is a race
      // condition where the automata processing could have gone further, but
      // the "set" message will arrive.
      while (GlobalEventFlow::instance->EventProcessingPending()) {
        usleep(100);
      }
    }
  } else {
    if (done)
      done->Notify();
  }
}

void BitRangeEventPC::HandleEventReport(EventReport* event, Notifiable* done) {
  done->Notify();
  if (event->event < event_base_)
    return;
  uint64_t d = (event->event - event_base_);
  bool new_value = !(d & 1);
  d >>= 1;
  if (d >= size_)
    return;
  int bit = d;
#ifdef DESCRIBE_VAR
  fprintf(stderr, "BitRange: IN  bit %x (%s) to %d\n", bit,
          GetNameForOffset(bit).c_str(), new_value);
#else
  LOG(VERBOSE, "BitRange: evt bit %x to %d", bit, new_value);
#endif

  uint32_t* ofs = nullptr;
  uint32_t mask = 0;
  GetBitAndMask(bit, &ofs, &mask);
  if (new_value) {
    *ofs |= mask;
  } else {
    *ofs &= ~mask;
  }
}

void BitRangeEventPC::HandleIdentifyProducer(EventReport* event,
                                             Notifiable* done) {
  HandleIdentifyBase(If::MTI_PRODUCER_IDENTIFIED_VALID, event, done);
}

void BitRangeEventPC::HandleIdentifyConsumer(EventReport* event,
                                             Notifiable* done) {
  HandleIdentifyBase(If::MTI_CONSUMER_IDENTIFIED_VALID, event, done);
}
void BitRangeEventPC::HandleIdentifyBase(If::MTI mti_valid, EventReport* event,
                                         Notifiable* done) {
  if (event->event < event_base_)
    return done->Notify();
  uint64_t d = (event->event - event_base_);
  bool new_value = !(d & 1);
  d >>= 1;
  if (d >= size_)
    return done->Notify();
  uint32_t* ofs = nullptr;
  uint32_t mask = 0;
  GetBitAndMask(d, &ofs, &mask);
  If::MTI mti = mti_valid;
  bool old_value = *ofs & mask;
  if (old_value != new_value) {
    mti++; // mti INVALID
  }

  event_write_helper1.WriteAsync(node_, mti, WriteHelper::global(),
                                 EventIdToBuffer(event->event), done);
}

uint64_t EncodeRange(uint64_t begin, unsigned size) {
  // We assemble a valid event range identifier that covers our block.
  uint64_t end = begin + size * 2;
  uint64_t shift = 1;
  while (begin + shift <= end) {
    begin &= ~shift;
    shift <<= 1;
  }
  if (begin & shift) {
    // last real bit is 1 => range ends with zero.
    return begin;
  } else {
    // last real bit is zero. Set all lower bits to 1.
    begin |= shift - 1;
    return begin;
  }
}

void BitRangeEventPC::HandleIdentifyGlobal(EventReport* event,
                                           Notifiable* done) {
  uint64_t range = EncodeRange(event_base_, size_);
  event_barrier.Reset(done);
  event_write_helper1.WriteAsync(node_, If::MTI_PRODUCER_IDENTIFIED_RANGE,
                                 WriteHelper::global(), EventIdToBuffer(range),
                                 event_barrier.NewChild());
  event_write_helper2.WriteAsync(node_, If::MTI_CONSUMER_IDENTIFIED_RANGE,
                                 WriteHelper::global(), EventIdToBuffer(range),
                                 event_barrier.NewChild());
  event_barrier.MaybeDone();
}

BitEventHandler::BitEventHandler(BitEventInterface* bit)
    : bit_(bit) {
  NMRAnetEventRegistry::instance()->RegisterHandler(this, 0, 0);
}

BitEventHandler::~BitEventHandler() {
  NMRAnetEventRegistry::instance()->UnregisterHandler(this, 0, 0);
}

void BitEventHandler::SendProducerIdentified() {
  bool value = bit_->GetCurrentState();
  If::MTI mti = If::MTI_PRODUCER_IDENTIFIED_VALID;
  if (!value)
    mti++; // INVALID
  event_write_helper1.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(bit_->event_on()),
                                 event_barrier.NewChild());
  if (!value) {
    mti--; // VALID
  } else {
    mti++; // INVALID
  }
  event_write_helper2.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(bit_->event_off()),
                                 event_barrier.NewChild());
}

void BitEventHandler::SendConsumerIdentified() {
  bool value = bit_->GetCurrentState();
  If::MTI mti = If::MTI_CONSUMER_IDENTIFIED_VALID;
  if (!value)
    mti++; // INVALID
  event_write_helper3.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(bit_->event_on()),
                                 event_barrier.NewChild());
  if (!value) {
    mti--; // VALID
  } else {
    mti++; // INVALID
  }
  event_write_helper4.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(bit_->event_off()),
                                 event_barrier.NewChild());
}

void BitEventHandler::SendEventReport(WriteHelper* writer, Notifiable* done) {
  bool value = bit_->GetCurrentState();
  uint64_t event = value ? bit_->event_on() : bit_->event_off();
  writer->WriteAsync(bit_->node(), If::MTI_EVENT_REPORT, WriteHelper::global(),
                     EventIdToBuffer(event), done);
}

void BitEventHandler::HandlePCIdentify(If::MTI mti, EventReport* event,
                                       Notifiable* done) {
  if (event->src_node.id == bit_->node()->node_id()) {
    // We don't respond to queries from our own node. This is not nice, but we
    // want to avoid to answering our own Query command.
    done->Notify();
    return;
  }
  bool active;
  if (event->event == bit_->event_on()) {
    active = bit_->GetCurrentState();
  } else if (event->event == bit_->event_off()) {
    active = !bit_->GetCurrentState();
  } else {
    done->Notify();
    return;
  }
  if (!active) {
    ++mti; // mti_invalid.
  }
  event_write_helper1.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(event->event), done);
}

void BitEventConsumer::HandleProducerIdentified(EventReport* event, Notifiable* done) {
  done->Notify();
  bool value;
  if (event->state == VALID) {
    value = true;
  } else if (event->state == INVALID) {
    value = false;
  } else {
    return;  // nothing to learn from this message.
  }
  if (event->event == bit_->event_on()) {
    bit_->SetState(value);
  } else if (event->event == bit_->event_off()) {
    bit_->SetState(!value);
  } else {
    return;  // uninteresting event id.
  }
}

void BitEventConsumer::SendQuery(WriteHelper* writer, Notifiable* done) {
  writer->WriteAsync(bit_->node(), If::MTI_PRODUCER_IDENTIFY, WriteHelper::global(),
                     EventIdToBuffer(bit_->event_on()), done);
}

void BitEventConsumer::HandleEventReport(EventReport* event, Notifiable* done) {
  if (event->event == bit_->event_on()) {
    bit_->SetState(true);
  } else if (event->event == bit_->event_off()) {
    bit_->SetState(false);
  }
  done->Notify();
}

void BitEventProducer::HandleIdentifyGlobal(EventReport* event,
                                            Notifiable* done) {
  event_barrier.Reset(done);
  SendProducerIdentified();
  event_barrier.MaybeDone();
}

void BitEventProducer::HandleIdentifyProducer(EventReport* event,
                                              Notifiable* done) {
  HandlePCIdentify(If::MTI_PRODUCER_IDENTIFIED_VALID, event, done);
}

void BitEventPC::HandleIdentifyProducer(EventReport* event, Notifiable* done) {
  HandlePCIdentify(If::MTI_PRODUCER_IDENTIFIED_VALID, event, done);
}

void BitEventConsumer::HandleIdentifyConsumer(EventReport* event,
                                              Notifiable* done) {
  HandlePCIdentify(If::MTI_CONSUMER_IDENTIFIED_VALID, event, done);
}

void BitEventConsumer::HandleIdentifyGlobal(EventReport* event,
                                            Notifiable* done) {
  event_barrier.Reset(done);
  SendConsumerIdentified();
  event_barrier.MaybeDone();
}

void BitEventPC::HandleIdentifyGlobal(EventReport* event, Notifiable* done) {
  event_barrier.Reset(done);
  SendProducerIdentified();
  SendConsumerIdentified();
  event_barrier.MaybeDone();
}

}; /* namespace NMRAnet */
