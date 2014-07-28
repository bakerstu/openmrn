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
#include "nmranet/EventService.hxx"

#ifdef __linux__
//#define DESCRIBE_VAR
#endif

#ifdef DESCRIBE_VAR
#include <string>
namespace nmranet
{
extern const string& GetNameForOffset(int);

__attribute__ ((weak))
const string& GetNameForOffset(int) { static string empty; return empty; }
}
#endif

namespace nmranet
{

BitRangeEventPC::BitRangeEventPC(Node *node,
                                 uint64_t event_base, uint32_t* backing_store,
                                 unsigned size)
    : event_base_(event_base), node_(node), data_(backing_store), size_(size) {
  unsigned mask = EventRegistry::align_mask(&event_base, size * 2);
  EventRegistry::instance()->register_handlerr(this, event_base, mask);
}

BitRangeEventPC::~BitRangeEventPC() {
  uint64_t event = event_base_;
  unsigned mask = EventRegistry::align_mask(&event, size_ * 2);
  EventRegistry::instance()->unregister_handlerr(this, event, mask);
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
                          BarrierNotifiable* done) {
  HASSERT(bit < size_);
  uint32_t* ofs;
  uint32_t mask;
  GetBitAndMask(bit, &ofs, &mask);
  bool old_value = new_value;
  HASSERT(ofs);
  if (ofs)
    old_value = (*ofs) & mask;
  if (old_value != new_value) {
#ifdef DESCRIBE_VAR
    fprintf(stderr, "BitRange: OUT bit %x (%s) to %d\n", bit,
            GetNameForOffset(bit).c_str(), new_value);
#else
    LOG(VERBOSE, "BitRange: set bit %x to %d", bit, new_value);
#endif
    if (new_value) {
      *ofs |= mask;
    } else {
      *ofs &= ~mask;
    }
    uint64_t event = event_base_ + bit * 2;
    if (!new_value)
      event++;
    writer->WriteAsync(node_, Defs::MTI_EVENT_REPORT, WriteHelper::global(),
                       EventIdToBuffer(event), done);
    if (!done) {
      // We wait for the sent-out event to come back. Otherwise there is a race
      // condition where the automata processing could have gone further, but
      // the "set" message will arrive.
      while (EventService::instance->event_processing_pending()) {
        usleep(100);
      }
    }
  } else {
#ifdef DESCRIBE_VAR
    fprintf(stderr, "BitRange: out bit %x (%s) to %d\n", bit,
            GetNameForOffset(bit).c_str(), new_value);
#endif
    if (done)
      done->notify();
  }
}

void BitRangeEventPC::HandleEventReport(EventReport* event, BarrierNotifiable* done) {
  done->notify();
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
                                             BarrierNotifiable* done) {
  HandleIdentifyBase(Defs::MTI_PRODUCER_IDENTIFIED_VALID, event, done);
}

void BitRangeEventPC::HandleIdentifyConsumer(EventReport* event,
                                             BarrierNotifiable* done) {
  HandleIdentifyBase(Defs::MTI_CONSUMER_IDENTIFIED_VALID, event, done);
}
void BitRangeEventPC::HandleIdentifyBase(Defs::MTI mti_valid, EventReport* event,
                                         BarrierNotifiable* done) {
  if (event->event < event_base_)
    return done->notify();
  uint64_t d = (event->event - event_base_);
  bool new_value = !(d & 1);
  d >>= 1;
  if (d >= size_)
    return done->notify();
  uint32_t* ofs = nullptr;
  uint32_t mask = 0;
  GetBitAndMask(d, &ofs, &mask);
  Defs::MTI mti = mti_valid;
  bool old_value = *ofs & mask;
  if (old_value != new_value) {
    mti++; // mti INVALID
  }

  event_write_helper1.WriteAsync(node_, mti, WriteHelper::global(),
                                 EventIdToBuffer(event->event), done);
}

uint64_t EncodeRange(uint64_t begin, unsigned size) {
  // We assemble a valid event range identifier that covers our block.
  uint64_t end = begin + size - 1;
  uint64_t shift = 1;
  while ((begin + shift) < end) {
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
                                           BarrierNotifiable* done) {
  if (event->dst_node && event->dst_node != node_) {
      return done->notify();
  }
  uint64_t range = EncodeRange(event_base_, size_ * 2);
  event_write_helper1.WriteAsync(node_, Defs::MTI_PRODUCER_IDENTIFIED_RANGE,
                                 WriteHelper::global(),
                                 EventIdToBuffer(range), done->new_child());
  event_write_helper2.WriteAsync(node_, Defs::MTI_CONSUMER_IDENTIFIED_RANGE,
                                 WriteHelper::global(),
                                 EventIdToBuffer(range), done->new_child());
  done->maybe_done();
}

ByteRangeEventC::ByteRangeEventC(Node *node,
                                 uint64_t event_base, uint8_t* backing_store,
                                 unsigned size)
    : event_base_(event_base), node_(node), data_(backing_store), size_(size) {
  unsigned mask = EventRegistry::align_mask(&event_base, size * 256);
  EventRegistry::instance()->register_handlerr(this, event_base, mask);
}

ByteRangeEventC::~ByteRangeEventC() {
  uint64_t event_base = event_base_;
  unsigned mask = EventRegistry::align_mask(&event_base, size_ * 256);
  EventRegistry::instance()->unregister_handlerr(this, event_base, mask);
}

void ByteRangeEventC::HandleEventReport(EventReport* event, BarrierNotifiable* done) {
  done->notify();
  uint8_t* storage;
  uint8_t value;
  if (!DecodeEventId(event->event, &storage, &value))
    return;
#ifdef DESCRIBE_VAR
  fprintf(stderr, "ByteRange: IN  byte %x to %d\n", storage - data_, value);
#else
  LOG(VERBOSE, "ByteRange: evt %x to %d", (unsigned)(storage - data_), value);
#endif
  *storage = value;
  notify_changed(storage - data_);
}

bool ByteRangeEventC::DecodeEventId(uint64_t event, uint8_t** storage, uint8_t*value) {
  *storage = nullptr;
  *value = 0;
  if (event < event_base_) return false;
  event -= event_base_;
  *value = event & 0xff;
  event >>= 8;
  if (event >= size_) return false;
  *storage = data_ + event;
  return true;
}

void ByteRangeEventC::HandleIdentifyConsumer(EventReport* event,
                                             BarrierNotifiable* done) {
  uint8_t* storage;
  uint8_t value;
  if (!DecodeEventId(event->event, &storage, &value)) {
    return done->notify();
  }
  Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
  if (*storage != value) {
    mti++; // mti INVALID
  }
  event_write_helper1.WriteAsync(node_, mti, WriteHelper::global(),
                                 EventIdToBuffer(event->event), done);
}

void ByteRangeEventC::HandleIdentifyGlobal(EventReport *event,
                                           BarrierNotifiable *done)
{
    if (event->dst_node && event->dst_node != node_) {
        return done->notify();
    }
    uint64_t range = EncodeRange(event_base_, size_ * 256);
    event_write_helper1.WriteAsync(
        node_, Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
        EventIdToBuffer(range), done->new_child());
    done->maybe_done();
}

ByteRangeEventP::ByteRangeEventP(Node *node,
                                 uint64_t event_base, uint8_t* backing_store,
                                 unsigned size)
    : ByteRangeEventC(node, event_base, backing_store, size) {}

void ByteRangeEventP::HandleEventReport(EventReport* event, BarrierNotifiable* done) {
    // Nothing to do for producers.
    done->notify();
}
void ByteRangeEventP::HandleIdentifyConsumer(EventReport* event, BarrierNotifiable* done) {
    // Nothing to do for producers.
    done->notify();
}

uint64_t ByteRangeEventP::CurrentEventId(unsigned byte) {
    return event_base_ + (byte << 8) + data_[byte];
}

void ByteRangeEventP::HandleIdentifyProducer(EventReport* event, BarrierNotifiable* done) {
    uint8_t* storage;
    uint8_t value;
    if (!DecodeEventId(event->event, &storage, &value)) {
        return done->notify();
    }
    Defs::MTI mti = Defs::MTI_PRODUCER_IDENTIFIED_VALID;
    if (*storage != value) {
        mti++; // mti INVALID
        // We also send off the currently valid value.
        Update(storage - data_, &event_write_helper2, done->new_child());
    }
    event_write_helper1.WriteAsync(node_, mti, WriteHelper::global(),
                                   EventIdToBuffer(event->event), done->new_child());
    done->maybe_done();
}
void ByteRangeEventP::HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done) {
  if (event->dst_node && event->dst_node != node_) {
      return done->notify();
  }
  uint64_t range = EncodeRange(event_base_, size_ * 256);
  event_write_helper1.WriteAsync(node_, Defs::MTI_PRODUCER_IDENTIFIED_RANGE,
                                 WriteHelper::global(), EventIdToBuffer(range),
                                 done);
}

void ByteRangeEventP::Update(unsigned byte, WriteHelper* writer, BarrierNotifiable* done) {
    // @TODO(balazs.racz): Should we use producer identified valid or event
    // report here?
    writer->WriteAsync(node_, Defs::MTI_EVENT_REPORT, WriteHelper::global(),
                       EventIdToBuffer(CurrentEventId(byte)), done);
}

// Responses to possible queries.
void ByteRangeEventP::HandleConsumerIdentified(EventReport* event, BarrierNotifiable* done) {
    uint8_t* storage;
    uint8_t value;
    if (!DecodeEventId(event->event, &storage, &value)) {
        return done->notify();
    }
    Update(storage - data_, &event_write_helper1, done);
}

void ByteRangeEventP::HandleConsumerRangeIdentified(EventReport* event, BarrierNotifiable* done) {
    /** @TODO(balazs.racz): We should respond with the correct signal aspect
     *  for each offset that we offer. */
    if (event->event + event->mask < event_base_) {
        return done->notify();
    }
    if (event->event >= event_base_ + size_ * 256) {
        return done->notify();
    }
    unsigned start_offset = 0;
    unsigned end_offset = 0;
    uint8_t* storage;
    uint8_t value;
    if (!DecodeEventId(event->event, &storage, &value)) {
        start_offset = 0;
    } else {
        start_offset = storage - data_;
    }
    if (!DecodeEventId(event->event + event->mask, &storage, &value)) {
        end_offset = size_;
    } else {
        end_offset = storage - data_ + 1;
    }
    unsigned cur = start_offset;
    if (cur < end_offset) {
        Update(cur++, &event_write_helper1, done->new_child());
    }
    if (cur < end_offset) {
        Update(cur++, &event_write_helper2, done->new_child());
    }
    if (cur < end_offset) {
        Update(cur++, &event_write_helper3, done->new_child());
    }
    if (cur < end_offset) {
        Update(cur++, &event_write_helper4, done->new_child());
    }
    // This will crash if more than four packets are to be produced. The above
    // code should be replaced by a background iteration in that case.
    HASSERT(cur >= end_offset);
    done->maybe_done();
}

BitEventHandler::BitEventHandler(BitEventInterface* bit)
    : bit_(bit) {
}

void BitEventHandler::register_handler() {
    if ((bit_->event_on() ^ bit_->event_off()) == 1ULL) {
        // Register once for two eventids.
        uint64_t id = bit_->event_on() & (~1ULL);
        EventRegistry::instance()->register_handlerr(this, id, 1);
    } else {
        EventRegistry::instance()->register_handlerr(this, bit_->event_on(), 0);
        EventRegistry::instance()->register_handlerr(this, bit_->event_off(), 0);
    }
}

void BitEventHandler::unregister_handler() {
    if ((bit_->event_on() ^ bit_->event_off()) == 1ULL) {
        // Register once for two eventids.
        uint64_t id = bit_->event_on() & (~1ULL);
        EventRegistry::instance()->unregister_handlerr(this, id, 1);
    } else {
        EventRegistry::instance()->unregister_handlerr(this, bit_->event_on(), 0);
        EventRegistry::instance()->unregister_handlerr(this, bit_->event_off(), 0);
    }
}

void BitEventHandler::SendProducerIdentified(BarrierNotifiable* done) {
  bool value = bit_->GetCurrentState();
  Defs::MTI mti = Defs::MTI_PRODUCER_IDENTIFIED_VALID;
  if (!value)
    mti++; // INVALID
  event_write_helper1.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(bit_->event_on()),
                                 done->new_child());
  if (!value) {
    mti--; // VALID
  } else {
    mti++; // INVALID
  }
  event_write_helper2.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(bit_->event_off()),
                                 done->new_child());
}

void BitEventHandler::SendConsumerIdentified(BarrierNotifiable* done) {
  bool value = bit_->GetCurrentState();
  Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
  if (!value)
    mti++; // INVALID
  event_write_helper3.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(bit_->event_on()),
                                 done->new_child());
  if (!value) {
    mti--; // VALID
  } else {
    mti++; // INVALID
  }
  event_write_helper4.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(bit_->event_off()),
                                 done->new_child());
}

void BitEventHandler::SendEventReport(WriteHelper* writer, Notifiable* done) {
  bool value = bit_->GetCurrentState();
  uint64_t event = value ? bit_->event_on() : bit_->event_off();
  writer->WriteAsync(bit_->node(), Defs::MTI_EVENT_REPORT, WriteHelper::global(),
                     EventIdToBuffer(event), done);
}

void BitEventHandler::HandlePCIdentify(Defs::MTI mti, EventReport* event,
                                       BarrierNotifiable* done) {
  if (event->src_node.id == bit_->node()->node_id()) {
    // We don't respond to queries from our own node. This is not nice, but we
    // want to avoid to answering our own Query command.
    done->notify();
    return;
  }
  bool active;
  if (event->event == bit_->event_on()) {
    active = bit_->GetCurrentState();
  } else if (event->event == bit_->event_off()) {
    active = !bit_->GetCurrentState();
  } else {
    done->notify();
    return;
  }
  if (!active) {
    ++mti; // mti_invalid.
  }
  event_write_helper1.WriteAsync(bit_->node(), mti, WriteHelper::global(),
                                 EventIdToBuffer(event->event), done);
}

void BitEventConsumer::HandleProducerIdentified(EventReport* event, BarrierNotifiable* done) {
  done->notify();
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

void BitEventConsumer::SendQuery(WriteHelper* writer, BarrierNotifiable* done) {
  writer->WriteAsync(bit_->node(), Defs::MTI_PRODUCER_IDENTIFY, WriteHelper::global(),
                     EventIdToBuffer(bit_->event_on()), done);
}

void BitEventConsumer::HandleEventReport(EventReport* event, BarrierNotifiable* done) {
  if (event->event == bit_->event_on()) {
    bit_->SetState(true);
  } else if (event->event == bit_->event_off()) {
    bit_->SetState(false);
  }
  done->notify();
}

void BitEventProducer::HandleIdentifyGlobal(EventReport* event,
                                            BarrierNotifiable* done) {
  if (event->dst_node && event->dst_node != bit_->node()) {
    return done->notify();
  }
  SendProducerIdentified(done);
  done->maybe_done();
}

void BitEventProducer::HandleIdentifyProducer(EventReport* event,
                                              BarrierNotifiable* done) {
  HandlePCIdentify(Defs::MTI_PRODUCER_IDENTIFIED_VALID, event, done);
}

void BitEventPC::HandleIdentifyProducer(EventReport* event, BarrierNotifiable* done) {
  HandlePCIdentify(Defs::MTI_PRODUCER_IDENTIFIED_VALID, event, done);
}

void BitEventConsumer::HandleIdentifyConsumer(EventReport* event,
                                              BarrierNotifiable* done) {
  HandlePCIdentify(Defs::MTI_CONSUMER_IDENTIFIED_VALID, event, done);
}

void BitEventConsumer::HandleIdentifyGlobal(EventReport* event,
                                            BarrierNotifiable* done) {
  if (event->dst_node && event->dst_node != bit_->node()) {
    return done->notify();
  }
  SendConsumerIdentified(done);
  done->maybe_done();
}

void BitEventPC::HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done) {
  if (event->dst_node && event->dst_node != bit_->node()) {
    return done->notify();
  }
  SendProducerIdentified(done);
  SendConsumerIdentified(done);
  done->maybe_done();
}

}; /* namespace nmranet */
