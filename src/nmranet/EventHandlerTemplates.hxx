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
 * \file EventHandlerTemplates.hxx
 *
 * Defines partial implementations for event handlers that are usable for
 * multiple event handler types.
 *
 * @author Balazs Racz
 * @date 19 October 2013
 */

#ifndef _NMRANET_EVENTHANDLERTEMPLATES_HXX_
#define _NMRANET_EVENTHANDLERTEMPLATES_HXX_

#include "nmranet/EventHandler.hxx"
#include "nmranet/WriteHelper.hxx"

namespace nmranet
{

// A proxy event handler has a single helper function that gets every event
// handler call with an indication of which call it is. It is helpful to create
// event containers that proxy calls to many event handler instances.
class ProxyEventHandler : public EventHandler {
 public:
  virtual ~ProxyEventHandler() {}

  // This function will be called for any other incoming event handler
  // function.
  virtual void HandlerFn(EventHandlerFunction fn,
                         EventReport* event,
                         BarrierNotifiable* done) = 0;

#define DEFPROXYFN(FN)                                    \
  virtual void FN(EventReport* event, BarrierNotifiable* done) { \
    HandlerFn(&EventHandler::FN, event, done);     \
  }

  DEFPROXYFN(HandleEventReport);
  DEFPROXYFN(HandleConsumerIdentified);
  DEFPROXYFN(HandleConsumerRangeIdentified);
  DEFPROXYFN(HandleProducerIdentified);
  DEFPROXYFN(HandleProducerRangeIdentified);
  DEFPROXYFN(HandleIdentifyGlobal);
  DEFPROXYFN(HandleIdentifyConsumer);
  DEFPROXYFN(HandleIdentifyProducer);

#undef DEFPROXYFN
};

// SimpleEventHandler ignores all non-essential callbacks.
class SimpleEventHandler : public EventHandler {
 public:
#define IGNOREFN(FN) \
  virtual void FN(EventReport* event, BarrierNotifiable* done) { done->notify(); }

  IGNOREFN(HandleEventReport);
  IGNOREFN(HandleConsumerIdentified);
  IGNOREFN(HandleConsumerRangeIdentified);
  IGNOREFN(HandleProducerIdentified);
  IGNOREFN(HandleProducerRangeIdentified);
  IGNOREFN(HandleIdentifyConsumer);
  IGNOREFN(HandleIdentifyProducer);
};

class BitEventInterface {
 public:
  BitEventInterface(uint64_t event_on, uint64_t event_off)
      : event_on_(event_on), event_off_(event_off) {}
  virtual bool GetCurrentState() = 0;
  virtual void SetState(bool new_value) = 0;
  uint64_t event_on() { return event_on_; }
  uint64_t event_off() { return event_off_; }
  virtual Node *node() = 0;
 private:
  uint64_t event_on_;
  uint64_t event_off_;

  DISALLOW_COPY_AND_ASSIGN(BitEventInterface);
};

template<class T> class MemoryBit : public BitEventInterface {
 public:
  MemoryBit(Node *node, uint64_t event_on, uint64_t event_off, T* ptr, T mask)
      : BitEventInterface(event_on, event_off),
        node_(node), ptr_(ptr), mask_(mask) {}

  virtual Node *node() { return node_; }
  virtual bool GetCurrentState() { return (*ptr_) & mask_; }
  virtual void SetState(bool new_value) {
    if (new_value) {
      *ptr_ |= mask_;
    } else {
      *ptr_ &= ~mask_;
    }
  }

 private:
  Node *node_;
  T* ptr_;
  T mask_;

  DISALLOW_COPY_AND_ASSIGN(MemoryBit);
};

class BitEventHandler : public SimpleEventHandler {
 public:
  BitEventHandler(BitEventInterface* bit);
  ~BitEventHandler();

  // Sends an event report packet (unconditionally).
  void SendEventReport(WriteHelper* writer,
                       BarrierNotifiable* done);

 protected:
  // Sends off two packets using write_event_handler{1,2} of ProducerIdentified
  // for handling a global identify events message. Allocated children from
  // barrier done.
  void SendProducerIdentified(BarrierNotifiable* done);

  // Sends off two packets using write_event_handler{3,4} of ConsumerIdentified
  // for handling a global identify events message. Allocated children from
  // barrier done.
  void SendConsumerIdentified(BarrierNotifiable* done);

  // Checks if the event in the report is something we are interested in, and
  // if so, sends off a {Producer|Consumer}Identify message. Uses
  // write_event_handler1.
  void HandlePCIdentify(Defs::MTI mti_valid, EventReport* event, BarrierNotifiable* done);

  BitEventInterface* bit_;

 private:
  DISALLOW_COPY_AND_ASSIGN(BitEventHandler);
};

class BitEventProducer : public BitEventHandler {
 public:
  BitEventProducer(BitEventInterface* bit)
      : BitEventHandler(bit) {}

  // Requests the event associated with the current value of the bit to be
  // produced (unconditionally).
  //
  // @param writer is the output flow to be used.
  //
  // @param done is the notification callback. If it is NULL, the writer will
  // be invoked inline and potentially block the calling thread.
  void Update(WriteHelper* writer,
              BarrierNotifiable* done) {
    SendEventReport(writer, done);
  }

  virtual void HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyProducer(EventReport* event, BarrierNotifiable* done);

 private:
  DISALLOW_COPY_AND_ASSIGN(BitEventProducer);
};

class BitEventConsumer : public BitEventHandler {
 public:
  BitEventConsumer(BitEventInterface* bit)
      : BitEventHandler(bit) {}

    /// Queries producers and acquires the current state of the bit.
  void SendQuery(WriteHelper* writer, BarrierNotifiable* done);

  virtual void HandleEventReport(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyConsumer(EventReport* event, BarrierNotifiable* done);
  virtual void HandleProducerIdentified(EventReport* event, BarrierNotifiable* done);
};

class BitEventPC : public BitEventConsumer {
 public:
  BitEventPC(BitEventInterface* bit)
      : BitEventConsumer(bit) {}

  virtual void HandleIdentifyProducer(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done);
};

class BitRangeEventPC : public SimpleEventHandler {
 public:
  // Creates a new bit range listener. backing store points to memory of at
  // least size bits (round up to multiple of 32). This class will advertise
  // producing and consuming size * 2 events contiguous from
  // event_base. event_base will turn bit 0 on, event_base + 1 will turn bit 0
  // off, event_base + 2 will turn bit 1 on, event_base + 3 will turn bit 1
  // off, etc.
  BitRangeEventPC(Node *node,
                  uint64_t event_base,
                  uint32_t* backing_store,
                  unsigned size);
  virtual ~BitRangeEventPC();

  // Requests the event associated with the current value of the bit to be
  // produced (unconditionally).
  //
  // @param node specifies the source node from which to produce the event.
  //
  // @param bit is the offset of the bit to set (0 <= bit < size)
  //
  // @param new_value is the new value of the bit
  //
  // @param writer is the output flow to be used.
  //
  // @param done is the notification callback. If it is NULL, the writer will
  // be invoked inline and potentially block the calling thread.
  void Set(unsigned bit, bool new_value, WriteHelper* writer, BarrierNotifiable* done);

  /// @returns the value of a given bit. 0 <= bit < size_.
  bool Get(unsigned bit) const;

  virtual void HandleEventReport(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyProducer(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyConsumer(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done);

 private:
  void HandleIdentifyBase(Defs::MTI mti_valid, EventReport* event, BarrierNotifiable* done);
  void GetBitAndMask(unsigned bit, uint32_t** data, uint32_t* mask) const;

  uint64_t event_base_;
  Node *node_;
  uint32_t* data_;
  unsigned size_;  //< number of bits stored.
};

class ByteRangeEventC : public SimpleEventHandler {
 public:
  // Creates a new byte range listener. backing store points to memory of at
  // least size bytes. This class will advertise consuming size * 256 events
  // contiguous from event_base. event_base will set byte 0 to value 0,
  // event_base + 1 will set byte 0 to value 1, event_base + 256 will set byte
  // 1 to value zero, event_base + 257 will set byte 1 to value 1, etc.
  ByteRangeEventC(Node *node,
                  uint64_t event_base,
                  uint8_t* backing_store,
                  unsigned size);
  virtual ~ByteRangeEventC();

  virtual void HandleEventReport(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyConsumer(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done);

 protected:
  // takes an event ID and checks if we are responsible for it. Returns false
  // if it is an uninteresting eventid, returns true and fills *data with the
  // byte pointer and *value with the corresponding value.
  bool DecodeEventId(uint64_t event_id, uint8_t** data, uint8_t* value);

  uint64_t event_base_;
  Node *node_;
  uint8_t* data_;
  unsigned size_;  //< number of bytes consumed.
};

class ByteRangeEventP : public ByteRangeEventC {
 public:
  // Creates a new byte range producer. backing store points to memory of at
  // least size bytes. This class will advertise producing size * 256 events
  // contiguous from event_base. event_base will set byte 0 to value 0,
  // event_base + 1 will set byte 0 to value 1, event_base + 256 will set byte
  // 1 to value zero, event_base + 257 will set byte 1 to value 1, etc.
  ByteRangeEventP(Node *node,
                  uint64_t event_base,
                  uint8_t* backing_store,
                  unsigned size);

  // Requests the event associated with the current value of a specific byte to
  // be produced (unconditionally).
  //
  // @param byte is the offset of the value to produce (0 <= byte < size)
  //
  // @param writer is the output flow to be used.
  //
  // @param done is the notification callback. Must not be NULL.
  void Update(unsigned byte, WriteHelper* writer, BarrierNotifiable* done);

  // Need to override C behavior.
  virtual void HandleEventReport(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyConsumer(EventReport* event, BarrierNotifiable* done);
  // Own behavior.
  virtual void HandleIdentifyProducer(EventReport* event, BarrierNotifiable* done);
  virtual void HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done);
  // Responses to possible queries.
  virtual void HandleConsumerIdentified(EventReport* event, BarrierNotifiable* done);
  virtual void HandleConsumerRangeIdentified(EventReport* event, BarrierNotifiable* done);

 private:
  // Creates the eventid of the currently valid value of a given byte.
  uint64_t CurrentEventId(unsigned byte);
};

}; /* namespace nmranet */

#endif  // _NMRANET_EVENTHANDLERTEMPLATES_HXX_
