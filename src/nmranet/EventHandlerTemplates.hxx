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

#ifndef _NMRAnet_EventHandlerTemplates_hxx_
#define _NMRAnet_EventHandlerTemplates_hxx_

#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/NMRAnetWriteFlow.hxx"

namespace NMRAnet
{

typedef void (NMRAnetEventHandler::*EventHandlerFunction)(EventReport* event,
                                                          Notifiable* done);

// A proxy event handler has a single helper function that gets every event
// handler call with an indication of which call it is. It is helpful to create
// event containers that proxy calls to many event handler instances.
class ProxyEventHandler : public NMRAnetEventHandler
{
public:
    virtual ~ProxyEventHandler()
    {
    }

    // This function will be called for any other incoming event handler
    // function.
    virtual void HandlerFn(EventHandlerFunction fn, EventReport* event,
                           Notifiable* done) = 0;

#define DEFPROXYFN(FN)                                                         \
    virtual void FN(EventReport* event, Notifiable* done)                      \
    {                                                                          \
        HandlerFn(&NMRAnetEventHandler::FN, event, done);                      \
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
class SimpleEventHandler : public NMRAnetEventHandler
{
public:
#define IGNOREFN(FN)                                                           \
    virtual void FN(EventReport* event, Notifiable* done)                      \
    {                                                                          \
        done->Notify();                                                        \
    }

    IGNOREFN(HandleEventReport);
    IGNOREFN(HandleConsumerIdentified);
    IGNOREFN(HandleConsumerRangeIdentified);
    IGNOREFN(HandleProducerIdentified);
    IGNOREFN(HandleProducerRangeIdentified);
    IGNOREFN(HandleIdentifyConsumer);
    IGNOREFN(HandleIdentifyProducer);
};

class BitEventInterface
{
public:
    BitEventInterface(uint64_t event_on, uint64_t event_off)
        : event_on_(event_on), event_off_(event_off)
    {
    }
    virtual bool GetCurrentState() = 0;
    virtual void SetState(bool new_value) = 0;
    uint64_t event_on()
    {
        return event_on_;
    }
    uint64_t event_off()
    {
        return event_off_;
    }
    virtual Node* node() = 0;

private:
    uint64_t event_on_;
    uint64_t event_off_;

    DISALLOW_COPY_AND_ASSIGN(BitEventInterface);
};

template <class T> class MemoryBit : public BitEventInterface
{
public:
    MemoryBit(Node* node, uint64_t event_on, uint64_t event_off, T* ptr, T mask)
        : BitEventInterface(event_on, event_off),
          node_(node),
          ptr_(ptr),
          mask_(mask)
    {
    }

    virtual Node* node()
    {
        return node_;
    }
    virtual bool GetCurrentState()
    {
        return (*ptr_) & mask_;
    }
    virtual void SetState(bool new_value)
    {
        if (new_value) {
            *ptr_ |= mask_;
        } else {
            *ptr_ &= ~mask_;
        }
    }

private:
    Node* node_;
    T* ptr_;
    T mask_;

    DISALLOW_COPY_AND_ASSIGN(MemoryBit);
};

class BitEventHandler : public SimpleEventHandler
{
public:
    BitEventHandler(BitEventInterface* bit);
    ~BitEventHandler();

protected:
    // Sends off two packets using write_event_handler{1,2} of
    // ProducerIdentified
    // for handling a global identify events message. Uses event_barrier.
    void SendProducerIdentified();

    // Sends off two packets using write_event_handler{3,4} of
    // ConsumerIdentified
    // for handling a global identify events message. Uses event_barrier.
    void SendConsumerIdentified();

    // Sends an event report packet (unconditionally).
    void SendEventReport(WriteHelper* writer, Notifiable* done);

    // Checks if the event in the report is something we are interested in, and
    // if so, sends off a {Producer|Consumer}Identify message. Uses
    // write_event_handler1.
    void HandlePCIdentify(If::MTI mti_valid, EventReport* event,
                          Notifiable* done);

    BitEventInterface* bit_;

private:
    DISALLOW_COPY_AND_ASSIGN(BitEventHandler);
};

class BitEventProducer : public BitEventHandler
{
public:
    BitEventProducer(BitEventInterface* bit) : BitEventHandler(bit)
    {
    }

    // Requests the event associated with the current value of the bit to be
    // produced (unconditionally).
    //
    // @param writer is the output flow to be used.
    //
    // @param done is the notification callback. If it is NULL, the writer will
    // be invoked inline and potentially block the calling thread.
    void Update(WriteHelper* writer, Notifiable* done)
    {
        SendEventReport(writer, done);
    }

    virtual void HandleIdentifyGlobal(EventReport* event, Notifiable* done);
    virtual void HandleIdentifyProducer(EventReport* event, Notifiable* done);

private:
    DISALLOW_COPY_AND_ASSIGN(BitEventProducer);
};

class BitEventConsumer : public BitEventHandler
{
public:
    BitEventConsumer(BitEventInterface* bit) : BitEventHandler(bit)
    {
    }

    virtual void HandleEventReport(EventReport* event, Notifiable* done);
    virtual void HandleIdentifyGlobal(EventReport* event, Notifiable* done);
    virtual void HandleIdentifyConsumer(EventReport* event, Notifiable* done);
};

class BitEventPC : public BitEventConsumer
{
public:
    BitEventPC(BitEventInterface* bit) : BitEventConsumer(bit)
    {
    }

    virtual void HandleIdentifyProducer(EventReport* event, Notifiable* done);
    virtual void HandleIdentifyGlobal(EventReport* event, Notifiable* done);
};

class BitRangeEventPC : public SimpleEventHandler
{
public:
    // Creates a new bit range listener. backing store points to memory of at
    // least size bits (round up to multiple of 32). This class will advertise
    // producing and consuming size * 2 events contiguous from
    // event_base. event_base will turn bit 0 on, event_base + 1 will turn bit 0
    // off, event_base + 2 will turn bit 1 on, event_base + 3 will turn bit 1
    // off, etc.
    BitRangeEventPC(Node* node, uint64_t event_base, uint32_t* backing_store,
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
    void Set(unsigned bit, bool new_value, WriteHelper* writer,
             Notifiable* done);

    //! @returns the value of a given bit. 0 <= bit < size_.
    bool Get(unsigned bit) const;

    virtual void HandleEventReport(EventReport* event, Notifiable* done);
    virtual void HandleIdentifyProducer(EventReport* event, Notifiable* done);
    virtual void HandleIdentifyConsumer(EventReport* event, Notifiable* done);
    virtual void HandleIdentifyGlobal(EventReport* event, Notifiable* done);

private:
    void HandleIdentifyBase(If::MTI mti_valid, EventReport* event,
                            Notifiable* done);
    void GetBitAndMask(unsigned bit, uint32_t** data, uint32_t* mask) const;

    uint64_t event_base_;
    Node* node_;
    uint32_t* data_;
    unsigned size_; //< number of bits stored.
};

}; /* namespace NMRAnet */

#endif // _NMRAnet_EventHandlerTemplates_hxx_
