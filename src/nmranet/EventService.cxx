//#define LOGLEVEL VERBOSE
#include "utils/logging.h"

#include <algorithm>
#include <vector>
#include <endian.h>

#include "nmranet/EventService.hxx"

#include "nmranet/EventServiceImpl.hxx"
#include "nmranet/EventHandler.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/EventHandlerContainer.hxx"
#include "nmranet/Defs.hxx"
#include "nmranet/EndianHelper.hxx"

namespace nmranet
{

/*static*/
EventService *EventService::instance = nullptr;

EventService::EventService(ExecutorBase *e) : Service(e)
{
    HASSERT(instance == nullptr);
    instance = this;
    impl_.reset(new Impl(this));
}

EventService::EventService(If *interface) : Service(interface->executor())
{
    HASSERT(instance == nullptr);
    instance = this;
    impl_.reset(new Impl(this));
    register_interface(interface);
}

EventService::~EventService()
{
    HASSERT(instance == this);
    instance = nullptr;
}

void EventService::register_interface(If *interface)
{
    impl()->ownedFlows_.emplace_back(new EventIteratorFlow(
        interface, this, EventService::Impl::MTI_VALUE_EVENT,
        EventService::Impl::MTI_MASK_EVENT));
    impl()->ownedFlows_.emplace_back(new EventIteratorFlow(
        interface, this, EventService::Impl::MTI_VALUE_GLOBAL,
        EventService::Impl::MTI_MASK_GLOBAL));
    impl()->ownedFlows_.emplace_back(new EventIteratorFlow(
        interface, this, EventService::Impl::MTI_VALUE_ADDRESSED_ALL,
        EventService::Impl::MTI_MASK_ADDRESSED_ALL));
}

EventService::Impl::Impl(EventService *service) : callerFlow_(service)
{
#ifdef TARGET_LPC11Cxx
    registry.reset(new VectorEventHandlers());
#else
    registry.reset(new TreeEventHandlers());
#endif
}

EventService::Impl::~Impl()
{
}

StateFlowBase::Action EventCallerFlow::entry()
{
    n_.reset(this);
    EventHandlerCall *c = message()->data();
    (c->handler->*(c->fn))(c->rep, &n_);
    return wait_and_call(STATE(call_done));
}

StateFlowBase::Action EventCallerFlow::call_done()
{
    return release_and_exit();
}

EventIteratorFlow::EventIteratorFlow(If *async_if, EventService *event_service,
                                     unsigned mti_value, unsigned mti_mask)
    : IncomingMessageStateFlow(async_if)
    , eventService_(event_service)
    , iterator_(event_service->impl()->registry->create_iterator())
{
    interface()->dispatcher()->register_handler(this, mti_value, mti_mask);
}

EventIteratorFlow::~EventIteratorFlow()
{
    interface()->dispatcher()->unregister_handler_all(this);
    delete iterator_;
}

/// Returns true if there are outstanding events that are not yet handled.
bool EventService::event_processing_pending()
{
    for (auto &f : impl()->ownedFlows_)
    {
        if (!f->is_waiting())
            return true;
    }
    return false;
}

void DecodeRange(EventReport *r)
{
    uint64_t e = r->event;
    if (e & 1)
    {
        r->mask = (e ^ (e + 1)) >> 1;
    }
    else
    {
        r->mask = (e ^ (e - 1)) >> 1;
    }
    r->event &= ~r->mask;
}

StateFlowBase::Action EventIteratorFlow::entry()
{
    // at this point: we have the mutex.
    LOG(VERBOSE, "GlobalFlow::HandleEvent");
    EventReport *rep = &eventReport_;
    rep->src_node = nmsg()->src;
    rep->dst_node = nmsg()->dstNode;
    if ((nmsg()->mti & Defs::MTI_EVENT_MASK) == Defs::MTI_EVENT_MASK)
    {
        if (nmsg()->payload.size() != 8)
        {
            LOG(INFO, "Invalid input event message, payload length %d",
                (unsigned)nmsg()->payload.size());
            return release_and_exit();
        }
        rep->event = NetworkToEventID(nmsg()->payload.data());
        rep->mask = 1;
    }
    else
    {
        // Message without event payload.
        rep->event = 0;
        /// @TODO(balazs.racz) refactor this into a
        rep->mask = 0xFFFFFFFFFFFFFFFFULL;
    }

    switch (nmsg()->mti)
    {
        case Defs::MTI_EVENT_REPORT:
            fn_ = &EventHandler::HandleEventReport;
            break;
        case Defs::MTI_CONSUMER_IDENTIFY:
            fn_ = &EventHandler::HandleIdentifyConsumer;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_RANGE:
            DecodeRange(rep);
            fn_ = &EventHandler::HandleConsumerRangeIdentified;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN:
            rep->state = UNKNOWN;
            fn_ = &EventHandler::HandleConsumerIdentified;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_VALID:
            rep->state = VALID;
            fn_ = &EventHandler::HandleConsumerIdentified;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_INVALID:
            rep->state = INVALID;
            fn_ = &EventHandler::HandleConsumerIdentified;
            break;
        case Defs::MTI_CONSUMER_IDENTIFIED_RESERVED:
            rep->state = RESERVED;
            fn_ = &EventHandler::HandleConsumerIdentified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFY:
            fn_ = &EventHandler::HandleIdentifyProducer;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_RANGE:
            DecodeRange(rep);
            fn_ = &EventHandler::HandleProducerRangeIdentified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN:
            rep->state = UNKNOWN;
            fn_ = &EventHandler::HandleProducerIdentified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_VALID:
            rep->state = VALID;
            fn_ = &EventHandler::HandleProducerIdentified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_INVALID:
            rep->state = INVALID;
            fn_ = &EventHandler::HandleProducerIdentified;
            break;
        case Defs::MTI_PRODUCER_IDENTIFIED_RESERVED:
            rep->state = RESERVED;
            fn_ = &EventHandler::HandleProducerIdentified;
            break;
        case Defs::MTI_EVENTS_IDENTIFY_ADDRESSED:
            if (!rep->dst_node)
            {
                LOG(INFO, "Invalid addressed identify all message, destination "
                          "node not found");
                return release_and_exit();
            }
        // fall through
        case Defs::MTI_EVENTS_IDENTIFY_GLOBAL:
            fn_ = &EventHandler::HandleIdentifyGlobal;
            break;
        default:
            DIE("Unexpected message arrived at the global event handler.");
    } //    case
    // The incoming message is not needed anymore.
    incomingDone_ = message()->new_child();
    release();

    iterator_->init_iteration(rep);
    return call_immediately(STATE(iterate_next));
}

StateFlowBase::Action EventIteratorFlow::iterate_next()
{
    EventHandler *handler = iterator_->next_entry();
    if (!handler)
    {
        if (incomingDone_)
        {
            incomingDone_->notify();
            incomingDone_ = nullptr;
        }
        return exit();
    }
    Buffer<EventHandlerCall> *b;
    /* This could be made an asynchronous allocation. Then the pool could be
     * made fixed size. */
    eventService_->impl()->callerFlow_.pool()->alloc(&b, nullptr);
    HASSERT(b);
    b->data()->reset(&eventReport_, handler, fn_);
    n_.reset(this);
    b->set_done(&n_);
    eventService_->impl()->callerFlow_.send(b);
    return wait();
}

} /* namespace nmranet */
