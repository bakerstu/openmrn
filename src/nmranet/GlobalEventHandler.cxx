//#define LOGLEVEL VERBOSE
#include "utils/logging.h"

#include <algorithm>
#include <vector>
#include <endian.h>

#include "nmranet/GlobalEventHandler.hxx"

#include "nmranet/GlobalEventHandlerImpl.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/EventManager.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/EndianHelper.hxx"

namespace NMRAnet
{

/*static*/
GlobalEventService *GlobalEventService::instance = nullptr;

GlobalEventService::GlobalEventService(ExecutorBase *e) : Service(e)
{
    HASSERT(instance == nullptr);
    instance = this;
    impl_.reset(new Impl(this));
}

GlobalEventService::GlobalEventService(AsyncIf *interface)
    : Service(interface->executor())
{
    HASSERT(instance == nullptr);
    instance = this;
    impl_.reset(new Impl(this));
    register_interface(interface);
}

GlobalEventService::~GlobalEventService()
{
    HASSERT(instance == this);
    instance = nullptr;
}

void GlobalEventService::register_interface(AsyncIf *interface)
{
    impl()->ownedFlows_.emplace_back(new GlobalEventFlow(
        interface, this, GlobalEventService::Impl::MTI_VALUE_EVENT,
        GlobalEventService::Impl::MTI_MASK_EVENT));
    impl()->ownedFlows_.emplace_back(new GlobalEventFlow(
        interface, this, GlobalEventService::Impl::MTI_VALUE_GLOBAL,
        GlobalEventService::Impl::MTI_MASK_GLOBAL));
    impl()->ownedFlows_.emplace_back(new GlobalEventFlow(
        interface, this, GlobalEventService::Impl::MTI_VALUE_ADDRESSED_ALL,
        GlobalEventService::Impl::MTI_MASK_ADDRESSED_ALL));
}

GlobalEventService::Impl::Impl(GlobalEventService *service)
    : callerFlow_(service)
{
    registry.reset(new VectorEventHandlers());
}

GlobalEventService::Impl::~Impl()
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

GlobalEventFlow::GlobalEventFlow(AsyncIf *async_if,
                                 GlobalEventService *event_service,
                                 unsigned mti_value, unsigned mti_mask)
    : IncomingMessageStateFlow(async_if)
    , eventService_(event_service)
    , iterator_(event_service->impl()->registry->create_iterator())
{
    interface()->dispatcher()->register_handler(this, mti_value, mti_mask);
}

GlobalEventFlow::~GlobalEventFlow()
{
    interface()->dispatcher()->unregister_handler_all(this);
    delete iterator_;
}

/// Returns true if there are outstanding events that are not yet handled.
bool GlobalEventService::event_processing_pending()
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

StateFlowBase::Action GlobalEventFlow::entry()
{
    // at this point: we have the mutex.
    LOG(VERBOSE, "GlobalFlow::HandleEvent");
    EventReport *rep = &eventReport_;
    rep->src_node = nmsg()->src;
    rep->dst_node = nmsg()->dstNode;
    if ((nmsg()->mti & If::MTI_EVENT_MASK) == If::MTI_EVENT_MASK) {
        if (nmsg()->payload.size() != 8)
        {
            LOG(INFO, "Invalid input event message, payload length %d",
                (unsigned)nmsg()->payload.size());
            return release_and_exit();
        }
        rep->event = NetworkToEventID(nmsg()->payload.data());
        rep->mask = 1;
    } else {
        // Message without event payload.
        rep->event = 0;
        /// @TODO(balazs.racz) refactor this into a 
        rep->mask = 0xFFFFFFFFFFFFFFFFULL;
    }

    switch (nmsg()->mti)
    {
        case If::MTI_EVENT_REPORT:
            fn_ = &NMRAnetEventHandler::HandleEventReport;
            break;
        case If::MTI_CONSUMER_IDENTIFY:
            fn_ = &NMRAnetEventHandler::HandleIdentifyConsumer;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_RANGE:
            DecodeRange(rep);
            fn_ = &NMRAnetEventHandler::HandleConsumerRangeIdentified;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_UNKNOWN:
            rep->state = UNKNOWN;
            fn_ = &NMRAnetEventHandler::HandleConsumerIdentified;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_VALID:
            rep->state = VALID;
            fn_ = &NMRAnetEventHandler::HandleConsumerIdentified;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_INVALID:
            rep->state = INVALID;
            fn_ = &NMRAnetEventHandler::HandleConsumerIdentified;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_RESERVED:
            rep->state = RESERVED;
            fn_ = &NMRAnetEventHandler::HandleConsumerIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFY:
            fn_ = &NMRAnetEventHandler::HandleIdentifyProducer;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_RANGE:
            DecodeRange(rep);
            fn_ = &NMRAnetEventHandler::HandleProducerRangeIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_UNKNOWN:
            rep->state = UNKNOWN;
            fn_ = &NMRAnetEventHandler::HandleProducerIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_VALID:
            rep->state = VALID;
            fn_ = &NMRAnetEventHandler::HandleProducerIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_INVALID:
            rep->state = INVALID;
            fn_ = &NMRAnetEventHandler::HandleProducerIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_RESERVED:
            rep->state = RESERVED;
            fn_ = &NMRAnetEventHandler::HandleProducerIdentified;
            break;
        case If::MTI_EVENTS_IDENTIFY_ADDRESSED:
            if (!rep->dst_node)
            {
                LOG(INFO, "Invalid addressed identify all message, destination "
                          "node not found");
                return release_and_exit();
            }
        // fall through
        case If::MTI_EVENTS_IDENTIFY_GLOBAL:
            fn_ = &NMRAnetEventHandler::HandleIdentifyGlobal;
            break;
        default:
            DIE("Unexpected message arrived at the global event handler.");
    } //    case
    // The incoming message is not needed anymore.
    release();

    iterator_->init_iteration(rep);
    return call_immediately(STATE(iterate_next));
}

StateFlowBase::Action GlobalEventFlow::iterate_next()
{
    NMRAnetEventHandler *handler = iterator_->next_entry();
    if (!handler)
    {
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

#if 0

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param node node that the packet is addressed to
 * @param data NMRAnet packet data
 */
void nmranet_event_packet_addressed(If::MTI mti, NodeHandle src, Node* node,
                                    const void* data)
{
    /*struct id_node* id_node = node;
    if (id_node->priv->state == NODE_UNINITIALIZED) {
      return;
      }*/

    GlobalEventMessage* m = GlobalEventFlow::instance->AllocateMessage();
    m->mti = mti;
    m->dst_node = node;
    m->src_node = src;
    m->event = 0;
    if (data)
    {
        memcpy(&m->event, data, sizeof(uint64_t));
        m->event = be64toh(m->event);
    }
    GlobalEventFlow::instance->PostEvent(m);

    /*  switch (mti) {
      default:
        break;
      case MTI_CONSUMER_IDENTIFY:
        // nmranet_identify_consumers(node, event, EVENT_EXACT_MASK);
        break;
      case MTI_CONSUMER_IDENTIFIED_RANGE:
        // nmranet_identify_consumers(node, event, identify_range_mask(event));
        break;
      case MTI_CONSUMER_IDENTIFIED_UNKNOWN: // fall through
      case MTI_CONSUMER_IDENTIFIED_VALID:   // fall through
      case MTI_CONSUMER_IDENTIFIED_INVALID: // fall through
      case MTI_CONSUMER_IDENTIFIED_RESERVED:
        break;
      case MTI_PRODUCER_IDENTIFY:
        // nmranet_identify_producers(node, event, EVENT_EXACT_MASK);
        break;
      case MTI_PRODUCER_IDENTIFIED_RANGE:
        // nmranet_identify_producers(node, event, identify_range_mask(event));
        break;
      case MTI_PRODUCER_IDENTIFIED_UNKNOWN: // fall through
      case MTI_PRODUCER_IDENTIFIED_VALID:   // fall through
      case MTI_PRODUCER_IDENTIFIED_INVALID: // fall through
      case MTI_PRODUCER_IDENTIFIED_RESERVED:
        break;
      case MTI_EVENTS_IDENTIFY_ADDRESSED: // fall through
      case MTI_EVENTS_IDENTIFY_GLOBAL:
        // nmranet_identify_consumers(node, 0, EVENT_ALL_MASK);
        // nmranet_identify_producers(node, 0, EVENT_ALL_MASK);
        break;
        }*/
}

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param src source Node ID
 * @param data NMRAnet packet data
 */
void nmranet_event_packet_global(If::MTI mti, NodeHandle src, const void* data)
{
    GlobalEventMessage* m = GlobalEventFlow::instance->AllocateMessage();
    m->mti = mti;
    m->dst_node = nullptr;
    m->src_node = src;
    m->event = 0;
    if (data)
    {
        memcpy(&m->event, data, sizeof(uint64_t));
        m->event = be64toh(m->event);
    }
    GlobalEventFlow::instance->PostEvent(m);

    /*  switch (mti) {
      default:
        break;
      case MTI_EVENT_REPORT: {
        // to save processing time in instantiations that include a large
        // number of nodes, consumers are sorted at the event level and
        // not at the node level.

        struct event_node* event_node;
              struct event_node event_lookup;

        uint64_t event;
        memcpy(&event, data, sizeof(uint64_t));

        event = be64toh(event);
        event_lookup.event = event;

        event_node = RB_FIND(event_tree, &eventHead, &event_lookup);
        if (event_node) {
          for (EventPriv* current = event_node->priv; current != NULL;
               current = current->next) {
            event_post(current->node, src, event);
          }
          }
        break;
      }
      case MTI_CONSUMER_IDENTIFY:
      // fall through
      case MTI_CONSUMER_IDENTIFIED_RANGE:
      // fall through
      case MTI_CONSUMER_IDENTIFIED_UNKNOWN:
      // fall through
      case MTI_CONSUMER_IDENTIFIED_VALID:
      // fall through
      case MTI_CONSUMER_IDENTIFIED_INVALID:
      // fall through
      case MTI_CONSUMER_IDENTIFIED_RESERVED:
      // fall through
      case MTI_PRODUCER_IDENTIFY:
      // fall through
      case MTI_PRODUCER_IDENTIFIED_RANGE:
      // fall through
      case MTI_PRODUCER_IDENTIFIED_UNKNOWN:
      // fall through
      case MTI_PRODUCER_IDENTIFIED_VALID:
      // fall through
      case MTI_PRODUCER_IDENTIFIED_INVALID:
      // fall through
      case MTI_PRODUCER_IDENTIFIED_RESERVED:
      // fall through
      case MTI_EVENTS_IDENTIFY_GLOBAL:
        //      os_mutex_lock(&nodeMutex);
        // global message, deliver all, non-subscribe
              for (node_t node = nmranet_node_next(NULL); node != NULL;
             node = nmranet_node_next(node)) {
          nmranet_event_packet_addressed(mti, node, data);
        }
        os_mutex_unlock(&nodeMutex);
        break;
    }
    */
}

void nmranet_identify_consumers(Node* node, uint64_t event, uint64_t mask)
{
    // Ignored: we'll do the global identify in IdentifyProducers.
}

void nmranet_identify_producers(Node* node, uint64_t event, uint64_t mask)
{
    nmranet_event_packet_global(If::MTI_EVENTS_IDENTIFY_GLOBAL, {0, 0}, NULL);
}

#endif // if 0

} /* namespace NMRAnet */
