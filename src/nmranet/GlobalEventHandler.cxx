//#define LOGLEVEL VERBOSE
#include "utils/logging.h"

#include <algorithm>
#include <vector>
#include <endian.h>

#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/EventManager.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/EndianHelper.hxx"

namespace NMRAnet
{

/*static*/
GlobalEventService *GlobalEventService::instance = nullptr;

struct GlobalEventFlow::Impl
{
    Impl(int max_event_slots)
    {
        message_block_.reset(new GlobalEventMessage[max_event_slots]);
        for (int i = 0; i < max_event_slots; ++i)
        {
            free_events_.TypedRelease(&message_block_[i]);
        }
    }

    // This is the queue of events that are coming from the read thread to the
    // handler thread. Every "released" event is a new incoming event message.
    // TypedAllocator<GlobalEventMessage> event_queue_;

    // This is the queue of global identify events.
    // TypedAllocator<GlobalEventMessage> global_event_queue_;

    // Freelist of event message objects.
    // TypedAllocator<GlobalEventMessage> free_events_;

    // Incoming event message, as it got off the event_queue.
    // GlobalEventMessage* message_;

    // Statically allocated structure for calling the event handlers from the
    // global event queue.
    EventReport global_event_report_;

    // Holds the memory used by the event messages for destruction.
    // std::unique_ptr<GlobalEventMessage[]> message_block_;
};

GlobalEventService::GlobalEventService(ExecutorBase *e) : Service(e)
{
    HASSERT(instance_ == nullptr);
    instance_ = this;
    impl_.reset(new Impl());
}

GlobalEventService::~GlobalEventService() {
    HASSERT(instance_ == this);
    instance_ = nullptr;
}

GlobalEventService::Impl::Impl()
{
    handler_.reset(new VectorEventHandlers(service()->executor()));
}

GlobalEventService::Impl::~Impl()
{
}

GlobalEventFlow::GlobalEventFlow(GlobalEventService *service,
                                 AsyncIf *interface)
    : IncomingMessageStateFlow(service),
      interface_(interface)
{
    interface()->dispatcher()->register_handler(
        GlobalEventService::Impl::MTI_VALUE_EVENT,
        GlobalEventService::Impl::MTI_MASK_EVENT,
        this);
}

GlobalEventFlow::~GlobalEventFlow()
{
    interface()->dispatcher()->unregister_handler(
        GlobalEventService::Impl::MTI_VALUE_EVENT,
        GlobalEventService::Impl::MTI_MASK_EVENT,
        this);
}

/// Returns true if there are outstanding events that are not yet handled.
bool GlobalEventService::event_processing_pending()
{
    HASSERT(0);
/*    // Protects against static instance == nullptr.
    if (!this)
        return false;
    /// @TODO(balazs.racz): maybe the order of these checks should be different.
    if (IsPendingOrRunning())
        return true;
    if (next_state() != ST(HandleEventArrived))
        return true;
    if (impl_->event_queue_.Peek())
        return true;
        return false;*/
}

StateFlowBase::Action GlobalEventFlow::entry()
{
    LOG(VERBOSE, "GlobalFlow::WaitForEvent");
    return allocate_and_call(STATE(call_handler), &event_handler_mutex);
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

StateFlowBase::Action GlobalEventFlow::call_handler()
{
    // at this point: we have the mutex.
    LOG(VERBOSE, "GlobalFlow::HandleEvent");
    EventReport *rep = &main_event_report_;
    rep->src_node = nmsg()->src_node;
    rep->dst_node = nmsg()->dstNode;
    if (nmsg()->payload.size() != 8) {
        LOG(INFO, "Invalid input event message, payload length %d",
            (unsigned)nmsg()->payload.size());
        event_handler_mutex.Unlock();
        return call_immediately(STATE(wait_for_message));
    }
    rep->event = NetworkToEventID(nmsg()->payload.data());
    rep->mask = 1;

    EventHandlerFunction fn;
    switch (nmsg()->mti)
    {
        case If::MTI_EVENT_REPORT:
            fn = &NMRAnetEventHandler::HandleEventReport;
            break;
        case If::MTI_CONSUMER_IDENTIFY:
            fn = &NMRAnetEventHandler::HandleIdentifyConsumer;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_RANGE:
            DecodeRange(rep);
            fn = &NMRAnetEventHandler::HandleConsumerRangeIdentified;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_UNKNOWN:
            rep->state = UNKNOWN;
            fn = &NMRAnetEventHandler::HandleConsumerIdentified;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_VALID:
            rep->state = VALID;
            fn = &NMRAnetEventHandler::HandleConsumerIdentified;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_INVALID:
            rep->state = INVALID;
            fn = &NMRAnetEventHandler::HandleConsumerIdentified;
            break;
        case If::MTI_CONSUMER_IDENTIFIED_RESERVED:
            rep->state = RESERVED;
            fn = &NMRAnetEventHandler::HandleConsumerIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFY:
            fn = &NMRAnetEventHandler::HandleIdentifyProducer;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_RANGE:
            DecodeRange(rep);
            fn = &NMRAnetEventHandler::HandleProducerRangeIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_UNKNOWN:
            rep->state = UNKNOWN;
            fn = &NMRAnetEventHandler::HandleProducerIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_VALID:
            rep->state = VALID;
            fn = &NMRAnetEventHandler::HandleProducerIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_INVALID:
            rep->state = INVALID;
            fn = &NMRAnetEventHandler::HandleProducerIdentified;
            break;
        case If::MTI_PRODUCER_IDENTIFIED_RESERVED:
            rep->state = RESERVED;
            fn = &NMRAnetEventHandler::HandleProducerIdentified;
            break;
        case If::MTI_EVENTS_IDENTIFY_ADDRESSED:
        case If::MTI_EVENTS_IDENTIFY_GLOBAL:
            fn = &NMRAnetEventHandler::HandleIdentifyGlobal;
            impl_->global_event_report_ = *rep;
            rep = &impl_->global_event_report_;
            break;
        default:
            DIE("Unexpected message arrived at the global event handler.");
    } //    case
    // The incoming message is not needed anymore.
    release();

    (service()->impl()->handler_.get()->*fn)(rep, this);
    return wait_and_call(STATE(handler_finished));
}

StateFlowBase::Action GlobalEventFlow::handler_finished()
{
    // We still have the event mutex.
    LOG(VERBOSE, "GlobalFlow::HandlerFinished");
    unsigned priority;
    Message* next_message = static_cast<Message*>(queue_next(&priority));
    if (!next_message) {
        // No pending message in the queue: releases the mutex and allows global
        // handlers to proceed.
        event_handler_mutex.AssertLocked();
        event_handler_mutex.Unlock();
        return exit();
    }
    reset_message(next_message, priority);
    // Passes on the event mutex here. Yields to achieve the appropriate
    // priority.
    return yield_and_call(STATE(call_handler));
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

#endif // CPP_EVENT_HANDLER

} /* namespace NMRAnet */
