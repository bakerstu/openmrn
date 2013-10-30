
#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"
#include "if/nmranet_if.h"
#include "core/nmranet_event.h"
#include "nmranet/EventHandlerTemplates.hxx"

/*static*/
GlobalEventFlow* GlobalEventFlow::instance = nullptr;

struct GlobalEventFlow::Impl {
  Impl(int max_event_slots) : pending_sem_(max_event_slots) {}

  // This is the queue of events that are coming from the read thread to the
  // handler thread. Every "released" event is a new incoming event message.
  TypedAllocator<GlobalEventMessage> event_queue_;

  // Statically allocated structure for calling the event handlers from the
  // main event queue.
  EventReport main_event_report_;

  // Each allocated GlobalEventMessage holds one value in this semaphore.
  OSSem pending_sem_;
};

class EventIterator {
 public:


};

GlobalEventFlow::GlobalEventFlow(Executor* executor, int max_event_slots)
    : ControlFlow(executor, CrashNotifiable::DefaultInstance()), impl_(new Impl(max_event_slots)) {
  StartFlowAt(ST(WaitForEvent));
}

ControlFlow::ControlFlowAction GlobalEventFlow::WaitForEvent() {
  return Allocate(&impl_->event_queue_, ST(HandleEvent));
}

void DecodeRange(EventReport* r) {
  uint64_t e = r->event;
  if (e&1) {
    r->mask = (e ^ (e+1)) >> 1;
  } else {
    r->mask = (e ^ (e-1)) >> 1;
  }
  r->event &= ~r->mask;
}

ControlFlow::ControlFlowAction GlobalEventFlow::HandleEvent() {
  GlobalEventMessage* m;
  GetAllocationResult(&m);
  EventReport* rep = &impl_->main_event_report_;
  rep->src_node = m->src_node;
  rep->dst_node = m->dst_node;
  rep->event = m->event;
  rep->mask = EVENT_EXACT_MASK;
  FreeMessage(m);

  EventHandlerFunction fn;
  switch (m->mti) {
    case MTI_EVENT_REPORT:
      fn = &NMRAnetEventHandler::HandleEventReport;
      break;
    case MTI_CONSUMER_IDENTIFY:
      fn = &NMRAnetEventHandler::HandleIdentifyConsumer;
      break;
    case MTI_CONSUMER_IDENTIFIED_RANGE:
      DecodeRange(rep);
      fn = &NMRAnetEventHandler::HandleConsumerRangeIdentified;
      break;
    case MTI_CONSUMER_IDENTIFIED_UNKNOWN:
      rep->state = UNKNOWN;
      fn = &NMRAnetEventHandler::HandleConsumerIdentified;
      break;
    case MTI_CONSUMER_IDENTIFIED_VALID:
      rep->state = VALID;
      fn = &NMRAnetEventHandler::HandleConsumerIdentified;
      break;
    case MTI_CONSUMER_IDENTIFIED_INVALID:
      rep->state = INVALID;
      fn = &NMRAnetEventHandler::HandleConsumerIdentified;
      break;
    case MTI_CONSUMER_IDENTIFIED_RESERVED:
      rep->state = RESERVED;
      fn = &NMRAnetEventHandler::HandleConsumerIdentified;
      break;
    case MTI_PRODUCER_IDENTIFY:
      fn = &NMRAnetEventHandler::HandleIdentifyProducer;
      break;
    case MTI_PRODUCER_IDENTIFIED_RANGE:
      DecodeRange(rep);
      fn = &NMRAnetEventHandler::HandleProducerRangeIdentified;
      break;
    case MTI_PRODUCER_IDENTIFIED_UNKNOWN:
      rep->state = UNKNOWN;
      fn = &NMRAnetEventHandler::HandleProducerIdentified;
      break;
    case MTI_PRODUCER_IDENTIFIED_VALID:
      rep->state = VALID;
      fn = &NMRAnetEventHandler::HandleProducerIdentified;
      break;
    case MTI_PRODUCER_IDENTIFIED_INVALID:
      rep->state = INVALID;
      fn = &NMRAnetEventHandler::HandleProducerIdentified;
      break;
    case MTI_PRODUCER_IDENTIFIED_RESERVED:
      rep->state = RESERVED;
      fn = &NMRAnetEventHandler::HandleProducerIdentified;
      break;
    case MTI_EVENTS_IDENTIFY_ADDRESSED:
    case MTI_EVENTS_IDENTIFY_GLOBAL:
      fn = &NMRAnetEventHandler::HandleIdentifyGlobal;
      break;
    default:
      DIE("Unexpected message arrived at the global event handler.");
  } //    case
  NMRAnetEventHandler* h = nullptr;
  (h->*fn)(rep, this);
  return YieldAndCall(ST(WaitForEvent));
}

GlobalEventMessage* GlobalEventFlow::AllocateMessage() {
  impl_->pending_sem_.wait();
  return new GlobalEventMessage();
}

void GlobalEventFlow::PostEvent(GlobalEventMessage* message) {
  impl_->event_queue_.TypedRelease(message);
}

void GlobalEventFlow::FreeMessage(GlobalEventMessage* m) {
  delete m;
  impl_->pending_sem_.post();
}

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param node node that the packet is addressed to
 * @param data NMRAnet packet data
 */
extern "C" void nmranet_event_packet_addressed(uint16_t mti,
                                               node_handle_t src,
                                               node_t node,
                                               const void* data) {
  /*struct id_node* id_node = node;
  if (id_node->priv->state == NODE_UNINITIALIZED) {
    return;
    }*/

  GlobalEventMessage* m = GlobalEventFlow::instance->AllocateMessage();
  m->mti = mti;
  m->dst_node = node;
  m->event = 0;
  if (data) {
    memcpy(&m->event, data, sizeof(uint64_t));
    m->event = be64toh(m->event);
  }
  GlobalEventFlow::instance->PostEvent(m);

  switch (mti) {
    default:
      break;
    case MTI_CONSUMER_IDENTIFY:
      //nmranet_identify_consumers(node, event, EVENT_EXACT_MASK);
      break;
    case MTI_CONSUMER_IDENTIFIED_RANGE:
      //nmranet_identify_consumers(node, event, identify_range_mask(event));
      break;
    case MTI_CONSUMER_IDENTIFIED_UNKNOWN: /* fall through */
    case MTI_CONSUMER_IDENTIFIED_VALID:   /* fall through */
    case MTI_CONSUMER_IDENTIFIED_INVALID: /* fall through */
    case MTI_CONSUMER_IDENTIFIED_RESERVED:
      break;
    case MTI_PRODUCER_IDENTIFY:
      //nmranet_identify_producers(node, event, EVENT_EXACT_MASK);
      break;
    case MTI_PRODUCER_IDENTIFIED_RANGE:
      //nmranet_identify_producers(node, event, identify_range_mask(event));
      break;
    case MTI_PRODUCER_IDENTIFIED_UNKNOWN: /* fall through */
    case MTI_PRODUCER_IDENTIFIED_VALID:   /* fall through */
    case MTI_PRODUCER_IDENTIFIED_INVALID: /* fall through */
    case MTI_PRODUCER_IDENTIFIED_RESERVED:
      break;
    case MTI_EVENTS_IDENTIFY_ADDRESSED: /* fall through */
    case MTI_EVENTS_IDENTIFY_GLOBAL:
      //nmranet_identify_consumers(node, 0, EVENT_ALL_MASK);
      //nmranet_identify_producers(node, 0, EVENT_ALL_MASK);
      break;
  }
}

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param src source Node ID
 * @param data NMRAnet packet data
 */
void nmranet_event_packet_global(uint16_t mti,
                                 node_handle_t src,
                                 const void* data) {
  GlobalEventMessage* m = GlobalEventFlow::instance->AllocateMessage();
  m->mti = mti;
  m->dst_node = nullptr;
  m->src_node = src;
  m->event = 0;
  if (data) {
    memcpy(&m->event, data, sizeof(uint64_t));
    m->event = be64toh(m->event);
  }
  GlobalEventFlow::instance->PostEvent(m);

  switch (mti) {
    default:
      break;
    case MTI_EVENT_REPORT: {
      /* to save processing time in instantiations that include a large
       * number of nodes, consumers are sorted at the event level and
       * not at the node level.
       */
      /*struct event_node* event_node;
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
        }*/
      break;
    }
    case MTI_CONSUMER_IDENTIFY:
    /* fall through */
    case MTI_CONSUMER_IDENTIFIED_RANGE:
    /* fall through */
    case MTI_CONSUMER_IDENTIFIED_UNKNOWN:
    /* fall through */
    case MTI_CONSUMER_IDENTIFIED_VALID:
    /* fall through */
    case MTI_CONSUMER_IDENTIFIED_INVALID:
    /* fall through */
    case MTI_CONSUMER_IDENTIFIED_RESERVED:
    /* fall through */
    case MTI_PRODUCER_IDENTIFY:
    /* fall through */
    case MTI_PRODUCER_IDENTIFIED_RANGE:
    /* fall through */
    case MTI_PRODUCER_IDENTIFIED_UNKNOWN:
    /* fall through */
    case MTI_PRODUCER_IDENTIFIED_VALID:
    /* fall through */
    case MTI_PRODUCER_IDENTIFIED_INVALID:
    /* fall through */
    case MTI_PRODUCER_IDENTIFIED_RESERVED:
    /* fall through */
    case MTI_EVENTS_IDENTIFY_GLOBAL:
      //      os_mutex_lock(&nodeMutex);
      /* global message, deliver all, non-subscribe */
      /*      for (node_t node = nmranet_node_next(NULL); node != NULL;
           node = nmranet_node_next(node)) {
        nmranet_event_packet_addressed(mti, node, data);
      }
      os_mutex_unlock(&nodeMutex);*/
      break;
  }
}
