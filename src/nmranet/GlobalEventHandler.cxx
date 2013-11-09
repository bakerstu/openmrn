//#define LOGLEVEL VERBOSE
#include "utils/logging.h"

#include <algorithm>
#include <vector>
#include <endian.h>

#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"
#include "if/nmranet_if.h"
#include "core/nmranet_event.h"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/EventManager.hxx"

/*static*/
GlobalEventFlow* GlobalEventFlow::instance = nullptr;

struct GlobalEventFlow::Impl {
  Impl(int max_event_slots) : pending_sem_(max_event_slots) {}

  // This is the queue of events that are coming from the read thread to the
  // handler thread. Every "released" event is a new incoming event message.
  TypedAllocator<GlobalEventMessage> event_queue_;

  // This is the queue of global identify events.
  TypedAllocator<GlobalEventMessage> global_event_queue_;

  // Incoming event message, as it got off the event_queue.
  GlobalEventMessage* message_;

  // Statically allocated structure for calling the event handlers from the
  // main event queue.
  EventReport main_event_report_;

  // Each allocated GlobalEventMessage holds one value in this semaphore.
  OSSem pending_sem_;

  // The implementation of the iterators.
  std::unique_ptr<NMRAnetEventHandler> handler_;
};

GlobalEventFlow::GlobalEventFlow(Executor* executor, int max_event_slots)
    : ControlFlow(executor, CrashNotifiable::DefaultInstance()),
      impl_(new Impl(max_event_slots)) {
  GlobalEventFlow::instance = this;
  impl_->handler_.reset(new VectorEventHandlers(executor));
  StartFlowAt(ST(WaitForEvent));
}

GlobalEventFlow::~GlobalEventFlow() {
  GlobalEventFlow::instance = nullptr;
}

//! Returns true if there are outstanding events that are not yet handled.
bool GlobalEventFlow::EventProcessingPending() {
  if (!this) return false;
  // TODO(balazs.racz): maybe the order of these checks should be different.
  if (IsPendingOrRunning()) return true;
  if (next_state() != ST(HandleEventArrived)) return true;
  if (impl_->event_queue_.Peek()) return true;
  return false;
}

ControlFlow::ControlFlowAction GlobalEventFlow::WaitForEvent() {
  LOG(VERBOSE, "GlobalFlow::WaitForEvent");
  return Allocate(&impl_->event_queue_, ST(HandleEventArrived));
}

ControlFlow::ControlFlowAction GlobalEventFlow::HandleEventArrived() {
  LOG(VERBOSE, "GlobalFlow::EventArrived");
  GetAllocationResult(&impl_->message_);
  return Allocate(&event_handler_mutex, ST(HandleEvent));
}

void DecodeRange(EventReport* r) {
  uint64_t e = r->event;
  if (e & 1) {
    r->mask = (e ^ (e + 1)) >> 1;
  } else {
    r->mask = (e ^ (e - 1)) >> 1;
  }
  r->event &= ~r->mask;
}

ControlFlow::ControlFlowAction GlobalEventFlow::HandleEvent() {
  LOG(VERBOSE, "GlobalFlow::HandleEvent");
  EventReport* rep = &impl_->main_event_report_;
  rep->src_node = impl_->message_->src_node;
  rep->dst_node = impl_->message_->dst_node;
  rep->event = impl_->message_->event;
  rep->mask = 1;

  EventHandlerFunction fn;
  switch (impl_->message_->mti) {
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
  }  //    case
  FreeMessage(impl_->message_);
  impl_->message_ = nullptr;

  (impl_->handler_.get()->*fn)(rep, this);
  // We insert an intermediate state to consume any pending notifications.
  return WaitAndCall(ST(HandlerFinished));
}

ControlFlow::ControlFlowAction GlobalEventFlow::WaitForHandler() {
  return WaitAndCall(ST(HandlerFinished));
}

ControlFlow::ControlFlowAction GlobalEventFlow::HandlerFinished() {
  LOG(VERBOSE, "GlobalFlow::HandlerFinished");
  impl_->message_ = impl_->event_queue_.TypedAllocateOrNull();
  if (impl_->message_) {
    return CallImmediately(ST(HandleEvent));
  }
  // No pending message in the queue: releases the mutex and allows global
  // handlers to proceed.
  event_handler_mutex.Unlock();
  return CallImmediately(ST(WaitForEvent));
}

GlobalEventMessage* GlobalEventFlow::AllocateMessage() {
  impl_->pending_sem_.wait();
  return new GlobalEventMessage();
}

void GlobalEventFlow::PostEvent(GlobalEventMessage* message) {
  impl_->event_queue_.TypedReleaseBack(message);
}

void GlobalEventFlow::FreeMessage(GlobalEventMessage* m) {
  delete m;
  impl_->pending_sem_.post();
}

#ifdef CPP_EVENT_HANDLER

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param node node that the packet is addressed to
 * @param data NMRAnet packet data
 */
extern "C" {

void nmranet_event_packet_addressed(uint16_t mti,
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
  m->src_node = src;
  m->event = 0;
  if (data) {
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

// This is a trick we play on the linker to pull in these symbols. Otherwise we
// won't be able to link the binary, since there are back-references to these
// symbols from lower-level libraries.
void (*unused_f)(node_t, uint64_t, uint64_t)=&nmranet_identify_consumers;
void (*unused_g)(node_t, uint64_t, uint64_t)=&nmranet_identify_producers;

} // extern C

#endif // CPP_EVENT_HANDLER
