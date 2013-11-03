#include <algorithm>
#include <vector>
#include <endian.h>

#define LOGLEVEL VERBOSE

#include "utils/logging.h"
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

// Abstract class for representing iteration through a container for event
// handlers.
class EventIterator : public ProxyNotifiable {
 public:
  //! Creates an EventIterator.
  //
  //! @param parent is a Notifiable. Any notifications coming into the
  //! Notifiables created by NewCallback will be forwarded to this callback.
  EventIterator()
      : event_(nullptr), done_(nullptr) {}

  //! Steps the iteration.
  //
  //! @returns the next entry or NULL if the iteration is done.
  //
  //! May be called many times after the iteratin is ended and should
  //! consistently return NULL.
  virtual NMRAnetEventHandler* NextEntry() = 0;

  //! Sets up variables used during iteration.
  //
  //! @param event is the EventReport that will be used for this iteration.
  //
  //! @param done is the Notifiable that shall be called when the iteration is
  //! completed.
  void InitIterator(EventReport* event, ::Notifiable* done) {
    HASSERT(event_ == nullptr);
    HASSERT(done_ == nullptr);
    event_ = event;
    done_ = done;
  }

  //! Resets iteration variables.
  void ClearIteration() {
    event_ = nullptr;
    done_ = nullptr;
  }

  //! @returns the EventReport structure used for this iteration.
  EventReport* event() { return event_; }

  //! @returns the callback to be called when the iteration is completed.
  ::Notifiable* done() { return done_; }

 private:
  // Stores iteration arguments.
  EventReport* event_;
  // Stores the iteration completion callback.
  ::Notifiable* done_;
  // Proxies incoming notifications to this parent.
  ::Notifiable* parent_;
  // When there is an incoming notification, this is set to true.
  bool was_notified_;
};

//! An implementation of the EventIterator abstract class that iterates through
//! an STL container's range by the begin-end iterators.
template <class IT>
class StlIterator : public EventIterator {
 public:
  StlIterator() {}

  //! Initializes the iteration with begin-end iterator pair.
  void Set(IT begin, IT end) {
    it_ = begin;
    end_ = end;
  }

  //! Returns the next iterated entry.
  virtual NMRAnetEventHandler* NextEntry() {
    if (it_ == end_)
      return NULL;
    else
      return *it_++;
  }

 private:
  //! The current iterator.
  IT it_;
  //! The end of the iteration.
  IT end_;
};


/**
 *  This control flow performs two iterations over event handlers in
 *  parallel. There isa high-priority iteration for regular incoming event
 *  calls, and there is a low-priority iteration for global identify
 *  calls. While there is any high-priority iteration, the low-priority
 *  iteration is stopped.
 * 
 *  LOCKING: 
 * 
 *  At any point in time there can be only one EventHandler child called. This
 *  is enforced by locking the event_handler_mutex when a child is
 *  called. All our incoming calls are also with this lock held.
 * 
 *  The high-priority iteration progresses without ever releasing the lock.
 *
 *  The low-priority iteration releases the mutex after every entry, and then
 *  immediately tries to reacquire it (thereby yielding to the back of the
 *  mutex's queue). This means that between any two children being called on
 *  the low-pri iteration and entire high-pri iteration will complete.
 *
 */
class DualIteratorFlow : public ControlFlow, public ProxyEventHandler {
 public:
  DualIteratorFlow(Executor* executor,
                   EventIterator* standard_iterator,
                   EventIterator* global_iterator)
      : ControlFlow(executor, NULL),
        standard_iterator_(standard_iterator),
        global_iterator_(global_iterator) {
    StartFlowAt(ST(StateInIteration));
  }

  //! Implementations have to override this to re-set the standard_iterator
  //! from the information in standard_iterator->event().
  virtual void InitStandardIterator() = 0;

  //! Handler function for high-priority event calls. Starts a
  //! StandardIterator.
  //
  //! This function is called (by the ProxyEventHandler) for any event handler
  //! functions we didn't override explicitly, that is, everything except
  //! Identify Global.
  virtual void HandlerFn(EventHandlerFunction fn,
                         EventReport* event,
                         Notifiable* done) {
    LOG(VERBOSE, "Dual::Handler, event %016llx", event->event);
    event_handler_mutex.AssertLocked();
    proxy_fn_ = fn;
    standard_iterator_->InitIterator(event, done);
    InitStandardIterator();
    this->Notify();
  }

  //! Implementations have to override this to re-set the global_iterator
  //! based on the information in global_iterator->event().
  virtual void InitGlobalIterator() = 0;

  virtual void HandleIdentifyGlobal(EventReport* event, Notifiable* done) {
    LOG(VERBOSE, "Dual::IdentifyGlobal");
    event_handler_mutex.AssertLocked();
    // We immediately release the global iterator mutex and will acquire it
    // inside when we try to call children.
    event_handler_mutex.Unlock();
    global_iterator_->InitIterator(event, done);
    InitGlobalIterator();
    this->Notify();
  }

 protected:
  ControlFlowAction StateInIteration() {
    // We first try a standard iteration.
    if (standard_iterator_->done())
      return CallImmediately(ST(TryStandardIteration));
    // Then a global iteration.
    if (global_iterator_->done())
      return CallImmediately(ST(GetGlobalIterationLock));
    // Give up and wait for incoming messages.
    return WaitForNotification();
  }

  ControlFlowAction TryStandardIteration() {
    NMRAnetEventHandler* handler = standard_iterator_->NextEntry();
    if (!handler) {
      // Iteration done.
      LOG(VERBOSE, "Standard iteration done");
      Notifiable* d = standard_iterator_->done();
      HASSERT(d);
      standard_iterator_->ClearIteration();
      d->Notify();
      return YieldAndCall(ST(StateInIteration));
    }
    (handler->*proxy_fn_)(standard_iterator_->event(),
                          standard_iterator_->NewCallback(this));
    return CallImmediately(ST(WaitForStandardReturn));
  }

  ControlFlowAction WaitForStandardReturn() {
    LOG(VERBOSE, "Standard entry return");
    if (!standard_iterator_->HasBeenNotified())
      return WaitForNotification();
    return CallImmediately(ST(TryStandardIteration));
  }

  ControlFlowAction GetGlobalIterationLock() {
    return Allocate(&event_handler_mutex, ST(CallGlobalIteration));
  }

  ControlFlowAction CallGlobalIteration() {
    NMRAnetEventHandler* handler = global_iterator_->NextEntry();
    if (!handler) {
      // Iteration done. We hand back the iterator lock.
      Notifiable* d = global_iterator_->done();
      HASSERT(d);
      global_iterator_->ClearIteration();
      d->Notify();
      global_iterator_->done()->Notify();
      return YieldAndCall(ST(StateInIteration));
    }
    handler->HandleIdentifyGlobal(global_iterator_->event(),
                                  global_iterator_->NewCallback(this));
    return CallImmediately(ST(WaitForGlobalReturn));
  }

  ControlFlowAction WaitForGlobalReturn() {
    if (!global_iterator_->HasBeenNotified())
      return WaitForNotification();
    // Releases the iteration lock and yields to other control flows that might
    // want it.
    event_handler_mutex.Unlock();
    return YieldAndCall(ST(StateInIteration));
  }

 private:
  //! Holds which event handler function the standard iteration should call.
  EventHandlerFunction proxy_fn_;
  EventIterator* standard_iterator_;
  EventIterator* global_iterator_;
};

class VectorEventHandlers : public DualIteratorFlow, public NMRAnetEventRegistry {
 public:
  VectorEventHandlers(Executor* executor)
      : DualIteratorFlow(executor,
                         &standard_iterator_impl_,
                         &global_iterator_impl_) {}

  virtual void InitStandardIterator() {
    standard_iterator_impl_.Set(handlers_.begin(), handlers_.end());
  }

  virtual void InitGlobalIterator() {
    global_iterator_impl_.Set(handlers_.begin(), handlers_.end());
  }

  virtual void RegisterHandler(NMRAnetEventHandler* handler, EventId event, unsigned mask) {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.push_back(handler);
  }
  virtual void UnregisterHandler(NMRAnetEventHandler* handler, EventId event, unsigned mask) {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.erase(std::remove(handlers_.begin(), handlers_.end(), handler));
  }
  
  virtual NMRAnetEventHandler* EventHandler() {
    return this;
  }

 private:
  typedef std::vector<NMRAnetEventHandler*> HandlersList;
  HandlersList handlers_;
  StlIterator<HandlersList::iterator> standard_iterator_impl_;
  StlIterator<HandlersList::iterator> global_iterator_impl_;
};

GlobalEventFlow::GlobalEventFlow(Executor* executor, int max_event_slots)
    : ControlFlow(executor, CrashNotifiable::DefaultInstance()),
      impl_(new Impl(max_event_slots)) {
  impl_->handler_.reset(new VectorEventHandlers(executor));
  StartFlowAt(ST(WaitForEvent));
}

GlobalEventFlow::~GlobalEventFlow() {}


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
  rep->mask = EVENT_EXACT_MASK;

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

  (impl_->handler_.get()->*fn)(rep, this);
  // We insert an intermediate state to consume any pending notifications.
  return WaitAndCall(ST(HandlerFinished));
}

ControlFlow::ControlFlowAction GlobalEventFlow::WaitForHandler() {
  return WaitAndCall(ST(HandlerFinished));
}

ControlFlow::ControlFlowAction GlobalEventFlow::HandlerFinished() {
  LOG(VERBOSE, "GlobalFlow::HandlerFinished");
  // @TODO(balazs.racz): When there is a new event in the main event queue, we
  // shouldn't release the mutex but go straight to the acquisition.
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


#endif // CPP_EVENT_HANDLER
