#ifndef _NMRANET_EVENTMANAGER_HXX_
#define _NMRANET_EVENTMANAGER_HXX_

#include <algorithm>
#include <vector>
#include <endian.h>

#ifndef LOGLEVEL
#define LOGLEVEL VERBOSE
#endif

#include "executor/control_flow.hxx"
#include "utils/logging.h"
//#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"
//#include "if/nmranet_if.h"
#include "core/nmranet_event.h"
#include "nmranet/EventHandlerTemplates.hxx"

namespace NMRAnet {

// Abstract class for representing iteration through a container for event
// handlers.
class EventIterator : public ProxyNotifiable {
 public:
  //! Creates an EventIterator.
  //
  //! @param parent is a Notifiable. Any notifications coming into the
  //! Notifiables created by NewCallback will be forwarded to this callback.
  EventIterator() : event_(nullptr), done_(nullptr) {}

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

 protected:
  //! The current iterator.
  IT it_;
  //! The end of the iteration.
  IT end_;
};

template <class IT>
class StraightStlIterator : public StlIterator<IT> {
 public:
  //! Returns the next iterated entry.
  virtual NMRAnetEventHandler* NextEntry() {
    if (this->it_ == this->end_)
      return NULL;
    else
      return *(this->it_)++;
  }
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
  DualIteratorFlow(Executor* executor, EventIterator* standard_iterator,
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
  virtual void HandlerFn(EventHandlerFunction fn, EventReport* event,
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
    if (!standard_iterator_->HasBeenNotified()) return WaitForNotification();
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
      return YieldAndCall(ST(StateInIteration));
    }
    handler->HandleIdentifyGlobal(global_iterator_->event(),
                                  global_iterator_->NewCallback(this));
    return CallImmediately(ST(WaitForGlobalReturn));
  }

  ControlFlowAction WaitForGlobalReturn() {
    if (!global_iterator_->HasBeenNotified()) return WaitForNotification();
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

class VectorEventHandlers : public DualIteratorFlow,
                            public NMRAnetEventRegistry {
 public:
  VectorEventHandlers(Executor* executor)
      : DualIteratorFlow(executor, &standard_iterator_impl_,
                         &global_iterator_impl_) {}

  virtual void InitStandardIterator() {
    standard_iterator_impl_.Set(handlers_.begin(), handlers_.end());
  }

  virtual void InitGlobalIterator() {
    global_iterator_impl_.Set(handlers_.begin(), handlers_.end());
  }

  virtual void RegisterHandler(NMRAnetEventHandler* handler, EventId event,
                               unsigned mask) {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.push_back(handler);
  }
  virtual void UnregisterHandler(NMRAnetEventHandler* handler, EventId event,
                                 unsigned mask) {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.erase(std::remove(handlers_.begin(), handlers_.end(), handler));
  }

  virtual NMRAnetEventHandler* EventHandler() { return this; }

 private:
  typedef std::vector<NMRAnetEventHandler*> HandlersList;
  HandlersList handlers_;
  StraightStlIterator<HandlersList::iterator> standard_iterator_impl_;
  StraightStlIterator<HandlersList::iterator> global_iterator_impl_;
};

}; /* namespace NMRAnet */

#endif  // _NMRANET_EVENTMANAGER_HXX_
