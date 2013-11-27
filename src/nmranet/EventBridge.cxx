// This file contains the functions bridging the C and C++ API for events. The
// three main components are:
//
// * Glue code that receives incoming messages from the IF layer (written in
//   C).
//
// * C++ implementation of the event dispatch functionality.
//
// * Implementation of the C interface towards the application.

#include "executor/control_flow.hxx"

#include <algorithm>

#include "os/OS.hxx"
#include "core/nmranet_event.h"
#include "nmranet/EventHandlerTemplates.hxx"

#define EXTERNC extern "C" {
#define EXTERNCEND }

using namespace NMRAnet;

// ============== Global event registry ===============

class EventVectorRegistry : public ProxyEventHandler {
 public:
  EventVectorRegistry(Executor* executor, int iterator_count)
      : executor_(executor),
        pending_iterators_(0),
        global_event_iterator_(this) {
    for (int i = 0; i < iterator_count; i++) {
      event_iterators_.TypedRelease(new AllocatedIteratedCall(this));
    }
  }
  virtual ~EventVectorRegistry() {}

  virtual void HandlerFn(EventHandlerFunction fn,
                         EventReport* event,
                         Notifiable* done) {
    TypedSyncAllocation<AllocatedIteratedCall> a(&event_iterators_);
    a.result()->RunFlow(fn, event, done);
  }

  virtual void HandleIdentifyGlobal(EventReport* event, Notifiable* done) {
    // If an identify global shows up, we reset the handling iterator to zero
    // and start it.
    global_event_iterator_.RunFlow(
        &NMRAnetEventHandler::HandleIdentifyGlobal, event, done);
  }

  void RegisterHandler(NMRAnetEventHandler* handler) {
    OSMutexLock l(&lock_);
    handlers_.push_back(handler);
  }

  void UnregisterHandler(NMRAnetEventHandler* handler) {
    OSMutexLock l(&lock_);
    // Since we cannot invalidate iterators, we have to wait for them to
    // finish.
    while (pending_iterators_ > 0) {
      lock_.unlock();
      usleep(10);
      lock_.lock();
    }
    handlers_.erase(std::remove(handlers_.begin(), handlers_.end(), handler),
                    handlers_.end());
  }

 private:
  typedef vector<NMRAnetEventHandler*> HandlerList;

  friend class IteratedCall;

  class IteratedCall : public ControlFlow {
   public:
    IteratedCall(EventVectorRegistry* parent)
        : ControlFlow(parent->executor_, NULL), parent_(parent) {}

    void RunFlow(EventHandlerFunction fn,
                 EventReport* event,
                 Notifiable* done) {
      Restart(done);
      fn_ = fn;
      event_ = event;
      {
        OSMutexLock l(&parent_->lock_);
        it_ = parent_->handlers_.begin();
        if (IsDone() || IsNotStarted())
          ++parent_->pending_iterators_;
      }
      StartFlowAt(ST(MainIteration));
    }

    ControlFlowAction MainIteration() {
      NMRAnetEventHandler* handler = nullptr;
      {
        OSMutexLock l(&parent_->lock_);
        if (it_ == parent_->handlers_.end()) {
          --parent_->pending_iterators_;
        } else {
          handler = *it_;
          it_++;
        }
      }
      if (handler) {
        (handler->*fn_)(event_, this);
      } else {
        return VExit();
      }
      return WaitForNotification();
    }

   protected:
    EventVectorRegistry* parent_;
    virtual ControlFlowAction VExit() { return Exit(); }

   private:
    HandlerList::const_iterator it_;
    EventHandlerFunction fn_;
    EventReport* event_;
  };

  class AllocatedIteratedCall : public IteratedCall {
   public:
    AllocatedIteratedCall(EventVectorRegistry* parent) : IteratedCall(parent) {}

   protected:
    virtual ControlFlowAction VExit() {
      return ReleaseAndExit(&parent_->event_iterators_, this);
    }
  };

  Executor* executor_;
  HandlerList handlers_;
  int pending_iterators_;
  OSMutex lock_;

  TypedAllocator<AllocatedIteratedCall> event_iterators_;
  IteratedCall global_event_iterator_;
};

// ================= Callbacks from interface core ===============

EXTERNC


EXTERNCEND

// =================== Implementation of the legacy interface ==============

EXTERNC

EXTERNCEND
