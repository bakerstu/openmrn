/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file EventCompatibility.hxx
 *
 * Compatibility layer for the C-style event handler interface.
 *
 * @author Balazs Racz
 * @date 3 Nov 2013
 */

#include "utils/logging.h"

#include <map>
#include <algorithm>

#include "nmranet/EventCompatibility.hxx"

#include "nmranet_config.h"
#include "core/nmranet_event.h"
#include "core/nmranet_node_private.h"
#include "if/nmranet_if.h"
#include "nmranet/EventManager.hxx"
#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"

#ifdef CPP_EVENT_HANDLER

class CompatEventHandler;

void EnsureCompatEventHandlerExists();

typedef std::map<std::pair<uint64_t, node_t>, CompatEventHandler> HandlerMap;

HandlerMap g_map;

HandlerMap::iterator current_standard_iterator;
HandlerMap::iterator current_global_iterator;

class CompatEventHandler : public SimpleEventHandler {
 private:
  node_t current_node(const HandlerMap::iterator* it) {
    return (*it)->first.second;
  }

  uint64_t current_eventid(const HandlerMap::iterator* it) {
    return (*it)->first.first;
  }

 public:
  CompatEventHandler()
      : has_producer_(0),
        producer_state_(EVENT_STATE_RESERVED),
        has_consumer_(0),
        consumer_state_(EVENT_STATE_RESERVED) {}

  void HandleEventReport(EventReport* event, Notifiable* done) {
    event_post(current_node(&current_standard_iterator), event->src_node,
               current_eventid(&current_standard_iterator));
    done->Notify();
  }

  void HandleIdentifyGlobal(EventReport* event, Notifiable* done) {
    event_barrier.Reset(done);
    if (has_producer_) {
      event_write_helper1.WriteAsync(
          current_node(&current_global_iterator),
          MTI_PRODUCER_IDENTIFIED_VALID + producer_state_,
          WriteHelper::Global(),
          EventIdToBuffer(current_eventid(&current_global_iterator)),
          event_barrier.NewChild());
    }
    if (has_consumer_) {
      event_write_helper2.WriteAsync(
          current_node(&current_global_iterator),
          MTI_CONSUMER_IDENTIFIED_VALID + consumer_state_,
          WriteHelper::Global(),
          EventIdToBuffer(current_eventid(&current_global_iterator)),
          event_barrier.NewChild());
    }
    event_barrier.MaybeDone();
  }

 public:
  unsigned has_producer_ : 1;
  unsigned producer_state_ : 2;
  unsigned has_consumer_ : 1;
  unsigned consumer_state_ : 2;
};

void nmranet_event_consumer(node_t node, uint64_t event, unsigned int state) {
  EnsureCompatEventHandlerExists();
  auto& v = g_map[make_pair(event, node)];
  v.has_consumer_ = 1;
  v.consumer_state_ = state & 3;
}

void nmranet_event_producer(node_t node, uint64_t event, unsigned int state) {
  EnsureCompatEventHandlerExists();
  auto& v = g_map[make_pair(event, node)];
  v.has_producer_ = 1;
  v.producer_state_ = state & 3;
}

void nmranet_event_produce(node_t node, uint64_t event, unsigned int state) {
  state &= 3;
  HandlerMap::iterator it = g_map.find(make_pair(event, node));
  if (it == g_map.end()) return;
  auto& v = it->second;
  if (!v.has_producer_) return;
  v.producer_state_ = state & 3;
  if (v.producer_state_ != EVENT_STATE_VALID) return;
  HASSERT(!nmranet_node_write(node, MTI_EVENT_REPORT, WriteHelper::Global(),
                              EventIdToBuffer(event)));
}

void nmranet_identify_consumers(node_t node, uint64_t event, uint64_t mask) {
  // Ignored: we'll do the global identify in IdentifyProducers.
}

void nmranet_identify_producers(node_t node, uint64_t event, uint64_t mask) {
  nmranet_event_packet_global(MTI_EVENTS_IDENTIFY_GLOBAL, {0, 0}, NULL);
}

// This is a trick we play on the linker to pull in these symbols. Otherwise we
// won't be able to link the binary, since there are back-references to these
// symbols from lower-level libraries.
void (*unused_h)(uint16_t, node_handle_t, node_t, const void*) =
    &nmranet_event_packet_addressed;

class CompatEventIterator : public StlIterator<HandlerMap::iterator> {
 public:
  CompatEventIterator(HandlerMap::iterator* exp) : export_(exp) {}

  //! Returns the next iterated entry.
  virtual NMRAnetEventHandler* NextEntry() {
    if (it_ == end_)
      return NULL;
    else {
      *export_ = it_++;
      return &(*export_)->second;
    }
  }

 private:
  HandlerMap::iterator* export_;
};

/** This class allows to route the incoming HandleEvent calls to the proper
 * event handler.
 */
class CompatEventManager : public DualIteratorFlow {
 public:
  CompatEventManager(Executor* executor)
      : DualIteratorFlow(executor, &std_, &global_),
        std_(&current_standard_iterator),
        global_(&current_global_iterator) {
    NMRAnetEventRegistry::instance()->RegisterHandler(this, 0, 0);
  }

  ~CompatEventManager() {
    NMRAnetEventRegistry::instance()->UnregisterHandler(this, 0, 0);
  }

  virtual void InitStandardIterator() {
    std_.Set(g_map.lower_bound(make_pair(std_.event()->event, (node_t)0)),
             g_map.upper_bound(make_pair(
                 std_.event()->event + std_.event()->mask - 1, (node_t) - 1)));
  }

  virtual void InitGlobalIterator() { global_.Set(g_map.begin(), g_map.end()); }

 private:
  CompatEventIterator std_;
  CompatEventIterator global_;
};


static EventCompatibilityLayer* g_compat_layer = nullptr;
Lockable g_compat_layer_lock;
Executor* g_event_thread = nullptr;
CompatEventManager* g_compat_event_manager = nullptr;

EventCompatibilityLayer::EventCompatibilityLayer() {
  g_compat_layer = this;
  if (!GlobalEventFlow::instance) {
    g_event_thread = new ThreadExecutor("global_event", 0, 1000);
    new GlobalEventFlow(g_event_thread, 10);
  }
  g_compat_event_manager = new CompatEventManager(g_event_thread);
}

EventCompatibilityLayer::~EventCompatibilityLayer() {
  delete g_compat_event_manager;
  g_compat_event_manager = nullptr;
  g_compat_layer = nullptr;
}

void EnsureCompatEventHandlerExists() {
  if (!g_compat_layer) {
    LockHolder l(&g_compat_layer_lock);
    if (g_compat_layer) return;
    new EventCompatibilityLayer();
  }
}

#endif
