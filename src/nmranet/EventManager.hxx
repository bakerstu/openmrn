#ifndef _NMRANET_EVENTMANAGER_HXX_
#define _NMRANET_EVENTMANAGER_HXX_

#include <algorithm>
#include <vector>
#include <endian.h>

#ifndef LOGLEVEL
#define LOGLEVEL VERBOSE
#endif

#include "utils/logging.h"
//#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"
//#include "if/nmranet_if.h"
//#include "core/nmranet_event.h"
#include "nmranet/EventHandlerTemplates.hxx"

namespace NMRAnet
{

template<class C> class FullContainerIterator : public EventIterator {
public:
    FullContainerIterator(C* container)
        : container_(container) {
        clear_iteration();
    }
    NMRAnetEventHandler* next_entry() OVERRIDE {
        if (it_ == container_->end()) return nullptr;
        NMRAnetEventHandler* h = *it_;
        ++it_;
        return h;
    }
    void clear_iteration() OVERRIDE {
        it_ = container_->end();
    }
    void init_iteration(EventReport*) {
        it_ = container_->begin();
    }

private:
    typename C::iterator it_;
    C* container_;
};

class VectorEventHandlers : public NMRAnetEventRegistry {
 public:
    VectorEventHandlers() {}

    // Creates a new event iterator. Caller takes ownership of object.
    EventIterator* create_iterator() OVERRIDE {
        return new FullContainerIterator<HandlersList>(&handlers_);
    }

  virtual void register_handler(NMRAnetEventHandler* handler, EventId event, unsigned mask) {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.push_back(handler);
  }
  virtual void unregister_handler(NMRAnetEventHandler* handler, EventId event, unsigned mask) {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.erase(std::remove(handlers_.begin(), handlers_.end(), handler));
  }
  
 private:
  typedef std::vector<NMRAnetEventHandler*> HandlersList;
  HandlersList handlers_;
};

}; /* namespace NMRAnet */

#endif  // _NMRANET_EVENTMANAGER_HXX_
