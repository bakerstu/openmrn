#ifndef _NMRANET_EVENTMANAGER_HXX_
#define _NMRANET_EVENTMANAGER_HXX_

#include <algorithm>
#include <vector>
#include <forward_list>
#include <endian.h>

#ifndef LOGLEVEL
#define LOGLEVEL VERBOSE
#endif

#include "utils/Atomic.hxx"
#include "utils/logging.h"
//#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"
//#include "if/nmranet_if.h"
//#include "core/nmranet_event.h"
#include "nmranet/EventHandlerTemplates.hxx"

namespace nmranet
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
    void init_iteration(EventReport*) OVERRIDE {
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

  virtual void register_handlerr(NMRAnetEventHandler* handler, EventId event, unsigned mask) {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.push_front(handler);
  }
  virtual void unregister_handlerr(NMRAnetEventHandler* handler, EventId event, unsigned mask) {
    // @TODO(balazs.racz): need some kind of locking here.
    handlers_.remove(handler);
  }

 private:
  typedef std::forward_list<NMRAnetEventHandler*> HandlersList;
  HandlersList handlers_;
};

class TreeEventHandlers : public NMRAnetEventRegistry, private Atomic {
public:
    TreeEventHandlers();

    EventIterator* create_iterator() OVERRIDE;
    void register_handlerr(NMRAnetEventHandler* handler, EventId event, unsigned mask) OVERRIDE;
    void unregister_handlerr(NMRAnetEventHandler* handler, EventId event, unsigned mask) OVERRIDE;

private:
    class Iterator;
    friend class Iterator;

    typedef std::multimap<uint64_t, NMRAnetEventHandler*> OneMaskMap;
    typedef std::map<uint8_t, OneMaskMap> MaskLookupMap;
    /** The registered handlers. The offset in the first map tell us how many
     * bits wide the registration is (it is the mask value in the register
     * call).*/
    MaskLookupMap handlers_;
};

}; /* namespace nmranet */

#endif  // _NMRANET_EVENTMANAGER_HXX_
