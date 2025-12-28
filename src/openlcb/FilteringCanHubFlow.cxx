#include "openlcb/FilteringCanHubFlow.hxx"

namespace openlcb
{

FilteringCanHubFlow::FilteringCanHubFlow(Service *service)
    : CanHubFlow(service)
    , isFiltering_(true)
{
}

StateFlowBase::Action FilteringCanHubFlow::entry()
{
    if (isFiltering_)
    {
        filter_.prepare_packet(message()->data());
    }
    return CanHubFlow::entry();
}

void FilteringCanHubFlow::unregister_port(CanHubFlow::port_type *port)
{
    filter_.remove_port(reinterpret_cast<uintptr_t>(port));
    CanHubFlow::unregister_port(port);
}

StateFlowBase::Action FilteringCanHubFlow::iterate()
{
    if (!isFiltering_)
    {
        return CanHubFlow::iterate();
    }

    // Filtering behavior
    ID id = get_message_id();
    {
        OSMutexLock l(&lock_);
        for (; currentIndex_ < handlers_.size(); ++currentIndex_)
        {
            auto &h = handlers_[currentIndex_];
            if (!h.handler)
            {
                continue;
            }
            if (negateMatch_ && (id & h.mask) == (h.id & h.mask))
            {
                continue;
            }
            if ((!negateMatch_) && (id & h.mask) != (h.id & h.mask))
            {
                continue;
            }

            // Additional check: filtering
            if (!filter_.is_matching(reinterpret_cast<uintptr_t>(h.handler)))
            {
                continue;
            }

            // At this point: we have another handler.
            if (!lastHandlerToCall_)
            {
                // This was the first we found.
                lastHandlerToCall_ = handlers_[currentIndex_].handler;
                continue;
            }
            break;
        }
    }
    if (currentIndex_ >= handlers_.size())
    {
        return iteration_done();
    }
    // Now: we have at least two different handler. We need to clone the
    // message. We use the pool of the last handler to call by default.
    return allocate_and_clone();
}

} // namespace openlcb
