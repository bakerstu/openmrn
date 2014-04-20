

#ifndef _NMRANET_GLOBAL_EVENT_HANDLER_
#define _NMRANET_GLOBAL_EVENT_HANDLER_

// This is a workaround for missing shared_ptr.h causing compilation errors. We
// do not use shared_ptr.
#ifndef __CR2_C___4_6_2_BITS_SHARED_PTR_H__
#define __CR2_C___4_6_2_BITS_SHARED_PTR_H__
#endif

#include <memory>

#include "utils/macros.h"
#include "executor/StateFlow.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/AsyncIf.hxx"

namespace NMRAnet
{

class AsyncNode;

class GlobalEventFlow;

/*
struct GlobalEventMessage {
public:
    uint64_t event;      ///< payload (event or range or zero)
    NodeHandle src_node; ///< sender of the message
    If::MTI mti;         ///< what message showed up
    AsyncNode* dst_node; ///< for addressed messages or else nullptr.
};
*/

class GlobalEventService : public Service
{
public:
    /** Creates a global event service with no interfaces registered. */
    GlobalEventService(ExecutorBase *e);
    /** Creates a global event service that runs on an interface's thread and
     * registers the interface. */
    GlobalEventService(AsyncIf *interface);
    ~GlobalEventService();

    /** Registers this global event handler with an interface. This operation
     * will be undone in the destructor. */
    void register_interface(AsyncIf *interface);

    struct Impl;
    Impl *impl()
    {
        return impl_.get();
    }

    /** Returns true if there are outstanding events that are not yet
     * handled. */
    bool event_processing_pending();

    static GlobalEventService *instance;

private:
    std::unique_ptr<Impl> impl_;
};

}; /* namespace NMRAnet */

#endif // _NMRANET_GLOBAL_EVENT_HANDLER_
