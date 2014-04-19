

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

namespace NMRAnet
{

class AsyncNode;

class GlobalEventFlow;

struct GlobalEventMessage
{
public:
    uint64_t event;      ///< payload (event or range or zero)
    NodeHandle src_node; ///< sender of the message
    If::MTI mti;         ///< what message showed up
    AsyncNode* dst_node; ///< for addressed messages or else nullptr.
};

// The global event handler is a control flow that runs in the user thread's
// executor. It listens to the incoming event queue, and runs the registered
// global event handlers when there is an incoming event message.
class GlobalEventFlow : public IncomingMessageStateFlow
{
public:
    GlobalEventFlow(AsyncIf* interface);
    ~GlobalEventFlow();

    /// Returns true if there are outstanding events that are not yet handled.
    bool EventProcessingPending();

    /// Statically points to the global instance of the event handler.
    static GlobalEventFlow* instance;

protected:
    Action entry() OVERRIDE;
    Action WaitForEvent();
    Action HandleEventArrived();
    Action HandleEvent();
    Action WaitForHandler();
    Action HandlerFinished();

private:
    class Impl;

    std::unique_ptr<Impl> impl_;
};

}; /* namespace NMRAnet */

#endif // _NMRANET_GLOBAL_EVENT_HANDLER_
