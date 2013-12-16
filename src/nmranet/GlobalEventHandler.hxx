

#ifndef _NMRANET_GLOBAL_EVENT_HANDLER_
#define _NMRANET_GLOBAL_EVENT_HANDLER_

// This is a workaround for missing shared_ptr.h causing compilation errors. We
// do not use shared_ptr.
#ifndef __CR2_C___4_6_2_BITS_SHARED_PTR_H__
#define __CR2_C___4_6_2_BITS_SHARED_PTR_H__
#endif

#include <memory>

#include "utils/macros.h"
#include "executor/control_flow.hxx"
#include "nmranet/NMRAnetIf.hxx"

namespace NMRAnet
{

class Node;

class GlobalEventFlow;

struct GlobalEventMessage : public QueueMember
{
public:
    uint64_t event;      ///< payload (event or range or zero)
    NodeHandle src_node; ///< sender of the message
    If::MTI mti;         ///< what message showed up
    Node* dst_node;      ///< for addressed messages or else nullptr.
private:
    DISALLOW_COPY_AND_ASSIGN(GlobalEventMessage);
    // We only allow allocation of this object by the GlobalEventFlow class.
    GlobalEventMessage()
    {
    }
    friend class GlobalEventFlow;
};

// The global event handler is a control flow that runs in the user thread's
// executor. It listens to the incoming event queue, and runs the registered
// global event handlers when there is an incoming event message.
class GlobalEventFlow : public ControlFlow
{
public:
    // @param max_event_slots sets the maximum number of pending incoming event
    // messages in the global event queue.
    GlobalEventFlow(Executor* executor, int max_event_slots);
    ~GlobalEventFlow();

    // This call will block until a slot can be acquired.
    GlobalEventMessage* AllocateMessage();
    //! @returns the allocator for new incoming messages.
    TypedAllocator<GlobalEventMessage>* message_allocator();
    // Sends a global event message to the handler flow. Can be called from any
    // thread. Will not block.
    void PostEvent(GlobalEventMessage* message);

    //! Returns true if there are outstanding events that are not yet handled.
    bool EventProcessingPending();

    static GlobalEventFlow* instance;

protected:
    ControlFlowAction WaitForEvent();
    ControlFlowAction HandleEventArrived();
    ControlFlowAction HandleEvent();
    ControlFlowAction WaitForHandler();
    ControlFlowAction HandlerFinished();

    void FreeMessage(GlobalEventMessage* m);

private:
    class Impl;

    std::unique_ptr<Impl> impl_;
};

}; /* namespace NMRAnet */

#endif // _NMRANET_GLOBAL_EVENT_HANDLER_
