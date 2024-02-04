# DirectHub design

DirectHub is a high performance router component that is suited to do the
forwarding of packets to multiple receivers with minimal CPU and latency
overhead.

It specifically addresses three performance issues with the traditional CanHub
/ Dispatcher infrastructure:

- DirectHub is zero-copy when forwarding packets between sockets. There is a
  buffer which is filled with a ::read on the source socket, and then the
  DirectHub passes around a reference to this buffer all the way to the output
  object, which then ::write's it to the output socket.
- CanHub and GcTcpHub as they operate together perform separate GridConnect
  formatting for every port. When data passes from one port in to three others
  out, there would be one parsing and three separate GridConnect rendering
  calls. DirectHub uses a single GridConnect representation and passes around
  that representation. Only one parsing is done when a CanIf needs a struct
  can_frame.
- DirectHub performs inline calls to the ports when forwarding the packet,
  while CanHub/GcTcpHub allocates a new copy of the buffer which then gets
  enqueued separately for each port's queue, on separate StateFlows. This means
  that the Executor is spinning a lot less for DirectHub, therefore the context
  switching overhead is much smaller. (note 1)

As future expansion, DirectHub by design will allow routing packets across
multiple interface types (e.g. CAN, GridConnect and native-TCP), apply packet
filtering, and admission control / fair queueing for multiple trafic sources.

_(note 1):_ There is a conceptual problem in `Buffer<T>*` in that it conflates
two different but equally important characteristics of data flow. A `Buffer<T>`
is reference-counted, and it can be queued. However, while different owners may
hold separate references (to the same memory), only one owner is allowed to
enqueue a `Buffer<T>` into a `Q`, `QList`, or `StateFlowWithQueue`. This is
because there is only one `QMember` pointer in the `BufferBase`. The result of
this conflation is that when a `Dispatcher` or a `Hub` / `CanHub` sends the
same data to multiple different ports or flows, it needs to actually create a
separate copy for each one of them, and taking a reference is not sufficient.


## Theory of operation


### Entry flow and threading model

In order to make the router have as little overhead as possible, almost
everything about the router should be happening inline instead of
asynchronously / via queueing. Virtual function calls are okay, but
StateFlowWithQueue operations should be avoided.

Inline calls mean that there is a difference in threading concept: most of the
time we use the thread of the caller. When concurrent calls are performed, we
have to hold one of those calls until the other is complete.

Upon an entry call (after the admission controller, see later) we want to first
check if the router is idle. If yes, we should grab a lock and start processing
the message inline. If the router is busy, we should queue the incoming
caller. To allow for both of these, the entry call doesn't actually give us a
message, we get a callback instead that we'll invoke. The sender renders the
actual message in that callback.

After processing the message, the router goes back to idle if the queue of held
callers is found to be empty.

If the queue is non-empty, that means that a different thread called the router
while we were sending a message on the current thread. We notice this in the
`on_done()` method of the service. In this case the router remains busy and the
queue front is taken out for processing. The queue front is always an
Executable and it will be scheduled on the Service's executor (effectively
yielding), while the inline caller's thread gets released.

A consequence is that the caller's send callback may be called either on the
caller's thread inline, or on the Service's thread, sometime later after the
caller signaled the intention of sending something to the DirectHub.

A special case of this threading model is that when the caller runs on the same
executor as the DirectHub, then the actual send callback is guaranteed to
happen on that executor. This is the typical case on a single-processor OpenMRN
application.

### Entry API

The Entry API defines how to send traffic to the DirectHub. It is defined by
`DirectHubInterface<T>` and `MessageAccessor<T>` in `DirectHub.hxx`.

This is an integrated API that will internally consult the admission controller
(not implemented, see later). There are three possible outcomes of an entry
call:
1. admitted and execute inline
2. admitted but queued
3. not admitted, blocked asynchronously. (this doesn't happen today)

When we queue or block the caller, a requirement is to not block the caller's
thread. This is necessary to allow Executors and StateFlows sending traffic to
the DirectHub.

When blocked, the best solution is to queue Executables (these are
queueable). So we put them into a queue, and we put the next one onto the
executor (yield) whenever we're ready, which is typically when the current
packet's processing and routing is done.

If we're idle (available) to process the packet upon the entry call, we want to
run it inline by calling run() on the Executable from the caller's thread.

In other words, assuming the caller is a StateFlow, the inline execution just
means that we `run()` the executable instead of `notify()`'ing it.

The syntax to prepare for both of this from a calling StateFlow (any
`StateFlowBase`):

```
Action have_message() {
    // Makes the next run() go to fill_request(), but does not put *this onto
    // the executor.
    wait_and_call(STATE(fill_request));
    // Will cause run() to be called now or later.
    target->enqueue_send(this);
    // Ensures we do not disturn state_ or the notification.
    return wait();
}

Action fill_request() {
    target->mutable_message()->set_...; // fills message buffer
    target->do_send();
    // should not be call_immediately() because threading is not certain at this
    // point.
    return yield_and_call(STATE(something_next)); 
}
```

There is a slightly more complicated sequence of states to do if the yield at
the end is undesired. The actual implementation of gridconnect / socket read
flows use this more complicated mechanism to process multiple gridconnect
frames that might have come with a single TCP packet.

### Exit API

The Exit API defines how to the DirectHub sends traffic to the ports. It is
defined by `DirectHubPort<T>` and the same `MessageAccessor<T>` in
`DirectHub.hxx`.

Since we are trying to make as much of the DirectHub processing happen inline,
the exit API is synchronous. The exit target is responsible for any queueing
that needs to happen. This is very much like the current FlowInterface<>.

The exit API does not by definition get a ref of the payload. If they need one,
they should take one inline. However, unlike `FlowInterface<T>`, this means
that they can not use a Buffer pointer they get from putting it into a
queue. If they need to queue, they have to allocate a new QMember
somewhere. See (note 1) in the introduction on a significant difference that
this makes.

It is guaranteed that there is at least one ref is held during the time of the
call, and the caller (the hub) will release that ref sometime after the exit
call has returned.

The exit call gets an accessor instead of a sequence of parameters. The memory
for the accessor is owned by the hub, and allows the target to inquire the
necessary parameters. The accessor is assumed to be available only during the
exit call and after the exit call has returned the accessor will be reused for
other messages. This is harmonized with the entry API where we are not queueing
_data_ but we are queueing _data sources_, which then fill in the data when we
are ready for them to do so.

API:

```
class DirectHubPort
{
    void send(MessageAccessor *message);
};

class MessageAccessor
{
    HubSource *source_;
    HubSource *dst_;
    BarrierNotifiable *done_;
    
    bool isFlush_;

    // For string typed hubs we have a BufferPtr<> data_ with a skip_ and size_
    // encapsulated in a class:
    LinkedDataPtr<uint8_t[]> payload_;
    
    // For arbitrary hubs we have a reference to a buffer:
    BufferPtr<T> payload_;
};
```

An important aspect is that the MessageAccessor is a constant sized object. The
real payload is always kept as a reference to a Buffer that was allocated by
the sender object. Output ports are allowed / encouraged to hold on to
references to this Buffer, which allows the zero-copy operation.

### Runner

The hub has at most one current message at any point in time (zero if the hub
is idle, one if the hub is busy). This is the message that is being sent by the
port that was last executed. The MessageAccessor is owned by the runner,
and accessed by the port during the entry API to fill in payload and message
parameters, and passed on to the ports as part of the exit API. There is no
queue of messages.

The runner is not a classic StateFlow, because of the lack of this queue. The
runner only manages the concurrency and queueing of the senders. After the
designated sender fills in the message in the MessageAccessor, the runner is
informed that it shall process the packet. This happens without yielding, by an
inline call to `do_send()` on the `DirectHubInterface`.

Internally, `do_send()` performs the iteration over output ports, calling all
the exit APIs synchronously. Once this is complete, the message gets cleared,
which releases the leftover reference owned by the DirectHub. Then the service
is informed that it may now look for additional callers
(`DirectHubService::on_done()`) that may have been enqueued. If none there, the
hub goes idle. For an inline caller, the control returns to the caller, and it
may attempt to send another packet. This allows a single caller to send a
sequence of messages without needing to yield or spin on an executor.

When we give the packet to an output port, that operation should never block
the router. We should rather block the incoming port than the router. It's the
job of the incoming admission controller to hold back; in the absence of that
the limit on the number and byte length of the buffers makes the data source
hold back.

### Output buffering

For TCP based output ports (both gridconnect-CAN-TCP and native TCP, but not
gridconnect-USB) we want to ensure that the number of kernel calls is much less
than the number of GridConnect packets that are being sent. This is essential
in keeping the costs low, especially that on the CC32xx platform where each
kernel call turns effectively into a packet to be sent to the network.

The DirectHub gets one call and one iteration for each GridConnect packet.

The mechanism that the legacy HubDevice infrastructure used is to create a
BufferPort, which internally reassembles these packets into larger buffers
whenever they come within a certain period of time. This results in data copies
unfortunately.

The DirectHub<uint8_t[]> infrastructure appraches this differently. Instead of
copying the input data into a separate buffer, it attempts to recognize when
the input data came from the same source and used consecutive bytes of the same
buffer. This is accomplished by comparing the Buffer references and offset/size
values of consecutive calls (see `LinkedDataBufferPtr::try_append_from()` in
`DataBuffer.hxx`). When two packets came from consecutive bytes of a single
input buffer, then the two references are united into a single reference with a
longer size. So long as the calls of the DirectHub are without yield, this
works until the entire input buffer is reassembled into a single output buffer,
which will be then written with a single `::write()` call to the socket.

While this mechanism is rather limited, it solves the the high-throughput
problem, when an input client is sending a datagram or stream with a large
number of CAN frames, a single 1460-byte read succeeds from the input socket,
then a sequence of sends happen through the directhub without yielding. On the
output there will be one write for almost all of the data, except a partial
GridConnect packet which had to be held until the next read.

Since the output object keeps the reference to the input buffer, the input
port's read flow will not observe the memory released until the output write
has completed. Since the input port has a limited number of such buffers, this
creates effective back-pressure on the input port not reading too much data
into memory.

### Message representation for untyped data in transit

See (note 1) in the introduction for background about the difference between
reference counted objects and queueable objects (QMembers). Specifically, it is
important to separate the queuing concept from the shared ownership of the data
payload. This is because the same data payload might be owned by multiple
output ports, but if the queue next pointer is tied to the buffer, then we
cannot effectively share.

Generally, all data during transit is represented in BufferPtr<T>. This is a
reference to an input buffer, but is not queueable. DirectHub does not use data
queues internally, so that's OK.

For untyped / string data, we need to ensure that we keep the length of the
contents as well. However, we don't generally know the length of the contents
until the read has happened and we have processed the incoming data on the
source port.

To avoid having to copy data, we perform a single longer read into a large
buffer (typically 1460 bytes, the size of a TCP frame), then we segment this
into individual messages. Each such message will have a reference to the longer
buffer, and an offset and a length attribute (called skip_ and size_).

A particular case to be handled is when one message spans reaches beyond the
end of one such buffer and into the beginning of the next buffer. It could also
happen that a message is longer than 1460 bytes.

For this purpose we keep `BufferBase` objects linked to each other using the
`next_` pointers. The queue created by the `next_` pointers means that the data
payload continues in the next buffer. This is different from the
`StateFlowWithQueue` infrastructure, and generally the `Q` ans `QList` classes,
where the `next_` pointer means that there is another data item (a different
message) waiting to be processed by the same StateFlow later.

The implementation of this mechanism is in `LinkedDataBufferPtr` in
`utils/DataBuffer.hxx`.

Some earlier notes:

BufferBase has the ability to do refcounting and shared ownership. It is
possible to have a BufferBase that has untyped payload (i.e., just
bytes). However, the BufferBase needs to know the amount of bytes as input; we
cannot trim down the actual bytes read from the BufferBase's size field, or
else we lose memory because after freeing the buffer will not be returned to
the right size. An alternative possibility is to have a buffer pool that
generates a single size buffer so everything is returned to the same
queue. Then size can be adjusted to the exact number of bytes read. This might
utilize a proxy buffer pool that takes buffer of a given size from the main
buffer pool and then returns them there upon freeing, resetting the size to
have them land in the appropriate bucket.

As an alternative, `shared_ptr<string>` is a standard template library solution
to the same problem. However, `shared_ptr<string>` causes two memory
allocations for payloads that are longer than 16 bytes, and it has a minimum of
36 bytes length (+payload length+4 if more than 16 bytes).

Note that the input message could be split between two shared buffer
ownerships. This means that the queue entry needs two buffer pointers, an
offset, and a length. We could use the buffer base next pointers to string up
buffers that have data from the same message, even if it's more than two. This
way we only need one buffer pointer. We have to assume that the respective
bytes always go together.

It might make sense to support appending another message to the end of the
buffers. This be especially true if the last buffer pointer is just
partially used up, and thus the bufferptr at the end of the string of
buffers is the same as the incoming next buffer.

### Input segmentation

When data arrives from the socket to be read, we will allocate a shareable
buffer, then execute the asynchronous read. As the read completes, the input
data will be passed on to the segmenter. The goal of the segmenter is to find
the boundary of the message, for gridconnect the `: ... ;` delimiter, and on
native OpenLCB-TCP the binary length of the message. Then the message can be
passed on to routing.

It is possible that during segmentation we start with one ref of a buffer, and
output two independent refs of the same buffer. This happens if a single kernel
read ends up with more than one message, which is rather typical in
GridConnect-TCP, but possibly even in GridConnect-USB.

It is also possible that the segmenter will retain a ref of the last read
buffer, waiting for the completion of the message that is present therein.

We must keep reading bytes from the hardware until the segmenter is happy to
send at least one packet onwards. Only thereafter should we send the packet (or
consult the admission controller). It is essential that a partial packet must
never be sent to the hub, because it is not guaranteed that we get the
completion of that packet before another port might try to send a different
packet. We can not interleave data from different packets, that would be an
unparseable outputs.

Implementation note:

There are several things that have to happen on the ingress port, and the order
in which we do these matters:

- allocate a BarrierNotificable* for accompanying the buffer.
- allocate a (1460-byte) buffer for the `::read` call.
- perform the `::read`
- call the segmenter (which might result in additional buffers needed and
  additional `::read` calls to be made)
- (not implemented yet) consult the admission controller on whether we are
  allowed to send.
- send the message to the hub.

The above list is the current order. There is one suboptimal part, which is
that we allocate a buffer earlier than when we know that there is data to read
from the fd or socket. We could theoretically wait until the fd is selectable
for read, and only then perform the buffer allocation. With the admission
controller this will get even more complicated.

### Legacy connection

We have two reasons to interact with a legacy `CanHub`:
- Running an OpenMRN `IfCan` and a `Node` requires this object to communicate
  with the external world.
- Interacting with a hardware CAN controller via SocketCan or via OpenMRN
  native CAN controller drivers can be done via `struct can_frame` today, and
  the implementation is in `HubDeviceSelect<struct can_frame>`.

To support these use-cases, there is a legacy bridge, which connects a
GridConnect typed `DirectHub` to a `CanHub`. It takes care of parsing the
GridConnect messages in one direction, formatting them in the other direction,
and the bridges the differences between the APIs.

When many CAN frames are generated consecutively, they typically get rendered
into a single text buffer. However, they don't typically get sent off without
a yield inbetween.

## Future features

**WARNING** These features are not currently implemented. They are described
here with requirements to guide a future implementation.

### Admission controller (not yet implemented)

When a caller has a packet to send, it goes first through an admission
controller. The admission controller is specific to the source port. If the
admission controller rejects the packet, we hold the caller, and save the
notification the caller gave us to let the caller know when they can
proceed.

QQ is it possible that a single source has more than one packet to send us,
possibly at different priorities, and the admission controller would block
one then allow a different? Probably yes; a priority based traffic selector
behaves like this. We must ensure that traffic of the same priority and
src/dst pair always is queued and never independently presented to the
admission controller.

Failed requests of the admission controller have to line up in a queue like
the Async Allocator. When there is a token available, the queue front
should be woken up / notified. This gives a constraint on what the class
interface of the incoming request should be: an Executable has an
alloc_result() call.

The most straightforward implementation of an admission controller is to
keep track of the number of inflight objects. This automatically pushes
back on the src-dst pairs that generate too high traffic even in the face
of smart routing.

There is also no admission control for CAN hardware, because we do not have the
ability to push back on traffic.

When admission control has woken up an input stream, that does not mean the
stream has the ability to execute directly on the hub. The hub might be busy.

QQ how long do we need to keep around the existence of the pending object?
Ideally exactly until the pending object fully leaves the memory of the
device. If there is an outgoing assembly buffer that collects bytes of outgoing
data, we must ensure that this outgoing buffer is also kept in
consideration. However, so long as there is empty space in this buffer we
should not prevent further ingress. Once there is no empty space in the output
buffer, we have to limit the number of items that queue up to be >= the maximum
single-source input entries. This will cause pushback on the ingress path. This
means that after the buffer is complete, we still have to queue some packets.

**Current State:** Since the admission controller is not implemented, each call
to the DirectHub will be enqueued on a first-come-first-served basis. One call
will be one GridConnect packet. A call to the hub never blocks, calls are
enqueued only if they are concurrect from different threads, which doesn't
typically happen when there is one main executor. One source port will perform
as many calls as it can from a single buffer -- until the segmenter says the
message in the buffer is partial. This is typically 1460 bytes
(`config_directhub_port_incoming_buffer_size()`). After that the port will
attempt to allocate a new buffer, which will make it pause. Each port can have
at most 2 buffers in flight
(`config_directhub_port_max_incoming_packets()`). At this point another port
can perform writes to the hub. When multiple ports are sending their traffic
concurrently, on the output we'll have roughly 1.5 kbytes from one port, then
1.5 kbyte from another, etc. If only one port is sending a lot of traffic, and
another wants to send just one packet, then typically 3 kbytes of traffic has
to drain from the one port before the other can send its packet. Since nothing
queues at the source port, it is possible for the stack to perform
prioritization of the packets against each other, for example when one source
port is sending a stream, while another sends a CAN control frame or an event.

### Connecting DirectHubs with each other (not yet implemented)

It is pretty important to have the Exit API compatible with the Entry API in
case we have a bridge that connects to different Hubs with a data conversion
inbetween.
However, there is no admission control between different Hubs.

For hubs that are 1:1 in their representation of messages, a synchronous
execution is desirable. This might be held up if the specific flow is busy
though. There can be a race condition where inputs from two different but
bidirectionally interconnected hubs are both receiving inbound packets, both
busy, and have to queue for accessing each other -- a deadlock.

Option 1) to work around this is to have 1:1 connected hubs not have
independent free/busy states, but share that state. This way all enquueing
happens on the entry point where separately defined flows or executors ensure
queueing is possible.

Option 2) to work around is to decouple the exit from the original flow if we
fail to acquire the target hub; this could be implemented by creating a
CallbackExecutable on the heap and handing out ownership of the buffer to it.

The conclusion from both of these is that the exit API is synchronous. The exit
target is responsible for any queueing that needs to happen. This is very much
like the current FlowInterface<>.

### Routing (not yet implemented)

The first thing the runner flow should determine is if we have a broadcast or
unicast packet. For unicast packet we will have to look up directly the output
port that it should go to. For broadcast packet we have to iterate. The output
port might have to go through an indirection: some output ports are going to be
represented by a bridge (e.g. CAN to TCP bridge) first and another routing hub
on the far side of it.

### Alternate output buffering (to be decided)

See background in section Output buffering.

We have two additional alternatives on how to handle this output buffer:

1) have a fixed memory pre-allocated, and as packets arrive at the output
port, we copy the bytes into this memory.

2) We queue up the packets in the output port with their original buffer,
keeping a ref to those buffers, and when we decide to flush, we allocate
the output buffer, copy the bytes into it, send off the data, then free the
output buffer.

Option 2 uses less memory because the half empty output buffer is not kept
around. Both options have the same number of memory copies (we cannot avoid
the etra copy unless we have some scatter-gather DMA thing). Option 1 is
easier for understanding when the original source admission controller can
release the token. Option 2 allows high priority packets to jump to the
front of the queue just before the flush is happening. Option 1 takes less
memory in that broadcast packets do not need to be queued at each single
output port.

QQ. How does the admission controller keep track of when the message really
exited the system? We must keep it in mind that the message might have been
translated to a differen physical format, and the memory for the specific
byte buffer be freed.

### Flushing (not implemented yet)

There should be a "flush" porperty of a message, which should cause
outgoing buffers to be immediately transmitted in order to achieve lower
roundtrip latency of important requests. We can be smart about what to
flush:
- last segment of datagrams (:X1Axxx, :X1Dxxx)
- last segments of stream writes (:X1F that has less than 8 data bytes)
- datagram ACK and stream proceed messages
- loco function control messages
- emergency stop messages

Current state: the isFlush_ attribute is there, but it is not filled in by the
traffic sources and not used in the sinks. Since the sinks don't hold data with
an output buffer / timerat this time, there is nothing to do really with this
information.

### Bridges (not yet implemented)

In a real CAN-TCP router we will need to have three separate instances of
this router type: one for TCP messages, one for CAN frames, one for
GridConnect text.

The respective instances have to be connected by bridges. A bridge is an
output port from one router and an input port to another router, allocates
memory for the data type conversion, but does not perform admisson control
(this is because admission control is end to end). The bridge might be a
state flow, because for ->CAN messages the bridge has to perform alias
lookups or allocation.

It is unclear whether we also need routers / bridges for internal only
reference of messages (e.g. Buffer<GenMessage>). Is the
AddressedMessageWriteFlow a component on the If side or is it a component
that is part of the connecting bridge? It has to exist in the bridge for
the purpose of routing messages between different types of Ifs.

### Route lookups (not yet implemented)

When a message shows up in a router, we have to perform a lookup to decide
where it goes. The output of this lookup is either "everywhere" (broadcast
message), which means every output except the same as where it came
from. The other alternative is "to a specific target" which is then one
single port. That port might not be on the current router though. Thus we
need a secondary routing table as well, which maps HubSources to ports or
bridges in the current, type-specific router. This secondary routing table
exists independently for each router; however, the primary routing table
only exists once globally.

In every case we will test the packet just before output against the
specific HubInput that it came from in order to avoid echoing it back.  The
HubInput may have to be changed between specific routers; or alternatively
we could use the secondary lookup table for the current router. It is
sufficient to perform that secondary lookup only once, as the current
router starts to process the message, if it has been identified that it is
a broadcast message. This basically means that the skipMember (source) can
be a class variable inside the hub flow itself.

The bridge could actually also directly represent the skipMember when
sending the packet to the target router. However, we want to make sure the
enqueueing of the message does not cause a loss of information. We could
have the enqueueing be based on an execution of a piece of code.
