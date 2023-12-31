# Throttling

Definition: *Throttling* is the ability of a program or a hardware device to
not accept more data on its input ports than how much data it can process and
emit on its output ports.

## Why is it important

In the absence of throttling, whenever there is an unbounded data flow coming
in on an input port, the program or device will have to grow in its memory use
indefinitely, or start dropping data.

Since dropping traffic is not acceptable for OpenLCB, the only choice remaining
is to grow unbounded in memory use. On embedded devices, this is quickly going
to cause a crash with out-of-memory.

It is clear that the only viable choice is to not accept the unbounded input
traffic stream.

## Traffic sources

An OpenMRN application or hardware device can sendreceive traffic in three
fundamentally different ways. These each have different characteristics in
terms of throughput and throttling.

### CAN-bus

CAN-bus does not really propvide an API for throttling incoming traffic.

The bit-rate of CAN-bus is fixed for OpenLCB, and it translates to about 890
frames/second at full frame size (e.g. events). The requirement to all hardware
devices is that they be able to process this volume of traffic. Whenever an
OpenMRN-based device is created, the manufacturer should stress-test the system
and verify that the CPU utilization is in the comfortable level even in the
presence of max bus utilization.
   
The data volume in GridConnect format is ~26 kbyte/sec, or 258 kbaud (over
serial).
   
### USB port

A native USB device can transmit a significant volume of data; up to 1 mbyte
per second over USB1.1/USB2 FS. This is significantly higher than the
   
The USB protocol contains several features that allow the device or the host to
control the data flow, generally in the form of NAK packets in response to the
proposed data frame.

USB hardware in microcontrollers always has the ability to send back NAK
packets when the buffers in use have not yet been freed by the application.

USB middlewares by different manufacturers have different methods of how they
expose this on their API. Famous counterexample is the STM32 USB middleware,
which has NO mechanism to throttle the data flow coming from the host. This is
just due to incredibly poor API design. The TinyUSB stack is a good alternative
that is better designed.
   
### TCP socket

TCP connections can scale to very high throughput. Even in embedded devices,
thropughput of several megabytes per second is common as a capability.

TCP provides flow control / throttling mechanism that is automatically
performed by the TCP implementations (at kernel level). Data is buffered in the
send buffer and in the receive buffer, which are limited in size. If the
receiving application does not read from the receive buffer, the TCP
implementation will send a zero-window message to the sender, instructing it to
stop sending further data. Any further writes will be kept in the send queue;
when the send queue is full, the sender application will pend in the write
call.

## Throttling in OpenMRN

