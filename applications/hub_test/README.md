Hub throughput testing application {#hub_test_application}
==================================

[TOC]

# Goal

The purpose of Hub testing is to verify an OpenLCB link or bus for qualitative
and quantitative performance characteristics. These are as follows.

Qualitative:
- The link has to be lossless.
- There is only limited reordering allowed within the link.
- There has to be proper throttling for ingress of packets to the link.

Quantitative:
- What is the maximum throughut of packets that can be accepted on the link.
- What is the round-trip-time of the link.
- What is the jitter on the timing of the packet round-trips.

For hubs, a relevant question is also how do these metrics change as the number
of clients on the hub are increased.

## Testing methodology

The application contains a load generator, and a receiver. For a given test,
one generator and one or more receivers have to be attached to the same bus.

The generator supplies a given load to the bus. This load may be a fixes
traffic, or the maximum traffic that the bus is willing to accept.

The receiver watches for the traffic coming from the generator, verifies the
qualitative attributes, and measures the quantitative attributes. Statistics
about this are printed to the screen.


