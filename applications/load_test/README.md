Load testing applications {#load_test_application}
=========================

[TOC]

# Goal

The purpose of load testing is the following:

- understand the performance characteristics of the current implementation of
  the protocol stack;
- understand the performance characteristics of a specific MCU platform;
- understand the benefits of a performance-improving change;
- identify performance regressions;
- by comparing to expected performance targets, identify whether there is a
  need for improving the performance;
- find and fix performance-impacting and buffering bugs such as priority
  inversions, insufficiently sized buffers and queues, or a bug that prevents
  traffic from draining from the node.

## Minimum performance

The requirement of the OpenLCB protocol standard is 100% packet reception. This
puts different requirements on nodes by the transport protocol:

- TCP (or any other flow controlled p2p stream mode) transport protocol clients
  are required to properly operate the flow control methods of the transport
  protocol in order to push back on incoming traffic when their CPU is
  overloaded.
- CAN-bus connected (or any other broadcast medium) nodes are required to be
  able to process packets arriving at 100% bus utilization rate for an extended
  period of time. On the current standard CAN-bus that means about 900 packets
  per second.

# Configurations

## Independent test (benchmarking)

In the independent configuration the target under test is not connected to a
bus. Instead, we supply a fake bus implementation that generates packets at a
specific rate, or at whichever rate the MCU is able to process them.

The output of the benchmark is a pkt/sec value that the MCU is able to process
at 100% CPU utilization. For CAN-bus clients this has to be over 900 to meet
the minimum requirement.

### Benchmark driver

Under the FreeRTOS full implementation (including FileIO drivers) packet
injection for benchmarking can be performed by the `BenchmarkCan` class in
`freertos_drivers/common/BenchmarkCan.hxx`. This exposes a device under the
name given by the constructor (e.g.`/dev/fakecan0`) which can be opened and
read/written as any regular device driver like `/dev/can0`.

The benchmark driver is initialized using a single given CAN frame, of which it
generates N copies inbound from the fake bus. Timing information is available
to determine how much wall time the software stack took to process all of the
incoming CAN frames.

### Example applications

in `applications/load_test/target/benchmark.*` there is a benchmark application
ported to a couple of different microcontrollers. This operates with 10'000
event report packets.

## Dependent test (Bus utilization load-test)

In the dependent form of benchmarking we have a real bus, with a target
under-test, and a load generator. The load generator creates a specific load on
the bus, and we observe health monitoring of the target, typically CPU
load. Trying a few different bus utilization numbers we can determine CPU load
at 100% bus utilization, or in case the performance targets are not met, the
approximate gap between actual performance and desired performance.

### Load generator

The load generator for computers is in the linux.x86 target under
`applications/load_test/targets/linux.x86`. Invoke it with the argument `-d
/dev/ttyACM0` to send load to a CAN-bus via LCC-Buffer. Driving load into
network hubs is also supported.

The generated load is configured with the `-s 100` argument. The number is the
packets/second to generate. Each packet is an event report.

There may be jitter in the exact timing of the packets generated, but there is
no drift, i.e. the speed averages to the desired throughput.

### Load generator for MCUs

There is a character driver `freertos_drivers/ti/TivaTestPacketSource.hxx`
which is able to generate a fixed stream of GridConnect packets. This is
specific to TI's driverlib SDK because it is using a hardware timer for
accurate packet timing and delivery.

The target `applications/load_test/targets/frertos.armv7m.ek-tm4c123gxl` is
using the fake packet source with a hub to emit it to a CAN-bus port. This
configuration also tests the gridconnect parsing code's efficiency.

# Measurement techniques

During the benchmarking and load testing it is important to measure the system
under test in a repeatable and objective way. Interesting metrics:

- the total CPU load, or CPU idle percentage
- CPU usage by different tasks and interrupts
- CPU usage by individual functions (hotspot detection)
- jitter of available CPU -- maximum burst length where no CPU is available.

## CPU load detection for FreeRTOS

### Overview

There is a shared driver for computing CPU load under FreeRTOS using a sampling
technique. The sampler shall be a hardware timer resource that ticks
independently of the FreeRTOS system tick, and upon each tick we sample whatthe
CPU is doing:
- is it in the IDLE thread?
- is it in a specific application thread?
- is it in a particular interrupt handler?

Then we average these samples over a benchmarking run or some period of time,
and give a visual or textual indicator to the operator on what the system under
load is doing.

Generally a sampling rate between 100Hz and 200Hz should be sufficient. I used
163 Hz to be relatively independent running of the FreeRTOS tick (usually at
1000Hz).

### Driver usage

- The driver is in `freertos_drivers/common/CpuLoad.hxx`.
- Create one global instance of CpuLoad class.
- Initalize a hardware timer resource, and in the timer tick, call the `extern
  "C"` function `cpuload_tick(unsigned irq)`. The argument should be zero if
  there is a task running. If the processor was in an interrupt, and the timer
  IRQ nested inside that interrupt, then give the nested interrupt number to
  the argument.

### Display using RGB LED

Using a red-green LED and the `src/utils/CpuDisplay.hxx` class we can display
the following thresholded CPU utilization:

- green: CPU load < 20%
- yellow: 20% < CPU load < 75%
- red: CPU load > 75%.

The class takes two Gpio* objects on the constructor. The Tiva 123 Launchpad
has such an RGB LED on board.

### Display using UART logging

In `src/freertos_drivers/common/CpuLoad.hxx` there is a StateFlow `CpuLoadLog`,
which will once every two seconds perform a log output of the CPU usage
details. It prints average utilization, longest streak of busy CPU, maximum
utilization of blocks of 16 consecutive ticks, and per-task breakdown of CPU
usage.

Due to the sampling nature of the CpuLoad driver, accuracy of measurement of
tasks using 5% or less CPU is pretty low.

To use, create an instance of `CpuLoadLog`, which may run on the main
executor. Ensure that there is a logging output configured, for example by
stdio_logging or by FdLog over the serial debug port.
