DCC decoder application {#dcc_application}
=======================

[TOC]

This application can be loaded to a TIVA LaunchPad, connected to a DCC track,
and it will print all DCC packets it receives in a human readable form to the
computer via tha USB port.

Note: This hardware and software combination has not been evaluated for NMRA
conformance of DCC packet reception.

# Requirements

- Tiva 123 Launchpad
- DCC command station/booster/track :)
- one or two 20k - 200k resistors
- Special grounding rules

# Special grounding rules

In order to use this application, either the DCC command station / booster, or
the computer has to have a floating ground. The computer and one of the tracks
will be grounded together. This is generally a supremely bad idea.

USE IT AT YOUR OWN RISK. YOU COULD CAUSE A FIRE. 

A simple way to achieve this is to use a laptop that is NOT plugged into the
wall power supply but is operating from battery.  An alternative (but less
secure) way to do the same thing is to use a DCC command station that is not
connected to anything (not connected to booster common, cab bus, loconet,
computer interface, etc), and check that the power supply has a 2-prong plug.

# Wiring

Connect one track to the GND pin of the launchpad. This exists both on J2 and
J3. Alternatively, connecting the Booster Common terminal on the CS/booster to
the GND pin might work as well.

Connect the other track through a series resistor (of any value between 20k and
270k) to pin PB1. This is on J1 (left hand side of the boosterpack pins).

To reduce your risk of fire or burning up things, you can add a series resistor
in the GND line as well.

# Operation

After these are connected, plug in the launchpad DEVICE port (the one on the
side, not on the top) to your computer. Open any terminal program (e.g. putty or
the Arduino IDE's serial monitor will do) to view the output.

# Output

```
73.526949 6229 [dcc] Short Address 51 F[5-8]=0000
   |      |    |     +--- textual description
   |      |    +------- protocol
   |      +--- packet sequence number
   +--- time (seconds.microseconds)
```
