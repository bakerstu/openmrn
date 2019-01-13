# Arduino Lib notes

- to build in PlatformIO it is mandatory to add the following to the platformio.ini of the project:
build_flags=-D__FreeRTOS__

- build is broken due to freertos_drivers/common/Can.cxx, freertos_drivers/common/Select.cxx
needing to use Atomic instead of portENTER_CRITICAL() and portEXIT_CRITICAL(), these methods
take an arg on the ESP32 and Atomic covers this use case but I was unsure how this should be
implemented in these files.

- library.json needs to be in the root of the repository for PlatformIO

- library.json has been updated to include the required source trees and exclude a couple files
that have been observed as bad, this includes include/freertos/dirent.h.

- library.properties needs to be in the root of the repository for Arduino IDE

- stropts.h is referenced from two paths:
src/freertos/can_ioctl.h:#include "freertos/stropts.h"
src/freertos_drivers/common/Devtab.hxx:#include <stropts.h>
both are required for ESP32 build to work

- all cxx files need to be renamed to cpp:
find src -name '*.cxx' -print0 | sed 's/.cxx//g' | xargs -0 -I % mv %.cxx %.cpp

- StreamBridge replaces both SerialBridge and CanBridge by using the base class of
Stream from Arduino, this covers both Serial and can be used for ESP32 hardware
CAN bus integration (thin wrapper to be created). This needs more work as Stream
doesn't have a way to read into a std::string which is used by loop_for_read().
This usage should be reviewed further as it doesn't have any limit on data to read.

- if possible we should remove this #pragma as it generates a TON if noise:
In file included from lib/OpenMRN/src/openlcb/SimpleNodeInfo.hxx:39:0,
from lib/OpenMRN/src/openlcb/ConfigRenderer.hxx:41,
from lib/OpenMRN/src/openlcb/ConfigEntry.hxx:44,
from lib/OpenMRN/src/openlcb/ConfigRepresentation.hxx:38,
from lib/OpenMRN/src/openlcb/SimpleStack.hxx:42,
from lib/OpenMRN/src/OpenMRN.h:4,
from include/DCCppESP32.h:91,
from include/Locomotive.h:20,
from include/DCCppESP32.h:88,
from src\WiFiInterface.cpp:18:
lib/OpenMRN/src/openlcb/SimpleInfoProtocol.hxx:404:0: warning: ignoring #pragma clang diagnostic [-Wunknown-pragmas]
#pragma clang diagnostic ignored "-Wunused-private-field"
^
- two samples are available, they do not compile cleanly currently.

- filelist.txt contains a find output of files that are "in" the DCCppESP32/lib directory for OpenMRN to compile cleanly. Note the examples are not in the DCCppESP32/lib/OpenMRN tree as they were created directly in openmrn/arduino tree (they also wouldn't be compiled under DCCppESP32/lib/OpenMRN)
