# Arduino Lib notes

- library.json needs to be in the root of the repository for PlatformIO
- library.properties needs to be in the root of the repository for Arduino IDE

- src/dirent.h *MUST* be removed for ESP32 compilation as it conflicts with the dirent.h from ESP-IDF

- stropts.h is referenced from two paths:
src/freertos/can_ioctl.h:#include "freertos/stropts.h"
src/freertos_drivers/common/Devtab.hxx:#include <stropts.h>

- freertos/can_ioctl.h is excluded on ESP32 as it doesn't exist, should it?

- all cxx files should get renamed to cpp, but this is not critical for PlatformIO.

- StreamBridge replaces both SerialBridge and CanBridge by using the base class of
Stream from Arduino, this covers both Serial and can be used for ESP32 hardware
CAN bus integration (thin wrapper to be created)

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

- two samples are available, they are 100% untested but may compile :)

- filelist.txt contains a find output of files that are "in" the DCCppESP32/lib directory for OpenMRN to compile cleanly. Note the examples are not in the DCCppESP32/lib/OpenMRN tree as they were created directly in openmrn/arduino tree (they also wouldn't be compiled under DCCppESP32/lib/OpenMRN)
