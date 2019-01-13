# Arduino Lib notes

to build in PlatformIO it is mandatory to add the following to the platformio.ini of the project:
build_flags=-D__FreeRTOS__
the project won't link currently due to:
.pioenvs\esp32dev\src\main.cpp.o:(.literal._ZN7OpenMRN3runEv[OpenMRN::run()]+0x0): undefined reference to `ExecutorBase::loop_once()'
.pioenvs\esp32dev\src\main.cpp.o:(.literal._ZN7OpenMRND5Ev[OpenMRN::~OpenMRN()]+0x4): undefined reference to `Executable::~Executable()'
.pioenvs\esp32dev\src\main.cpp.o:(.literal._ZN7OpenMRNC5Ey[OpenMRN::OpenMRN(unsigned long long)]+0x0): undefined reference to `openlcb::SimpleCanStack::SimpleCanStack(unsigned long long)'
.pioenvs\esp32dev\src\main.cpp.o:(.literal._ZN7OpenMRNC5Ey[OpenMRN::OpenMRN(unsigned long long)]+0x4): undefined reference to `openlcb::SimpleCanStackBase::start_stack(bool)'
.pioenvs\esp32dev\src\main.cpp.o: In function `OpenMRN::run()':
main.cpp:(.text._ZN7OpenMRN3runEv[OpenMRN::run()]+0x6): undefined reference to `ExecutorBase::loop_once()'
.pioenvs\esp32dev\src\main.cpp.o: In function `OpenMRN::~OpenMRN()':
main.cpp:(.text._ZN7OpenMRND2Ev[OpenMRN::~OpenMRN()]+0x17): undefined reference to `Executable::~Executable()'
.pioenvs\esp32dev\src\main.cpp.o: In function `OpenMRN::OpenMRN(unsigned long long)':
main.cpp:(.text._ZN7OpenMRNC2Ey[OpenMRN::OpenMRN(unsigned long long)]+0x4e): undefined reference to `openlcb::SimpleCanStack::SimpleCanStack(unsigned long long)'
main.cpp:(.text._ZN7OpenMRNC2Ey[OpenMRN::OpenMRN(unsigned long long)]+0x5e): undefined reference to `openlcb::SimpleCanStackBase::start_stack(bool)'
main.cpp:(.text._ZN7OpenMRNC2Ey[OpenMRN::OpenMRN(unsigned long long)]+0x72): undefined reference to `Executable::~Executable()'
.pioenvs\esp32dev\src\main.cpp.o: In function `setup()':
main.cpp:(.text._Z5setupv+0x22): undefined reference to `openlcb::SimpleCanStackBase::start_stack(bool)'

- library.json needs to be in the root of the repository for PlatformIO
- library.properties needs to be in the root of the repository for Arduino IDE

- src/dirent.h should not be present in the arduino lib as it is provided in the base libraries

- stropts.h is referenced from two paths:
src/freertos/can_ioctl.h:#include "freertos/stropts.h"
src/freertos_drivers/common/Devtab.hxx:#include <stropts.h>
both are required for ESP32 build to work

- all cxx files should get renamed to cpp, but this is not critical for PlatformIO.

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
