-include ../openmrnpath.mk
include $(OPENMRNPATH)/etc/core_target.mk
SRCDIR = $(OPENMRNPATH)/src
TESTBLACKLIST += utils/GcTcpHub.cxxtest # needs sockets
TESTBLACKLIST += nmranet/Bootloader.cxxtest # needs true multiple threads
TESTBLACKLIST += nmranet/IfCanStress.cxxtest # needs a true timer

include $(OPENMRNPATH)/etc/core_test.mk
