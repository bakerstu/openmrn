-include ../openmrnpath.mk
include $(OPENMRNPATH)/etc/core_target.mk
SRCDIR = $(OPENMRNPATH)/src
TESTBLACKLIST += utils/GcTcpHub.cxxtest # needs sockets
TESTBLACKLIST += nmranet/AliasAllocator.cxxtest # some deadlock problem
TESTBLACKLIST += nmranet/Bootloader.cxxtest # needs true multiple threads
TESTBLACKLIST += nmranet/IfCan.cxxtest # TODO
TESTBLACKLIST += nmranet/IfCanStress.cxxtest # needs multiple executors
TESTBLACKLIST += nmranet/PolledProducer.cxxtest # TODO
TESTBLACKLIST += nmranet/RefreshLoop.cxxtest # TODO

include $(OPENMRNPATH)/etc/core_test.mk
