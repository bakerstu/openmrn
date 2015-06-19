
include hardware.mk

all: cdi.xml


cdi.xml: config.hxx $(OPENMRNPATH)/src/nmranet/CompileCdiMain.cxx
	g++ -o ./compile_cdi -I. -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include --std=c++11 -MD -MF $*.d $(OPENMRNPATH)/src/nmranet/CompileCdiMain.cxx
	./compile_cdi > cdi.xml
