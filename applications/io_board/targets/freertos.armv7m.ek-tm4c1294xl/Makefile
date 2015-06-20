
include hardware.mk

$(EXECUTABLE)$(EXTENTION): cdi.xml

-include *.d

cdi.xml: compile_cdi
	./compile_cdi > cdi.xml

compile_cdi: config.hxx $(OPENMRNPATH)/src/nmranet/CompileCdiMain.cxx
	g++ -o $@ -I. -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include --std=c++11 -MD -MF $@.d $(OPENMRNPATH)/src/nmranet/CompileCdiMain.cxx

clean: clean_cdi

.PHONY: clean_cdi

clean_cdi:
	rm -f cdi.xml compile_cdi	
