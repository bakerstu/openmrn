include hardware.mk

OBJS += cdi.o

$(EXECUTABLE)$(EXTENTION): cdi.o

-include *.d

cdi.nxml: compile_cdi
	./compile_cdi > $@

cdi.o : cdi.nxml $(OPENMRNPATH)/bin/build_cdi.py
	$(OPENMRNPATH)/bin/build_cdi.py -i $< -o cdi.cxxout
	$(CXX) $(CXXFLAGS) -MD -MF $*.d -x c++ cdi.cxxout -o $@

compile_cdi: config.hxx $(OPENMRNPATH)/src/nmranet/CompileCdiMain.cxx
	g++ -o $@ -I. -I$(OPENMRNPATH)/src -I$(OPENMRNPATH)/include $(CDIEXTRA) --std=c++11 -MD -MF $@.d $(OPENMRNPATH)/src/nmranet/CompileCdiMain.cxx

clean: clean_cdi

.PHONY: clean_cdi

clean_cdi:
	rm -f cdi.xml compile_cdi	
