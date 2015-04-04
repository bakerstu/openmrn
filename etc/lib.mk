include $(OPENMRNPATH)/etc/path.mk

ifndef TARGET
TARGET := $(shell basename `cd ../; pwd`)
export TARGET
endif
BASENAME := $(shell basename `pwd`)

ifdef PARENTDIR
LIBBASENAME := $(PARENTLIB)_$(BASENAME)
REL_DIR := $(PARENTDIR)/$(BASENAME)
else
LIBBASENAME := $(BASENAME)
REL_DIR := $(BASENAME)
endif

SRCDIR := $(OPENMRNPATH)/src/$(REL_DIR)
TGTDIR := $(OPENMRNPATH)/targets/$(TARGET)/$(REL_DIR)

VPATH = $(SRCDIR)
export PARENTDIR := $(REL_DIR)
export PARENTLIB := $(LIBBASENAME)

INCLUDES += -I./ -I$(OPENMRNPATH)/src/ -I$(OPENMRNPATH)/include
include $(OPENMRNPATH)/etc/$(TARGET).mk

exist := $(wildcard $(SRCDIR)/sources)
ifneq ($(strip $(exist)),)
include $(VPATH)/sources
else
exist := $(wildcard sources)
ifneq ($(strip $(exist)),)
include sources
else
FULLPATHCSRCS = $(wildcard $(VPATH)/*.c)
FULLPATHCXXSRCS = $(wildcard $(VPATH)/*.cxx)
FULLPATHCPPSRCS = $(wildcard $(VPATH)/*.cpp)
FULLPATHASMSRCS = $(wildcard $(VPATH)/*.S)
CSRCS = $(notdir $(FULLPATHCSRCS))
CXXSRCS = $(notdir $(FULLPATHCXXSRCS))
CPPSRCS = $(notdir $(FULLPATHCPPSRCS))
ASMSRCS = $(notdir $(FULLPATHASMSRCS))
endif
endif

OBJS = $(CXXSRCS:.cxx=.o) $(CPPSRCS:.cpp=.o) $(CSRCS:.c=.o) $(ARM_CSRCS:.c=.o) $(ASMSRCS:.S=.o)
LIBNAME = lib$(LIBBASENAME).a

ARM_CSRCS ?=
ARM_OBJS = $(ARM_CSRCS:.c=.o)

ARM_CFLAGS += $(INCLUDES)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)

DEPS += TOOLPATH
MISSING_DEPS:=$(call find_missing_deps,$(DEPS))

ifneq ($(MISSING_DEPS),)

all docs clean veryclean tests mksubdirs:
	@echo "******************************************************************"
	@echo "*"
	@echo "*   Unable to build for $(TARGET)/$(REL_DIR), missing dependencies: $(MISSING_DEPS)"
	@echo "*"
	@echo "******************************************************************"

else
.PHONY: all
all: $(LIBNAME)

ifneq ($(SUBDIRS),)
include $(OPENMRNPATH)/etc/recurse.mk
endif

-include $(OBJS:.o=.d)

.SUFFIXES:
.SUFFIXES: .o .c .cxx .cpp .S

.cpp.o:
	$(CXX) $(CXXFLAGS) -MD -MF $*.d $< -o $@

.cxx.o:
	$(CXX) $(CXXFLAGS) -MD -MF $*.d $< -o $@

.S.o:
	$(AS) $(ASFLAGS) -MD -MF $*.d $< -o $@

.c.o:
	$(CC) $(CFLAGS) -MD -MF $*.d $< -o $@

$(ARM_OBJS): %.o : %.c
	$(CC) $(ARM_CFLAGS) -MD -MF $*.d $< -o $@


$(LIBNAME): $(OBJS)
	$(AR) crs$(AROPTS) $(LIBNAME) $(OBJS)
	ln -sf $(TGTDIR)/$(LIBNAME) $(OPENMRNPATH)/targets/$(TARGET)/lib
	touch $(OPENMRNPATH)/targets/$(TARGET)/lib/timestamp

.PHONY: clean
clean:
	rm -rf *.o *.d *.a *.so *.dll *.otest *.dtest *.test *.gcda *.map *.gcno *.md5

.PHONY: veryclean
veryclean: clean


.PHONY: tests
tests : 

.PHONY: mksubdirs
mksubdirs : 

endif
