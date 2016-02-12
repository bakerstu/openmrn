# Get the $(CFLAGSENV), $(CXXFLAGSENV), $(LDFLAGSENV)
include $(OPENMRNPATH)/etc/env.mk

# Get the $(TOOLPATH)
include $(OPENMRNPATH)/etc/path.mk
TOOLPATH:=$(PRUCGTPATH)

PREFIX = $(TOOLPATH)/bin/

AS = $(PREFIX)asmpru
CC = $(PREFIX)clpru
CXX = $(PREFIX)clpru
AR = $(PREFIX)arpru
LD = $(PREFIX)clpru
SIZE = size
OBJCOPY = objcopy
#OBJDUMP = objdump

AROPTS=D
OBJDUMPOPTS=-C

STARTGROUP :=
ENDGROUP :=

INCLUDES += -I$(TOOLPATH)/include \
            -I$(OPENMRNPATH)/src \
            -I$(PRUICSSPATH)/include \
            -I$(PRUICSSPATH)/include/am335x

ASFLAGS +=

CFLAGS += -v3 -O4 --opt_for_speed=0 -g --c99 --define=am3359 --define=pru0 \
          --diag_warning=225 --display_error_number --diag_wrap=off \
          --endian=little --hardware_mac=on --obj_extension=.o \
          --emit_warnings_as_errors --verbose_diagnostics \
          $(INCLUDES) $(CFLAGSEXTRA) $(CFLAGSENV)

LDFLAGS += -v3 -O4 --opt_for_speed=0 -g --c99 \
           --diag_warning=225 --display_error_number --diag_wrap=off \
           --endian=little --hardware_mac=on -z --reread_libs \
           --warn_sections --rom_model target.cmd \
           -i$(TOOLPATH)/lib $(LDFLAGSEXTRA) $(LDFLAGSENV)

SYSLIB_SUBDIRS +=
SYSLIBRARIES += -llibc.a $(SYSLIBRARIESEXTRA)

EXTENTION = .out

