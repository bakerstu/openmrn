OPENMRNPATH ?= $(abspath ../..)
include $(OPENMRNPATH)/etc/core_target.mk

SRCDIR = $(OPENMRNPATH)/src
HOST_TARGET=1

include $(OPENMRNPATH)/etc/core_test.mk

utils/OpenSSLAesCcm.test: SYSLIBRARIESEXTRA+=-lcrypto

clean veryclean: clean-gtest
