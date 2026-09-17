include $(OPENMRNPATH)/etc/test.mk
export TESTOPTIMIZATION := -m32 -O0
SYSLIBRARIES = -lrt -lpthread $(SYSLIBRARIESEXTRA)
