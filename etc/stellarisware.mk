STELLARISWAREPATH ?= $(shell \
sh -c "if [ \"X`printenv STELLARISWAREPATH`\" != \"X\" ]; then printenv STELLARISWAREPATH; \
     elif [ -d /opt/StellarisWare/driverlib ]; then echo /opt/StellarisWare; \
     elif [ -d ~/StellarisWare/driverlib ]; then echo ~/StellarisWare; \
     else echo STELLARISWAREPATH not found; fi" \
)

ifneq ($(STELLARISWAREPATH),STELLARISWAREPATH not found)
INCLUDES += -I$(STELLARISWAREPATH)
endif
