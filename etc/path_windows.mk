# Path definitions that are specific to Windows builds


################### FreeRTOS #####################
FREERTOSPATH ?= $(shell \
sh -c "if [ -d /a/openmrn/FreeRTOS/default/Source ]; then echo a:/openmrn/FreeRTOS/default; \
     elif [ -d /b/openmrn/FreeRTOS/default/Source ]; then echo b:/openmrn/FreeRTOS/default; \
     elif [ -d /c/openmrn/FreeRTOS/default/Source ]; then echo c:/openmrn/FreeRTOS/default; \
     elif [ -d /d/openmrn/FreeRTOS/default/Source ]; then echo d:/openmrn/FreeRTOS/default; \
     elif [ -d /e/openmrn/FreeRTOS/default/Source ]; then echo e:/openmrn/FreeRTOS/default; \
     elif [ -d /f/openmrn/FreeRTOS/default/Source ]; then echo f:/openmrn/FreeRTOS/default; \
     elif [ -d /g/openmrn/FreeRTOS/default/Source ]; then echo g:/openmrn/FreeRTOS/default; \
     else echo ; fi" \
)

################### ARM-GCC #####################
ARMGCCPATH ?= $(shell \
sh -c "if [ -d /a/openmrn/windows/armgcc/default/bin ]; then echo a:/openmrn/windows/armgcc/default; \
     elif [ -d /b/openmrn/windows/armgcc/default/bin ]; then echo b:/openmrn/windows/armgcc/default; \
     elif [ -d /c/openmrn/windows/armgcc/default/bin ]; then echo c:/openmrn/windows/armgcc/default; \
     elif [ -d /d/openmrn/windows/armgcc/default/bin ]; then echo d:/openmrn/windows/armgcc/default; \
     elif [ -d /e/openmrn/windows/armgcc/default/bin ]; then echo e:/openmrn/windows/armgcc/default; \
     elif [ -d /f/openmrn/windows/armgcc/default/bin ]; then echo f:/openmrn/windows/armgcc/default; \
     elif [ -d /g/openmrn/windows/armgcc/default/bin ]; then echo g:/openmrn/windows/armgcc/default; \
     else echo ; fi" \
)

################### TIVAWAREPATH #####################
TIVAWAREPATH ?= $(shell \
sh -c "if [ -d /a/openmrn/ti/TivaWare/default/driverlib ]; then echo a:/openmrn/ti/TivaWare/default; \
     elif [ -d /b/openmrn/ti/TivaWare/default/driverlib ]; then echo b:/openmrn/ti/TivaWare/default; \
     elif [ -d /c/openmrn/ti/TivaWare/default/driverlib ]; then echo c:/openmrn/ti/TivaWare/default; \
     elif [ -d /d/openmrn/ti/TivaWare/default/driverlib ]; then echo d:/openmrn/ti/TivaWare/default; \
     elif [ -d /e/openmrn/ti/TivaWare/default/driverlib ]; then echo e:/openmrn/ti/TivaWare/default; \
     elif [ -d /f/openmrn/ti/TivaWare/default/driverlib ]; then echo f:/openmrn/ti/TivaWare/default; \
     elif [ -d /g/openmrn/ti/TivaWare/default/driverlib ]; then echo g:/openmrn/ti/TivaWare/default; \
     else echo ; fi" \
)

