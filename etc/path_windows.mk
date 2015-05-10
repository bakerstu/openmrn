# Path definitions that are specific to Windows builds


################### FreeRTOS #####################
ARMGCC ?= $(shell \
sh -c "if [ -d /a/openmrn/FreeRTOS/default/Source ]; then echo /a/openmrn/FreeRTOS/default; \
     elif [ -d /b/openmrn/FreeRTOS/default/Source ]; then echo /b/openmrn/FreeRTOS/default; \
     elif [ -d /c/openmrn/FreeRTOS/default/Source ]; then echo /c/openmrn/FreeRTOS/default; \
     elif [ -d /d/openmrn/FreeRTOS/default/Source ]; then echo /d/openmrn/FreeRTOS/default; \
     elif [ -d /e/openmrn/FreeRTOS/default/Source ]; then echo /e/openmrn/FreeRTOS/default; \
     elif [ -d /f/openmrn/FreeRTOS/default/Source ]; then echo /f/openmrn/FreeRTOS/default; \
     elif [ -d /g/openmrn/FreeRTOS/default/Source ]; then echo /g/openmrn/FreeRTOS/default; \
     else echo ; fi" \
)

################### ARM-GCC #####################
ARMGCC ?= $(shell \
sh -c "if [ -d /a/openmrn/windows/armgcc/default/bin ]; then echo /a/openmrn/windows/armgcc/default; \
     elif [ -d /b/openmrn/windows/armgcc/default/bin ]; then echo /b/openmrn/windows/armgcc/default; \
     elif [ -d /c/openmrn/windows/armgcc/default/bin ]; then echo /c/openmrn/windows/armgcc/default; \
     elif [ -d /d/openmrn/windows/armgcc/default/bin ]; then echo /d/openmrn/windows/armgcc/default; \
     elif [ -d /e/openmrn/windows/armgcc/default/bin ]; then echo /e/openmrn/windows/armgcc/default; \
     elif [ -d /f/openmrn/windows/armgcc/default/bin ]; then echo /f/openmrn/windows/armgcc/default; \
     elif [ -d /g/openmrn/windows/armgcc/default/bin ]; then echo /g/openmrn/windows/armgcc/default; \
     else echo ; fi" \
)

