#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile).#
.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk image

# ------------------------------------------------------------------------------------
# Rules for buildStep: build and debug
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
image: D:/tmpOpenMRN/applications/blink_raw/targets/freertos.mips4k.pic32mx.lccbuffer/blink_raw.elf nbproject/Makefile-default.mk 
else
image: D:/tmpOpenMRN/applications/blink_raw/targets/freertos.mips4k.pic32mx.lccbuffer/blink_raw.hex nbproject/Makefile-default.mk 
endif

.PHONY: D:/tmpOpenMRN/applications/blink_raw/targets/freertos.mips4k.pic32mx.lccbuffer/blink_raw.elf
D:/tmpOpenMRN/applications/blink_raw/targets/freertos.mips4k.pic32mx.lccbuffer/blink_raw.elf: 
	cd /D . && make all

.PHONY: D:/tmpOpenMRN/applications/blink_raw/targets/freertos.mips4k.pic32mx.lccbuffer/blink_raw.hex
D:/tmpOpenMRN/applications/blink_raw/targets/freertos.mips4k.pic32mx.lccbuffer/blink_raw.hex: 
	cd /D . && make all


# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	cd /D . && make rclean clean

