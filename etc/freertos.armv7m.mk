TOOLPATH = /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI
PREFIX = arm-none-eabi-

CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AR = $(PREFIX)ar
LD = $(PREFIX)g++

FREERTOSPATH = $(HOME)/FreeRTOS

INCLUDES += -I$(FREERTOSPATH)/Source/include \
            -I$(FREERTOSPATH)/Source/portable/GCC/ARM_CM3 \
            -I$(LEVEL)include/freertos \
            -I$(LEVEL)src/freertos_drivers/common

#ARCHOPTIMIZATION =
ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

CFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -Werror -MD -MP -std=gnu99 -D__FreeRTOS__ \
         -fno-builtin -fpic -msingle-pic-base -mpic-register=r10 \
         -march=armv7-m -mthumb -mfloat-abi=soft -Wstrict-prototypes \
         -fno-stack-protector -DGCC_ARMCM3 -DTARGET_IS_FIRESTORM_RA2
CXXFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -Werror -MD -MP -D__FreeRTOS__ \
           -fno-builtin -fpic -msingle-pic-base -mpic-register=r10 \
           -march=armv7-m -mthumb -mfloat-abi=soft \
           -fno-stack-protector -fno-exceptions -DGCC_ARMCM3 -DTARGET_IS_FIRESTORM_RA2

LDFLAGS = -g -T target.ld -L$(TOOLPATH)/arm-none-eabi/lib/thumb2
SYSLIBRARIES = -lfreertos -Wl,-whole-archive -lfreertos_drivers -Wl,-no-whole-archive

EXTENTION = .elf

