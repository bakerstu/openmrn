PREFIX = arm-none-eabi-

CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AR = $(PREFIX)ar
LD = $(PREFIX)g++

INCLUDES += -I/home/stu/nuttx-6.22/nuttx-export-6.22/include \
            -I $(LEVEL)include/nuttx

#CFLAGS = -c -g -O3 -Wall -Werror -MD -MP -std=gnu99 -D__nuttx__
#CXXFLAGS = -c -g -O3 -Wall -Werror -MD -MP -D__nuttx__

#ARCHOPTIMIZATION =
ARCHOPTIMIZATION = -O3 -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer

CFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -MD -MP -std=gnu99 -D__nuttx__ \
         -fno-builtin -fpic -msingle-pic-base -mpic-register=r10 \
         -march=armv7-m -mthumb -mfloat-abi=soft -Wstrict-prototypes \
         -fno-stack-protector
CXXFLAGS = -c -g $(ARCHOPTIMIZATION) -Wall -MD -MP -D__nuttx__ \
           -fno-builtin -fpic -msingle-pic-base -mpic-register=r10 \
           -march=armv7-m -mthumb -mfloat-abi=soft -Wstrict-prototypes \
           -fno-stack-protector -fno-exceptions

LDFLAGS = -g
SYSLIBRARIES = 

EXTENTION =

