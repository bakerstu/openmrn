CC = gcc
CXX = g++
AR = ar
LD = g++

CFLAGS = -c -g -O0 -Wall -Werror -MD -MP -std=gnu99 -m32 -fno-stack-protector \
         -D_GNU_SOURCE
CXXFLAGS = -c -g -O0 -Wall -Werror -MD -MP -m32 -fno-stack-protector \
           -D_GNU_SOURCE

LDFLAGS = -g -m32
SYSLIBRARIES = -lrt -lpthread

EXTENTION =

