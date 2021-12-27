#!/bin/bash
#
# usage: find-distcc.sh /opt/armgcc/default/bin/arm-none-eabi-g++
#
# Checks if distcc is available right now,. If yes, returns `distcc
# compiler-short-name` otherwise returns the absolute path to the compiler for
# local execution.
#
# The example output for the above incovation might be:
# /opt/armgcc/default/bin/arm-none-eabi-g++
# or
# distcc armgcc-2018-q4-g++
#
# See DISTCC.md for additional information.

DISTCC_PORT=$(cat ~/.distcc/port 2>/dev/null)
if [ -z "${DISTCC_PORT}" ] ; then
    # .distcc/port file is not set up. This means the user does not want to use
    # distcc.
    echo "$1"
    exit
fi

if ! netstat --tcp -l -n | grep -q ":${DISTCC_PORT} " ; then
    # nobody is listening to the distcc port
    echo "$1"
    exit
fi

# Always enable remoting g++ and gcc
if [ "$1" == "gcc" -o "$1" == "g++" ]; then
    echo distcc "$1"
    exit
fi

#Find masquerading compiler name
#echo find ~/bin -type l -lname "$1" -print 
CNAME=$(find ~/bin -type l -lname "$1" -print)

if [ -z "${CNAME}" ] ; then
    echo missing distcc compiler link for "$1" >&2
    echo unknown-distcc-compiler
    exit
fi

echo distcc $(basename "${CNAME}")
