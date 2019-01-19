#!/bin/bash

TARGET_LIB_DIR=$1
OPENMRNPATH=$2

if [ "${TARGET_LIB_DIR}x" == "x" ]; then
    echo "TARGET_LIB_DIR NOT DEFINED"
fi

if [ "${OPENMRNPATH}x" == "x" ]; then
    echo "OPENMRNPATH NOT DEFINED"
fi

cp ${OPENMRNPATH}/arduino/library.json \
    ${OPENMRNPATH}/arduino/library.properties \
    ${OPENMRNPATH}/arduino/keywords.txt \
    ${TARGET_LIB_DIR}
cp -r ${OPENMRNPATH}/arduino/examples \
    ${TARGET_LIB_DIR}

mkdir ${TARGET_LIB_DIR}/src
cp ${OPENMRNPATH}/arduino/OpenMRN.cpp \
    ${OPENMRNPATH}/arduino/OpenMRN.h \
    ${OPENMRNPATH}/include/can_frame.h \
    ${OPENMRNPATH}/include/nmranet_config.h \
    ${OPENMRNPATH}/include/freertos/endian.h \
    ${TARGET_LIB_DIR}/src

mkdir ${TARGET_LIB_DIR}/src/dcc
cp ${OPENMRNPATH}/src/dcc/*.hxx \
    ${OPENMRNPATH}/src/dcc/*.h \
    ${TARGET_LIB_DIR}/src/dcc

mkdir ${TARGET_LIB_DIR}/src/executor
cp ${OPENMRNPATH}/src/executor/*.hxx \
    ${OPENMRNPATH}/src/executor/*.cxx \
    ${TARGET_LIB_DIR}/src/executor

mkdir ${TARGET_LIB_DIR}/src/freertos_drivers
mkdir ${TARGET_LIB_DIR}/src/freertos_drivers/arduino
cp ${OPENMRNPATH}/src/freertos_drivers/arduino/* \
    ${OPENMRNPATH}/src/freertos_drivers/common/DeviceBuffer.cxx \
    ${OPENMRNPATH}/src/freertos_drivers/common/DeviceBuffer.hxx \
    ${OPENMRNPATH}/src/freertos_drivers/common/GpioWrapper.hxx \
    ${TARGET_LIB_DIR}/src/freertos_drivers/arduino

mkdir ${TARGET_LIB_DIR}/src/openlcb
cp ${OPENMRNPATH}/src/openlcb/*.cxx \
    ${OPENMRNPATH}/src/openlcb/*.hxx \
    ${TARGET_LIB_DIR}/src/openlcb

rm -f ${TARGET_LIB_DIR}/src/openlcb/CompileCdiMain.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.hxx

mkdir ${TARGET_LIB_DIR}/src/os
cp ${OPENMRNPATH}/src/os/*.c \
    ${OPENMRNPATH}/src/os/*.h \
    ${OPENMRNPATH}/src/os/*.hxx \
    ${TARGET_LIB_DIR}/src/os

mkdir ${TARGET_LIB_DIR}/src/sys
cp ${OPENMRNPATH}/include/sys/tree.hxx \
    ${TARGET_LIB_DIR}/src/sys

mkdir ${TARGET_LIB_DIR}/src/utils
cp ${OPENMRNPATH}/src/utils/*.cxx \
    ${OPENMRNPATH}/src/utils/*.hxx \
    ${OPENMRNPATH}/src/utils/*.c \
    ${OPENMRNPATH}/src/utils/*.h \
    ${TARGET_LIB_DIR}/src/utils

rm -f ${TARGET_LIB_DIR}/src/utils/ReflashBootloader.cxx \
    ${TARGET_LIB_DIR}/src/utils/HubDeviceSelect.cxx \
    ${TARGET_LIB_DIR}/src/utils/HubDeviceSelect.hxx

find ${TARGET_LIB_DIR}/src -name '*.cxx' -print0 | sed 's/.cxx//g' | xargs -0 -I % mv %.cxx %.cpp
