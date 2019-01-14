#!/bin/bash

TARGET_LIB_DIR=$1
OPENMRN_ROOT=$2

if [ "${TARGET_LIB_DIR}x" == "x" ]; then
    echo "TARGET_LIB_DIR NOT DEFINED"
fi

if [ "${OPENMRN_ROOT}x" == "x" ]; then
    echo "OPENMRN_ROOT NOT DEFINED"
fi

cp ${OPENMRN_ROOT}/arduino/library.json \
    ${OPENMRN_ROOT}/arduino/library.properties \
    ${OPENMRN_ROOT}/arduino/keywords.txt \
    ${TARGET_LIB_DIR}
cp -r ${OPENMRN_ROOT}/arduino/examples \
    ${TARGET_LIB_DIR}

mkdir ${TARGET_LIB_DIR}/src
cp ${OPENMRN_ROOT}/arduino/OpenMRN.cpp \
    ${OPENMRN_ROOT}/arduino/OpenMRN.h \
    ${OPENMRN_ROOT}/include/can_frame.h \
    ${OPENMRN_ROOT}/include/nmranet_config.h \
    ${OPENMRN_ROOT}/include/freertos/endian.h \
    ${TARGET_LIB_DIR}/src

mkdir ${TARGET_LIB_DIR}/src/dcc
cp ${OPENMRN_ROOT}/src/dcc/*.hxx \
    ${OPENMRN_ROOT}/src/dcc/*.h \
    ${TARGET_LIB_DIR}/src/dcc

mkdir ${TARGET_LIB_DIR}/src/executor
cp ${OPENMRN_ROOT}/src/executor/*.hxx \
    ${OPENMRN_ROOT}/src/executor/*.cxx \
    ${TARGET_LIB_DIR}/src/executor

mkdir ${TARGET_LIB_DIR}/src/freertos_drivers
mkdir ${TARGET_LIB_DIR}/src/freertos_drivers/arduino
cp ${OPENMRN_ROOT}/src/freertos_drivers/arduino/* \
    ${OPENMRN_ROOT}/src/freertos_drivers/common/DeviceBuffer.cxx \
    ${OPENMRN_ROOT}/src/freertos_drivers/common/DeviceBuffer.hxx \
    ${OPENMRN_ROOT}/src/freertos_drivers/common/GpioWrapper.hxx \
    ${TARGET_LIB_DIR}/src/freertos_drivers/arduino

mkdir ${TARGET_LIB_DIR}/src/openlcb
cp ${OPENMRN_ROOT}/src/openlcb/*.cxx \
    ${OPENMRN_ROOT}/src/openlcb/*.hxx \
    ${TARGET_LIB_DIR}/src/openlcb

rm -f ${TARGET_LIB_DIR}/src/openlcb/CompileCdiMain.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.hxx

mkdir ${TARGET_LIB_DIR}/src/os
cp ${OPENMRN_ROOT}/src/os/*.c \
    ${OPENMRN_ROOT}/src/os/*.h \
    ${OPENMRN_ROOT}/src/os/*.hxx \
    ${TARGET_LIB_DIR}/src/os

mkdir ${TARGET_LIB_DIR}/src/sys
cp ${OPENMRN_ROOT}/include/sys/tree.hxx \
    ${TARGET_LIB_DIR}/src/sys

mkdir ${TARGET_LIB_DIR}/src/utils
cp ${OPENMRN_ROOT}/src/utils/*.cxx \
    ${OPENMRN_ROOT}/src/utils/*.hxx \
    ${OPENMRN_ROOT}/src/utils/*.c \
    ${OPENMRN_ROOT}/src/utils/*.h \
    ${TARGET_LIB_DIR}/src/utils

rm -f ${TARGET_LIB_DIR}/src/utils/ReflashBootloader.cxx \
    ${TARGET_LIB_DIR}/src/utils/HubDeviceSelect.cxx \
    ${TARGET_LIB_DIR}/src/utils/HubDeviceSelect.hxx

find ${TARGET_LIB_DIR}/src -name '*.cxx' -print0 | sed 's/.cxx//g' | xargs -0 -I % mv %.cxx %.cpp
