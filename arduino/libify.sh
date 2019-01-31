#!/bin/bash
#
# 
#

function usage() {
    echo
    echo 'usage: libify.sh path/to/arduino/library/output path/to/openmrn [-f] [-l]'
    echo 'exports OpenMRN code as an arduino library.'
    echo 'example: libify.sh ~/Arduino/libraries/OpenMRN .. -l'
    echo '(options must come after the path specification)'
    echo '-f will erase the target library before exporting.'
    echo '-l will create symlinks instead of copying files.'
    exit 1
}

TARGET_LIB_DIR=$(realpath $1)
OPENMRNPATH=$(realpath $2)

shift; shift

if [ "${TARGET_LIB_DIR}x" == "x" ]; then
    echo "TARGET_LIB_DIR NOT DEFINED"
    usage
fi

if [ "${OPENMRNPATH}x" == "x" ]; then
    echo "OPENMRNPATH NOT DEFINED"
    usage
fi

USE_LINK=

while [ "x$1" != "x" ] ; do
    case $1 in
        -f)
            rm -rf ${TARGET_LIB_DIR}/*
            ;;
        -l)
            USE_LINK=-s
            ;;
    esac
    shift
done

# Arguments:
# $1 is the relative path in the library directory
# $2... is the relative path in openmrn tree with the filename
# Will create necessary directories internally.
function copy_file() {
    REL_DIR=$1
    shift
    mkdir -p ${TARGET_LIB_DIR}/${REL_DIR}
    pushd ${TARGET_LIB_DIR}/${REL_DIR} >/dev/null
    while [ "x$1" != "x" ] ; do
        cp -fax ${USE_LINK} ${OPENMRNPATH}/${1} .
        shift
    done
    popd >/dev/null
}


# Arguments:
# $1 is the relative path in the library directory
# $2 is the relative path in openmrn tree with the
# Will create necessary target directories internally.
function copy_dir() {
    mkdir -p ${TARGET_LIB_DIR}/$1
    pushd ${TARGET_LIB_DIR}/$1 >/dev/null
    
    cp -faxr ${USE_LINK} ${OPENMRNPATH}/$2 .
    popd >/dev/null
}

copy_file . arduino/library.json arduino/library.properties arduino/keywords.txt
copy_dir . arduino/examples

copy_file src arduino/OpenMRN.{h,cpp} include/{can_frame.h,nmranet_config.h} include/freertos/endian.h

copy_file src/dcc src/dcc/*.hxx src/dcc/*.h
copy_file src/executor src/executor/*.hxx src/executor/*.cxx
copy_file src/openlcb src/openlcb/*.hxx src/openlcb/*.cxx

rm -f ${TARGET_LIB_DIR}/src/openlcb/CompileCdiMain.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.hxx

copy_file src/freertos_drivers/arduino \
          src/freertos_drivers/common/DeviceBuffer.{hxx,cxx} \
          src/freertos_drivers/common/GpioWrapper.hxx \
          src/freertos_drivers/arduino/*

copy_file src/os src/os/*.h src/os/*.c src/os/*.hxx

copy_file src/sys include/sys/tree.hxx

copy_file src/utils src/utils/*.{cxx,hxx,c,h}

rm -f ${TARGET_LIB_DIR}/src/utils/ReflashBootloader.cxx \
    ${TARGET_LIB_DIR}/src/utils/HubDeviceSelect.cxx \
    ${TARGET_LIB_DIR}/src/utils/HubDeviceSelect.hxx

find ${TARGET_LIB_DIR}/src -name '*.cxx' -print0 | sed 's/.cxx//g' | xargs -0 -I % mv %.cxx %.cpp
