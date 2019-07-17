#!/bin/bash
#
# This script exports the OpenMRN code into an Arduino compatible library
# format.
#

#set -x

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

function realpath_macOS() {
  OURPWD=$PWD
  cd "$(dirname "$1")"
  LINK=$(readlink "$(basename "$1")")
  while [ "$LINK" ]; do
    cd "$(dirname "$LINK")"
    LINK=$(readlink "$(basename "$1")")
  done
  REALPATH="$PWD/$(basename "$1")"
  cd "$OURPWD"
  echo "$REALPATH"
}

if [[ "$OSTYPE" == "darwin"* ]]; then
REALPATH=realpath_macOS
else
REALPATH=realpath
fi

TARGET_LIB_DIR=$($REALPATH $1 2>/dev/null)
OPENMRNPATH=$($REALPATH $2 2>/dev/null)

if [[ -z ${TARGET_LIB_DIR} ]]; then
  if [[ $1 ]]; then
    echo "$1 does not exist, creating $1"
    mkdir -p $1
    TARGET_LIB_DIR=$(realpath $1 2>/dev/null)
  fi
fi

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
VERBOSE=

while [ "x$1" != "x" ] ; do
    case $1 in
        -f)
            echo "Cleaning ${TARGET_LIB_DIR}"
            rm -rf ${TARGET_LIB_DIR}/*
            ;;
        -l)
            USE_LINK=-s
            ;;
        -v)
            VERBOSE=1
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
    if [ "x$VERBOSE" != "x" ]; then
        echo "Creating ${TARGET_LIB_DIR}/${REL_DIR}"
    fi
    mkdir -p ${TARGET_LIB_DIR}/${REL_DIR}
    pushd ${TARGET_LIB_DIR}/${REL_DIR} >/dev/null
    while [ "x$1" != "x" ] ; do
        if [ "x$VERBOSE" != "x" ]; then
            echo "${OPENMRNPATH}/${1} ==> ${TARGET_LIB_DIR}/${REL_DIR}"
        fi

        if [[ "$OSTYPE" == "darwin"* ]]; then
            cp -fa ${USE_LINK} ${OPENMRNPATH}/${1} .
        else
            cp -fax ${USE_LINK} ${OPENMRNPATH}/${1} .
        fi

        shift
    done
    popd >/dev/null
}

# Arguments:
# $1 is the relative path in the library directory
# $2 is the relative path in openmrn tree with the
# Will create necessary target directories internally.
function copy_dir() {
    if [ "x$VERBOSE" != "x" ]; then
        echo "Creating ${TARGET_LIB_DIR}/$1"
    fi
    mkdir -p ${TARGET_LIB_DIR}/$1
    pushd ${TARGET_LIB_DIR}/$1 >/dev/null
    
    if [ "x$VERBOSE" != "x" ]; then
        echo "${OPENMRNPATH}/${2} ==> ${TARGET_LIB_DIR}/$1"
    fi

    if [[ "$OSTYPE" == "darwin"* ]]; then
        cp -fa ${USE_LINK} ${OPENMRNPATH}/$2 .
    else
        cp -faxr ${USE_LINK} ${OPENMRNPATH}/$2 .
    fi

    popd >/dev/null
}

copy_file . arduino/{library.json,library.properties,keywords.txt,README.md,LICENSE,CONTRIBUTING.md}
copy_dir . arduino/examples

copy_file src arduino/OpenMRNLite.{h,cpp} \
    include/{can_frame.h,nmranet_config.h,openmrn_features.h} \
    include/freertos/{freertos_includes.h,endian.h} \
    include/freertos_select/ifaddrs.h

# General DCC related files (all headers and DCC packet related cxx)
copy_file src/dcc src/dcc/*.hxx src/dcc/*.h src/dcc/{DccDebug,Packet}.cxx

# RailCom related DCC files
copy_file src/dcc src/dcc/{RailCom,RailcomBroadcastDecoder,RailcomDebug}.cxx

# Command Station DCC related files
copy_file src/dcc src/dcc/{Loco,SimpleUpdateLoop,UpdateLoop}.cxx

# remove test framework related file
rm -f ${TARGET_LIB_DIR}/src/dcc/dcc_test_utils.hxx

copy_file src/executor src/executor/*.hxx src/executor/*.cxx
copy_file src/openlcb src/openlcb/*.hxx src/openlcb/*.cxx

rm -f ${TARGET_LIB_DIR}/src/openlcb/CompileCdiMain.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.hxx

copy_file src/freertos_drivers/arduino \
          src/freertos_drivers/common/DeviceBuffer.{hxx,cxx} \
          src/freertos_drivers/common/GpioWrapper.hxx \
          src/freertos_drivers/common/CpuLoad.{hxx,cxx} \
          src/freertos_drivers/common/WifiDefs.{hxx,cxx} \
          src/freertos_drivers/arduino/*

copy_file src/freertos_drivers/esp32 \
          src/freertos_drivers/esp32/*

copy_file src/os src/os/*.h src/os/*.c src/os/*.hxx \
          src/os/{OSImpl,MDNS,OSSelectWakeup}.cxx

copy_file src/sys include/sys/tree.hxx

copy_file src/utils src/utils/*.{cxx,hxx,c,h}

rm -f ${TARGET_LIB_DIR}/src/utils/ReflashBootloader.cxx \
    ${TARGET_LIB_DIR}/src/utils/async_datagram_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/async_if_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/async_traction_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/hub_test_utils.hxx \
    ${TARGET_LIB_DIR}/src/utils/if_tcp_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/test_main.hxx

if [ "x$VERBOSE" != "x" ]; then
    echo "Renaming all cxx to cpp under ${TARGET_LIB_DIR}/src"
fi
find ${TARGET_LIB_DIR}/src -name '*.cxx' -print0 | sed 's/.cxx//g' | xargs -0 -I % mv %.cxx %.cpp
