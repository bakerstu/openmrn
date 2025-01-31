#!/bin/bash
#
# This script exports the OpenMRN code into an Arduino compatible library
# format.
#

#set -x

function usage() {
    echo
    echo 'usage: libify.sh path/to/arduino/library/output path/to/openmrn [-f] [-l] [-i] [-r]'
    echo 'exports OpenMRN code as an arduino library.'
    echo 'example: libify.sh ~/Arduino/libraries/OpenMRNLite .. -l'
    echo '(options must come after the path specification)'
    echo '-f will erase the target library before exporting.'
    echo '-l will create symlinks instead of copying files.'
    echo '-i will create OpenMRNIDF repository instead of arduino.'
    echo '-r will create relative symlinks. OpenMRNPath has to be a relative path from the library export directory back to openmrn, starting with ../'
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
ORIGOMRNPATH="${2}"

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
TARGET_IDF=

while [ "x$1" != "x" ] ; do
    case $1 in
        -f)
            echo "Cleaning ${TARGET_LIB_DIR}"
            rm -rf ${TARGET_LIB_DIR}/*
            ;;
        -l)
            USE_LINK=-s
            ;;
        -i)
            TARGET_IDF=1
            ;;
        -v)
            VERBOSE=1
            ;;
        -r)
            USE_LINK=-s
            export RELATIVE=1
            OPENMRNPATH="${ORIGOMRNPATH}"
            ;;
    esac
    shift
done

# Creates a relative path to the toplevel of the directory.
# $1 is a directory path without trailing /, such as
# 'src/freertos_drivers/esp32"
# prints to stdout a relative path like "../../.." to get back to the toplevel
# from the given subdiretory.
function get_relative() {
    if [ "x${RELATIVE}" == "x" ]; then
       # print nothing
       return
    fi      
    if [ "x${1}" == "x." ] ; then
       echo "./"
       return
    fi
    SUB="$(echo $1 | sed s/[^/]//g)"
    case ${SUB} in
        "")
            echo "../"
            ;;
        "/")
            echo "../../"
            ;;
        "//")
            echo "../../../"
            ;;
        *)
            echo UNKNOWN SUBTREE "'"${SUB}"'"
    esac
}

# Arguments:
# $1 is the relative path in the library directory
# $2... is the relative path in openmrn tree with the filename
# Will create necessary directories internally.
function copy_file() {
    REL_DIR="$1"
    INVERSE_DIR="$(get_relative $1)"
    shift
    if [ "x$VERBOSE" != "x" ]; then
        echo "Creating ${TARGET_LIB_DIR}/${REL_DIR}"
    fi
    mkdir -p ${TARGET_LIB_DIR}/${REL_DIR}
    pushd ${TARGET_LIB_DIR}/${REL_DIR} >/dev/null
    while [ "x$1" != "x" ] ; do
        if [ "x$VERBOSE" != "x" ]; then
            echo "${INVERSE_DIR}${OPENMRNPATH}/${1} ==> ${TARGET_LIB_DIR}/${REL_DIR}"
        fi

        if [[ "$OSTYPE" == "darwin"* ]]; then
            cp -fa ${USE_LINK} ${INVERSE_DIR}${OPENMRNPATH}/${1} .
        else
            cp -fax ${USE_LINK} ${INVERSE_DIR}${OPENMRNPATH}/${1} .
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
        echo "${INVERSE_DIR}${OPENMRNPATH}/${2} ==> ${TARGET_LIB_DIR}/$1"
    fi

    if [[ "$OSTYPE" == "darwin"* ]]; then
        cp -fa ${USE_LINK} ${INVERSE_DIR}${OPENMRNPATH}/$2 .
    else
        cp -faxr ${USE_LINK} ${INVERSE_DIR}${OPENMRNPATH}/$2 .
    fi

    popd >/dev/null
}

if [ "x$TARGET_IDF" == "x" ]; then
    copy_file . arduino/{library.json,library.properties,keywords.txt,README.md,LICENSE,CONTRIBUTING.md}
    copy_dir . arduino/examples
    copy_file src arduino/OpenMRNLite.{h,cpp}
else
    copy_file . arduino/LICENSE arduino/idf/{CMakeLists.txt,README.md}
    copy_file src arduino/idf/library.properties arduino/keywords.txt
fi

copy_file src arduino/CDIXMLGenerator.hxx \
    include/{can_frame.h,nmranet_config.h,openmrn_features.h,i2c.h,i2c-dev.h} \
    include/freertos/{bootloader_hal.h,can_ioctl.h,endian.h,freertos_includes.h,stropts.h} \
    include/freertos_select/ifaddrs.h

# General DCC related files (all headers and DCC packet related cxx)
copy_file src/dcc src/dcc/*.hxx src/dcc/*.h src/dcc/{Defs,dcc_constants,DccDebug,LocalTrackIf,Packet}.cxx

# RailCom related DCC files
copy_file src/dcc src/dcc/{RailCom,RailcomBroadcastDecoder,RailcomDebug}.cxx

# Command Station DCC related files
copy_file src/dcc src/dcc/{Loco,SimpleUpdateLoop,UpdateLoop}.cxx

# remove test framework related file
rm -f ${TARGET_LIB_DIR}/src/dcc/dcc_test_utils.hxx

copy_file src/executor src/executor/*.hxx src/executor/*.cxx
copy_file src/openlcb src/openlcb/*.hxx src/openlcb/*.cxx

rm -f ${TARGET_LIB_DIR}/src/openlcb/CompileCdiMain.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/EventHandlerMock.hxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.cxx \
    ${TARGET_LIB_DIR}/src/openlcb/Stream.hxx

copy_file src/freertos_drivers/arduino \
          src/freertos_drivers/arduino/* \
          src/freertos_drivers/common/DeviceBuffer.{hxx,cxx} \
          src/freertos_drivers/common/DummyGPIO.hxx \
          src/freertos_drivers/common/GpioWrapper.hxx \
          src/freertos_drivers/common/CpuLoad.{hxx,cxx} \
          src/freertos_drivers/common/WifiDefs.{hxx,cxx} \
          src/freertos_drivers/common/libatomic.c \
          src/freertos_drivers/common/PWM.hxx \
          src/freertos_drivers/common/RailcomDriver.hxx

copy_file src/freertos_drivers/esp32 \
          src/freertos_drivers/esp32/*

copy_file src/freertos_drivers/stm32 \
          src/freertos_drivers/st/Stm32Can.* \
          arduino/stm32f_hal_conf.hxx \

copy_file src/os src/os/*.h src/os/*.c src/os/*.hxx \
          src/os/{OSImpl,MDNS,OSSelectWakeup}.cxx

copy_file src/sys include/sys/tree.hxx

copy_file src/utils src/utils/*.{cxx,hxx,c,h}

rm -f ${TARGET_LIB_DIR}/src/utils/ReflashBootloader.cxx \
    ${TARGET_LIB_DIR}/src/utils/AesCcmTestVectors.hxx \
    ${TARGET_LIB_DIR}/src/utils/AesCcmTestVectorsEx.hxx \
    ${TARGET_LIB_DIR}/src/utils/async_datagram_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/async_if_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/async_stream_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/async_traction_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/EEPROMEmuTest.hxx \
    ${TARGET_LIB_DIR}/src/utils/hub_test_utils.hxx \
    ${TARGET_LIB_DIR}/src/utils/if_tcp_test_helper.hxx \
    ${TARGET_LIB_DIR}/src/utils/test_main.hxx \
    ${TARGET_LIB_DIR}/src/utils/ShaTestVectors.hxx

if [ "x$VERBOSE" != "x" ]; then
    echo "Renaming all cxx to cpp under ${TARGET_LIB_DIR}/src"
fi
find ${TARGET_LIB_DIR}/src -name '*.cxx' -print0 | sed 's/.cxx//g' | xargs -0 -I % mv %.cxx %.cpp
