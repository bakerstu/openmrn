#!/bin/bash

###########
# Options #
###########
usage="$(basename "$0") [-h] [-je] [-d] -- download and install OpenMRN tools

where:
    -h  show this help text
    -j  include JMRI in the install
    -e  include eclipse in the install
    -d  installation directory, default=/opt"

INSTALL_JMRI=0
INSTALL_ECLIPSE=0
INSTALL_DIR="/opt"
TMPDIR="/tmp/openmrntoolsinstall"

while getopts ':hjed:' option; do
    case "$option" in
        h) echo "$usage"
            exit
            ;;
        j) INSTALL_JMRI=1
            ;;
        e) INSTALL_ECLIPSE=1
            ;;
        d) INSTALL_DIR=$OPTARG
            ;;
        :) echo "Option -$OPTARG must specify a directory"
            exit 1
            ;;
    esac
done

####################
# Permissions Test #
####################
if [ "$(id -u)" != 0 ] && [ "$INSTALL_DIR" == "/opt" ]; then
    echo "Installation to directory $INSTALL_DIR requires root privilages"
    exit 1
fi

###################################
# OS dependent download mechanism #
###################################
if [[ "$OSTYPE" == "linux-gnu" ]]; then
    function download
    {
        wget -P $1 $2
    }
elif [[ "$OSTYPE" == "darwin*" ]]; then
    function download
    {
        curl $1 -o $2
    }
fi

#####################
# Download packages #
#####################
if [[ "$OSTYPE" == "linux-gnu" ]]; then
    JMRIINSTALLNAME="JMRI.3.10.1-r28327.tgz"
    ARMGCCINSTALLNAME="gcc-arm-none-eabi-4_8-2014q1-20140314-linux.tar.bz2"
    ECLISEINSTALLNAME="eclipse-cpp-mars-R-linux-gtk.tar.gz"
elif [[ "$OSTYPE" == "darwin*" ]]; then
    JMRIINSTALLNAME="JMRI.3.10.1-r28327.dmg"
    ARMGCCINSTALLNAME="gcc-arm-none-eabi-4_8-2014q1-20140314-mac.tar.bz2"
    ECLIPSEINSTALLNAME="eclipse-cpp-mars-R-macosx-cocoa-x86_64.tar.gz"
fi

GMOCKURL="http://googlemock.googlecode.com/files/gmock-1.7.0.zip"
FREERTOSURL="http://downloads.sourceforge.net/project/freertos/FreeRTOS/V8.2.1/FreeRTOSV8.2.1.zip"
TIVAWAREURL="http://software-dl.ti.com/download/tiva-c/AYZBDHVNPVSW_EERZ8XXRAETY5KK48QW/SW-TM4C-2.1.1.71.exe"
LPCOPEN1769URL="http://www.lpcware.com/system/files/lpcopen_2_10_lpcxpresso_nxp_lpcxpresso_1769.zip"
STM32CUBEF0URL="http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware/stm32cubef0.zip"

JMRIURL="http://downloads.sourceforge.net/project/jmri/production%20files/$JMRIINSTALLNAME"
ARMGCCURL="https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q1-update/+download/$ARMGCCINSTALLNAME"
ECLIPSEURL="http://ftp.osuosl.org/pub/eclipse/technology/epp/downloads/release/mars/R/$ECLIPSEINSTALLNAME"

rm -rf $TMPDIR
mkdir $TMPDIR
if [[ $INSTALL_ECLIPSE -ne 0 ]]; then
    download $TMPDIR $ECLIPSEURL
fi
if [[ $INSTALL_JMRI -ne 0 ]]; then
    download $TMPDIR $JMRIURL
fi
download $TMPDIR $ARMGCCURL
download $TMPDIR $GMOCKURL
download $TMPDIR $FREERTOSURL
download $TMPDIR $LPCOPEN1769URL
download $TMPDIR $TIVAWAREURL
download $TMPDIR $STM32CUBEF0URL

####################
# Install packages #
####################
if [[ $INSTALL_ECLIPSE -ne 0 ]]; then
    #unpack Eclipse
    #tar -xzf eclipse-cpp-mars-R-linux-gtk.tar.gz
fi
if [[ $INSTALL_JMRI -ne 0 ]]; then
    #wget -P $TMPDIR $JMRIURL
fi

mkdir -p $INSTALL_DIR/ti/TivaWare
mkdir -p $INSTALL_DIR/nxp/lpc_chip
mkdir -p $INSTALL_DIR/st/STM32Cube_FW_F0
mkdir -p $INSTALL_DIR/FreeRTOS
mkdir -p $INSTALL_DIR/armgcc
mkdir -p $INSTALL_DIR/gmock

# unpack TivaWare
unzip -d $INSTALL_DIR/ti/TivaWare/SW-TM4C-2.1.1.71 $TMPDIR/SW-TM4C-2.1.1.71.exe
rm -f $INSTALL_DIR/ti/TivaWare/default
ln -rs $INSTALL_DIR/ti/TivaWare/SW-TM4C-2.1.1.71 $INSTALL_DIR/ti/TivaWare/default

# unpack LPCopen
unzip -d $INSTALL_DIR/nxp/lpc_chip/lpcopen_2_10_lpcxpresso_nxp_lpcxpresso_1769 $TMPDIR/lpcopen_2_10_lpcxpresso_nxp_lpcxpresso_1769.zip
rm -f $INSTALL_DIR/nxp/lpc_chip/lpc_chip_17xx_40xx
ln -rs $INSTALL_DIR/nxp/lpc_chip/lpcopen_2_10_lpcxpresso_nxp_lpcxpresso_1769/lpc_chip_175x_6x $INSTALL_DIR/nxp/lpc_chip/lpc_chip_17xx_40xx

#unpack STM32CubeF0
unzip -d $INSTALL_DIR/st/STM32Cube_FW_F0 $TMPDIR/stm32cubef0.zip
rm -f $INSTALL_DIR/st/STM32Cube_FW_F0/default
ln -rs $INSTALL_DIR/st/STM32Cube_FW_F0/STM32Cube_FW_F0_V1.2.0 $INSTALL_DIR/st/STM32Cube_FW_F0/default

#unpack FreeRTOS
unzip -d $INSTALL_DIR/FreeRTOS/ $TMPDIR/FreeRTOSV8.2.1.zip
rm -f $INSTALL_DIR/FreeRTOS/default
ln -rs $INSTALL_DIR/FreeRTOS/FreeRTOSV8.2.1/FreeRTOS $INSTALL_DIR/FreeRTOS/default

#unpack ARMGCC
tar -xjf $TMPDIR/gcc-arm-none-eabi-4_8-2014q1-20140314-linux.tar.bz2 -C $INSTALL_DIR/armgcc/ 
rm -f $INSTALL_DIR/armgcc/default
ln -rs $INSTALL_DIR/armgcc/gcc-arm-none-eabi-4_8-2014q1 $INSTALL_DIR/armgcc/default

#unpack GMock
unzip -d $INSTALL_DIR/gmock $TMPDIR/gmock-1.7.0.zip
rm -f $INSTALL_DIR/gmock/default
ln -rs $INSTALL_DIR/gmock/gmock-1.7.0 $INSTALL_DIR/gmock/default

printf "\n\n"
printf "*********************************************************************\n"
printf "* Post-install steps:\n"
printf "*     1) cd $INSTALL_DIR/gmock/default\n"
printf "*     2) ./configure\n"
printf "*     3) make\n"
printf "*\n"
printf "* Depending on the installation directory path, sudo may be reqired:\n"
printf "*     1) cd $INSTALL_DIR/gmock/default\n"
printf "*     2) sudo ./configure\n"
printf "*     3) sudo make\n"
printf "*********************************************************************\n"

