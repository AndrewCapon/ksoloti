#!/usr/bin/env bash

# Downloads and builds all the required dependencies and toolchain executables
# Items already present are skipped to save your bandwidth.

set -e

PLATFORM_ROOT="$(cd $(dirname $0); pwd -P)"

cd "$PLATFORM_ROOT"

if [ ! -d "${PLATFORM_ROOT}/bin" ]; 
then
    mkdir "${PLATFORM_ROOT}/bin"
fi

if [ ! -d "${PLATFORM_ROOT}/lib" ]; 
then
    mkdir "${PLATFORM_ROOT}/lib"
fi

if [ ! -d "${PLATFORM_ROOT}/src" ]; 
then
    mkdir "${PLATFORM_ROOT}/src"
fi

## Obsolete... Chibios is included in the repo now
# if [ ! -d "${PLATFORM_ROOT}/../chibios" ]; 
# then
#     cd "${PLATFORM_ROOT}/src"
#     CH_VERSION=2.6.9
#     ARDIR=ChibiOS-ver${CH_VERSION}
#     ARCHIVE=${ARDIR}.zip
#     if [ ! -f ${ARCHIVE} ]; 
#     then
#         printf "\ndownloading ${ARCHIVE}\n"
# 		curl -L https://github.com/ChibiOS/ChibiOS/archive/ver${CH_VERSION}.zip > ${ARCHIVE}
#     else
#         printf "\n${ARCHIVE} already downloaded\n"
#     fi
#     unzip -q -o ${ARCHIVE}
#     mv ${ARDIR} chibios
#     cd chibios/ext
#     unzip -q -o ./fatfs-0.*-patched.zip
#     cd ../../
#     mv chibios ../..
# else
#     printf "\nchibios directory already present, skipping...\n"
# fi

if [ ! -f "$PLATFORM_ROOT/bin/arm-none-eabi-gcc" ]; 
then
    cd "${PLATFORM_ROOT}/src"
    ARCHIVE=gcc-arm-none-eabi-9-2020-q2-update-mac.tar.bz2
    if [ ! -f ${ARCHIVE} ]; 
    then
        printf "\ndownloading ${ARCHIVE}\n"
        curl -L https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/$ARCHIVE > $ARCHIVE
    else
        printf "\n${ARCHIVE} already downloaded\n"
    fi
    tar xfj ${ARCHIVE}
    cp -r gcc-arm-none-eabi-9-2020-q2-update/* ..
    rm -r gcc-arm-none-eabi-9-2020-q2-update
else
    printf "\nbin/arm-none-eabi-gcc already present, skipping...\n"
fi


if [ ! -f "$PLATFORM_ROOT/bin/libusb-1.0.0.dylib" ]; 
then
    cd "${PLATFORM_ROOT}/src"
    ARDIR=libusb-1.0.19
    ARCHIVE=${ARDIR}.tar.bz2
    if [ ! -f ${ARCHIVE} ]; 
    then
        printf "\ndownloading ${ARCHIVE}\n"
        curl -L http://sourceforge.net/projects/libusb/files/libusb-1.0/$ARDIR/$ARCHIVE/download > $ARCHIVE
    else
        printf "\n${ARCHIVE} already downloaded\n"
    fi
    tar xfj ${ARCHIVE}
    
    cd "${PLATFORM_ROOT}/src/libusb-1.0.19"

    patch -N -p1 < ../libusb.stdfu.patch

    ./configure --prefix="${PLATFORM_ROOT}/i386" CFLAGS="-arch i386 -mmacosx-version-min=10.6" LDFLAGS="-arch i386"
    make 
    make install
    make clean
    ./configure --prefix="${PLATFORM_ROOT}/x86_64" CFLAGS="-arch x86_64 -mmacosx-version-min=10.6" LDFLAGS="-arch x86_64"
    make 
    make install
    make clean

    cd $PLATFORM_ROOT/
    lipo -create x86_64/lib/libusb-1.0.0.dylib i386/lib/libusb-1.0.0.dylib -output lib/libusb-1.0.0.dylib

    cd $PLATFORM_ROOT/lib
    install_name_tool -id libusb-1.0.0.dylib libusb-1.0.0.dylib
else
    printf "\nlibusb already present, skipping...\n"
fi

if [ ! -f "${PLATFORM_ROOT}/bin/dfu-util" ]; 
then
    cd "${PLATFORM_ROOT}/src"
    ARDIR=dfu-util-0.11
    ARCHIVE=${ARDIR}.tar.gz
    if [ ! -f $ARCHIVE ]; 
    then
        printf "\ndownloading ${ARCHIVE}\n"
        curl -L http://dfu-util.sourceforge.net/releases/$ARCHIVE > $ARCHIVE
    else
        printf "\n$ARCHIVE already downloaded\n"
    fi
    tar xfz ${ARCHIVE}

    cd "${PLATFORM_ROOT}/src/${ARDIR}"
    ./configure --prefix="${PLATFORM_ROOT}/i386" USB_LIBS="${PLATFORM_ROOT}/lib/libusb-1.0.0.dylib" USB_CFLAGS=-I${PLATFORM_ROOT}/i386/include/libusb-1.0/ CFLAGS="-arch i386 -mmacosx-version-min=10.6" LDFLAGS="-arch i386"
    make 
    make install
    make clean

    cd "$PLATFORM_ROOT/src/$ARDIR"
    make clean
    ./configure --prefix="${PLATFORM_ROOT}/x86_64" USB_LIBS="${PLATFORM_ROOT}/lib/libusb-1.0.0.dylib" USB_CFLAGS=-I${PLATFORM_ROOT}/x86_64/include/libusb-1.0/ CFLAGS="-arch x86_64 -mmacosx-version-min=10.6" LDFLAGS="-arch x86_64"
    make 
    make install
    make clean

    cd "$PLATFORM_ROOT"
    lipo -create x86_64/bin/dfu-util i386/bin/dfu-util -output bin/dfu-util
else
    printf "\ndfu-util already present, skipping...\n"
fi

if [ ! -f "$PLATFORM_ROOT/bin/make" ]; 
then
    cd "${PLATFORM_ROOT}/src"
    ARDIR=make-4.3
    ARCHIVE=${ARDIR}.tar.gz

    if [ ! -f ${ARCHIVE} ]; 
    then
        printf "\ndownloading ${ARCHIVE}\n"
        curl -L http://ftp.gnu.org/gnu/make/$ARCHIVE > $ARCHIVE
    else
        printf "\n${ARCHIVE} already downloaded\n"
    fi

    tar xfz $ARCHIVE

    cd "${PLATFORM_ROOT}/src/${ARDIR}"
    ./configure --prefix="${PLATFORM_ROOT}/i386" CFLAGS="-arch i386 -mmacosx-version-min=10.6" LDFLAGS="-arch i386"
    make 
    make install
    make clean

    cd "${PLATFORM_ROOT}/src/${ARDIR}"
    ./configure --prefix="${PLATFORM_ROOT}/x86_64" CFLAGS="-arch x86_64 -mmacosx-version-min=10.6" LDFLAGS="-arch x86_64"
    make 
    make install
    make clean

    cd "${PLATFORM_ROOT}"
    lipo -create x86_64/bin/make i386/bin/make -output bin/make
fi

cd "${PLATFORM_ROOT}/../jdks"

JDK_ARCHIVE_LINUX="zulu21.42.19-ca-jdk21.0.7-linux_x64.zip"
if [ ! -f "${JDK_ARCHIVE_LINUX}" ];
then
    echo "##### downloading ${JDK_ARCHIVE_LINUX} #####"
    curl -L https://cdn.azul.com/zulu/bin/$JDK_ARCHIVE_LINUX > $JDK_ARCHIVE_LINUX
else
    echo "##### ${JDK_ARCHIVE_LINUX} already downloaded #####"
fi

JDK_ARCHIVE_MAC="zulu21.42.19-ca-jdk21.0.7-macosx_x64.zip"
if [ ! -f "${JDK_ARCHIVE_MAC}" ];
then
    echo "##### downloading ${JDK_ARCHIVE_MAC} #####"
    curl -L https://cdn.azul.com/zulu/bin/$JDK_ARCHIVE_MAC > $JDK_ARCHIVE_MAC
else
    echo "##### ${JDK_ARCHIVE_MAC} already downloaded #####"
fi

JDK_ARCHIVE_WINDOWS="zulu21.42.19-ca-jdk21.0.7-win_x64.zip"
if [ ! -f "${JDK_ARCHIVE_WINDOWS}" ];
then
    echo "##### downloading ${JDK_ARCHIVE_WINDOWS} #####"
    curl -L https://cdn.azul.com/zulu/bin/$JDK_ARCHIVE_WINDOWS > $JDK_ARCHIVE_WINDOWS
else
    echo "##### ${JDK_ARCHIVE_WINDOWS} already downloaded #####"
fi

#cp -v "${PLATFORM_ROOT}/lib/"*.dylib "${PLATFORM_ROOT}/bin/"

file "${PLATFORM_ROOT}/bin/make"
file "${PLATFORM_ROOT}/bin/dfu-util"
file "${PLATFORM_ROOT}/bin/libusb-1.0.0.dylib"

printf "\n##### building firmware... #####\n"
cd "${PLATFORM_ROOT}"/..
./firmware/compile_firmware.sh BOARD_AXOLOTI_CORE
./firmware/compile_firmware.sh BOARD_KSOLOTI_CORE

printf "\n##### building GUI... #####\n"
ant

printf "\nDONE!\n"
