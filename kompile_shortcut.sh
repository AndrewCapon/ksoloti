#!/bin/bash
set -e

BUILD_AXOLOTI=0
BUILD_KSOLOTI=1
BUILD_NORMAL=1
BUILD_USBAUDIO=1
BUILD_SPILINK=0
BUILD_FLASHER=0
BUILD_MOUNTER=0

platform='unknown'
unamestr=`uname`
case "$unamestr" in
    Linux)
        platform='linux'
        rootdir="$(dirname $(readlink -f $0))"
    ;;
    Darwin)
        platform='mac'
        rootdir="$(cd $(dirname $0); pwd -P)"
    ;;
    MINGW*)
        platform='windows'
        rootdir="$(cd $(dirname $0); pwd -P)"
    ;;
    MSYS*)
        platform='windows_bash'
        rootdir="$(cd $(dirname $0); pwd -P)"
    ;;
    *)
        echo "Unknown OS: $unamestr - aborting..."
        exit
    ;;
esac

case "$platform" in
    mac)
        rm -f firmware.log

        # # compile board mode and firmware options
        if [ $BUILD_AXOLOTI -eq 1 ] 
        then
            echo "********************"
            echo "* Building Axoloti *"
            echo "********************"

            sh ./platform_osx/compile_firmware.sh BOARD_AXOLOTI_CORE $BUILD_NORMAL $BUILD_USBAUDIO $BUILD_SPILINK $BUILD_FLASHER $BUILD_MOUNTER $2>&1 | tee firmware.log
        fi

        # compile board mode and firmware options
        if [ $BUILD_KSOLOTI -eq 1 ] 
        then
            echo "********************"
            echo "* Building Ksoloti *"
            echo "********************"

            sh ./platform_osx/compile_firmware.sh BOARD_KSOLOTI_CORE $BUILD_NORMAL $BUILD_USBAUDIO $BUILD_SPILINK $BUILD_FLASHER $BUILD_MOUNTER 2>&1 | tee -a firmware.log
        fi
    ;;
    linux)
        # rm -f ./firmware/build/*.*
        # sh ./qlean.sh

        # compile board mode and firmware options
        sh ./platform_linux/compile_firmware.sh BOARD_AXOLOTI_CORE 2>&1 | tee firmware.log

        # compile board mode and firmware options
        sh ./platform_linux/compile_firmware.sh BOARD_KSOLOTI_CORE 2>&1 | tee -a firmware.log
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/ksoloti/usbaudio/ksoloti_usbaudio.elf > ./firmware/build/ksoloti_usbaudio.lst
    ;;
    windows)
        # rm -f ./firmware/build/*.*
        # sh ./qlean.sh
        cd platform_win

        # compile board mode and firmware options
        cmd "//C path.bat && compile_firmware.bat BOARD_AXOLOTI_CORE 2>&1 | tee ..\firmware.log"

        # compile board mode and firmware options
        cmd "//C path.bat && compile_firmware.bat BOARD_KSOLOTI_CORE 2>&1 | tee -a ..\firmware.log"

        cd ..
        sh ./qlean.sh
    ;;
    windows_bash)
        # rm -f ./firmware/build/*.*
        # sh ./qlean.sh

        # compile board mode and firmware options
        sh ./platform_win/compile_firmware.sh BOARD_AXOLOTI_CORE 2>&1 | tee firmware.log

        # compile board mode and firmware options
        sh ./platform_win/compile_firmware.sh BOARD_KSOLOTI_CORE 2>&1 | tee -a firmware.log
    ;;
esac