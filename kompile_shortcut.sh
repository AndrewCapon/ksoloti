#!/bin/bash
set -e

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
        rm firmware.log
        #rm -f ./firmware/build/*.*
        #sh ./qlean.sh

        # compile board mode and firmware options
        sh ./platform_osx/compile_firmware.sh BOARD_AXOLOTI_CORE 2>&1 | tee firmware.log
        # # create .lst files
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/flasher/flasher_build/axoloti_flasher/axoloti_flasher.elf > ./firmware/flasher/flasher_build/axoloti_flasher.lst
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/mounter/mounter_build/axoloti_mounter/axoloti_mounter.elf > ./firmware/mounter/mounter_build/axoloti_mounter.lst 
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/axoloti/normal/axoloti.elf > ./firmware/build/axoloti.lst
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/axoloti/spilink/axoloti_spilink.elf > ./firmware/build/axoloti_spilink.lst
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/axoloti/usbaudio/axoloti_usbaudio.elf > ./firmware/build/axoloti_usbaudio.lst

        # compile board mode and firmware options
        sh ./platform_osx/compile_firmware.sh BOARD_KSOLOTI_CORE 2>&1 | tee -a firmware.log
        # create .lst files
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/flasher/flasher_build/ksoloti_flasher/ksoloti_flasher.elf > ./firmware/flasher/flasher_build/ksoloti_flasher.lst
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/mounter/mounter_build/ksoloti_mounter/ksoloti_mounter.elf > ./firmware/mounter/mounter_build/ksoloti_mounter.lst
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/ksoloti/normal/ksoloti.elf > ./firmware/build/ksoloti.lst
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/ksoloti/spilink/ksoloti_spilink.elf > ./firmware/build/ksoloti_spilink.lst
        ./platform_osx/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/ksoloti/usbaudio/ksoloti_usbaudio.elf > ./firmware/build/ksoloti_usbaudio.lst
        # sh ./qlean.sh
    ;;
    linux)
        # rm -f ./firmware/build/*.*
        # sh ./qlean.sh

        # compile board mode and firmware options
        sh ./platform_linux/compile_firmware.sh BOARD_AXOLOTI_CORE 2>&1 | tee firmware.log
        # create .lst files
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/flasher/flasher_build/axoloti_flasher/axoloti_flasher.elf > ./firmware/flasher/flasher_build/axoloti_flasher.lst
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/mounter/mounter_build/axoloti_mounter/axoloti_mounter.elf > ./firmware/mounter/mounter_build/axoloti_mounter.lst 
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/axoloti/normal/axoloti.elf > ./firmware/build/axoloti.lst
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/axoloti/spilink/axoloti_spilink.elf > ./firmware/build/axoloti_spilink.lst
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/axoloti/usbaudio/axoloti_usbaudio.elf > ./firmware/build/axoloti_usbaudio.lst

        # compile board mode and firmware options
        sh ./platform_linux/compile_firmware.sh BOARD_KSOLOTI_CORE 2>&1 | tee -a firmware.log
        # create .lst files
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/flasher/flasher_build/ksoloti_flasher/ksoloti_flasher.elf > ./firmware/flasher/flasher_build/ksoloti_flasher.lst
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/mounter/mounter_build/ksoloti_mounter/ksoloti_mounter.elf > ./firmware/mounter/mounter_build/ksoloti_mounter.lst
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/ksoloti/normal/ksoloti.elf > ./firmware/build/ksoloti.lst
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/ksoloti/spilink/ksoloti_spilink.elf > ./firmware/build/ksoloti_spilink.lst
        ./platform_linux/bin/arm-none-eabi-objdump --source-comment --demangle --disassemble ./firmware/build/ksoloti/usbaudio/ksoloti_usbaudio.elf > ./firmware/build/ksoloti_usbaudio.lst
        # sh ./qlean.sh
    ;;
    windows)
        # rm -f ./firmware/build/*.*
        # sh ./qlean.sh
        cd platform_win

        # compile board mode and firmware options
        cmd "//C path.bat && compile_firmware.bat BOARD_AXOLOTI_CORE 2>&1 | tee ..\firmware.log"
        # create .lst files
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\flasher\flasher_build\axoloti_flasher.elf > ..\firmware\flasher\flasher_build\axoloti_flasher.lst"
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\mounter\mounter_build\axoloti_mounter.elf > ..\firmware\mounter\mounter_build\axoloti_mounter.lst"
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\build\axoloti\normal\axoloti.elf > ..\firmware\build\axoloti.lst"
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\build\axoloti\spilink\axoloti_spilink.elf > ..\firmware\build\axoloti_spilink.lst"
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\build\axoloti\usbaudio\axoloti_usbaudio.elf > ..\firmware\build\axoloti_usbaudio.lst"

        # compile board mode and firmware options
        cmd "//C path.bat && compile_firmware.bat BOARD_KSOLOTI_CORE 2>&1 | tee -a ..\firmware.log"
        # create .lst files
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\flasher\flasher_build\ksoloti_flasher.elf > ..\firmware\flasher\flasher_build\ksoloti_flasher.lst"
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\mounter\mounter_build\ksoloti_mounter.elf > ..\firmware\mounter\mounter_build\ksoloti_mounter.lst"
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\build\ksoloti\normal\ksoloti.elf > ..\firmware\build\ksoloti.lst"
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\build\ksoloti\spilink\ksoloti_spilink.elf > ..\firmware\build\ksoloti_spilink.lst"
        cmd "//C path.bat && arm-none-eabi-objdump --source-comment --demangle --disassemble ..\firmware\build\ksoloti\usbaudio\ksoloti_usbaudio.elf > ..\firmware\build\ksoloti_usbaudio.lst"

        cd ..
        sh ./qlean.sh
    ;;
    windows_bash)
        # rm -f ./firmware/build/*.*
        # sh ./qlean.sh

        # compile board mode and firmware options
        sh ./platform_win/compile_firmware.sh BOARD_AXOLOTI_CORE 2>&1 | tee firmware.log
        # create .lst files
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/flasher/flasher_build/axoloti_flasher/axoloti_flasher.elf > ./firmware/flasher/flasher_build/axoloti_flasher.lst
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/mounter/mounter_build/axoloti_mounter/axoloti_mounter.elf > ./firmware/mounter/mounter_build/axoloti_mounter.lst 
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/build/axoloti/normal/axoloti.elf > ./firmware/build/axoloti.lst
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/build/axoloti/spilink/axoloti_spilink.elf > ./firmware/build/axoloti_spilink.lst
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/build/axoloti/usbaudio/axoloti_usbaudio.elf > ./firmware/build/axoloti_usbaudio.lst

        # compile board mode and firmware options
        sh ./platform_win/compile_firmware.sh BOARD_KSOLOTI_CORE 2>&1 | tee -a firmware.log
        # create .lst files
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/flasher/flasher_build/ksoloti_flasher/ksoloti_flasher.elf > ./firmware/flasher/flasher_build/ksoloti_flasher.lst
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/mounter/mounter_build/ksoloti_mounter/ksoloti_mounter.elf > ./firmware/mounter/mounter_build/ksoloti_mounter.lst
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/build/ksoloti/normal/ksoloti.elf > ./firmware/build/ksoloti.lst
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/build/ksoloti/spilink/ksoloti_spilink.elf > ./firmware/build/ksoloti_spilink.lst
        ./platform_win/bin/arm-none-eabi-objdump.exe --source-comment --demangle --disassemble ./firmware/build/ksoloti/usbaudio/ksoloti_usbaudio.elf > ./firmware/build/ksoloti_usbaudio.lst
        # sh ./qlean.sh
    ;;
esac