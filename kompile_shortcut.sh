#!/bin/bash
# sh ./qlean.sh
rm firmware/build/axoloti.elf
sh ./platform_osx/compile_firmware.sh 2>&1 | tee firmware.log
