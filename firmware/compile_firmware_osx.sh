#!/bin/sh
set -e

export PATH=${axoloti_runtime}/platform_osx/bin:$PATH
echo $path
which arm-none-eabi-gcc

echo "Compiling Ksoloti firmware... ${axoloti_firmware}"
cd "${axoloti_firmware}"
make -j16 -f Makefile.patch clean

mkdir -p build/obj
mkdir -p build/lst
if ! make $1 -j16; then
    exit 1
fi
# rm -rf .dep
# rm -rf build/obj
# rm -rf build/lst

# echo "Compiling Ksoloti firmware flasher..."
# cd flasher
# mkdir -p flasher_build/obj
# mkdir -p flasher_build/lst
# make -j16 $1
# # rm -rf .dep
# # rm -rf flasher_build/obj
# # rm -rf flasher_build/lst
# cd ..

# echo "Compiling Ksoloti firmware mounter..."
# cd mounter
# mkdir -p mounter_build/obj
# mkdir -p mounter_build/lst
# make -j16 $1
# # rm -rf .dep
# # rm -rf mounter_build/obj
# # rm -rf mounter_build/lst
# cd ..
