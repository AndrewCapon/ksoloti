#!/bin/sh
set -e

export PATH=${axoloti_runtime}/platform_osx/bin:$PATH

cd "${axoloti_firmware}"
make BOARDDEF=$1 -f Makefile.patch.mk clean

if [ $1 == "BOARD_KSOLOTI_CORE" ]; then
  FLASHER_PROJECT=ksoloti_flasher
  MOUNTER_PROJECT=ksoloti_mounter
else if [ $1 == "BOARD_AXOLOTI_CORE" ]; then
  FLASHER_PROJECT=axoloti_flasher
  MOUNTER_PROJECT=axoloti_mounter
fi
fi

echo "Compiling firmware flasher... $1"
cd flasher
mkdir -p .dep
mkdir -p flasher_build/$FLASHER_PROJECT/lst
mkdir -p flasher_build/$FLASHER_PROJECT/obj
if ! make -j16 BOARDDEF=$1; then
    exit 1
fi
cp flasher_build/$FLASHER_PROJECT/$FLASHER_PROJECT.* flasher_build/
cd ..

echo "Compiling firmware mounter... $1"
cd mounter
mkdir -p .dep
mkdir -p mounter_build/$MOUNTER_PROJECT/lst
mkdir -p mounter_build/$MOUNTER_PROJECT/obj
if ! make -j16 BOARDDEF=$1; then
    exit 1
fi
cp mounter_build/$MOUNTER_PROJECT/$MOUNTER_PROJECT.* mounter_build/
cd ..

echo "Compiling firmware... $1"
export BUILDDIR=build/normal
mkdir -p .dep
mkdir -p $BUILDDIR/lst
mkdir -p $BUILDDIR/obj
if ! make -j16 BOARDDEF=$1; then
    exit 1
fi
cp $BUILDDIR/ksoloti.* build

echo "Compiling firmware... $1 FW_SPILINK"
export BUILDDIR=build/spilink
mkdir -p .dep
mkdir -p $BUILDDIR/lst
mkdir -p $BUILDDIR/obj
if ! make -j16 BOARDDEF=$1 FWOPTIONDEF=FW_SPILINK; then
    exit 1
fi
cp $BUILDDIR/ksoloti_spilink.* build


echo "Compiling firmware... $1 FW_USBAUDIO"
export BUILDDIR=build/usbaudio
mkdir -p .dep
mkdir -p $BUILDDIR/lst
mkdir -p $BUILDDIR/obj
if ! make -j16 BOARDDEF=$1 FWOPTIONDEF=FW_USBAUDIO; then
    exit 1
fi
cp $BUILDDIR/ksoloti_usbaudio.* build
