#!/bin/bash
platformdir="$(cd $(dirname $0); pwd -P)"

export axoloti_release=${axoloti_release:="$platformdir/.."}
export axoloti_runtime=${axoloti_runtime:="$platformdir/.."}
export axoloti_firmware=${axoloti_firmware:="$axoloti_release/firmware"}
export axoloti_home=${axoloti_home:="$platformdir"}

cd ${platformdir}/bin
./dfu-util --transfer-size 4096 --device 0483:df11 -i 0 -a 0 -D "${axoloti_firmware}/build/$1" --dfuse-address=0x08000000:leave
