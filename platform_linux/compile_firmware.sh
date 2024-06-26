#!/bin/sh
set -e
platformdir="$(dirname $(readlink -f $0))"

export axoloti_release=${axoloti_release:="$platformdir/.."}
export axoloti_runtime=${axoloti_runtime:="$platformdir/.."}
export axoloti_firmware=${axoloti_firmware:="$axoloti_release/firmware"}
export axoloti_legacy_firmware=${axoloti_legacy_firmware:="$axoloti_release/firmware_axoloti_legacy"}
export axoloti_home=${axoloti_home:="$platformdir/.."}

echo "\n\n\n-----------------------------------------------------------------------------------"
"${axoloti_legacy_firmware}/compile_firmware_linux.sh" $1
echo "\n\n\n-----------------------------------------------------------------------------------"
cd ..
cd "${axoloti_firmware}"
"${axoloti_firmware}/compile_firmware_linux.sh" $1
