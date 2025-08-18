#!/bin/sh

if [ $# -lt 1 ]; then
  echo "USAGE: build-avr.sh BOARD [-d]"
  echo "EXAMPLE: build-avr.sh uno"
  echo "EXAMPLE: build-avr.sh nano"
  exit 1
fi

dbg=
if [[ $# -gt 1 && $2 == "-d" ]]; then dbg=--optimize-for-debug; fi

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

fqbn=arduino:avr:$1 

echo "building for $fqbn"

arduino-cli compile --fqbn $fqbn --libraries $script_dir/../ -e $dbg .

