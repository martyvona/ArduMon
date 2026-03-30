#!/usr/bin/env bash

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if [[ $# -lt 1 ]]; then
  echo "USAGE: build-avr.sh BOARD [-d]"
  echo "EXAMPLE: build-avr.sh uno"
  echo "EXAMPLE: build-avr.sh nano"
  exit 1
fi

dbg=
[[ $# -gt 1 && $2 == "-d" ]] && dbg=--optimize-for-debug

fqbn=arduino:avr:$1 

what=`pwd`
what=${what##*/}
echo "building $what for $fqbn"

arduino-cli compile --fqbn $fqbn --libraries $script_dir/../../ --libraries $script_dir/../ -e $dbg .

