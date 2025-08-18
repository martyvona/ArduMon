#!/bin/sh

if [ $# -lt 1 ]; then
  echo "USAGE: build-esp32.sh FQBN"
  echo "EXAMPLE: build-esp32.sh esp32:esp32:esp32"
  echo "EXAMPLE: build-esp32.sh arduino:esp32:nano_nora"
  exit 1
fi

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

fqbn=$1

echo "building for $fqbn"

arduino-cli compile --fqbn $fqbn --libraries $script_dir/../ -e .
