#!/bin/sh

if [ $# -lt 1 ]; then
  echo "USAGE: build-avr.sh BOARD [-d]"
  echo "EXAMPLE: build-avr.sh uno"
  echo "EXAMPLE: build-avr.sh nano"
  exit 1
fi

dbg=
if [[ $# -gt 1 && $2 == "-d" ]]; then dbg=--optimize-for-debug; fi

arduino-cli compile --fqbn arduino:avr:$1 --libraries ../ -e $dbg .

