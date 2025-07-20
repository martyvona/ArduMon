#!/bin/sh

if [ $# -lt 1 ]; then
  echo "USAGE: build-avr.sh BOARD"
  echo "EXAMPLE: build-avr.sh uno"
  echo "EXAMPLE: build-avr.sh nano"
  exit 1
fi

arduino-cli compile --fqbn arduino:avr:$1 --libraries ../ -e .

