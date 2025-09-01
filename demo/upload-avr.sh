#!/bin/sh

if [ $# -lt 2 ]; then
  echo "USAGE: upload-avr.sh BOARD PORT"
  echo "EXAMPLE: upload-avr.sh uno /dev/cu.usbserial-110"
  echo "EXAMPLE: upload-avr.sh nano /dev/cu.usbserial-110"
  echo "EXAMPLE: upload-avr.sh nano:cpu=atmega328old /dev/cu.usbserial-110"
  exit 1
fi

fqbn=arduino:avr:$1 
port=$2

what=`pwd`
what=${what##*/}
echo "uploading $what for $fqbn to port $port"

arduino-cli upload --fqbn $fqbn -p $port

