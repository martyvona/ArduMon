#!/bin/sh

if [ $# -lt 2 ]; then
  echo "USAGE: upload-esp32.sh FQBN PORT"
  echo "EXAMPLE: upload-esp32.sh esp32:esp32:esp32 /dev/cu.usb*"
  exit 1
fi

fqbn=$1
port=$2

what=`pwd`
what=${what##*/}
echo "uploading $what for $fqbn to port $port"

arduino-cli upload --fqbn $fqbn -p $port --board-options "FlashMode=dio,UploadSpeed=115200"

