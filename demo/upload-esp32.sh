#!/bin/sh

if [ $# -lt 2 ]; then
  echo "USAGE: upload-esp32.sh FQBN PORT"
  echo "EXAMPLE: upload-esp32.sh esp32:esp32:esp32 /dev/cu.usb*"
  exit 1
fi

arduino-cli upload --fqbn $1 -p $2 --board-options "FlashMode=dio,UploadSpeed=115200"

