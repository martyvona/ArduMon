#!/bin/sh

if [ $# -lt 2 ]; then
  echo "USAGE: upload-stm32-mass-storage.sh FQBN BOARD"
  echo "EXAMPLE: upload-stm32-mass-storage.sh STMicroelectronics:stm32:Nucleo_32 NUCLEO_G431KB"
  echo "EXAMPLE: upload-stm32-mass-storage.sh STMicroelectronics:stm32:Nucleo_64 NUCLEO_G431RB"
  echo "EXAMPLE: upload-stm32-mass-storage.sh STMicroelectronics:stm32:Nucleo_64 NUCLEO_F411RE"
  exit 1
fi

fqbn=$1
board=$2

what=`pwd`
what=${what##*/}
echo "uploading $what for $fqbn $board (mass storage)"

arduino-cli upload --fqbn $fqbn --board-options "pnum=$board,upload_method=MassStorage"

