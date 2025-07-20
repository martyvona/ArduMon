#!/bin/sh

if [ $# -lt 2 ]; then
  echo "USAGE: upload-stm32-swd.sh FQBN BOARD"
  echo "EXAMPLE: upload-stm32-swd.sh STMicroelectronics:stm32:GenF4 BLACKPILL_F411CE"
  echo "EXAMPLE: upload-stm32-swd.sh STMicroelectronics:stm32:Nucleo_32 NUCLEO_G431KB"
  echo "EXAMPLE: upload-stm32-swd.sh STMicroelectronics:stm32:Nucleo_64 NUCLEO_G431RB"
  echo "EXAMPLE: upload-stm32-swd.sh STMicroelectronics:stm32:Nucleo_64 NUCLEO_F411RE"
  exit 1
fi

STM32_CUBE_OS_X_NEW=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin
if [ -d $STM32_CUBE_OS_X_NEW ]; then export PATH=$STM32_CUBE_OS_X_NEW:$PATH; fi

arduino-cli upload --fqbn $1 --board-options "pnum=$2,upload_method=swdMethod"

