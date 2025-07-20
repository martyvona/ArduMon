#!/bin/sh

if [ $# -lt 2 ]; then
  echo "USAGE: upload-stm32-dfu.sh FQBN BOARD"
  echo "EXAMPLE: upload-stm32-dfu.sh STMicroelectronics:stm32:GenF4 BLACKPILL_F411CE"
  exit 1
fi

STM32_CUBE_OS_X_NEW=/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin
if [ -d $STM32_CUBE_OS_X_NEW ]; then export PATH=$STM32_CUBE_OS_X_NEW:$PATH; fi

echo "make sure your device is in DFU mode:"
echo "hold down the RST button, press the BOOT button, release RST, then release BOOT"
echo "arduino-cli board list should show a device with protocol dfu and type USB DFU"
arduino-cli upload --fqbn $1 --board-options "pnum=$2,upload_method=dfuMethod"

