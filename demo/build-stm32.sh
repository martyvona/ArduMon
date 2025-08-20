#!/bin/bash

if [ $# -lt 2 ]; then
  echo "USAGE: build-stm32.sh FQBN PNUM"
  echo "EXAMPLE: build-stm32.sh STMicroelectronics:stm32:GenF4 BLACKPILL_F411CE"
  echo "EXAMPLE: build-stm32.sh STMicroelectronics:stm32:Nucleo_64 NUCLEO_F411RE"
  echo "EXAMPLE: build-stm32.sh STMicroelectronics:stm32:Nucleo_32 NUCLEO_G431KB"
  echo "EXAMPLE: build-stm32.sh STMicroelectronics:stm32:Nucleo_64 NUCLEO_G431RB"
  exit 1
fi

USB=CDCgen
if [[ $2 == NUCLEO_* ]]; then USB=none; fi

# blackpill
# https://www.adafruit.com/product/4877
# https://www.dfrobot.com/product-2338.html
# https://www.amazon.com/dp/B09V7GCMRX
# https://blog.hobbycomponents.com/?p=758
# STM32F411CEU6 512k Flash 128k SRAM 48 pin
# arduino-cli board details --fqbn STMicroelectronics:stm32:GenF4

# https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html
# STM32F411CE 512k flash 128k SRAM 64 pin
# STM32G431KB 128k flash 32k SRAM 32 pin
# STM32G431RB 128k flash 32k SRAM 64 pin
# arduino-cli board details --fqbn STMicroelectronics:stm32:Nucleo_32
# arduino-cli board details --fqbn STMicroelectronics:stm32:Nucleo_64

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

fqbn=$1

what=`pwd`
what=${what##*/}
echo "building $what for $fqbn $2"

arduino-cli compile --fqbn $fqbn --board-options "pnum=$2,usb=$USB" --libraries $script_dir/../ -e .
