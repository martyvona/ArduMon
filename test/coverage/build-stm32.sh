#!/bin/sh

# https://www.adafruit.com/product/4877
# https://www.dfrobot.com/product-2338.html
# https://www.amazon.com/dp/B09V7GCMRX
# STM32F411CEU6 512k Flash 128k SRAM
# arduino-cli board details --fqbn STMicroelectronics:stm32:GenF4
echo "building for STM32 black pill (STMicroelectronics:stm32:GenF4)..."
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF4 --board-options "pnum=BLACKPILL_F411CE" --libraries ../../ -e .

# https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html
# STM32G431RB 128k flash 32k SRAM
# arduino-cli board details --fqbn STMicroelectronics:stm32:Nucleo_32
# arduino-cli board details --fqbn STMicroelectronics:stm32:Nucleo_64
echo "building for STM32 Nucleo-32 G431KB (STMicroelectronics:stm32:Nucleo_32)..."
arduino-cli compile --fqbn STMicroelectronics:stm32:Nucleo_32 --board-options "pnum=NUCLEO_G431KB" --libraries ../../ -e .
echo "building for STM32 Nucleo-64 G431RB (STMicroelectronics:stm32:Nucleo_64)..."
arduino-cli compile --fqbn STMicroelectronics:stm32:Nucleo_64 --board-options "pnum=NUCLEO_G431RB" --libraries ../../ -e .
