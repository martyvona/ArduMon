#!/bin/bash

../build-avr.sh uno || exit
../build-esp32.sh arduino:esp32:nano_nora || exit
../build-stm32.sh STMicroelectronics:stm32:GenF4 BLACKPILL_F411CE || exit
