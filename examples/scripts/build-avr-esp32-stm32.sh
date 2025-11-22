#!/bin/bash

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

"$script_dir"/build-avr.sh uno || exit
"$script_dir"/build-esp32.sh arduino:esp32:nano_nora || exit
"$script_dir"/build-stm32.sh STMicroelectronics:stm32:GenF4 BLACKPILL_F411CE || exit
