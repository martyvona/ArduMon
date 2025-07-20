#!/bin/sh

if [ $# -lt 1 ]; then
  echo "USAGE: build-esp32.sh FQBN"
  echo "EXAMPLE: build-esp32.sh esp32:esp32:esp32"
  echo "EXAMPLE: build-esp32.sh arduino:esp32:nano_nora"
  exit 1
fi

arduino-cli compile --fqbn $1 --libraries ../ -e .

# https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display

# this should also work for Arduino Nano ESP32:
# arduino-cli compile --fqbn esp32:esp32:nano_nora --libraries ../ -e .
# but appears broken in esp32 core 3.2.0, probably fixed in 3.2.1
# but as of 7/6/25 arduino-cli still thinks 3.2.0 is latest (3.2.1 was released 7/3/25)
# https://github.com/espressif/arduino-esp32/pull/11315
