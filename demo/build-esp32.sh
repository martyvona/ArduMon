#!/bin/sh

echo "building for nano ESP32 (arduino:esp32:nano_nora).."
arduino-cli compile --fqbn arduino:esp32:nano_nora --libraries ../ -e .
# this should also work for nano ESP32 but appears broken in esp32 core 3.2.0, probably fixed in 3.2.1
# arduino-cli compile --fqbn esp32:esp32:nano_nora --libraries ../ -e .
# but as of 7/6/25 arduino-cli still thinks 3.2.0 is latest (3.2.1 was released 7/3/25)
# https://github.com/espressif/arduino-esp32/pull/11315

# https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display
echo "building for generic ESP32/CYD (esp32:esp32:esp32)..."
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries ../ -e .

