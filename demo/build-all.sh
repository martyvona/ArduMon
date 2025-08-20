#!/bin/sh

echo "building text server for Arduino platforms..."
./build-avr.sh uno || exit
./build-esp32.sh arduino:esp32:nano_nora || exit
./build-stm32.sh STMicroelectronics:stm32:GenF4 BLACKPILL_F411CE || exit

echo "\nbuilding binary server for Arduino platforms..."
(cd binary_server && ./build-all.sh) || exit

echo "\nbuilding binary client for Arduino platforms..."
(cd binary_client && ./build-all.sh) || exit

echo "\nbuilding for native demo client and server..."
(cd native && ./build-native.sh) || exit
