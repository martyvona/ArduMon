#!/bin/bash

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "building simple example for Arduino platforms..."
cd "$script_dir"/../simple
../scripts/build-avr-esp32-stm32.sh

echo "building text demo for Arduino platforms..."
cd "$script_dir"/../demo
../scripts/build-avr-esp32-stm32.sh

echo "\nbuilding binary server for Arduino platforms..."
(cd binary_server && ../../scripts/build-avr-esp32-stm32.sh) || exit

echo "\nbuilding binary client for Arduino platforms..."
(cd binary_client && ../../scripts/build-avr-esp32-stm32.sh) || exit

echo "\nbuilding native demo client and server..."
(cd native && ./build-native.sh) || exit
