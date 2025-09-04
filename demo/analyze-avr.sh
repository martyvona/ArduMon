#!/bin/bash

# https://arduino.stackexchange.com/questions/31190/detailed-analyse-of-memory-usage

if [ $# -lt 1 ]; then
  echo "USAGE: analyze-avr.sh BOARD"
  echo "EXAMPLE: analyze-avr.sh uno"
  echo "EXAMPLE: analyze-avr.sh nano"
  exit 1
fi

adir=`arduino-cli config get directories.data`
for d in `ls -d $adir/packages/arduino/tools/avr-gcc/*/bin/`; do export PATH="$d:$PATH"; done

elf=`ls ./build/arduino.avr.${1}/*.elf`
asm=${elf%.elf}.asm
inf=${elf%.elf}.inf

echo "creating disassembly $asm from $elf"
avr-objdump -Sz -I./ -I../ $elf > $asm

echo "creating info $inf from $elf"
avr-readelf -a -W $elf > $inf


