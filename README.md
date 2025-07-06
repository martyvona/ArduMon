<div style="width: 50em"> 

# Arduino Command Monitor

## Building

OS X:
```
brew install arduino-cli
softwareupdate --install-rosetta
```

Linux (including Raspbian) - install into `~/bin`:
```
cd ~
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
# make sure ~/bin is in your PATH
```

One time config to install the Arduino AVR core:
```
arduino-cli config init --additional-urls https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
arduino-cli core update-index
arduino-cli core list --all
arduino-cli core install arduino:avr
arduino-cli core install arduino:esp32
arduino-cli core install esp32:esp32
arduino-cli core install STMicroelectronics:stm32
arduino-cli board listall
```

Compile:
```
arduino-cli compile --fqbn arduino:avr:uno -e .
```
