<div style="width: 50em"> 

# ArduMon

**add a CLI *and* a packet API to your Arduino project**

**CURRENT STATUS: WORK IN PROGRESS**

Copyright 2025 Marsette A. Vona (martyvona@gmail.com)

[MIT License](./LICENSE.txt)

Once upon a time, it was not uncommon during microcontroller firmware development to implement a *firmware monitor* as a a command line interface (CLI) exposed by the firmware over a serial port.  (Terminology: the word *monitor* has been used both for microcontroller firmware that exposes a user interface as well as for the serial terminal program which would [connect](#connecting-to-ardumon) to it.)  Though modern controllers now offer more [advanced options](https://www.st.com/en/development-tools/stm32cubemonitor.html), serial monitors are still implemented, particularly on smaller platforms.  And there is [no shortage](https://github.com/gpb01/SerialCmd) of [existing](https://github.com/ppedro74/Arduino-SerialCommands) [Arduino](https://github.com/argandas/SerialCommand) [libraries](https://github.com/naszly/Arduino-StaticSerialCommands) to help implement them.

*ArduMon* is yet another one of these, but with a few features that I didn't see in most of the existing ones:

* The same command handler code can operate in either text or binary mode.  Text mode would be used for a traditional CLI-style monitor user interface.  Binary mode reuses the same command implementations as text mode, but turns them into an efficient packet-based application programming interface (API).
* Supported command argument and response data types include: character, string, boolean, 8/16/32/64 bit signed and unsigned integers, and 32 and 64 bit floating point numbers.  Each command can accept zero or more parameters of these types and also respond with zero or more of them.  Text mode responses can also respond with free-form text, or with VT100 control sequences, e.g. to implement a dynamically updating text display.  In text mode, integers can be read and written in hexadecimal or decimal, and floating point numbers can be read and written in decimal or scientific format.
* ArduMon implements a basic command line user experience in text mode: echo is supported, an optional text prompt is displayed, the user may hit backspace to erase the most recent character, and hitting the up arrow re-enters the previous command if there was space to save it.

ArduMon also
* is header only
* is 8 bit AVR compatible
* supports AVR, ESP32, and STM32 Arduino compatible boards
* can optionally also be built for PC: OS X, Linux, [Windows Subsystem for Linux (WSL)](https://learn.microsoft.com/en-us/windows/wsl/install)
* can have a relatively low memory footprint (configurable)
* uses no heap allocations.

In many cases the same command handler can work in both binary and text mode: call the ArduMon `recv(...)` APIs to read command parameters and the `send(...)` APIs to write results, and finally `end_cmd()`.  It is also possible to make a handler behave differently in text and binary mode, e.g. by checking the `is_binary_mode()` and `is_txt_mode()` ArduMon APIs.  For example, a handler could stream a text response with VT100 control codes to update a live display on the terminal, but in binary mode it could instead send a stream of binary packets.

In text mode the entire received command string must fit in the ArduMon receive buffer, the size of which is set at compile time.  There is no limit on the amount of data that can be returned by a command in text mode, though sending may block the handler if enough data is sent fast enough relative to the Arduino serial send buffer size, typically 64 bytes, and the serial baudrate.   The ArduMon send buffer is not used in text mode, and can be set to size 0 at compile time if binary mode will not be used.

In binary mode both commands and responses are sent in variable length packets of up to 255 bytes.  The ArduMon receive and send buffers must be sized at compile time to fit the largest used packets; if an application is receive-only then the ArduMon recive buffer can be set to size 0.  The first byte of each packet gives the packet length in bytes (2-255) and the last byte is a checksum.  The max payload size per received packet is 252 bytes, as there are three overhead bytes: length (byte 0), command code (byte 1), and checksum (the last byte).  The max payload size per response packet is 253 bytes since there are only 2 bytes of overhead there: length and checksum.  Zero or more packets can be returned in series from a single command handler, see `send_packet()`.

Most of the ArduMon APIs return a boolean error flag: if the return is false call `get_err()` to check the error, then `clear_err()`.  There is no built in handling of errors, e.g. an ACK/NACK protocol, retries, etc.  Error handling could be implemented by applications if desired.

Flow control is also up to the application.  In interactive use the operator can wait as appropriate and/or verify a response before sending another command.  Automation can do similar if necessary in binary mode.  Only one command is handled at a time.  If a new command starts coming in while one is still being handled then the new command will start to fill the Arduino serial input buffer, which is typically 64 bytes.  Once the Arduino serial input buffer fills, further received bytes will be silently dropped; the Arduino serial receive interrupt unfortunately [does not signal overflow](https://arduino.stackexchange.com/a/14035).

ArduMon also has an optional receive timeout (`set_recv_timeout_ms()`) which will reset the command interpreter if too much time has passed between receiving the first and last bytes of a command.  This is disabled by default; it probably makes more sense for automation than for interactive use.

The ArduMon `update()` API should be "pumped" from the Arduino `loop()` method.  Command handlers are run directly from `update()`, so if they run long, they will block `loop()`.  A handler may return before handling is complete, as long as `end_cmd()` is eventually called.

In text mode a single handler can return an arbitrary amount of data.  In binary mode a single handler can send an arbitrary number of response packets (see `send_packet()`).  (This would require breaking the handler up so that `loop()` is not blocked.)  It is also acceptable to send data when a command handler is not actually running, as long as all command parameters have been read before calling `end_cmd()`.  For example, a command could trigger sending periodic status strings in text mode, or status packets in binary mode, indefinitely until another command is received to end the stream.  ArduMon can even be used for send-only applications, e.g. which just autonomously send a never ending stream of packets in binary mode.

Handlers can also implement their own sub-protocols, reading and optionally writing directly to the serial port (typically via the Arduino serial send and receive buffers).  Command receive is disabled while a handler is running, so during that time a handler can consume serial data that's not intended for the command processor.  For example, an interactive text mode handler that is updating a live display on the terminal could exit when a keypress is received from the user.

## Text Mode

For each command in text mode:

1.  An optional prompt is sent, preceded by carriage return and line feed, and suffixed with a space.
1.  Characters are read, with optional echo, until a carriage return (`\r`, `0x0D`) or line feed (`\n`, `0x0A`) is
    received. In echo mode both a carriage return and line feed are sent when the end of command is received.  Both
    Unix/OS X style `\r` line endings as well as Windows style `\r\n` are supported, though the latter will incur an
    ignored empty command and an extra `\r\n` response if echo is enabled.  Terminal programs like minicom typically
    send only `\r` when the user hits the `return` key, so the other line ending types are more likely to be encountered
    when receiving a file of commands, in which case echo would typically be disabled, and ignored empty commands should    not matter.
1.  If the backspace character (`\b`, `0x08`) is received and there was a previously received character in the command
    then (a) the previously received character is ignored and (b) VT100 control codes are sent to erase the previous
    character on the terminal if echo is enbled.
1.  If the received command is empty then it is ignored and the command interpreter resets.
1.  If the received command including the terminating carriage return is larger than the receive buffer then
    `RECV_OVERFLOW` and the command interpreter resets.
1.  The received command is tokenized on whitespace.  End-of-line comments starting with `#` are ignored.  Character
    tokens can be unquoted or surrounded by single quotes.  In the quoted form the backslash escape sequences below
    are supported.  The quoted form also allows `'#'` where the `#` will not be interpreted as the start of a comment.
    String tokens must be surrounded by double quotes unless they contain no whitespace, quotes, or `#`, in which case
    the quotes are optional.
1.  If the first token of the command is not in the command table then `BAD_CMD` and the command interpreter resets.
1.  Otherwise, the command handler is executed.  It may call the `recv(...)` APIs to parse the command tokens in order.
    Call `recv()` with no arguments to skip a token, including the command token itself.  If there are no more tokens
    then `RECV_UNDERFLOW`.  If the next token is not in the expected form then `BAD_ARG`.
1.  The command handler may also call the `send(...)` APIs at any point to stream results back to the serial port.  In
    text mode returned data is sent incrementally.  There is no limit to the amount of data that can be sent in
    response to a command, but the `send(...)` APIs will block if the send buffer is full. Each sent value after the
    first will be prefixed by a space, unless the handler calls `send_CRLF()` which will replace the space with a
    `\r\n` sequence that will set the cursor to the start of the next line.  Sent characters will be single quoted and
    backslash escaped if they contain a quote or whitespace character. Sent strings will be double quoted and backslash
    escaped if they contain a quote or whitespace character.  The `send_raw()` APIs can be used to avoid the space
    prefix, quote, and escape behaviors.
1.  The command handler must end with a call to `end_cmd()`, which will reset the command interpreter.

Supported backslash escape sequences in text mode:

* `\'` `0x27` single quote
* `\"` `0x22` double quote
* `\\` `0x5c` backslash
* `\a` `0x07` bell
* `\b` `0x08` backspace
* `\f` `0x0c` form feed
* `\n` `0x0a` line feed
* `\r` `0x0d` carriage return
* `\t` `0x09` horizontal tab
* `\v` `0x0b` vertical tab
* `\e` `0x1b` escape (nonstandard)
* `\d` `0x7f` delete (nonstandard)

Integers are parsed and formatted in decimal or hexadecimal in text mode, and floating point numbers are parsed and formatted with optional scientific notation by default.

## Binary Mode

Each command in binary mode is a packet consisting of

1.  A packet length `L` as a single unsigned byte.  The packet length includes the length byte itself, the command code
    byte, and the checksum byte.  If `L < 3` then `BAD_COMMAND` and the command interpreter resets.
2.  A command code as a single unsigned byte.
3.  `L-3` payload bytes
4.  A single byte checksum such that the 8 bit sum of the bytes of the entire packet from the length byte through the
    checksum byte itself is 0.

In binary mode a command handler is called when the full length of a packet with a valid checksum (otherwise `BAD_PACKET`) and known command code (otherwise `BAD_CMD`) is received.  The command handler may call the `recv(...)` APIs to access the received data bytes in order.  The first byte returned will be the command code itself; call `recv()` with no arguments to skip a byte.  Attempts to `recv(...)` beyond the end of the payload will result in `RECV_UNDERFLOW`.  The command handler may also call the `send(...)` APIs at any point to append data to the send buffer.  Sending more than `min(send_buf_sz - 2, 253)` bytes results in `SEND_OVERFLOW`.  When `send_packet()` or `end_cmd()` is called the send buffer is enabled for transfer to the serial port.  As much of it as possible is sent immediately, blocking up to `send_wait_ms` (0 by default).  Any remaining bytes will be drained in later calls to `update()`. The sent data will be prefixed with an unsigned length byte which includes itself, and suffixed with a checksum byte, which is also included in the length.  The checksum will be computed such that the 8 bit sum of the bytes of the entire packet from the first (length) byte through the checksum byte itelf is 0.

Multibyte quantities are read and written in little endian byte order in binary mode, which matches the endianness of the architectures this library is intended to target.  (Compilation will intentionally fail on a big endian target.)

## Running the Demo Without an Arduino

It's possible to run the ArduMon demo entirely on your host computer, with no Arduino involved.  This can be useful for development and testing.

Install `g++` on Linux including WSL:
```
sudo apt install g++ # or use the package manger for your distribution
```

Install `g++` on OS X:
```
xcode-select --install
```

Compile demo for native host (OS X, Linux, WSL):
```
cd demo/native
./build-native.sh
```

Run the demo on the native host:
```
cd demo/native
./demo_native foo
```

A UNIX socket called "foo" (use any name you want) will be created as as a file in the curent directory, and will be deleted when the demo program terminates.

This should work on Linux and OS X.  It should also work on Windows under WSL, but [UNIX sockets are currently only supported in WSL 1](https://stackoverflow.com/a/73067921), not WSL 2.  You can [switch](https://learn.microsoft.com/en-us/windows/wsl/install#upgrade-version-from-wsl-1-to-wsl-2) an existing WSL installation to WSL 1 with the command `wsl --set-version DISTRO_NAME 1` where `DISTRO_NAME` is one of the installed WSL Linux distributions shown in `wsl --list`.  You can also switch back to WSL 2 later with `wsl --set-version DISTRO_NAME 2`.

Once `demo_native` is running, open another terminal and use [minicom](#connecting-with-minicom) to attach to it with a command like

```
minicom -D unix#PATH_TO_UNIX_SOCKET_FILE
```

where `PATH_TO_UNIX_SOCKET_FILE` is the full path to the Unix socket file that was created by `demo_native`.  To end the demo, enter the command `quit` in minicom, which will cause `demo_native` to terminate and the UNIX socket file to be deleted.  Then exit minicom by hitting ctrl-A (option-Z on OS X), then Q, then enter.

## Building the Demo for Arduino

OS X:
```
brew install arduino-cli
softwareupdate --install-rosetta
```

Linux and Windows (using [git bash](https://about.gitlab.com/blog/git-command-line-on-windows-with-git-bash) or WSL) - install into `~/bin`:
```
cd ~
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
# make sure ~/bin is in your PATH
```

One time config to install the Arduino cores:

```
arduino-cli config init --additional-urls https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
arduino-cli config set network.connection_timeout 600s
arduino-cli core update-index
arduino-cli core list --all
arduino-cli core install arduino:avr
arduino-cli core install arduino:esp32 # for e.g. Arduino Nano ESP32
arduino-cli core install esp32:esp32 # for e.g. cheap yellow display
arduino-cli core install STMicroelectronics:stm32 # for STM32 boards including BlackPill, Nucleo32, Nucleo64, etc
arduino-cli board listall
```

Unfortunately `STMicroelectronics:stm32` is [not currently supported](https://github.com/stm32duino/Arduino_Core_STM32/issues/708) on Linux/ARM (e.g. Raspberry Pi).

Compile the demo for AVR, ESP32, and STM32:

```
cd demo
./build-avr.sh uno # or replace uno with nano or any board in arduino-cli board listall | grep arduino:avr
./build-esp32.sh arduino:esp32:nano_nora
./build-esp32.sh esp32:esp32:esp32 # for CYD; or replace with any board in arduino-cli board listall | grep esp32:esp32
./build-stm32.sh STMicroelectronics:stm32:GenF4 BLACKPILL_F411CE # or any board in arduino-cli board listall | grep stm32
./build-stm32.sh STMicroelectronics:stm32:Nucleo_64 NUCLEO_F411RE # or any part number in arduino-cli board details --fqbn STMicroelectronics:stm32:Nucleo_64
```

## Uploading the Demo to an Arduino Board

Attach your Arduino using a USB cable, then verify you can see it:

```
arduino-cli board list
```

### Upload the Demo for AVR

```
cd demo
./upload-avr.sh arduino:avr:uno /dev/cu.usb*
```

Replace `/dev/cu.usb*` with `/dev/ttyUSB*` on Linux, `/dev/ttySn` on WSL, or `COMn` on windows, where n is the serial port number shown in `arduino-cli board list`.

### Upload the Demo for ESP32 CYD (Cheap Yellow Display)

Attach the CYD by USB. Note that some CYD variants have a [USB-C port which does not work properly](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/blob/main/cyd.md#the-usb-c-port-doesnt-work) with USB C-C cables; try a USB A-C cable instead.

You can also attach the CYD to a USB-to-serial adapter with +5V/TX/RX/GND connected to port P1 on the CYD, push the reset button on the board, then the boot button, then release reset, then release boot.  However, there are several caveats with this method.  The [ESP32-2432S028 CYD with both USB micro and USB C ports](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/blob/main/cyd.md#my-cyd-has-two-usb-ports) appears to have a design flaw where the on-board CH340C USB to serial chip holds the RX line high even when USB is not connected (this also probably applies to the [original CYD with only USB micro](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/blob/main/cyd.md#what-is-a-cheap-yellow-display-cyd)).  This can be corrected with a board modification to create a [diode-OR](https://en.wikipedia.org/wiki/Diode-or_circuit) as shown in [this photo](./photos/CYD-diode-or.jpg), though this requires some tricky soldering and would be challenging without a microscope: remove the 100 Ohm resistor connected to the RX pin on P1 and replace it with any standard signal diode, e.g. 1N4148, with the cathode attached to the RX pin on P1; cut the trace leading to pin 2 on the CH340C right next to the IC, then bridge that cut with another of the same type of diode, with its cathode attached to the CH340C.  This diode arrangement also solves a secondary issue, which is that the P1 RX pin is not 5V tolerant, since the ESP32 input pin is itself not 5V tolerant.  The diode-OR [resolves this](https://forum.arduino.cc/t/5v-to-3-3v-logic-signal-diodes-to-drop-voltage/372343/9) by relying on the internal pullup on the ESP32 RX pin to bring it high.  Some other CYD variants have a 0 Ohm resistor installed instead of the 100 Ohm one on the P1 RX pin, and probably some other changes as well.  Those *may* work unmodified, but they likely still have the issue with applying 5V to RX (which may work in practice, but is out of spec).  So, either use a [USB-to-serial](./photos/CYD-3_3V-adapter.jpg) adapter [capable of 3.3V](https://www.amazon.com/dp/B087RJ7X32), or put a [diode in-line](./photos/CYD-inline-diode.jpg) with its anode attached to the P1 RX pin.

```
cd demo
./upload-esp32.sh esp32:esp32:esp32 /dev/cu.usb*
```

Replace `/dev/cu.usb*` with `/dev/ttyUSB*` on Linux, `/dev/ttySn` on WSL, or `COMn` on Windows, where n is the serial port number shown in `arduino-cli board list`.

### Upload the Demo for STM32

arduino-cli supports uploading to STM32 boards in several different ways.  If you are using a board like the STM32F411CEU6 [BlackPill](https://www.adafruit.com/product/4877) ([DFRobot](https://www.dfrobot.com/product-2338.html
), [Amazon](https://www.amazon.com/dp/B09V7GCMRX)) you can use USB device firmware upgrade (DFU) mode.  You will first need to install the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) software.  Push the reset button on the board, then the boot button, then release reset, then release boot.  Then:

```
cd demo
upload-stm32-dfu.sh STMicroelectronics:stm32:GenF4 BLACKPILL_F411CE
```

If you are using one of the many [Nucleo](https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html) boards, DFU mode is not available (to my knowledge), but you can use either USB mass storage mode or serial wire debug (SWD).  Mass storage does not require STM32CubeProgrammer to be installed:

```
cd demo
upload-stm32-mass-storage.sh STMicroelectronics:stm32:Nucleo_32 NUCLEO_G431KB
```

SWD uploading *does* require STM32CubeProgrammer to be installed, but for Nucleo boards does *not* require a separate hardware dongle:

```
cd demo
upload-stm32-swd.sh STMicroelectronics:stm32:Nucleo_32 NUCLEO_G431KB
```

SWD mode can also be used on boards like the BlackPill, but does require a separate hardware dongle like like [ST-LINK V2](https://www.amazon.com/dp/B07SQV6VLZ) or [ST-LINK V3](https://www.dalbert.net/stlink-v3-minie).  You may want to check if the firmware on the dongle needs to be upgraded first with [this tool](https://www.st.com/en/development-tools/stsw-link007.html).

## Connecting to ArduMon

This section explains how to interact with an Arduino running ArduMon using a terminal emulator.  This assumes you have the Arduino connected to your computer, typically with a USB cable or USB-to-serial adapter.  If you are using a CYD, see the [above caveats](#upload-the-demo-for-esp32-cyd-cheap-yellow-display) about the functionality of the USB-C and P1 (TTL serial) ports.

First determine the serial port device on your computer corresponding to your Arduino:
```
arduino-cli board list
```

The example commands below expect there to be exactly one Arduino connected; if there is more than one then you will need to specify the one to use exactly instead of using a wildcard as in the example commands below.

### Connecting with Minicom

To install [minicom](https://en.wikipedia.org/wiki/Minicom) on OS X:

```
brew install minicom
```

To install minicom on Linux including WSL:

```
sudo apt install minicom # or use the package manger for your distribution
```

To connect with minicom:

```
minicom -D /dev/cu.usb* -b 115200
```

Replace `/dev/cu.usb*` with `/dev/ttyUSB*` on Linux, `/dev/ttySn` on WSL, or `COMn` on Windows, where n is the serial port number shown in `arduino-cli board list`.

To exit minicom hit ctrl-A (option-Z on OS X), then Q.  If you haven't already, you will need to check "use option as meta key" in terminal -> settings -> profiles -> keyboard.

If nothing happens when you type, ensure hardware flow control is OFF: ctrl-A (option-Z on OS X), then O, then serial port setup, then F to toggle hardware flow control.  You can then "save setup as dfl" to persist this setting.

### Connecting with Screen

To connect with screen:

To install [screen](https://www.gnu.org/software/screen/manual/screen.html) on OS X:

```
brew install screen
```

To install screen on Linux including WSL:

```
sudo apt install screen # or use the package manger for your distribution
```

To connect with screen:

```
screen /dev/cu.usb* 115200
```

Replace `/dev/cu.usb*` with `/dev/ttyUSB*` on Linux, `/dev/ttySn` on WSL, or `COMn` on Windows, where n is the serial port number shown in `arduino-cli board list`.

To exit screen hit ctrl-A then \ (backslash) then Y.

### Connecting with C-Kermit

To install [c-kermit](https://www.kermitproject.org/ck90.html) on OS X:

```
brew install c-kermit
```

Currently c-kermit packages do not appear to be well maintained for Linux distributions including in WSL.

To connect with c-kermit:

```
cd demo
./kermit.scr /dev/cu.usb* 115200
```

Replace `/dev/cu.usb*` with `/dev/ttyUSB*` on Linux, `/dev/ttySn` on WSL, or `COMn` on Windows, where n is the serial port number shown in `arduino-cli board list`.

To exit kermit hit ctrl-\ then Q.

### Connecting with `arduino-cli monitor`

```
arduino-cli monitor --port /dev/cu.usb* --config 115200 --quiet --raw
```

Replace `/dev/cu.usb*` with `/dev/ttyUSB*` on Linux, `/dev/ttySn` on WSL, or `COMn` on Windows, where n is the serial port number shown in `arduino-cli board list`.

To exit the arduino-cli monitor hit ctrl-C.  You may need to then enter the `reset` command to reinitialize your terminal.

### Connecting with the Arduino IDE

Select your bord in the "Select Board" dropdown.  Then select Tools -> Serial Monitor.  Make sure the baud rate is set to 115200.

