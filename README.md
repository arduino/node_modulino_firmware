## Firmware project for MCU based Modulino boards

This repository contains the complete source code for STM32C011 and an helper for easily uploading it to any preflashed Modulino.

### Compile and flash

```
git clone --recursive https://github.com/arduino/node_modulino_firmware
./build.sh
# symlink / copy the sketches/libraries/Modulino folder to your sketchbook
# compile and run the FirmwareUpdater.ino example while only one modulino is connected to a UNO R4
```
