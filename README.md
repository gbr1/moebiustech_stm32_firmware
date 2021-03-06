# moebiustech_stm32_firmware

## How to install the stm32duino core
  1.  Install Arduino IDE (tested on ide 2 beta10);
  2.  go into preferences and add `http://dan.drown.org/stm32duino/package_STM32duino_index.json` 
  3.  in board manager install stm32duino core for **STM32F1** boards
  4.  open terminal and type `$``cd /home/$USER/.arduino15/packages/stm32duino/tools/stm32tools/2021.5.31/linux/`
  5.  run `./install.sh`
  6.  close terminal and go back to Arduino IDE
  7.  select `tools -> board -> STM32F1 Boards (Arduino_STM32) -> "Generic STM32F103R series"`
  8.  select `tools -> variant -> STM32F103RC (48k RAM 256 Flash)`
  9.  select `tools -> cpu speed -> 72MHz`
  10.  select `tools -> optimize -> "smallest default"`
  11.  select `tools -> port -> /dev/ttyUSB0`

## How add ros_lib
Copy **ros_lib** into your Arduino libraries folder.

## How to upload
If you are on linux you need an STLINK v2 and you need to:<br>
  1.  select `tools -> upload method -> stlink`
  2.  press upload

If you are on macOS you can also upload just using the serial port.
  1. select `tools -> upload methos -> serial`
  2. press upload

*NOTE: if you encounter some issues on uploading, try to select `tools -> variant -> STM32F103RE`*

## How to run
You need to use this [ros package](https://github.com/gbr1/moebiustech_stm32_ros)

---
> ***Copyright © 2021 Giovanni di Dio Bruno under MIT license***
---

