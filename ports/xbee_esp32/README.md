# Platform Support: ESP32

## Overview

This port is based on the EFM32 with SLSTK3701A Starter Kit implementation 
of the driver. As such, platform layer files might include some redundant code.

**These drivers should be used with the default ESP32 runtime with FreeRTOS.**

The drivers utilize ESP-IDF C++ API and are not pure C anymore.

## platform_config.h
Header file that defines platform specific macros and `xbee_serial_t` structure, 
which is used by the adaptation layer mapping functions.

## xbee_platform_esp32.c
The timers and readline function for user interaction are implemented here.
Timers need FreeRTOS to work!

## xbee_serial_config_esp32.h
UART line configuration. Define your UART pins here.

## xbee_serial_esp32.c
The platform-specific layer that maps a consistent serial API for the upper 
layers of the driver to the device's native serial API.
All required functions are implemented, optional ones are stubbed.
