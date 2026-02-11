# Platform Support: ESP32

## Overview

This port is based on the EFM32 with SLSTK3701A Starter Kit implementation of the driver.
**These drivers should be used with the default ESP32 runtime with FreeRTOS.**

## platform_config.h
Header file that defines a bunch of platform specific macros.
Most of them remain untouched.

The main structure used by the adaptation layer was edited to allow the XBee drivers more 
finessed control over the ESP IDF UART driver.
```C
typedef struct xbee_serial_t {
    uint32_t        uart_number;
    uint32_t		baudrate;
    uint32_t		flow_control;
} xbee_serial_t;
```

## xbee_platform_esp32.c
The timers and readline function for user interaction are implemented here.
Timers utilize FreeRTOS timers.

## xbee_serial_config_esp32.h
UART line configuration. Only pin definitions are in use right now.

## xbee_serial_esp32.c
The platform-specific layer that maps a consistent serial API for the upper 
layers of the driver to the device's native serial API.
All required functions are implemented, optional ones are stubbed.
