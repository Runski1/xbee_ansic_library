/*
 * Copyright (c) 2017 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */
/**
	@defgroup hal_efm32 HAL: EFM32/Simplicity Studio
	@ingroup hal
	@{
	@file xbee_platform_efm32.c
	Platform header for efm32
*/
#ifndef __XBEE_PLATFORM_ESP32
#define __XBEE_PLATFORM_ESP32

// verbose print for debugging

// #define XBEE_ATCMD_VERBOSE
// #define XBEE_DEVICE_VERBOSE
// #define XBEE_SERIAL_VERBOSE


// Necessary includes for ESP32 board
	// stdint.h for int8_t, uint8_t, int16_t, etc. types
	#include <stdint.h>
	#include <errno.h>
	#include <inttypes.h>
    #include <stdbool.h>

	#define _f_memcpy memcpy
	#define _f_memset memset

	// Must define since GDK doesn't have endian.h (the values must be unique)
// // not sure if needed
	#define LITTLE_ENDIAN 4321 
	#define BIG_ENDIAN 1234    
	#define BYTE_ORDER LITTLE_ENDIAN //This is hard-coded, but can be checked

	// macros used to declare a packed structure (no alignment of elements)
    // The more-flexible XBEE_PACKED() replaced PACKED_STRUCT in 2019.
	#define PACKED_STRUCT		struct __attribute__ ((__packed__))
	#define XBEE_PACKED(name, decl)	PACKED_STRUCT name decl

	
	typedef struct xbee_serial_t {
        uint32_t        uart_number;
		uint32_t		baudrate;
		uint32_t		flow_control;
	} xbee_serial_t;

    typedef bool bool_t;

	#define XBEE_MS_TIMER_RESOLUTION 1 // Our timer has 1 ms resolution
	#define ZCL_TIME_EPOCH_DELTA 0

#if defined(__cplusplus)
	extern "C" {
#endif
	int xbee_platform_init(void);
	#define XBEE_PLATFORM_INIT() xbee_platform_init()
#if defined(__cplusplus)
	}
#endif
#endif /* __XBEE_PLATFORM_ESP32 */
///@}
