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
	@addtogroup hal_efm32
	@{
	@file xbee_platform_efm32.c
	Platform-specific functions for use by the
	XBee Driver on EFM32 uC platform.
	Documented in platform.h
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <stdbool.h>
#include <errno.h>
#include "driver/usb_serial_jtag.h"
#include <string.h>

#define XBEE_MAX_LINELENGTH 256

/**
 * Timer required by xbee driver to report elapsed time
 */
	uint32_t xbee_millisecond_timer()
	{
		return (uint32_t) (esp_timer_get_time() / 1000ULL);
	}
/**
 * Timer required by xbee driver to report elapsed time
 */
	uint32_t xbee_seconds_timer()
	{
		return xbee_millisecond_timer() / 1000UL;
	}


	void xbee_platform_init(void)
	{
		return; //nothing to initialize here
	}
	
/**
   @brief
   This function is a non-blocking version of gets(), used to read a line of
   input from the user.

   It waits for a string from stdin terminated by a return.  It should be
   called repeatedly, until it returns a value other than -EAGAIN.  The input
   string, stored in \a buffer is null-terminated without the return.

   The user should make sure only one process calls this function at a time.

   @param[in,out] buffer   buffer to store input from user
   @param[in]     length   size of \a buffer

   @retval  >=0      User ended the input with a newline, return value is
                     number of bytes written.
   @retval  -EAGAIN  User has not completed a line.
   @retval  -EINVAL  NULL buffer or length is less than 1.
   @retval  -ENODATA User entered CTRL-D to end input.
*/

int xbee_readline( char *buffer, int length){
    if (!buffer || length < 1) { return -EINVAL;}

    char c {'\0'};
    static int bytes_read {0};
    static char temp_buffer[XBEE_MAX_LINELENGTH] {'\0'};

    while (usb_serial_jtag_read_bytes(&c, 1, 0) > 0) {

            if (c == 0x04) return -ENODATA; // EOT (CTRL+D)
            if (c == '\n' || c == '\r') { // copy string to buffer when \n or \r
                int n = bytes_read < (length - 1) ? bytes_read : (length - 1);
                memcpy(buffer, temp_buffer, n);
                buffer[n] = '\0';
                bytes_read = 0;
                return n; // return num of bytes written to buffer
            }
            if (bytes_read < XBEE_MAX_LINELENGTH) {
            temp_buffer[bytes_read++] = c; // Add character to temp buffer
        }
        }
    return -EAGAIN;

}
///@}
