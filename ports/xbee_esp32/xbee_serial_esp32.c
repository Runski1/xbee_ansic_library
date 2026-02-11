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
	@addtogroup hal_esp32
	@{
	@file xbee_serial_esp32.c
	Serial Interface for XBee Module (ESP32 Microcontroller)
*/
// NOTE: Documentation for the public functions can be found in xbee/serial.h.


#include "esp_err.h"
#include "hal/uart_types.h"
#include "xbee_serial_config_esp32.h"
#include "platform_config.h"

#include "xbee/platform.h"
#include "xbee/serial.h"
#include "xbee/cbuf.h"
#include <errno.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <limits.h>



#define BUFFER_UPPER_BOUND	4  // how much space before rts is de-asserted
#define	BUFFER_LOWER_BOUND	RX_BUFF_SIZE/3  // how much before it's re-asserted

/* Local Prototypes */
void serialInit(xbee_serial_t *serial);


// NOTE: AFAIK we don't need these here
static bool_t		flow_control_enabled = TRUE;	/* is FC enabled?   */
//toggled by xbee_ser_break, which is now stubbed out
static bool_t		tx_break = FALSE;					/* are we breaking  */

/**
 * NOTE: The original code implemented a lot of features here, such as:
 * * Buffer fill checking
 * * TX/RX interrupts
 * * Manual interrupt triggering
 * * utilized the drivers own ring buffer implementation cbuf.h
 * We're using ESP-IDF UART drivers so I've cut out a lot of stuff.
 *  - Matias
 */
/******************************************************************************
 * LOCAL FUNCTIONS
 *****************************************************************************/

void serialInit(xbee_serial_t *serial) {
    /**
     * TODO:
     * I need AT LEAST store the UART instance identity in xbee_serial_t
     * (It's XBEE_UART_NUMBER here)
     * and pass that as parameter. The uart_config could also be member of that 
     * structure. 
     * This function is not defined in xbee/serial.h, so most likely it's
     * not called by anything in the driver
     *
     *
     * NOTE:
     *      @typedef xbee_serial_t
     *      Must be a structure with uint32_t member \c baudrate and any additional
     *      members required by the functions in xbee/serial.h.
     * */

    // UART configuration
    const uart_config_t uart_config = {
        .baud_rate = serial->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_DISABLE,
        .flow_ctrl = serial->flow_control, // UART_HW_FLOWCTRL_CTS_RTS
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };
    /*
     * Either pass XBEE_UART_NUMBER as parameter to this function, or
     * store the UART number here to *serial
     */
    ESP_ERROR_CHECK(uart_param_config(serial->uart_number, &uart_config));
    
    // UART pins defined in xbee_serial_config_esp32.h
    ESP_ERROR_CHECK(uart_set_pin(
        XBEE_UART_NUMBER,
        XBEE_TXPIN,
        XBEE_RXPIN,
        XBEE_RTSPIN,
        XBEE_CTSPIN
    ));

    ESP_ERROR_CHECK(uart_driver_install(
        XBEE_UART_NUMBER,
        RX_BUFF_SIZE,
        TX_BUFF_SIZE,
        0, // event queue size
        NULL, // TODO: add event queue
        0 // interrupt allocation flags, use?
    ));

    ESP_ERROR_CHECK(uart_flush(serial->uart_number));

	xbee_ser_break(serial, FALSE);
	xbee_ser_flowcontrol(serial, TRUE);
}

/******************************************************************************
 * PUBLIC FUNCTIONS
 *****************************************************************************/

/**
   @brief
   Helper function used by other xbee_serial functions to
   validate the \a serial parameter.

   Confirms that it is non-\c NULL and is set to a valid port.

   @param[in]  serial   XBee serial port

   @retval  1 \a serial is not a valid XBee serial port
   @retval  0 \a serial is a valid XBee serial port
 * NOTE: Stub, should be expanded
 */
bool_t xbee_ser_invalid(xbee_serial_t *serial)
{
    //UART_NUM_MAX -1 is maximum possible uart number
    return (serial == NULL || serial->baudrate == 0);
}


int xbee_ser_write(xbee_serial_t *serial, const void FAR *buffer, int length)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	if (length < 0 || tx_break) {
		return -EIO;
	}
	
	if (length > TX_BUFF_SIZE) {
		length = TX_BUFF_SIZE;
	}
	return uart_write_bytes(serial->uart_number, buffer, length);
}


int xbee_ser_read(xbee_serial_t *serial, void FAR *buffer, int bufsize)
{
	int ret;
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	if (bufsize < 0) {
		return -EIO;
	}
	
	if (bufsize > RX_BUFF_SIZE) {
		bufsize = RX_BUFF_SIZE;
	}
	ret = uart_read_bytes(serial->uart_number, buffer, bufsize, 0);

    if (ret == -1) {
        return -EIO; // if uart read fails
    }
	return ret;
}


int xbee_ser_putchar(xbee_serial_t *serial, uint8_t ch)
{
    int ret;
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	if (tx_break){
		return -EIO;
	}
    // I think this function blocks until there's space in UART driver's TX buffer
    ret = uart_write_bytes(serial->uart_number, &ch, 1);	
    if (ret == -1) {
        return -EIO; // parameter error
    }
    
	return 0;
}

int xbee_ser_getchar(xbee_serial_t *serial)
{
    int ret;
    char ch;

	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	ret = uart_read_bytes(serial->uart_number, &ch, 1, 0);

    if (ret <= 0) {
		return -ENODATA; // this function shouldn't return -EIO (-1)
	}

	return ch;
}


int xbee_ser_open(xbee_serial_t *serial, uint32_t baudrate)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	serialInit(serial);
	return xbee_ser_baudrate(serial, baudrate);
}


int xbee_ser_baudrate(xbee_serial_t *serial, uint32_t baudrate)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	switch (baudrate)
	{
		case 9600:
		case 19200:
		case 38400:
		case 57600:
		case 115200:
		case 230400:
		case 460800:
		case 921600:
			break;
		default:
			return -EIO;
	}

	serial->baudrate = baudrate;
    ESP_ERROR_CHECK(uart_set_baudrate(serial->uart_number, baudrate));
	return 0;
}


int xbee_ser_close(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	/* Disable interrupts */
    ESP_ERROR_CHECK(uart_driver_delete(serial->uart_number));
	return 0;
}

/******************************************************************************
 * STUBBED OUT FUNCTIONS
 *
 *   `xbee/serial.h`.  During development, you can probably stub out the
 * following functions (which are only used by `xbee_firmware.c` and
 * `xbee_atmode.c` at the moment) and implement them in a second phase of
 * development:
 *  - `xbee_ser_tx_free` and `xbee_ser_rx_free` -- always return `MAX_INT`
 *  - `xbee_ser_tx_used` and `xbee_ser_rx_used` -- always return 0
 *  - `xbee_ser_tx_flush`, `xbee_ser_rx_flush`, `xbee_ser_break`,
 *    `xbee_ser_flowcontrol` and `xbee_ser_set_rts` -- do nothing
 *  - `xbee_ser_get_cts` -- always return 1
 *****************************************************************************/


/* NOTE: ESP IDF doesn't allow for setting break indefinitely.
 * A break with timeout can be set with `uart_write_bytes_with_break()`
 * or an indefinite timeout could be achieved by manually pulling the pin low.
 * That might require deleting existing UART driver, clearing buffers etc.
 */
int xbee_ser_break(xbee_serial_t *serial, bool_t enabled)
{
	return 0;
}

/* NOTE: ESP IDF supports hardware and software flow control. Those can be controlled 
 * with `uart_set_hw_flow_control()` and `uart_set_sw_flow_control()` respectively.
 */
int xbee_ser_flowcontrol(xbee_serial_t *serial, bool_t enabled)
{
	return 0;
}


/* NOTE: STUB
 */
int xbee_ser_set_rts(xbee_serial_t *serial, bool_t asserted)
{
    return 0;
}

/* NOTE: STUB
 */
int xbee_ser_get_cts(xbee_serial_t *serial)
{
    return 1;
}

/* NOTE: STUB
 */
int xbee_ser_tx_free(xbee_serial_t *serial)
{
    return INT_MAX;
}

/* NOTE: STUB
 */
int xbee_ser_tx_used(xbee_serial_t *serial)
{
    return 0;
}

/* NOTE: STUB
 */
int xbee_ser_tx_flush(xbee_serial_t *serial)
{
    return 0;
}

/* NOTE: STUB
 */
int xbee_ser_rx_free(xbee_serial_t *serial)
{
    return INT_MAX;
}

/* NOTE: STUB
 */
int xbee_ser_rx_used(xbee_serial_t *serial)
{
    return 0;
}

/* NOTE: STUB
 */
int xbee_ser_rx_flush(xbee_serial_t *serial)
{
    return 0;
}
///@}
