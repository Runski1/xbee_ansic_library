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

#define BUFFER_UPPER_BOUND	4  // how much space before rts is de-asserted
#define	BUFFER_LOWER_BOUND	RX_BUFF_SIZE/3  // how much before it's re-asserted


/* DO NOT CHANGE THINGS BELOW (they change based off of USART_NUMBER) */
#define XBEE_RX_IRQn		CONCAT3(USART, USART_NUMBER, _RX_IRQn)			/* IRQRX number */
#define XBEE_TX_IRQn		CONCAT3(USART, USART_NUMBER, _TX_IRQn)			/* IRQTX number */
#define XBEE_RX_IRQ_NAME	CONCAT3(USART, USART_NUMBER, _RX_IRQHandler)	/* USART IRQ Handler */
#define XBEE_TX_IRQ_NAME 	CONCAT3(USART, USART_NUMBER, _TX_IRQHandler)	/* USART IRQ Handler */
#define XBEE_USART			CONCAT(USART, USART_NUMBER)						/* USART instance */
#define XBEE_CLK			CONCAT3(cmuClock, _USART, USART_NUMBER)			/* HFPER Clock */
#define XBEE_ROUTE_LOC_TX	CONCAT(USART_ROUTELOC0_TXLOC_LOC,TX_LOC)		/* HFPER Clock */
#define XBEE_ROUTE_LOC_RX	CONCAT(USART_ROUTELOC0_RXLOC_LOC,TX_LOC)
#define XBEE_ROUTE_LOC_CTS	CONCAT(USART_ROUTELOC1_CTSLOC_LOC, CTS_LOC)

/* Helper Macros, DO NOT CHANGE */
#define XBEE_USART_STRING	TOSTRINGMACRO(CONCAT(USART_, USART_NUMBER))       /* String version of USART_# */
#define TOSTRINGMACRO(s)	TOSTRINGMACRO_(s)
#define TOSTRINGMACRO_(s)	#s
#define CONCAT_(A, B)		A ## B
#define CONCAT(A, B)		CONCAT_(A, B)
#define CONCAT3_(A, B, C)	A ## B ## C
#define CONCAT3(A, B, C)	CONCAT3_(A, B, C)

/* Local Prototypes */
void checkRxBufferUpper(void);
void checkRxBufferLower(void);
void serialInit(xbee_serial_t *serial);

static xbee_cbuf_t *rx_buffer;
static xbee_cbuf_t *tx_buffer;
static uint8_t internal_rx_buffer[RX_BUFF_SIZE + XBEE_CBUF_OVERHEAD];
static uint8_t internal_tx_buffer[TX_BUFF_SIZE + XBEE_CBUF_OVERHEAD];

static bool_t		flow_control_enabled = TRUE;	/* is FC enabled?   */
static bool_t		tx_break = FALSE;					/* are we breaking  */

/******************************************************************************
 * IRQS
 *****************************************************************************/
//TODO:
void XBEE_RX_IRQ_NAME(void)
{
	uint32_t interruptFlags = USART_IntGetEnabled(XBEE_USART);
	USART_IntClear(XBEE_USART, interruptFlags);

	if (XBEE_USART->STATUS & USART_STATUS_RXDATAV) {
		// Store Data
		xbee_cbuf_putch(rx_buffer, USART_Rx(XBEE_USART));
		checkRxBufferUpper();
	}
}

//TODO:
void XBEE_TX_IRQ_NAME(void)
{
	uint32_t interruptFlags = USART_IntGetEnabled(XBEE_USART);
	USART_IntClear(XBEE_USART, interruptFlags);

	if (XBEE_USART->STATUS & USART_STATUS_TXBL) {
		int tx;
		if (tx_break) {
			USART_IntDisable(XBEE_USART, UART_IF_TXBL);
			return;
		}
		tx = xbee_cbuf_getch(tx_buffer);
		if (tx != -1) {
			USART_Tx(XBEE_USART, (uint8_t) tx);
		}
		else { //nothing to send
			USART_IntDisable(XBEE_USART, UART_IF_TXBL);
		}
	}
}


/******************************************************************************
 * LOCAL FUNCTIONS
 *****************************************************************************/
void checkRxBufferUpper(void)
{
	if ((xbee_cbuf_free(rx_buffer)) <= BUFFER_UPPER_BOUND) {
		/* The buffer is almost full, de-assert RTS */
		GPIO_PinOutSet(XBEE_RTSPORT, XBEE_RTSPIN);
	}
}


void checkRxBufferLower(void)
{
	if (xbee_cbuf_used(rx_buffer) <= BUFFER_LOWER_BOUND) {
		/* The buffer is empty enough, assert RTS */
		GPIO_PinOutClear(XBEE_RTSPORT, XBEE_RTSPIN);
	}
}

/* It should be safe to call this multiple times */
void serialInit(xbee_serial_t *serial) {
    /* TODO: read and double check
     * https://github.com/espressif/esp-idf/blob/v5.5.2/examples/peripherals/uart/uart_echo/main/uart_echo_example_main.c 
     *
     * also check hardware flow control config and use
     * */

    // RX and TX software ring buffers
	rx_buffer = (xbee_cbuf_t *) internal_rx_buffer;
	xbee_cbuf_init(rx_buffer, RX_BUFF_SIZE);

	tx_buffer = (xbee_cbuf_t *) internal_tx_buffer;
	xbee_cbuf_init(tx_buffer, TX_BUFF_SIZE);

    // UART configuration
    const uart_config_t uart_config = {
        .baud_rate = XBEE_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS,
        .parity = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_param_config(XBEE_UART_NUMBER, &uart_config));
    
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

    uart_flush(XBEE_UART_NUMBER);

	xbee_ser_break(serial, FALSE);
	xbee_ser_flowcontrol(serial, TRUE);
}

/******************************************************************************
 * PUBLIC FUNCTIONS
 *****************************************************************************/
//TODO:
bool_t xbee_ser_invalid(xbee_serial_t *serial)
{
	return (serial == NULL || serial->baudrate == 0);
}

const char *xbee_ser_portname(xbee_serial_t *serial)
{
	return XBEE_USART_STRING;
}

//TODO:
int xbee_ser_write(xbee_serial_t *serial, const void FAR *buffer, int length)
{
	int ret = 0;
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	if (length < 0 || tx_break) {
		return -EIO;
	}
	
	if (length > TX_BUFF_SIZE) {
		length = TX_BUFF_SIZE;
	}
	ret = xbee_cbuf_put(tx_buffer, buffer, length);
	USART_IntEnable(XBEE_USART, UART_IF_TXBL);
	return ret;
}

//TODO:
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
	ret = xbee_cbuf_get(rx_buffer, buffer, bufsize);
	if (flow_control_enabled) {
		checkRxBufferLower();
	}
	return ret;
}

//TODO:
int xbee_ser_putchar(xbee_serial_t *serial, uint8_t ch)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	if (tx_break){
		return -EIO;
	}
	
	USART_IntEnable(XBEE_USART, UART_IF_TXBL);
	if (xbee_cbuf_putch(tx_buffer, ch)) {
		return 0;
	}
	return -ENOSPC;
}

//TODO:
int xbee_ser_getchar(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	int ch = xbee_cbuf_getch(rx_buffer);
	
	if (ch == -1) {
		return -ENODATA;
	}
	
	if (flow_control_enabled) {
		checkRxBufferLower();			
	}
	return ch;
}

//TODO:
int xbee_ser_tx_free(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	return xbee_cbuf_free(tx_buffer);
}

//TODO:
int xbee_ser_tx_used(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	return xbee_cbuf_used(tx_buffer);
}

//TODO:
int xbee_ser_tx_flush(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	xbee_cbuf_flush(tx_buffer);
	return 0;
}

//TODO:
int xbee_ser_rx_free(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	return xbee_cbuf_free(rx_buffer);
}

//TODO:
int xbee_ser_rx_used(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	return xbee_cbuf_used(rx_buffer);
}

//TODO:
int xbee_ser_rx_flush(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	xbee_cbuf_flush(rx_buffer);
	return 0;
}

//TODO:
int xbee_ser_open(xbee_serial_t *serial, uint32_t baudrate)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	serialInit(serial);
	return xbee_ser_baudrate(serial, baudrate);
}

//TODO:
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
	USART_BaudrateAsyncSet(XBEE_USART, 0, baudrate, usartOVS16);
	return 0;
}

//TODO:
int xbee_ser_close(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	/* Disable interrupts */
	USART_IntDisable(XBEE_USART, USART_IF_RXDATAV);
	USART_IntDisable(XBEE_USART, UART_IF_TXBL);
	NVIC_DisableIRQ(XBEE_RX_IRQn);
	NVIC_DisableIRQ(XBEE_TX_IRQn);
	xbee_ser_rx_flush(serial);
	xbee_ser_tx_flush(serial);
	return 0;
}

//TODO:
int xbee_ser_break(xbee_serial_t *serial, bool_t enabled)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	if (enabled) {
		tx_break = TRUE;
		USART_IntDisable(XBEE_USART, UART_IF_TXBL);
		GPIO_PinModeSet(XBEE_TXPORT, XBEE_TXPIN, gpioModePushPull, 0);
		usart->ROUTEPEN &= ~(_USART_ROUTEPEN_MASK & USART_ROUTEPEN_TXPEN);
		GPIO_PinOutClear(XBEE_TXPORT, XBEE_TXPIN);
	}
	else {
		GPIO_PinModeSet(XBEE_TXPORT, XBEE_TXPIN, gpioModePushPull, 1);
		GPIO_PinOutSet(XBEE_TXPORT, XBEE_TXPIN);
		usart->ROUTEPEN |= _USART_ROUTEPEN_MASK & USART_ROUTEPEN_TXPEN;
		tx_break = FALSE;
		USART_IntEnable(XBEE_USART, UART_IF_TXBL);
	}
	return 0;
}

//TODO:
int xbee_ser_flowcontrol(xbee_serial_t *serial, bool_t enabled)
{

	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}

	flow_control_enabled = enabled; /* To disable software RTS */

	if (enabled) {
		/* Ensure we have RTS asserted correctly */
		checkRxBufferLower();
		checkRxBufferUpper();
		/* Enable CTS flow control */
		usart->CTRLX |= (_USART_CTRLX_MASK & (USART_CTRLX_CTSEN));
	}
	else {
		/* Disable CTS and RTS flow control */
		usart->CTRLX &= (_USART_CTRLX_MASK & (~USART_CTRLX_CTSEN));
		GPIO_PinOutClear(XBEE_RTSPORT, XBEE_RTSPIN);
	}
	return 0;
}

// TODO:
int xbee_ser_set_rts(xbee_serial_t *serial, bool_t asserted)
{
	int ret;
	
	ret = xbee_ser_flowcontrol(serial, FALSE);
	if (ret == 0) {
		
		if (asserted) {
			GPIO_PinOutClear(XBEE_RTSPORT, XBEE_RTSPIN);
		}
		else {
			GPIO_PinOutSet(XBEE_RTSPORT, XBEE_RTSPIN);
		}
	}
	return ret;
}

// TODO:
int xbee_ser_get_cts(xbee_serial_t *serial)
{
	if (xbee_ser_invalid(serial)) {
		return -EINVAL;
	}
	return (0 == GPIO_PinInGet(XBEE_CTSPORT, XBEE_CTSPIN));
}

///@}
