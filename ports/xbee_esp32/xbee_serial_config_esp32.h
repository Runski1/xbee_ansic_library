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
	@file xbee_platform_esp32.c
	Serial configurations
*/
#ifndef __SERIAL_CONFIG_H
#define __SERIAL_CONFIG_H

	#define RX_BUFF_SIZE		255				/* Receive buffer = 2^n -1 for some integer n) */
	#define TX_BUFF_SIZE		RX_BUFF_SIZE	/* Transmit buffer (same as Rx buffer) */
	#define XBEE_UART_NUMBER    1				/* The USART peripheral number you are using */
    #define XBEE_UART_BAUD_RATE 115200

	/* USART Pin Locations */
	#define XBEE_TXPIN			17				/* UART1 transmission pin */
	#define XBEE_RXPIN			18				/* UART1 reception pin */
	#define XBEE_RTSPIN			19				/* UART1 RTS flow control pin */
	#define XBEE_CTSPIN			20				/* UART1 CTS flow control pin */


#endif /* __SERIAL_CONFIG_H */
///@}
