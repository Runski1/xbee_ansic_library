/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Written by Matias Ruonala
 * =======================================================================
 */

#include "../../../samples/common/_atinter.h"
#include "freertos/projdefs.h"
#include "../xbee_platform_esp32.c"

#include "driver/uart.h"

#define INPUT_BUFFER_SIZE 0xFF

#include "xbee/atcmd.h"       // for XBEE_FRAME_HANDLE_LOCAL_AT
#include "xbee/device.h"

const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
   XBEE_FRAME_HANDLE_LOCAL_AT,
   XBEE_FRAME_MODEM_STATUS_DEBUG,
   XBEE_FRAME_TABLE_END
};

/*
   main

   Initiate communication with the XBee module, then accept AT commands from
   STDIO, pass them to the XBee module and print the result.
*/
int app_main(void)
{
    xbee_dev_t my_xbee;
    xbee_serial_t serial = {
        .uart_number = 1,
        .baudrate = 115200,
        .flow_control = UART_HW_FLOWCTRL_CTS_RTS
    };

    xbee_dev_init(&my_xbee, &serial, NULL, NULL);
    xbee_ser_open(&serial, 115200);


    char ibuf[INPUT_BUFFER_SIZE];

    while (1) {
        xbee_readline(ibuf, INPUT_BUFFER_SIZE);
        xbee_dev_tick(&my_xbee);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


