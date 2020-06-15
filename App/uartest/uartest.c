/*
 * uart.c
 *
 *  Created on: 2020.06.14
 *      Author: Taohanyi
 */

/******************************************************************************
* LOCAL INCLUDE FILES
******************************************************************************/
#include <stdio.h>
#include "cmsis_os.h"
#include "usart.h"
#include "uart.h"

/******************************************************************************
* LOCAL MACROS AND DEFINITIONS
******************************************************************************/
#define UART_PORT    huart2
#define UART_DEBUG   huart1

/******************************************************************************
* LOCAL TYPE DECLARATIONS
******************************************************************************/

/******************************************************************************
* LOCAL CONSTANTS
******************************************************************************/

/******************************************************************************
* LOCAL VARIABLES
******************************************************************************/
static UART_HANDLE_T *_uart_handle = NULL;
static UART_HANDLE_T *_uart_dbg_handle = NULL;

/******************************************************************************
* LOCAL FUNCTION DECLARATIONS
******************************************************************************/
static void uart_rx_callback(uint8_t *pdata, uint16_t dlen);
static void uart_dbg_rx_callback(uint8_t *pdata, uint16_t dlen);

/******************************************************************************
* EXPORTED FUNCTIONS
******************************************************************************/
void uartest_init(void) {
    _uart_handle = uart_creat(&UART_PORT, 128, 32);
    uart_register_rcv_cb(_uart_handle, uart_rx_callback);

    _uart_dbg_handle = uart_creat(&UART_DEBUG, 32, 32);
    uart_register_rcv_cb(_uart_dbg_handle, uart_dbg_rx_callback);
}

/******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************/
static void uart_rx_callback(uint8_t *pdata, uint16_t dlen) {
    uart_send(_uart_dbg_handle, pdata, dlen);

    uint8_t pTxData[8];
    if(dlen == 7) {
        pTxData[0] = 0x55;
        pTxData[1] = 0xAA;
        pTxData[2] = 0x03;
        pTxData[3] = 0x00;
        pTxData[4] = 0x00;
        pTxData[5] = 0x01;
        pTxData[6] = 0x00;
        pTxData[7] = 0x03;
        if(pdata[3] == 0x00) {
            uart_send(_uart_handle, pTxData, 8);
            uart_send(_uart_dbg_handle, pTxData, 8);
        }
    }
}

static void uart_dbg_rx_callback(uint8_t *pdata, uint16_t dlen) {
    uint8_t str[] = "Receive data:";
    uart_send(_uart_dbg_handle, str, 13);
    uart_send(_uart_dbg_handle, pdata, dlen);
}

