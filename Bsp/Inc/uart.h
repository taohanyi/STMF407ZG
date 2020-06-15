/*
 * uart.h
 *
 *  Created on: 2020.06.15
 *      Author: Taohanyi
 */

#ifndef __UART_H
#define __UART_H

/******************************************************************************
* INCLUDE FILES
******************************************************************************/
#include <stdint.h>

/******************************************************************************
* EXPORTED MACROS
******************************************************************************/

/******************************************************************************
* EXPORTED TYPES
******************************************************************************/
typedef void (*uart_rx_callback_t)(uint8_t *pdata, uint16_t dlen);

struct TS_UART_HANDLE;
typedef struct TS_UART_HANDLE UART_HANDLE_T;

/******************************************************************************
* EXPORTED VARIABLES
******************************************************************************/

/******************************************************************************
* EXPORTED FUNCTIONS
******************************************************************************/
extern UART_HANDLE_T *uart_creat(UART_HandleTypeDef *huart, uint16_t rxBufSize, uint16_t txBufSize);
extern void uart_register_rcv_cb(UART_HANDLE_T *puart, uart_rx_callback_t cb);
extern int8_t uart_send(UART_HANDLE_T *puart, uint8_t *pdata, uint16_t data_len);

#endif
