/*
 * uart.c
 *
 *  Created on: 2020.06.15
 *      Author: Taohanyi
 */

/******************************************************************************
* LOCAL INCLUDE FILES
******************************************************************************/
#include <stddef.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "uart.h"

/******************************************************************************
* LOCAL MACROS AND DEFINITIONS
******************************************************************************/
#define UART_TIMEOUT      (3)

/******************************************************************************
* LOCAL TYPE DECLARATIONS
******************************************************************************/
typedef struct TS_UART_TRANSFER {
    uint8_t *pbuf;
    uint16_t bufSize;
    uint16_t bufCount;
    uint16_t bufWriteIndex;
    uint16_t bufReadIndex;
} TS_UART_TRANSFER;

typedef struct TS_UART_HANDLE {
    struct TS_UART_HANDLE *pnext;
    UART_HandleTypeDef *huart;
    osTimerId htimer;
    uart_rx_callback_t rx_callback;
    TS_UART_TRANSFER rx;
    TS_UART_TRANSFER tx;
} TS_UART_HANDLE;

/******************************************************************************
* LOCAL CONSTANTS
******************************************************************************/

/******************************************************************************
* LOCAL VARIABLES
******************************************************************************/
static TS_UART_HANDLE *_uart_list_head = NULL;

/******************************************************************************
* LOCAL FUNCTION DECLARATIONS
******************************************************************************/
static void uart_timer_callback(void const *argument);
static void uart_rx_callback(TS_UART_HANDLE *puart);
static void uart_tx_callback(TS_UART_HANDLE *puart);
static uint16_t uart_next_index(uint16_t index, uint16_t bufLen);
static void uart_buffer_clear(TS_UART_TRANSFER *pbuf);

/******************************************************************************
* EXPORTED FUNCTIONS
******************************************************************************/
UART_HANDLE_T *uart_creat(UART_HandleTypeDef *huart, uint16_t rxBufSize, uint16_t txBufSize) {
    TS_UART_HANDLE *puart;

    /* init uart data */
    puart = (TS_UART_HANDLE *)pvPortMalloc(sizeof(TS_UART_HANDLE));
    puart->huart = huart;
    puart->rx.pbuf = (uint8_t *)pvPortMalloc(rxBufSize);
    puart->rx.bufSize = rxBufSize;
    puart->rx.bufCount = 0;
    puart->rx.bufWriteIndex = 0;
    puart->rx.bufReadIndex = 0;
    puart->tx.pbuf = (uint8_t *)pvPortMalloc(txBufSize);
    puart->tx.bufSize = txBufSize;
    puart->tx.bufCount = 0;
    puart->tx.bufWriteIndex = 0;
    puart->tx.bufReadIndex = 0;

    /* insert into end of list */
    if(_uart_list_head == NULL) {
        _uart_list_head = puart;
    }
    else {
        TS_UART_HANDLE *temp_uart = _uart_list_head;
        while(temp_uart->pnext != NULL) {
            temp_uart = temp_uart->pnext;
        }
        temp_uart->pnext = puart;
    }

    return puart;
}

void uart_register_rcv_cb(UART_HANDLE_T *puart, uart_rx_callback_t cb) {
    /* definition and creation of Timer */
    osTimerDef(uartTimer, uart_timer_callback);
    puart->htimer = osTimerCreate(osTimer(uartTimer), osTimerOnce, puart);
    puart->rx_callback = cb;

    /* Start receive data */
    HAL_UART_Receive_IT(puart->huart, puart->rx.pbuf, 1);
}

int8_t uart_send(UART_HANDLE_T *puart, uint8_t *pdata, uint16_t data_len) {
    /* Check buffer is full */
    if(data_len > (puart->tx.bufSize - puart->tx.bufCount)) {
        return -1;
    }

    /* Copy data to TX buffer */
    for(uint16_t id = 0; id < data_len; id++) {
        *(puart->tx.pbuf + puart->tx.bufWriteIndex) = pdata[id];
        /* Move tx write pointer to next index */
        puart->tx.bufWriteIndex = uart_next_index(puart->tx.bufWriteIndex, puart->tx.bufSize);
    
        if(puart->tx.bufWriteIndex == puart->tx.bufReadIndex) {
            uart_buffer_clear(&puart->tx);
            return -1;
        }
    }

    if(puart->tx.bufCount == 0) {
        /* Send uart data */
        HAL_UART_Transmit_IT(puart->huart, puart->tx.pbuf + puart->tx.bufReadIndex, 1);
    }

    puart->tx.bufCount += data_len;
    return 0;
}

/******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************/
static void uart_timer_callback(void const *argument) {
    TS_UART_HANDLE *puart = pvTimerGetTimerID((osTimerId)argument);

    /* frame data received send to application layer */
    uint8_t * pdata = (uint8_t *)pvPortMalloc(puart->rx.bufCount);
    uint16_t size = puart->rx.bufCount;

    /* Copy data to RX buffer */
    for(uint16_t id = 0; id < size; id++) {
        pdata[id] = *(puart->rx.pbuf + puart->rx.bufReadIndex);
        /* Move rx read pointer to next index */
        puart->rx.bufReadIndex = uart_next_index(puart->rx.bufReadIndex, puart->rx.bufSize);
    }
    puart->rx.bufCount = 0;

    /* send received data */
    puart->rx_callback(pdata, size);

    vPortFree(pdata);
}

static void uart_rx_callback(TS_UART_HANDLE *puart) {
    /* receive one byte data */
    puart->rx.bufCount++;

    /* Move rx write index */
    puart->rx.bufWriteIndex = uart_next_index(puart->rx.bufWriteIndex, puart->rx.bufSize);

    /* Check buffer is overflow */
    if(puart->rx.bufWriteIndex == puart->rx.bufReadIndex) {
        uart_buffer_clear(&puart->rx);
    }

    /* Start receive next data */
    HAL_UART_Receive_IT(puart->huart, puart->rx.pbuf + puart->rx.bufWriteIndex, 1);

    /* start software timer to wait data receive finish */
    osTimerStart(puart->htimer, UART_TIMEOUT);
}

static void uart_tx_callback(TS_UART_HANDLE *puart) {
    /* Send one byte data */
    puart->tx.bufCount--;

    /* Move tx read pointer */
    puart->tx.bufReadIndex = uart_next_index(puart->tx.bufReadIndex, puart->tx.bufSize);

    /* Check is tx buffer empty */
    if(puart->tx.bufCount > 0) {
        HAL_UART_Transmit_IT(puart->huart, puart->tx.pbuf + puart->tx.bufReadIndex, 1);
    }
}

static uint16_t uart_next_index(uint16_t index, uint16_t bufLen) {
    index++;
    if(index == bufLen) {
        index = 0;
    }
    return index;
}

static void uart_buffer_clear(TS_UART_TRANSFER *pbuf) {
    pbuf->bufReadIndex = 0;
    pbuf->bufWriteIndex = 0;
    pbuf->bufCount = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    TS_UART_HANDLE *temp_uart = _uart_list_head;
    while(temp_uart != NULL) {
        if(temp_uart->huart->Instance == huart->Instance) {
            uart_rx_callback(temp_uart);
        }
        temp_uart = temp_uart->pnext;
    };
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    TS_UART_HANDLE *temp_uart = _uart_list_head;
    while(temp_uart != NULL) {
        if(temp_uart->huart->Instance == huart->Instance) {
            uart_tx_callback(temp_uart);
        }
        temp_uart = temp_uart->pnext;
    };
}

