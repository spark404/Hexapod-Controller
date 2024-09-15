#ifndef DYNAMIXEL_LL_UART_H
#define DYNAMIXEL_LL_UART_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "dynamixel.h"

#define DYNAMIXEL_DMA_TX_CPLT 0x01
#define DYNAMIXEL_DMA_RX_CPLT 0x02
#define DYNAMIXEL_DMA_ERR     0x04

typedef struct {
	UART_HandleTypeDef *huart;
	osThreadId_t callerThread;
} dynamixel_ll_uart_context;

dynamixel_result_t dynamixel_write_uart_dma(uint8_t *txBuffer, size_t size, void *pvContext);
dynamixel_result_t dynamixel_read_uart_dma(uint8_t *rxBuffer, size_t size, void *pvContext);


#endif /* DYNAMIXEL_LL_UART_H */
