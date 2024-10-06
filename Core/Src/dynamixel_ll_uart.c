#include "dynamixel_ll_uart.h"

// Bit of a hack, but we need to know which thread to get back to
extern osThreadId_t servoCallbackThreadId;

dynamixel_result_t dynamixel_write_uart_dma(uint8_t *txBuffer, size_t size, void *pvContext) {
	if (pvContext == NULL) {
		return DNM_LL_ERR;
	}

	dynamixel_ll_uart_context *context = (dynamixel_ll_uart_context *)pvContext;
	UART_HandleTypeDef *huart = context->huart;
	servoCallbackThreadId = context->callerThread;

	while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY) {}

	// Enable the transmitter and start the DMA transfer
	HAL_HalfDuplex_EnableTransmitter(huart);
	if (HAL_UART_Transmit_DMA(huart, txBuffer, size) != HAL_OK) {
		return DNM_LL_ERR;
	}

	// Wait for the TX complete flag
	uint32_t flags = osThreadFlagsWait(DYNAMIXEL_DMA_TX_CPLT | DYNAMIXEL_DMA_ERR, osFlagsWaitAny, pdMS_TO_TICKS(50));

	if (flags == (uint32_t)osErrorTimeout) {
		return DNM_LL_TIMEOUT;
	}

	if (flags != DYNAMIXEL_DMA_TX_CPLT) {
		return DNM_LL_ERR;
	}

	return DNM_OK;
}

dynamixel_result_t dynamixel_read_uart_dma(uint8_t *rxBuffer, size_t size, void *pvContext) {
	if (pvContext == NULL) {
		return DNM_LL_ERR;
	}

	dynamixel_ll_uart_context *context = (dynamixel_ll_uart_context *)pvContext;
	UART_HandleTypeDef *huart = context->huart;
	servoCallbackThreadId = context->callerThread;

	while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY) {}

	// Enable the transmitter and start the DMA transfer
	HAL_HalfDuplex_EnableReceiver(huart);
	if (HAL_UART_Receive_DMA(huart, rxBuffer, 14) != HAL_OK) {
		return DNM_LL_ERR;
	}

	// Wait for the RX complete flag
	uint32_t flags = osThreadFlagsWait(DYNAMIXEL_DMA_RX_CPLT | DYNAMIXEL_DMA_ERR, osFlagsWaitAny, pdMS_TO_TICKS(50));

	if (flags == (uint32_t)osErrorTimeout) {
		HAL_UART_DMAStop(huart);
		return DNM_LL_TIMEOUT;
	}

	if (flags != DYNAMIXEL_DMA_RX_CPLT) {
		HAL_UART_DMAStop(huart);
		return DNM_LL_ERR;
	}

	HAL_UART_DMAStop(huart);
	return DNM_OK;
}
