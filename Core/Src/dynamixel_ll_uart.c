/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dynamixel_ll_uart.h"

#include "log.h"

// Bit of a hack, but we need to know which thread to get back to
extern osThreadId_t servoCallbackThreadId;

ssize_t dynamixel_write_uart_dma(const uint8_t *txBuffer, const size_t size, void *pvContext) {
	if (pvContext == NULL) {
		return -1;
	}

	dynamixel_ll_uart_context *context = (dynamixel_ll_uart_context *)pvContext;
	UART_HandleTypeDef *huart = context->huart;
	servoCallbackThreadId = context->callerThread;

	while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY) {}

	// Enable the transmitter and start the DMA transfer
	HAL_HalfDuplex_EnableTransmitter(huart);
	if (HAL_UART_Transmit_DMA(huart, txBuffer, size) != HAL_OK) {
		return -1;
	}

	// Wait for the TX complete flag
	const uint32_t flags = osThreadFlagsWait(DYNAMIXEL_DMA_TX_CPLT | DYNAMIXEL_DMA_ERR, osFlagsWaitAny, pdMS_TO_TICKS(50));

	if (flags == (uint32_t)osErrorTimeout) {
		HAL_UART_DMAStop(huart);
		// LOG_DEBUG("dynamixel_write_uart_dma: osThreadFlagsWait timeout");
		return -1;
	}

	if (flags & (1U << 31)) {
		HAL_UART_DMAStop(huart);
		LOG_DEBUG("dynamixel_write_uart_dma: osThreadFlagsWait error %ld", flags);
		return -1;
	}

	if (flags == DYNAMIXEL_DMA_ERR) {
		HAL_UART_DMAStop(huart);
		LOG_DEBUG("dynamixel_write_uart_dma: uart error");
		return -1;
	}

	return (ssize_t)size;
}

ssize_t dynamixel_read_uart_dma(uint8_t *rxBuffer, const size_t size, void *pvContext) {
	if (pvContext == NULL) {
		return -1;
	}

	const dynamixel_ll_uart_context *context = (dynamixel_ll_uart_context *)pvContext;
	UART_HandleTypeDef *huart = context->huart;
	servoCallbackThreadId = context->callerThread;

	while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY) {}

	// Enable the transmitter and start the DMA transfer
	HAL_HalfDuplex_EnableReceiver(huart);
	if (HAL_UART_Receive_DMA(huart, rxBuffer, size) != HAL_OK) {
		return -1;
	}

	// Wait for the RX complete flag
	const uint32_t flags = osThreadFlagsWait(DYNAMIXEL_DMA_RX_CPLT | DYNAMIXEL_DMA_ERR, osFlagsWaitAny, pdMS_TO_TICKS(15));

	if (flags == (uint32_t)osErrorTimeout) {
		HAL_UART_DMAStop(huart);
		// LOG_DEBUG("dynamixel_read_uart_dma: osThreadFlagsWait timeout");
		return -1;
	}

	if (flags & (1U << 31)) {
		HAL_UART_DMAStop(huart);
		LOG_DEBUG("dynamixel_read_uart_dma: osThreadFlagsWait error %ld", flags);
		return -1;
	}

	if (flags == DYNAMIXEL_DMA_ERR) {
		HAL_UART_DMAStop(huart);
		LOG_DEBUG("dynamixel_read_uart_dma: uart error");
		return -1;
	}

	HAL_UART_DMAStop(huart);
	return (ssize_t)size;
}
