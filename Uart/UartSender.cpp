/*
 * UartSender.cpp
 *
 *  Created on: 13 juil. 2018
 *      Author: jonathan.michel
 */

#include "UartSender.h"


UartSender* UartSender::instance = nullptr;

UartSender::UartSender() {
	uart[0] = nullptr;
	uart[1] = nullptr;
}


UartSender* UartSender::getInstance() {
	if(instance == nullptr)
		instance = new UartSender();
	return instance;
}

void UartSender::config(UART_HandleTypeDef *uartPc, UART_HandleTypeDef *uartPi) {
	uart[0] = uartPc;
	uart[1] = uartPi;
}

bool UartSender::send(UartDestination destination, uint8_t *pData, uint16_t length) {
	if(uart[destination] == nullptr)
		errorHandler();

	HAL_StatusTypeDef ack = HAL_UART_Transmit(uart[destination], pData, length, 500);
	if(ack != HAL_OK)
		return false;
	return true;
}

// Please call UartSender::getInstance()->config() with UART to use before send() method
void UartSender::errorHandler() {
	while(1)
		;
}
