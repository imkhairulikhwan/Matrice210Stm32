/*
 * UartSender.h
 *
 *  Created on: 13 juil. 2018
 *      Author: jonathan.michel
 */

#ifndef UARTSENDER_H_
#define UARTSENDER_H_

#include <cstdio>

#include "stm32f4xx_hal.h"

class UartSender {
public:
	typedef enum UartDestination_ {
		PC,
		Pi
	} UartDestination;
	static UartSender* getInstance();
	void config(UART_HandleTypeDef *pc, UART_HandleTypeDef *pi);
	bool send(UartDestination destination, uint8_t *pData, uint16_t length);
private:
	UartSender();
	virtual ~UartSender() {}
	void errorHandler();
	static UartSender* instance;
	UART_HandleTypeDef* uart[2];
};

#endif /* UARTSENDER_H_ */
