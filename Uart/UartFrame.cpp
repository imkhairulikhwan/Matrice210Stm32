/*
 * 	UartFrame.cpp
 *
 *  Created on: 13 juil. 2018
 *      Author: jonathan.michel
 */

#include "UartFrame.h"

#include "UartSender.h"
#include "Log.h"

UartFrame::UartFrame() {
	framePtr = frame;
}


bool UartFrame::pushInt(int32_t value) {
	char tempBf[BUFFER_SIZE];
	sprintf(tempBf, "%ld|", value);
	size_t length = strlen(tempBf);

	return addData(tempBf, length);
}

bool UartFrame::pushFloat(float value) {
	char tempBf[BUFFER_SIZE];
	sprintf(tempBf, "%.6f|", value);
	size_t length = strlen(tempBf);

	return addData(tempBf, length);
}

bool UartFrame::send() {
	framePtr--;
	*framePtr = '@';
	size_t length = (size_t)(framePtr - frame) + 1;
	return UartSender::getInstance()->send(UartSender::Pi, (uint8_t*)frame, length);
}

bool UartFrame::addData(const char* data, size_t length) {
	if(framePtr + length <= &frame[BUFFER_SIZE]) {
		memcpy(framePtr, data, length);
		framePtr = framePtr + length;
		return true;
	} else {
		Log::debug("Buffer filled, please ");
	}
	return false;
}
