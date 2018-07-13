/*
 * Frame.h
 *
 *  Created on: 13 juil. 2018
 *      Author: jonathan.michel
 */

#ifndef UARTFRAME_H_
#define UARTFRAME_H_

#define BUFFER_SIZE 256

#include <cstdio>
#include <cstring>

using namespace std;

class UartFrame {
private:
	char frame[BUFFER_SIZE];
	char* framePtr;
	bool addData(const char* data, size_t length);
public:
	UartFrame();
	virtual ~UartFrame() {}
	bool pushInt(int32_t value);
	bool pushFloat(float value);
	bool send();
};

#endif /* UARTFRAME_H_ */
