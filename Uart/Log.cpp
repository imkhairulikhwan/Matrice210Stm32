/*
 * Log.cpp
 *
 *  Created on: 13 juil. 2018
 *      Author: jonathan.michel
 */

#include "Log.h"

#include "UartSender.h"

void Log::debug( const char * format, ... ) {
  char buffer[256];
  va_list args;
  va_start (args, format);
  vsprintf (buffer,format, args);
  va_end (args);
  UartSender::getInstance()->send(UartSender::PC, (uint8_t*)"\r\n", 2);
  UartSender::getInstance()->send(UartSender::PC, (uint8_t *)buffer, strlen(buffer));
}
