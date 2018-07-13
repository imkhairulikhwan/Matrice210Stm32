/*
 * Log.h
 *
 *  Created on: 13 juil. 2018
 *      Author: jonathan.michel
 */

#ifndef LOG_H_
#define LOG_H_

#include <cstdarg>
#include <cstring>
#include <cstdio>

using namespace std;

class Log {
private:
	Log() {}
	virtual ~Log() {}
public:
	static void debug( const char * format, ... );
};

#endif /* LOG_H_ */
