/*
 * DVAReader.cpp
 *
 *  Created on: 6 août 2018
 *      Author: jonathan.michel
 */

#include "DVAReader.h"

DVAReader::DVAReader() {

}

DVAReader::~DVAReader() {

}

bool DVAReader::addAdcValue(uint32_t value) {
	measures.push_back(value);
	// Get max value each second
	// This process takes 1.5ms
	// Can be called from ADC interrupt if
	// isr period is long enough
	if(measures.size() == 40) {
		measures.sort();
		current = measures.back();
		measures.clear();
		return true;
	}
	return false;
}

uint32_t DVAReader::getCurrentValue() {
	return current;
}
