/*
 * DVAReader.cpp
 *
 *  Created on: 6 août 2018
 *      Author: jonathan.michel
 */

#include "DVAReader.h"

DVAReader::DVAReader() {
	newValue = false;
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
		// Enough values have been read to determine current
		// antenna value
		// We cannot directly use ADC value as antenna value
		// because the signal is pulsed

		// Antenna value will be read in main
		measures.sort();
		current = measures.back();
		measures.clear();
		newValue = true;
		return true;
	}
	return false;
}

bool DVAReader::isNewValue() {
	if(newValue) {
		newValue = false;
		return true;
	}
	return false;
}

uint32_t DVAReader::getCurrentValue() {
	return current;
}
