/*
 * DVAReader.h
 *
 *  Created on: 6 août 2018
 *      Author: jonathan.michel
 */

#ifndef DVAREADER_H_
#define DVAREADER_H_

#include <list>
#include <cstdint>


using namespace std;
/**
 * DVA signal has a frequency of 457kHz
 * but is pulsed at a frequency of 1 Hz
 * Pulse duration is ~80-100 ms
 * ADC pin is connected to a circuit that generates
 * the signal envelope.
 * The goal of this class is to read enough data from ADC
 * to determine antenna value. To do that, we read ADC
 * values each 25ms. After 40 measurements (1 second),
 * we extract max value and consider it as the antenna
 * value
 */
class DVAReader {
public:
	DVAReader();
	virtual ~DVAReader();
	bool addAdcValue(uint32_t value);
	uint32_t getCurrentValue();
private:
	list<uint32_t>measures;
	uint32_t current;
};

#endif /* DVAREADER_H_ */
