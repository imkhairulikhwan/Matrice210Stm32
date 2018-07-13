/*
  * crossSearch.h
 *
 *  Created on: 28 juin 2018
 *      Author: vincent.bontempe
 */
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <list>
#include "stm32f429i_discovery_lcd.h"

#define MEAS_TRIG 300
#define MEAS_GAP_RANGE 200
#define MEAS_NUMB 20
#define OVERPASSCOUNT 2

#ifndef CROSSSEARCH_H_
#define CROSSSEARCH_H_

enum algoState
{IDLE,
START,
FORWARD,
BACKWARD};
//enum treatementState {WAITING, LIST1, LIST2};

class crossSearch
{
public:
	crossSearch();
	virtual ~crossSearch();
	void init();
	void add(uint32_t measure);
	algoState process();
	void startMeasure();



private:
	/*list*/
	std::list<int32_t>meas_set;
	std::list<int32_t>savedMeas;
	uint8_t meas_counter;
	std::list<int32_t>::iterator lastTreatMeas;

	bool dataUpdated; //20 value of measure have been done
	bool direction; // have to change direction

	int32_t old_max;
	uint8_t decreaseCounter;
	char displayBuffer [15];//for debug

	/*State machines*/
	algoState algoSMState;

	//treatementState treatSMState;

	/*Direction s draws*/
	uint16_t forward_xPos = 125;
	uint16_t forward_yPos = 30;
	uint16_t backward_xPos = 125;
	uint16_t backward_yPos = 280;
	uint16_t radius = 20;

	/*functions*/
	uint32_t sortSavedList();
	void treatData();

};

#endif /* CROSSSEARCH_H_ */
