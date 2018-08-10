/*
  * CrossSearch.h
 *
 *  Created on: 28 juin 2018
 *      Author: vincent.bontempe
 */

#define MEAS_TRIG 300
#define MEAS_GAP_RANGE 200
#define MEAS_NUMB 20
#define OVERPASSCOUNT 2

#ifndef CROSSSEARCH_H_
#define CROSSSEARCH_H_

#include <cstdint>
#include <cstdio>
//#include <iostream>

enum algoState
{IDLE,
START,
FORWARD,
BACKWARD,
ROTATE};
//enum treatementState {WAITING, LIST1, LIST2};
class UartFrame;


class CrossSearch
{
public:
	CrossSearch();
	virtual ~CrossSearch();
	void init();
	algoState process(uint32_t newValue);




private:

	bool dirChange; // direction has already change
	int32_t old_max;
	uint8_t decreaseCounter;
	bool decreasing;
	char displayBuffer [15];//for debug

	UartFrame* orderFrame;

	/*State machines*/
	algoState algoSMState;

	/*Direction s draws*/
	uint16_t forward_xPos = 125;
	uint16_t forward_yPos = 30;
	uint16_t backward_xPos = 125;
	uint16_t backward_yPos = 280;
	uint16_t radius = 20;

	/*functions*/
	void treatData(int32_t newValue);
	void goForward();
	void goBackward();
	void rotate(int32_t angle);
	bool playbackOrders(uint8_t orders[]);


};

#endif /* CROSSSEARCH_H_ */
