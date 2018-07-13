/*
 * crossSearch.cpp
 *
 *  Created on: 28 juin 2018
 *      Author: vincent.bontempe
 */

#include "crossSearch.h"
//#include "math.h"

crossSearch::crossSearch() {

}

void crossSearch::init()
{
	meas_counter = 0;
	dataUpdated = false;
	direction = false;
	algoSMState = IDLE;
	old_max = 0;
	decreaseCounter = OVERPASSCOUNT;
	lastTreatMeas = meas_set.begin();
}
crossSearch::~crossSearch() {

}

void crossSearch::add(uint32_t measure)
{
	/*add to current list*/
	meas_set.push_back(measure);
	meas_counter++;

	/*check if the list is full*/
	if(meas_counter >= 20)
	{
		dataUpdated = true;
		meas_counter = 0;
	}
}

uint32_t crossSearch::sortSavedList()
{
	savedMeas.sort();
	return savedMeas.back();
}

algoState crossSearch::process()
{
	/*debug*/
	BSP_LCD_ClearStringLine(8);
	sprintf(displayBuffer, "%lu", old_max);
	BSP_LCD_DisplayStringAtLine(8, (uint8_t*) displayBuffer);

	switch (algoSMState)
	{
	case IDLE:
		/*Displaying*/
		BSP_LCD_DrawCircle(forward_xPos, forward_yPos, radius);
		BSP_LCD_DrawCircle(backward_xPos, backward_yPos, radius);
		break;

	case START:
		BSP_LCD_DisplayStringAtLine(3, (uint8_t*) "Looking for");
		BSP_LCD_DisplayStringAtLine(4, (uint8_t*) "signal");

		if(meas_set.front() > MEAS_TRIG)
		{
			algoSMState = FORWARD;
			BSP_LCD_ClearStringLine(3);
			BSP_LCD_ClearStringLine(4);
			meas_set.clear();
			meas_counter = 0;
		}
		/*to have only one value in list during the signal research*/
		meas_set.clear();
		meas_counter = 0;
		break;

	case FORWARD:
		/*Displaying*/
		BSP_LCD_ClearStringLine(10);
		BSP_LCD_ClearStringLine(11);
		BSP_LCD_ClearStringLine(12);
		BSP_LCD_DrawCircle(forward_xPos, forward_yPos, radius);
		if(dataUpdated)
		{
			dataUpdated = false;
			treatData();
			if(direction)
			{
				algoSMState = BACKWARD;
				direction = false;
			}

		}
		break;

	case BACKWARD:
		BSP_LCD_ClearStringLine(0);
		BSP_LCD_ClearStringLine(1);
		BSP_LCD_ClearStringLine(2);
		BSP_LCD_DrawCircle(backward_xPos, backward_yPos, radius);
		if(dataUpdated)
		{
			dataUpdated = false;
			treatData();
			if(direction)
			{
				algoSMState = FORWARD;
				direction = false;
			}

		}
		break;

	default:
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAtLine(5, (uint8_t*) "DIRECTION");
		BSP_LCD_DisplayStringAtLine(6, (uint8_t*) "ERROR");
		break;

	}
	return algoSMState;
}

void crossSearch::treatData()
{
	__disable_irq();
	/*copy the 20 first measure in save measure list*/
	if(meas_set.size() >= MEAS_NUMB)
	{
		savedMeas.assign(std::next(meas_set.begin(), 0), std::next(meas_set.begin() , 19));

		/*setting the iterator on the 20th measure of measure list*/
		lastTreatMeas = meas_set.begin();
		std::advance (lastTreatMeas,19);

		/*erase the 20 first measure of the current measure list*/
		meas_set.erase(meas_set.begin(),lastTreatMeas);
	}
	__enable_irq();

	/*sort the saved list*/
	int32_t temp_max = sortSavedList();

	//
	if((old_max - temp_max) > MEAS_GAP_RANGE)
	{
		/*Make sure that the magnetic field value are really decreasing
		 * Make sure that a least 3 values are different from MEASGAPRANGE before changing*/
		if(decreaseCounter == 0)
		{
			//change direction
			direction = true;
			//reset counter
			decreaseCounter = OVERPASSCOUNT;
		}
		decreaseCounter--;
	}
	old_max = temp_max;
	savedMeas.clear();
}


void crossSearch::startMeasure()
{
	algoSMState = START;
}
