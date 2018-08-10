/*
 * CrossSearch.cpp
 *
 *  Created on: 28 juin 2018
 *      Author: vincent.bontempe
 */

#include "CrossSearch.h"
#include <string.h>
#include <list>
#include "stm32f429i_discovery_lcd.h"
#include "UartFrame.h"


CrossSearch::CrossSearch() {

}

void CrossSearch::init()
{
	decreasing = false;
	dirChange = false;
	algoSMState = FORWARD;
	old_max = 0;
	decreaseCounter = OVERPASSCOUNT;
	orderFrame = new UartFrame();
}
CrossSearch::~CrossSearch() {
	delete orderFrame;
}


algoState CrossSearch::process(uint32_t newValue)
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

		//if(meas_set.front() > MEAS_TRIG)
		//{
			algoSMState = FORWARD;
			BSP_LCD_ClearStringLine(3);
			BSP_LCD_ClearStringLine(4);
		//}
		/*to have only one value in list during the signal research*/
		break;

	case FORWARD:
		/*Displaying*/
		BSP_LCD_ClearStringLine(10);
		BSP_LCD_ClearStringLine(11);
		BSP_LCD_ClearStringLine(12);
		BSP_LCD_DrawCircle(forward_xPos, forward_yPos, radius);
		treatData(newValue);
		if(decreasing)
		{
			if(dirChange)
			{
				algoSMState = ROTATE;
				dirChange = false;
				//reset
			}
			else
			{
				algoSMState = BACKWARD;
				dirChange = true;
			}
		}

		break;

	case BACKWARD:
		BSP_LCD_ClearStringLine(0);
		BSP_LCD_ClearStringLine(1);
		BSP_LCD_ClearStringLine(2);
		BSP_LCD_DrawCircle(backward_xPos, backward_yPos, radius);
		treatData(newValue);
		if(decreasing)
		{
			if(dirChange)
			{
				algoSMState = ROTATE;
				dirChange = false;
				//reset
			}
			else
			{
				algoSMState = FORWARD;
				dirChange = true;
			}
		}
		break;

	case ROTATE:
		/*play back last orders to be
		 * back where the max intensity
		 * was measured*/

		//Send rotate order
		//wait for order receive ACK
		//wait for order reach ACK
		algoSMState = FORWARD;
		break;

	default:
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAtLine(5, (uint8_t*) "DIRECTION");
		BSP_LCD_DisplayStringAtLine(6, (uint8_t*) "ERROR");
		break;

	}
	return algoSMState;
}

void CrossSearch::treatData(int32_t newValue)
{
	if((old_max - newValue) > MEAS_GAP_RANGE)
	{
		/*Make sure that the magnetic field value are really decreasing
		 * Make sure that a least 3 values are different from MEASGAPRANGE before changing*/
		if(decreaseCounter == 0)
		{
			//change direction
			decreasing = true;
			//reset counter
			decreaseCounter = OVERPASSCOUNT;
		}
		decreaseCounter--;
	}
	old_max = newValue;
}

void CrossSearch::goForward()
{
	orderFrame->pushInt(1);//it s an order
	orderFrame->pushInt(1); //go forward
	orderFrame->send();
}
void CrossSearch::goBackward()
{
	orderFrame->pushInt(1);//it s an order
	orderFrame->pushInt(2); //go backward
	orderFrame->send();
}
void CrossSearch::rotate(int32_t angle)
{
	orderFrame->pushInt(1);//it s an order
	orderFrame->pushInt(3); //rotation
	orderFrame->send();
}

bool CrossSearch::playbackOrders(uint8_t orders[])
{

}
