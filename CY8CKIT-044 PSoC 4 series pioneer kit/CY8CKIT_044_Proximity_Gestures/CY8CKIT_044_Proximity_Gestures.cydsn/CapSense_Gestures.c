/*****************************************************************************
* File Name		: CapSense_Gestures.c
* Version		: 1.0 
*
* Description:
*  This file defines the function used to detect the proximity gesture made 
*  by an approaching hand.
*
******************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#include "CapSense_Gestures.h"
#include "LED_Control.h"
#include "project.h"

extern uint16 brightness;
/*******************************************************************************
* Function Name: DetectGesture
********************************************************************************
* Summary:
*  This function scans the proximity sensors individually and determines if a 
*  valid gesture has been made.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
uint8 DetectGesture(void)
{
	/* Variables to store proximity states. */
	uint8 prox_0_state = INACTIVE;
	uint8 prox_1_state = INACTIVE;
	
	/* Variable declared static to remember the state machine
	 * state between function calls. */
	static uint8 currState = INACTIVE;
	static uint8 prevState = INACTIVE;
	static uint8 proxCounter = RESET;
	
	uint8 gesture = NO_GESTURE;
	
	/* Get the proximity sensor states. */
	prox_0_state = CapSense_CheckIsWidgetActive(CapSense_PROXIMITYSENSOR0__PROX);	
	prox_1_state = CapSense_CheckIsWidgetActive(CapSense_PROXIMITYSENSOR1__PROX);
	
	/* Determine if a gesture is detected by checking if the 
	 * sensors are active or inactive in the current state. */
	switch(currState)
	{
		case INACTIVE:
			if(prox_0_state == ACTIVE && prox_1_state == INACTIVE)
			{
				proxCounter++;
				if(proxCounter > PROX_DETECT_LIMIT)
				{
					proxCounter = RESET;
					currState = UP_ACTIVE;
				}
			}
			
			if(prox_0_state == INACTIVE && prox_1_state == ACTIVE)
			{
				proxCounter++;
				if(proxCounter > PROX_DETECT_LIMIT)
				{
					proxCounter = RESET;
					currState = DOWN_ACTIVE;
				}
			}
			
			if(prox_0_state == ACTIVE && prox_1_state == ACTIVE)
			{
				currState = BRIGHTNESS_CONTROL;
			}

			if(prox_0_state == INACTIVE && prox_1_state == INACTIVE)
			{
				currState = INACTIVE;
				gesture = SENSORS_INACTIVE;
			}
			prevState = INACTIVE;
			
			break;
			
		case UP_ACTIVE:
			if(prox_0_state == INACTIVE && prox_1_state == ACTIVE)
			{
				currState = UP_END;
			}
			
			if((prox_0_state == ACTIVE && prox_1_state == ACTIVE) || (prox_0_state == ACTIVE && prox_1_state == INACTIVE))
			{
				currState = BRIGHTNESS_CONTROL;
			}
			
			if(prox_0_state == INACTIVE && prox_1_state == INACTIVE)
			{
				currState = INACTIVE;
				gesture = SENSORS_INACTIVE;
			}
			prevState = UP_ACTIVE;
			break;
			
		case UP_END:
			if(prox_0_state == ACTIVE && prox_1_state == ACTIVE)
			{
				currState = BRIGHTNESS_CONTROL;
			}
			
			if(prox_0_state == INACTIVE)
			{
				gesture = DOWN_SWIPE;
				currState = INACTIVE;
			}
			prevState = UP_END;
			break;
			
		case DOWN_ACTIVE:
			if(prox_0_state == ACTIVE && prox_1_state == INACTIVE)
			{
				currState = DOWN_END;
			}
			
			if((prox_0_state == ACTIVE && prox_1_state == ACTIVE) || (prox_0_state == INACTIVE && prox_1_state == ACTIVE))
			{
				currState = BRIGHTNESS_CONTROL;
			}
			
			if(prox_0_state == INACTIVE && prox_1_state == INACTIVE)
			{
				currState = INACTIVE;
				gesture = SENSORS_INACTIVE;
			}
			prevState = DOWN_ACTIVE;
			break;
			
		case DOWN_END:
			if(prox_0_state == ACTIVE && prox_1_state == ACTIVE)
			{
				currState = BRIGHTNESS_CONTROL;
			}
			
			if(prox_1_state == INACTIVE)
			{
				gesture = UP_SWIPE;
				currState = INACTIVE;
			}
			prevState = DOWN_END;
			break;
			
		case BRIGHTNESS_CONTROL:
			if(SENSOR_SIGNAL < PROX_UPPER_LIMIT)
			{
				brightness = ((MAX_BRIGHT * (SENSOR_SIGNAL - PROX_LOWER_LIMIT))/(PROX_UPPER_LIMIT - PROX_LOWER_LIMIT));
			}
			else
			{
				brightness = MAX_BRIGHT;
			}
			switch(prevState)
			{
				case UP_ACTIVE:
				case UP_END:
					if(prox_0_state == INACTIVE && prox_1_state == ACTIVE)
					{
						currState = UP_END;
					}
					break;
				
				case DOWN_END:
				case DOWN_ACTIVE:
					if(prox_0_state == ACTIVE && prox_1_state == INACTIVE)
					{
						currState = DOWN_END;
					}
					break;
			}

			if(prox_0_state == INACTIVE && prox_1_state == INACTIVE)
			{
				currState = INACTIVE;
				gesture = SENSORS_INACTIVE;
			}
			DisplayLEDOutput(BRIGHTNESS_CONTROL);
			break;
			
		default:
			/* Reset the state machine for any unknown state. */
			currState = INACTIVE;
			gesture = SENSORS_INACTIVE;
			break;				
	}
	
	return(gesture);
}

/* [] END OF FILE */
