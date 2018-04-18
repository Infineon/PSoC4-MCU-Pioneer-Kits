/*****************************************************************************
* File Name		: CapSense_Gestures.c
* Version		: 1.0 
*
* Description:
*  This file defines the function used to detect the touch gesture made 
*  by the user.
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
#include "project.h"

/* Variables to store touch states. */
static uint8 left_btnCurrent = INACTIVE;
static uint8 right_btnCurrent = INACTIVE;
static uint8 up_btnCurrent = INACTIVE;
static uint8 down_btnCurrent = INACTIVE;
static uint8 centre_btnCurrent = INACTIVE;

/* Variables to update the status of touch buttons on uC/Probe */
extern uint8 uC_left_btnCurrent;
extern uint8 uC_right_btnCurrent;
extern uint8 uC_up_btnCurrent;
extern uint8 uC_down_btnCurrent;
extern uint8 uC_centre_btnCurrent;

/*******************************************************************************
* Function Name: DetermineActiveButton
********************************************************************************
* Summary:
*  This function determines the button with the maximum signal. This function 
*  does not consider the centre button during the calculation.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void DetermineActiveButton(uint8 centreButtonStatus)
{
	/* Local variables to store the signal values for buttons. */
	uint16 left = ZERO, up = ZERO, right = ZERO, down = ZERO, centre = ZERO;

	/* Read the signal value if the button is active. The variable is 
	 * set to zero by default if button is not active. */
	if(left_btnCurrent == ACTIVE)
	{
		left = CapSense_SensorSignal[CapSense_SENSOR_LEFT_BUTTON__BTN];
	}
	
	/* Read the signal value if the button is active. The variable is 
	 * set to zero by default if button is not active. */
	if(right_btnCurrent == ACTIVE)
	{
		right = CapSense_SensorSignal[CapSense_SENSOR_RIGHT_BUTTON__BTN];
	}
	
	/* Read the signal value if the button is active. The variable is 
	 * set to zero by default if button is not active. */
	if(up_btnCurrent == ACTIVE)
	{
		up = CapSense_SensorSignal[CapSense_SENSOR_UP_BUTTON__BTN];
	}
	
	/* Read the signal value if the button is active. The variable is 
	 * set to zero by default if button is not active. */
	if(down_btnCurrent == ACTIVE)
	{
		down = CapSense_SensorSignal[CapSense_SENSOR_DOWN_BUTTON__BTN];
	}
	
	/* Read the signal value if the button is active. The variable is 
	 * set to zero by default if button is not active. */
	if(centre_btnCurrent == ACTIVE && centreButtonStatus == WITH_CENTRE)
	{
		centre = CapSense_SensorSignal[CapSense_SENSOR_CENTRE_BUTTON__BTN];
	}
	
	/* Logic to determine the button with maximum signal. */
	if(left > up)
	{
		up_btnCurrent = INACTIVE;
		if(left > right)
		{
			right_btnCurrent = INACTIVE;
			if(left > down)
			{
				down_btnCurrent = INACTIVE;
				if(left > centre)
				{
					centre_btnCurrent = INACTIVE;
				}
				else
				{
					left_btnCurrent = INACTIVE;
				}
			}
			else
			{
				left_btnCurrent = INACTIVE;
				if(down > centre)
				{
					centre_btnCurrent = INACTIVE;
				}
				else
				{
					down_btnCurrent = INACTIVE;
				}
			}
		}
		else
		{
			left_btnCurrent = INACTIVE;
			if(right > down)
			{
				down_btnCurrent = INACTIVE;
				if(right > centre)
				{
					centre_btnCurrent = INACTIVE;
				}
				else
				{
					right_btnCurrent = INACTIVE;
				}
			}
			else
			{
				right_btnCurrent = INACTIVE;
				if(down > centre)
				{
					centre_btnCurrent = INACTIVE;
				}
				else
				{
					down_btnCurrent = INACTIVE;
				}
			}
		}		
	}
	else
	{
		left_btnCurrent = INACTIVE;
		if(up > right)
		{
			right_btnCurrent = INACTIVE;
			if(up > down)
			{
				down_btnCurrent = INACTIVE;
				if(up > centre)
				{
					centre_btnCurrent = INACTIVE;
				}
				else
				{
					up_btnCurrent = INACTIVE;
				}
			}
			else
			{
				up_btnCurrent = INACTIVE;
				if(down > centre)
				{
					centre_btnCurrent = INACTIVE;
				}
				else
				{
					down_btnCurrent = INACTIVE;
				}
			}
		}
		else
		{
			up_btnCurrent = INACTIVE;
			if(right > down)
			{
				down_btnCurrent = INACTIVE;
				if(right > centre)
				{
					centre_btnCurrent = INACTIVE;
				}
				else
				{
					right_btnCurrent = INACTIVE;
				}
			}
			else
			{
				right_btnCurrent = INACTIVE;
				if(down > centre)
				{
					centre_btnCurrent = INACTIVE;
				}
				else
				{
					down_btnCurrent = INACTIVE;
				}
			}
		}
	}
		
}

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
*  uint8: Detected gesture.
*
*******************************************************************************/
uint8 DetectGesture(void)
{
	/* Variable declared static to remember the state machine
	 * state between function calls. */
	static uint8 currState = INACTIVE;
	
	uint8 gesture = NO_GESTURE;
	
	/* Variable to save the previously activated button of gesture pad. */
	static uint8 prevActButton = NONE;
	static uint8 loopingActive = INACTIVE;
	
	/* Get the sensor states  */
	left_btnCurrent = CapSense_CheckIsWidgetActive(CapSense_SENSOR_LEFT_BUTTON__BTN);	
	up_btnCurrent = CapSense_CheckIsWidgetActive(CapSense_SENSOR_UP_BUTTON__BTN);
	right_btnCurrent = CapSense_CheckIsWidgetActive(CapSense_SENSOR_RIGHT_BUTTON__BTN);
	down_btnCurrent = CapSense_CheckIsWidgetActive(CapSense_SENSOR_DOWN_BUTTON__BTN);
	centre_btnCurrent = CapSense_CheckIsWidgetActive(CapSense_SENSOR_CENTRE_BUTTON__BTN);
	
	/* Code to update the button status in uC/Probe. */
	uC_left_btnCurrent = left_btnCurrent;
	uC_right_btnCurrent = right_btnCurrent;
	uC_up_btnCurrent = up_btnCurrent;
	uC_down_btnCurrent = down_btnCurrent;
	uC_centre_btnCurrent = centre_btnCurrent;
	
	/* If all sensors are inactive, report it back to the calling 
	 * function. Reset the state machine. */
	if((left_btnCurrent == INACTIVE) && (up_btnCurrent == INACTIVE) && \
 	(right_btnCurrent == INACTIVE) && (down_btnCurrent == INACTIVE) && \
	(centre_btnCurrent == INACTIVE))
	{
		gesture = SENSORS_INACTIVE;
		if(loopingActive == ACTIVE)
		{
			loopingActive = INACTIVE;
			currState = INACTIVE;
		}
	}
	else
	{
		if(loopingActive == ACTIVE)
		{
			DetermineActiveButton(IGNORE_CENTRE);
		}
		else
		{
			DetermineActiveButton(WITH_CENTRE);
		}
	}
					
	/* Combines the touch states to get the gesture. The following code 
	 * implements a state machine to recognize the gesture made by the user.
	 * All the states of the state machine are defined in the file 
	 * CapSense_Gestures.h. The state machine transitions to next state as 
	 * soon as the user touches one of the sensors. The state name is same 
	 * as that of the sensor on which user touched. The state machine waits
	 * for a subsequent touch from the user to determine the intended gesture
	 * from the user, and waits until the user completes the gesture. If the
	 * user removes the finger before completing a gesture, the state machine
	 * is reset. 
	 * State machine also supports accelerate clockwise/counter-clockwise gesture
	 * recognition. If the user begins a clockwise/counter-clockwise gesture, 
	 * after the fist complete rotation the state machine reflects the gesture
	 * output for subsequent rotations after 3/4 rotations. */
	switch(currState)
	{
		/* State machine reset state. If any sensor is active, the 
		 * state machine transitions to the corresponding state. */
		case INACTIVE:
			if(left_btnCurrent == ACTIVE)
			{
				currState = LEFT_START;
			}
			
			if(up_btnCurrent == ACTIVE)
			{
				currState = UP_START;
			}
			
			if(right_btnCurrent == ACTIVE)
			{
				currState = RIGHT_START;
			}
			
			if(down_btnCurrent == ACTIVE)
			{
				currState = DOWN_START;
			}
			prevActButton = NONE;
			break;
		
		/* State machine detected beginning of gesture. User 
		 * started the gesture by touching the LEFT sensor. */
		case LEFT_START:
			if((centre_btnCurrent == ACTIVE) && (up_btnCurrent == INACTIVE) && (down_btnCurrent == INACTIVE) && (loopingActive == INACTIVE))
			{
				currState = RIGHT_SWIPE_BEGIN;
			}
			
			if(up_btnCurrent == ACTIVE)
			{
				currState = CLOCK_SWIPE_BEGIN;
				prevActButton = LEFT_BUTTON;
			}
			
			if(down_btnCurrent == ACTIVE)
			{
				currState = COUNTER_CLOCK_SWIPE_BEGIN;
				prevActButton = LEFT_BUTTON;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
		
		/* State machine detected beginning of gesture. User 
		 * started the gesture by touching the UP sensor. */
		case UP_START:
			if((centre_btnCurrent == ACTIVE) && (right_btnCurrent == INACTIVE) && (left_btnCurrent == INACTIVE) && (loopingActive == INACTIVE))
			{
				currState = DOWN_SWIPE_BEGIN;
			}
			
			if(right_btnCurrent == ACTIVE)
			{
				currState = CLOCK_SWIPE_BEGIN;
				prevActButton = UP_BUTTON;
			}
			
			if(left_btnCurrent == ACTIVE)
			{
				currState = COUNTER_CLOCK_SWIPE_BEGIN;
				prevActButton = UP_BUTTON;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;

		/* State machine detected beginning of gesture. User 
		 * started the gesture by touching the RIGHT sensor. */
		case RIGHT_START:
			if((centre_btnCurrent == ACTIVE) && (up_btnCurrent == INACTIVE) && (down_btnCurrent == INACTIVE) && (loopingActive == INACTIVE))
			{
				currState = LEFT_SWIPE_BEGIN;
			}
			
			if(down_btnCurrent == ACTIVE)
			{
				currState = CLOCK_SWIPE_BEGIN;
				prevActButton = RIGHT_BUTTON;
			}
			
			if(up_btnCurrent == ACTIVE)
			{
				currState = COUNTER_CLOCK_SWIPE_BEGIN;
				prevActButton = RIGHT_BUTTON;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
			
		/* State machine detected beginning of gesture. User 
		 * started the gesture by touching the DOWN sensor. */
		case DOWN_START:
			if((centre_btnCurrent == ACTIVE) && (right_btnCurrent == INACTIVE) && (left_btnCurrent == INACTIVE) && (loopingActive == INACTIVE))
			{
				currState = UP_SWIPE_BEGIN;
			}
			
			if(left_btnCurrent == ACTIVE)
			{
				currState = CLOCK_SWIPE_BEGIN;
				prevActButton = DOWN_BUTTON;
			}
			
			if(right_btnCurrent == ACTIVE)
			{
				currState = COUNTER_CLOCK_SWIPE_BEGIN;
				prevActButton = DOWN_BUTTON;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
			
		/* State machine decided that the gesture could be a right swipe.
		 * Waiting for user to complete gesture. */
		case RIGHT_SWIPE_BEGIN:
			if(right_btnCurrent == ACTIVE)
			{
				currState = INACTIVE;
				gesture = RIGHT;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
				
		/* State machine decided that the gesture could be a up swipe.
		 * Waiting for user to complete gesture. */	
		case UP_SWIPE_BEGIN:
			if(up_btnCurrent == ACTIVE)
			{
				gesture = UP;
				currState = INACTIVE;
			}
		
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
					
		/* State machine decided that the gesture could be a left swipe.
		 * Waiting for user to complete gesture. */
		case LEFT_SWIPE_BEGIN:
			if(right_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
			{
				gesture = LEFT;
				currState = INACTIVE;
			}

			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
					
		/* State machine decided that the gesture could be a down swipe.
		 * Waiting for user to complete gesture. */	
		case DOWN_SWIPE_BEGIN:
			if(up_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
			{
				gesture = DOWN;
				currState = INACTIVE;
			}

			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
					
		/* State machine decided that the gesture could be a clockwise swipe.
		 * Waiting for user to complete gesture. */	
		case CLOCK_SWIPE_BEGIN:
			
			/* Since the clockwise swipe can begin at any sensor, state machine 
			 * continuously checks the current and previous active sensors to 
			 * determine the completion of a swipe gesture. */
			switch(prevActButton)
			{
				case LEFT_BUTTON:
					if(left_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_MID;
						prevActButton = UP_BUTTON;
					}
					else if(left_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case UP_BUTTON:
					if(up_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_MID;
						prevActButton = RIGHT_BUTTON;
					}
					else if(up_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case RIGHT_BUTTON:
					if(right_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_MID;
						prevActButton = DOWN_BUTTON;
					}
					else if(right_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case DOWN_BUTTON:
					if(down_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_MID;
						prevActButton = LEFT_BUTTON;
					}
					else if(down_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				default:
					currState = INACTIVE;
					break;
			}			
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
					
		/* State machine determined that the gesture is a clockwise swipe.
		 * User has completed half way through the gesture pad in process 
		 * of completing the swipe. Waiting for user to complete gesture. */	
		case CLOCK_SWIPE_MID:
			switch(prevActButton)
			{
				case LEFT_BUTTON:
					if(left_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_LAST;
						prevActButton = UP_BUTTON;
					}
					else if(left_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case UP_BUTTON:
					if(up_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_LAST;
						prevActButton = RIGHT_BUTTON;
					}
					else if(up_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case RIGHT_BUTTON:
					if(right_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_LAST;
						prevActButton = DOWN_BUTTON;
					}
					else if(right_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case DOWN_BUTTON:
					if(down_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_LAST;
						prevActButton = LEFT_BUTTON;
					}
					else if(down_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				default:
					currState = INACTIVE;
					break;
			}			
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
					
		/* State machine determined that the gesture is a right swipe.
		 * User completed 3/4 way through the completion of gesture.
		 * Waiting for user to complete gesture. */
		case CLOCK_SWIPE_LAST:
			switch(prevActButton)
			{
				case LEFT_BUTTON:
					if(left_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_END;
						prevActButton = UP_BUTTON;
					}
					else if(left_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_BEGIN;
						prevActButton = NONE;
					}
					break;
					
				case UP_BUTTON:
					if(up_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_END;
						prevActButton = RIGHT_BUTTON;
					}
					else if(up_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_BEGIN;
						prevActButton = NONE;
					}
					break;
					
				case RIGHT_BUTTON:
					if(right_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_END;
						prevActButton = DOWN_BUTTON;
					}
					else if(right_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_BEGIN;
						prevActButton = NONE;
					}			
					break;
					
				case DOWN_BUTTON:
					if(down_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_END;
						prevActButton = LEFT_BUTTON;
					}
					else if(down_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_BEGIN;
						prevActButton = NONE;
					}
					break;
					
				default:
					currState = INACTIVE;
					break;
			}
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
		
		/* User has completed the gesture. Note that the state machine is reset if 
		 * user removes the finger. Instead, if user continues with the rotation, 
		 * state machine gives a head-start by skipping one state thereby allowing user
		 * to complete the gesture with only 3/4 rotation for subsequent rotations. */
		case CLOCK_SWIPE_END:
			switch(prevActButton)
			{
				case LEFT_BUTTON:
					if(left_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						gesture = CLOCKWISE;
						loopingActive = ACTIVE;
						currState = INACTIVE;
					}
					else if(left_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case UP_BUTTON:
					if(up_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						gesture = CLOCKWISE;
						loopingActive = ACTIVE;
						currState = INACTIVE;
					}
					else if(up_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case RIGHT_BUTTON:
					if(right_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						gesture = CLOCKWISE;
						loopingActive = ACTIVE;
						currState = INACTIVE;
					}
					else if(right_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case DOWN_BUTTON:
					if(down_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						gesture = CLOCKWISE;
						loopingActive = ACTIVE;
						currState = INACTIVE;
					}
					else if(down_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				default:
					currState = INACTIVE;
					break;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
			
		/* State machine determined that the gesture is a counter-clockwise swipe.
		 * Waiting for user to complete gesture. */		
		case COUNTER_CLOCK_SWIPE_BEGIN:
			switch(prevActButton)
			{
				case LEFT_BUTTON:
					if(left_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_MID;
						prevActButton = DOWN_BUTTON;
					}
					else if(left_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case UP_BUTTON:
					if(up_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_MID;
						prevActButton = LEFT_BUTTON;
					}
					else if(up_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case RIGHT_BUTTON:
					if(right_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_MID;
						prevActButton = UP_BUTTON;
					}
					else if(right_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case DOWN_BUTTON:
					if(down_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_MID;
						prevActButton = RIGHT_BUTTON;
					}
					else if(down_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				default:
					currState = INACTIVE;
					break;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
		
		/* State machine determined that the gesture is a counter-clockwise swipe.
		 * User has completed half way through the gesture pad in process 
		 * of completing the swipe. Waiting for user to complete gesture. */
		case COUNTER_CLOCK_SWIPE_MID:
			switch(prevActButton)
			{
				case LEFT_BUTTON:
					if(left_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_LAST;
						prevActButton = DOWN_BUTTON;
					}
					else if(left_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case UP_BUTTON:
					if(up_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_LAST;
						prevActButton = LEFT_BUTTON;
					}
					else if(up_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case RIGHT_BUTTON:
					if(right_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_LAST;
						prevActButton = UP_BUTTON;
					}
					else if(right_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case DOWN_BUTTON:
					if(down_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_LAST;
						prevActButton = RIGHT_BUTTON;
					}
					else if(down_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				default:
					currState = INACTIVE;
					break;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
		
		/* State machine determined that the gesture is a right swipe.
		 * User completed 3/4 way through the completion of gesture.
		 * Waiting for user to complete gesture. */
		case COUNTER_CLOCK_SWIPE_LAST:
			switch(prevActButton)
			{
				case LEFT_BUTTON:
					if(left_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_END;
						prevActButton = DOWN_BUTTON;
					}
					else if(left_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_BEGIN;
						prevActButton = NONE;
					}			
					break;
					
				case UP_BUTTON:
					if(up_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_END;
						prevActButton = LEFT_BUTTON;
					}
					else if(up_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_BEGIN;
						prevActButton = NONE;
					}
					break;
					
				case RIGHT_BUTTON:
					if(right_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_END;
						prevActButton = UP_BUTTON;
					}
					else if(right_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_BEGIN;
						prevActButton = NONE;
					}
					break;
					
				case DOWN_BUTTON:
					if(down_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = COUNTER_CLOCK_SWIPE_END;
						prevActButton = RIGHT_BUTTON;
					}
					else if(down_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = CLOCK_SWIPE_BEGIN;
						prevActButton = NONE;
					}
					break;
					
				default:
					currState = INACTIVE;
					break;
			}
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
		
		/* User has completed the gesture. Note that the state machine is reset if 
		 * user removes the finger. Instead, if user continues with the rotation, 
		 * state machine gives a head-start by skipping one state thereby allowing user
		 * to complete the gesture with only 3/4 rotation for subsequent rotations. */
		case COUNTER_CLOCK_SWIPE_END:
			switch(prevActButton)
			{
				case LEFT_BUTTON:
					if(left_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						gesture = COUNTER_CLOCKWISE;
						loopingActive = ACTIVE;
						currState = INACTIVE;
					}
					else if(left_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case UP_BUTTON:
					if(up_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						gesture = COUNTER_CLOCKWISE;
						loopingActive = ACTIVE;
						currState = INACTIVE;
					}
					else if(up_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case RIGHT_BUTTON:
					if(right_btnCurrent == INACTIVE && up_btnCurrent == ACTIVE)
					{
						gesture = COUNTER_CLOCKWISE;
						loopingActive = ACTIVE;
						currState = INACTIVE;
					}
					else if(right_btnCurrent == INACTIVE && down_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				case DOWN_BUTTON:
					if(down_btnCurrent == INACTIVE && right_btnCurrent == ACTIVE)
					{
						gesture = COUNTER_CLOCKWISE;
						loopingActive = ACTIVE;
						currState = INACTIVE;
					}
					else if(down_btnCurrent == INACTIVE && left_btnCurrent == ACTIVE)
					{
						currState = INACTIVE;
						prevActButton = NONE;
					}
					break;
					
				default:
					currState = INACTIVE;
					break;
			}
			
			if(gesture == SENSORS_INACTIVE)
			{
				currState = INACTIVE;
			}
			break;
		
		/* State machine reset for unknown conditions. */	
		default:
			currState = INACTIVE;
			gesture = NO_GESTURE;
			break;
	}

	return(gesture);
}

/* [] END OF FILE */
