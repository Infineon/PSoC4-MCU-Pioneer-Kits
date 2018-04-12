/*******************************************************************************
* File Name: LedControl.c
*
* Version 1.0
*
* Description: This file contains routines for RGB LED control.
*
*******************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the Right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*******************************************************************************/

#include "project.h"
#include "LedControl.h"
#include "main.h"


/*******************************************************************************
*	Static Variable Declarations
********************************************************************************/

static bool isRgbLedOn = false;
static tLedColor ledColor = INIT_LED_COLOR;
static tBrightLevel brightness = INIT_BRIGHT_LEVEL;


/*******************************************************************************
*   External Variable Declarations
*******************************************************************************/


/* Indicates the shift or delta in the slider centroid, accumulated over
 * multiple scans. 
 */
extern uint32 accSliderDelta;


/*******************************************************************************
* Function Name: UpdateLedColor
********************************************************************************
* Summary:
*  Changes the color of the onboard RGB LED depending on the gesture 
*  value passed to the function. This function also saves the new LED color 
*  information in the global variable prevLEDColor.
*
* Parameters:
*  gestureId - Id of the gesture detected.
*
* Return:
*  None
*
*******************************************************************************/
void UpdateLedColor(tGestureId gestureId)
{
	if(capsenseButtonStatus == CENTRE_BTN_MASK)
	{
		/* Centre button is used to turn on/off the RGB LED */
		if(isRgbLedOn)
		{
			SetLedColor(COLOR_OFF, brightness); 
		}
		else
		{
			SetLedColor(ledColor, brightness);
		}
	}
	else if(isRgbLedOn)
	{
		/* Check the gesture and control the output accordingly */	
		switch (gestureId)
		{		
		/* Change color in order of VIBGYOR. */
		case GESTURE_INNER_CLKWISE:
			switch (ledColor)
			{
			case COLOR_VIOLET:
				ledColor = COLOR_INDIGO;
				break;
			
			case COLOR_INDIGO:
				ledColor = COLOR_BLUE;
				break;
				
			case COLOR_BLUE:
				ledColor = COLOR_GREEN;
				break;
			
			case COLOR_GREEN:
				ledColor = COLOR_YELLOW;
				break;
			
			case COLOR_YELLOW:
				ledColor = COLOR_ORANGE;
				break;
			
			case COLOR_ORANGE:
				ledColor = COLOR_RED;
				break;
			
			case COLOR_RED:
				ledColor = COLOR_VIOLET;
				break;

			default:
				ledColor = COLOR_VIOLET;
				break;
			}		
			break;
			
			/* Fine color change in reverse order of VIBGYOR. */
		case GESTURE_INNER_COUNTER_CLKWISE:
			switch (ledColor)
			{
			case COLOR_VIOLET:
				ledColor = COLOR_RED;
				break;
			
			case COLOR_INDIGO:
				ledColor = COLOR_VIOLET;
				break;
				
			case COLOR_BLUE:
				ledColor = COLOR_INDIGO;
				break;
			
			case COLOR_GREEN:
				ledColor = COLOR_BLUE;
				break;
			
			case COLOR_YELLOW:
				ledColor = COLOR_GREEN;
				break;
			
			case COLOR_ORANGE:
				ledColor = COLOR_YELLOW;
				break;
			
			case COLOR_RED:
				ledColor = COLOR_ORANGE;
				break;

			default:
				ledColor = COLOR_RED;
				break;
			}	
				break;
		
		case GESTURE_OUTER_CLKWISE:													
			switch(brightness)
			{
				case BRIGHT_LEVEL_0:
					brightness = BRIGHT_LEVEL_1;
					break;
				
				case BRIGHT_LEVEL_1:
					brightness = BRIGHT_LEVEL_2;
					break;
				
				case BRIGHT_LEVEL_2:
					brightness = BRIGHT_LEVEL_3;
					break;
				
				case BRIGHT_LEVEL_3:
					brightness = BRIGHT_LEVEL_4;
					break;
				
				case BRIGHT_LEVEL_4:
					brightness = BRIGHT_LEVEL_5;
					break;
				
				case BRIGHT_LEVEL_5:
					brightness = BRIGHT_LEVEL_6;
					break;
				
				case BRIGHT_LEVEL_6:
					brightness = BRIGHT_LEVEL_7;
					break;
				
				case BRIGHT_LEVEL_7:
					brightness = BRIGHT_LEVEL_8;
					break;
				
				case BRIGHT_LEVEL_8:
					brightness = BRIGHT_LEVEL_9;
					break;
				
				case BRIGHT_LEVEL_9:
				/* Saturate at the maximum brightness. */
					brightness = BRIGHT_LEVEL_9;
					break;
				
				default:
					/* Start with a default brightness of 70%. */
					brightness = BRIGHT_LEVEL_7;
					break;		
			}			
			break;
			
		case GESTURE_OUTER_COUNTER_CLKWISE:										
			switch(brightness)
			{
				case BRIGHT_LEVEL_0:
				
					/* Saturate at the minimum brightness. */
					brightness = BRIGHT_LEVEL_0;
					break;
				
				case BRIGHT_LEVEL_1:
					brightness = BRIGHT_LEVEL_0;
					break;
				
				case BRIGHT_LEVEL_2:
					brightness = BRIGHT_LEVEL_1;
					break;
				
				case BRIGHT_LEVEL_3:
					brightness = BRIGHT_LEVEL_2;
					break;
				
				case BRIGHT_LEVEL_4:
					brightness = BRIGHT_LEVEL_3;
					break;
				
				case BRIGHT_LEVEL_5:
					brightness = BRIGHT_LEVEL_4;
					break;
				
				case BRIGHT_LEVEL_6:
					brightness = BRIGHT_LEVEL_5;
					break;
				
				case BRIGHT_LEVEL_7:
					brightness = BRIGHT_LEVEL_6;
					break;
				
				case BRIGHT_LEVEL_8:
					brightness = BRIGHT_LEVEL_7;
					break;
				
				case BRIGHT_LEVEL_9:
					brightness = BRIGHT_LEVEL_8;
					break;
				
				default:
					/* Start with a default brightness of 70%. */
					brightness = BRIGHT_LEVEL_7;
					break;
				}
				
				break;
					
			default:
				break;
		}
		
		/* Show the color on the RGB LED. */
		SetLedColor(ledColor, brightness);		
	}	
}


/******************************************************************************
* Function Name: SetLedColor
*******************************************************************************
* Summary:
*  Shows a specific color on the onboard RGB LED at a specified
*  brightness level depending on the parameters.
*
* Parameters:
*  color - LED color to be shown on the RGB LED.
*  tempBrightness - Brightness level of the LED.
*
* Return:
*  None
*
*******************************************************************************/
void SetLedColor(tLedColor color, tBrightLevel tempBrightness)
{
	/* Show the color on the LED by controlling the PWMs. 
	 * Note that the colors are only approximate. */
	
	if(color == COLOR_OFF)
	{
		isRgbLedOn = false; 
	}
	else 
	{
		isRgbLedOn = true;
		/* Copy the color and brightness values to the static variables */
		ledColor = color;
		brightness = tempBrightness;
	}
	
	switch(color)
	{
	case COLOR_OFF:
		PWM_Red_WriteCompare(LED_OFF_PWM_PERIOD);
		PWM_Blue_WriteCompare(LED_OFF_PWM_PERIOD);
		PWM_Green_WriteCompare(LED_OFF_PWM_PERIOD);
		break;
	
	case COLOR_WHITE:
		PWM_Red_WriteCompare(brightness);
		PWM_Blue_WriteCompare(brightness);
		PWM_Green_WriteCompare(brightness);
		break;
	
	case COLOR_VIOLET:
		PWM_Red_WriteCompare(brightness);
		PWM_Blue_WriteCompare(brightness);
		PWM_Green_WriteCompare(LED_OFF_PWM_PERIOD);
		break;
	
	case COLOR_INDIGO:
		PWM_Red_WriteCompare(brightness/4);
		PWM_Blue_WriteCompare(brightness);
		PWM_Green_WriteCompare(LED_OFF_PWM_PERIOD);
		break;
		
	case COLOR_BLUE:
		PWM_Red_WriteCompare(LED_OFF_PWM_PERIOD);
		PWM_Blue_WriteCompare(brightness);
		PWM_Green_WriteCompare(LED_OFF_PWM_PERIOD);
		break;
	
	case COLOR_GREEN:
		PWM_Red_WriteCompare(LED_OFF_PWM_PERIOD);
		PWM_Blue_WriteCompare(LED_OFF_PWM_PERIOD);
		PWM_Green_WriteCompare(brightness);
		break;
	
	case COLOR_YELLOW:
		PWM_Red_WriteCompare(brightness);
		PWM_Blue_WriteCompare(LED_OFF_PWM_PERIOD);
		PWM_Green_WriteCompare(brightness);
		break;
	
	case COLOR_ORANGE:
		PWM_Red_WriteCompare(brightness);
		PWM_Blue_WriteCompare(LED_OFF_PWM_PERIOD);
		PWM_Green_WriteCompare(brightness/2);	
		break;
		
	case COLOR_RED:
		PWM_Red_WriteCompare(brightness);
		PWM_Blue_WriteCompare(LED_OFF_PWM_PERIOD);
		PWM_Green_WriteCompare(LED_OFF_PWM_PERIOD);
		break;
		
	default:
		break;
	}
}


/* [] END OF FILE */

