/*****************************************************************************
* File Name		: LED_Control.c
* Version		: 1.0 
*
* Description:
*  This file defines the functions to control the color of the RGB LED on the 
*  CY8CKIT-044 PSoC 4 M-Series Pioneer Kit. 
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
#include "project.h"
#include "LED_Control.h"
#include "CapSense_Gestures.h"

extern uint32 uC_Probe_LED;
extern uint8 prevLEDColor;
uint16 brightness = MIN_BRIGHTNESS;

/*******************************************************************************
* Function Name: DisplayLEDOutput
********************************************************************************
* Summary:
*  Changes the color of the onboard RGB LED depending on the gesture 
*  value passed to the function. This function also saves the new LED color 
*  information to the global variable prevLEDColor.
*
* Parameters:
*  gesture: Gesture detected by the CapSense CSD Component.
*
* Return:
*  void
*
*******************************************************************************/
void DisplayLEDOutput(uint8 command)
{
	/* Variable used to save LED color states between function calls. */
	static uint8 ledColor = WHITE;
	
	/* Find the color to be written to the LEDs based on the command received */
	switch (command)
	{
		case OFF:
			ledColor = BLACK;
			break;
		
		case ON:
			ledColor = WHITE;
			break;
		
		/* Rotate the colors in order RED > GREEN > BLUE > RED */
		case UP_SWIPE:
			switch (ledColor)
			{
				case RED:
					ledColor = GREEN;
					break;
				
				case GREEN:
					ledColor = BLUE;
					break;
				
				case BLUE:
					ledColor = RED;
					break;
				
				default:
					ledColor = RED;
					break;
			}
			break;
		
		/* Rotate the colors in order RED > BLUE > GREEN > RED */
		case DOWN_SWIPE:
			switch (ledColor)
			{
				case RED:
					ledColor = BLUE;
					break;
				
				case BLUE:
					ledColor = GREEN;
					break;
				
				case GREEN:
					ledColor = RED;
					break;
				
				default:
					ledColor = RED;
					break;
			}
			break;
		
		/* Case statements to make sure that the function 
		 * can also handle color values as inputs. */
		case RED:
			ledColor = RED;
			break;
			
		case GREEN:
			ledColor = GREEN;
			break;

		case BLUE:
			ledColor = BLUE;
			break;
			
		case BRIGHTNESS_CONTROL:
			break;
			
		default:
			/* Show white color on LED by default. */
			ledColor = WHITE;
	}
	
	/* Save the LED color information. This is not required if the LED
	 * is set to be completely ON or OFF. */
	if((ledColor != WHITE) && (ledColor != BLACK))
	{
		prevLEDColor = ledColor;
	}
	
	/* If the variable ledStatus holds an arbitrary value 0xIIRRGGBB, 
	 * the uC/Probe interprets the value in the following format: 
	 * BB = Green component in the color to be displayed (0 to 255)
	 * GG = Green component in the color to be displayed (0 to 255)
	 * RR = Red component in the color to be displayed (0 to 255)
	 * BB = Opacity of the color to be displayed (0 to 255).*/	
	/* LED set to full brightness. */
	uC_Probe_LED = UC_PROBE_LED_DEFAULT_VAL;

	/* Show the color on the LED by controlling the PWMs */	
	switch(ledColor)
	{
		case OFF:
			PWM_Red_WriteCompare(OFF);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(OFF);
			uC_Probe_LED &= (UC_PROBE_MIN << BRIGHTNESS_SHIFT_POS);
			break;
		
		case ON:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(brightness);
			PWM_Green_WriteCompare(brightness);
			uC_Probe_LED = UC_PROBE_WHITE;
			break;
		
		case RED:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(OFF);
			uC_Probe_LED |= (UC_PROBE_MAX << RED_SHIFT_POS);
			break;
		
		case GREEN:
			PWM_Red_WriteCompare(OFF);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(brightness);
			uC_Probe_LED |= (UC_PROBE_MAX << GREEN_SHIFT_POS);
			break;
		
		case BLUE:
			PWM_Red_WriteCompare(OFF);
			PWM_Blue_WriteCompare(brightness);
			PWM_Green_WriteCompare(OFF);
			uC_Probe_LED |= (UC_PROBE_MAX << BLUE_SHIFT_POS);
			break;
			
		default:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(OFF);
			uC_Probe_LED |= (UC_PROBE_MAX << RED_SHIFT_POS);
			break;			
	}
}

/* [] END OF FILE */
