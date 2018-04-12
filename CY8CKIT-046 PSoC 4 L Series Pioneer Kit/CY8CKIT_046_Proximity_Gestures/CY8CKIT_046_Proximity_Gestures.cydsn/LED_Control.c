/*****************************************************************************
* File Name		: LED_Control.c
* Version		: 1.0 
*
* Description:
*  This file defines the functions to control the color of the RGB LED on the 
*  CY8CKIT-046 PSoC 4 L-Series Pioneer Kit. 
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
* Cypress reserves the right to make changes to the Software without notice. 
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
#include "LED_Control.h"

extern uint8 prevLEDColor;
uint16 brightness = 0;

/*******************************************************************************
* Function Name: DisplayLEDOutput
********************************************************************************
* Summary:
*  Changes the color of the onboard RGB LED depending on the gesture 
*  value passed to the function. This function also saves the new LED color 
*  information to the global variable prevLEDColor.
*
* Parameters:
*  command: Command to be used to update the LED colors.
*
* Return:
*  void
*
*******************************************************************************/
void DisplayLEDOutput(DISPLAY_COMMAND command)
{
	/* Variable used to save LED color states between function calls. */
	static LED_COLOR ledColor = LED_COLOR_RED;
	
	/* Find the color to be written to the LEDs based on the command received */
	switch (command)
	{
	case DISPLAY_LED_OFF:
		brightness = 0;
		break;
	
	case DISPLAY_LED_WHITE:
		ledColor = LED_COLOR_WHITE;
		break;
		
	case DISPLAY_UP_SWIPE:
        /* Rotate the colors in order RED > GREEN > BLUE > RED */
		switch (ledColor)
		{
		case LED_COLOR_RED:
			ledColor = LED_COLOR_GREEN;
			break;
		
		case LED_COLOR_GREEN:
			ledColor = LED_COLOR_BLUE;
			break;
		
		case LED_COLOR_BLUE:
			ledColor = LED_COLOR_RED;
			break;
		
		default:
			ledColor = LED_COLOR_RED;
			break;
		}
		break;
		
	case DISPLAY_DOWN_SWIPE:
		/* Rotate the colors in order RED > BLUE > GREEN > RED */
        switch (ledColor)
		{
		case LED_COLOR_RED:
			ledColor = LED_COLOR_BLUE;
			break;
		
		case LED_COLOR_BLUE:
			ledColor = LED_COLOR_GREEN;
			break;
		
		case LED_COLOR_GREEN:
			ledColor = LED_COLOR_RED;
			break;
		
		default:
			ledColor = LED_COLOR_RED;
			break;
		}
		break;
		
	/* Case statements to make sure that the function 
	 * can also handle color values as inputs. */
	case DISPLAY_LED_RED:
		ledColor = LED_COLOR_RED;
		break;
		
	case DISPLAY_LED_GREEN:
		ledColor = LED_COLOR_GREEN;
		break;

	case DISPLAY_LED_BLUE:
		ledColor = LED_COLOR_BLUE;
		break;
        
    case DISPLAY_LED_PREV_COLOR:
        break;    
			
	default:
		/* Show white color on LED by default. */
		ledColor = LED_COLOR_WHITE;
	}

	/* Show the color on the LED by controlling the PWMs */	
	switch(ledColor)
	{
	case LED_COLOR_WHITE:
		PWM_Red_WriteCompare(brightness);
		PWM_Blue_WriteCompare(brightness);
		PWM_Green_WriteCompare(brightness);
		break;
	
	case LED_COLOR_RED:
		PWM_Red_WriteCompare(brightness);
		PWM_Blue_WriteCompare(OFF);
		PWM_Green_WriteCompare(OFF);
		break;
	
	case LED_COLOR_GREEN:
		PWM_Red_WriteCompare(OFF);
		PWM_Blue_WriteCompare(OFF);
		PWM_Green_WriteCompare(brightness);
		break;
	
	case LED_COLOR_BLUE:
		PWM_Red_WriteCompare(OFF);
		PWM_Blue_WriteCompare(brightness);
		PWM_Green_WriteCompare(OFF);
		break;
		
	default:
		PWM_Red_WriteCompare(brightness);
		PWM_Blue_WriteCompare(OFF);
		PWM_Green_WriteCompare(OFF);
		break;			
	}
}

/* [] END OF FILE */
