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

/* This variable is used to store LED brightness.*/
uint16 brightness = BRIGHT_LEVEL_7;
uint8 prevLEDColor = WHITE;

/*******************************************************************************
* Function Name: DisplayLEDOutput
********************************************************************************
* Summary:
*  Changes the color of the onboard RGB LED depending on the gesture 
*  value passed to the function. This function also saves the new LED color 
*  information in the global variable prevLEDColor.
*
* Parameters:
*  gesture: Gesture detected by the CapSense CSD Component.
*
* Return:
*  void
*
*******************************************************************************/
void DisplayLEDOutput(uint8 gesture)
{
	/* Variable used to save LED color states between function calls. */
	static uint8 ledColor = WHITE;
	
	/* Check the gesture and control the output accordingly */	
	switch (gesture)
	{
		case ON:
			ledColor = WHITE;
			break;
		
		case OFF:
			ledColor = BLACK;
			break;
		
		/* Coarse color change. RED > GREEN > BLUE > RED. */
		case LEFT:
			switch(ledColor)
			{
				case VIOLET:
					ledColor = BLUE;
					break;
				
				case INDIGO:
					ledColor = BLUE;
					break;
					
				case BLUE:
					ledColor = GREEN;
					break;
				
				case GREEN:
					ledColor = RED;
					break;
				
				case YELLOW:
					ledColor = RED;
					break;
				
				case ORANGE:
					ledColor = RED;
					break;
				
				case RED:
					ledColor = BLUE;
					break;

				default:
					ledColor = RED;
					break;
			}
			break;
			
		/* Coarse color change. RED > BLUE > GREEN > RED. */
		case RIGHT:
			switch(ledColor)
			{
				case VIOLET:
					ledColor = RED;
					break;
				
				case INDIGO:
					ledColor = RED;
					break;
					
				case BLUE:
					ledColor = RED;
					break;
				
				case GREEN:
					ledColor = BLUE;
					break;
				
				case YELLOW:
					ledColor = GREEN;
					break;
				
				case ORANGE:
					ledColor = GREEN;
					break;
				
				case RED:
					ledColor = GREEN;
					break;

				default:
					ledColor = RED;
					break;
			}
			break;
		
		/* Brightness Increase.  */
		case UP:
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
					brightness = BRIGHT_LEVEL_10;
					break;
				
				case BRIGHT_LEVEL_10:
				
					/* Saturate maximum brightness at 100%. */
					brightness = BRIGHT_LEVEL_10;
					break;
				
				default:
					/* Start with a default brightness of 70%. */
					brightness = BRIGHT_LEVEL_7;
					break;		
			}
			break;
		
		/* Brightness Decrease.  */
		case DOWN:
			switch(brightness)
			{
				case BRIGHT_LEVEL_0:
				
					/* Saturate minimum brightness at 0%. */
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
				
				case BRIGHT_LEVEL_10:
					brightness = BRIGHT_LEVEL_9;
					break;
				
				default:
					/* Start with a default brightness of 70%. */
					brightness = BRIGHT_LEVEL_7;
					break;
			}
			break;
		
		/* Fine color change in order VIBGYOR. */
		case CLOCKWISE:
			switch (ledColor)
			{
				case VIOLET:
					ledColor = INDIGO;
					break;
				
				case INDIGO:
					ledColor = BLUE;
					break;
					
				case BLUE:
					ledColor = GREEN;
					break;
				
				case GREEN:
					ledColor = YELLOW;
					break;
				
				case YELLOW:
					ledColor = ORANGE;
					break;
				
				case ORANGE:
					ledColor = RED;
					break;
				
				case RED:
					ledColor = VIOLET;
					break;

				default:
					ledColor = VIOLET;
					break;
			}
			break;
		
		/* Fine color change in reverse order of VIBGYOR. */
		case COUNTER_CLOCKWISE:
			switch (ledColor)
			{
				case VIOLET:
					ledColor = RED;
					break;
				
				case INDIGO:
					ledColor = VIOLET;
					break;
					
				case BLUE:
					ledColor = INDIGO;
					break;
				
				case GREEN:
					ledColor = BLUE;
					break;
				
				case YELLOW:
					ledColor = GREEN;
					break;
				
				case ORANGE:
					ledColor = YELLOW;
					break;
				
				case RED:
					ledColor = ORANGE;
					break;

				default:
					ledColor = RED;
					break;
			}
			break;
			
		/* Code to make sure that the function can accept 
		 * individual colors also as parameters. */
		case VIOLET:
			ledColor = VIOLET;
			break;
		
		case INDIGO:
			ledColor = INDIGO;
			break;
		
		case BLUE:
			ledColor = BLUE;
			break;
		
		case GREEN:
			ledColor = GREEN;
			break;
		
		case YELLOW:
			ledColor = YELLOW;
			break;
		
		case ORANGE:
			ledColor = ORANGE;
			break;
		
		case RED:
			ledColor = RED;
			break;
			
		default:
			ledColor = WHITE;
	}
	
	/* Show the color on the RGB LED. */
	LED_Show(ledColor, brightness);
	
	/* Save the LED color information. This is not required if the LED
	 * is set to be OFF. */
	if(ledColor != BLACK)
	{
		prevLEDColor = ledColor;
	}
}

/******************************************************************************
* Function Name: LED_Show
*******************************************************************************
* Summary:
*  Shows a specific color on the onboard RGB LED at a specified
*  brightness level depending on the parameters.
*
* Parameters:
*  ledColor: LED color to be shown on the RGB LED.
*  brightness: Brightness level of the LED.
*
* Return:
*  void
*
*******************************************************************************/
void LED_Show(uint8 ledColor, uint16 brightness)
{
	/* Show the color on the LED by controlling the PWMs. 
	 * Note that the colors are only approximate. */
	switch(ledColor)
	{
		case BLACK:
			PWM_Red_WriteCompare(OFF);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(OFF);
			break;
		
		case WHITE:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(brightness);
			PWM_Green_WriteCompare(brightness);
			break;
		
		case VIOLET:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(brightness);
			PWM_Green_WriteCompare(OFF);
			break;
		
		case INDIGO:
			PWM_Red_WriteCompare(brightness/4);
			PWM_Blue_WriteCompare(brightness);
			PWM_Green_WriteCompare(OFF);
			break;
			
		case BLUE:
			PWM_Red_WriteCompare(OFF);
			PWM_Blue_WriteCompare(brightness);
			PWM_Green_WriteCompare(OFF);
			break;
		
		case GREEN:
			PWM_Red_WriteCompare(OFF);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(brightness);
			break;
		
		case YELLOW:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(brightness);
			break;
		
		case ORANGE:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(brightness/2);	
			break;
			
		case RED:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(OFF);
			PWM_Green_WriteCompare(OFF);
			break;
			
		default:
			PWM_Red_WriteCompare(brightness);
			PWM_Blue_WriteCompare(brightness);
			PWM_Green_WriteCompare(brightness);
			break;
	}
}

/* [] END OF FILE */
