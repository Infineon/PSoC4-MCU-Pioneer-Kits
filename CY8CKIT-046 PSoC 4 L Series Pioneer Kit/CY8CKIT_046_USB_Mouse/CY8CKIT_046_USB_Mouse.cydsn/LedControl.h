/*****************************************************************************
* File Name		: LeControl.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  LedControl.c
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

#ifndef LED_CONTROL_H		/* Guard to prevent multiple inclusions */
#define LED_CONTROL_H 

#include "cytypes.h"
#include "Gesture.h"


/*******************************************************************************
* 	Data Type Definitions
********************************************************************************/
	
typedef enum
{
	COLOR_OFF, 
	COLOR_VIOLET, 
	COLOR_INDIGO,
	COLOR_BLUE,
	COLOR_GREEN,
	COLOR_YELLOW,
	COLOR_ORANGE,
	COLOR_RED,
	COLOR_WHITE
}tLedColor;

/* Exponential values for brightness level control. */
typedef enum
{
	BRIGHT_LEVEL_0 = 500, 
	BRIGHT_LEVEL_1 = 1000,
	BRIGHT_LEVEL_2 = 2000,
	BRIGHT_LEVEL_3 = 4000,
	BRIGHT_LEVEL_4 = 8000,
	BRIGHT_LEVEL_5 = 12000,
	BRIGHT_LEVEL_6 = 16000,
	BRIGHT_LEVEL_7 = 20000,
	BRIGHT_LEVEL_8 = 24000,
	BRIGHT_LEVEL_9 = 28000
}tBrightLevel;


/*******************************************************************************
* 	Macro Definitions
********************************************************************************/

/* Defines the initial color of the RGB LED at power-up. */
#define INIT_LED_COLOR					COLOR_OFF

/* Defines the initial brightness level of the RGB LED at power-up. */
#define INIT_BRIGHT_LEVEL				BRIGHT_LEVEL_0

/* Defines the minimum delta or shift in the slider centroid (accumulated over
 * multiple scans) required to change the brightness level. 
 */
#define BRIGHT_CHANGE_ACCU_SLIDER_DELTA		(20)

/* Defines the PWM period value that turns off the LED */
#define LED_OFF_PWM_PERIOD				(0)


/*******************************************************************************
* 	Function Prototypes
*******************************************************************************/

void UpdateLedColor(tGestureId gestureId);
void SetLedColor(tLedColor ledColor, tBrightLevel brightness);

	
#endif /* #ifndef LED_CONTROL_H */


/* [] END OF FILE */
