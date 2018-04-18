/*****************************************************************************
* File Name		: LED_Control.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  LED_Control.c
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
#if !defined(LED_CONTROL_H) 
#define LED_CONTROL_H 

#include "cytypes.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define OFF							0x00
#define ON							0xFF

#define BLACK						0x00
#define VIOLET						0x11
#define INDIGO						0x22
#define BLUE						0x33
#define GREEN						0x44
#define YELLOW						0x55
#define ORANGE						0x66
#define RED							0x77
#define WHITE						0xFF

/* Exponential values for brightness level control. */
#define BRIGHT_LEVEL_0				500
#define BRIGHT_LEVEL_1				1000
#define BRIGHT_LEVEL_2				2000
#define BRIGHT_LEVEL_3				4000
#define BRIGHT_LEVEL_4				8000
#define BRIGHT_LEVEL_5				12000
#define BRIGHT_LEVEL_6				16000
#define BRIGHT_LEVEL_7				20000
#define BRIGHT_LEVEL_8				24000
#define BRIGHT_LEVEL_9				28000
#define BRIGHT_LEVEL_10				32000

#define LED_INFO_SIZE				0x03
#define COLOR_LOC					0x00
#define BRIGHTNESS_MSB				0x01
#define BRIGHTNESS_LSB				0x02
#define MSB_SHIFT					0x08
#define MSB_MASK					0xFF00

/*****************************************************************************
* Data Type Definition
*****************************************************************************/


/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/


/*****************************************************************************
* Data Structure Definition
*****************************************************************************/


/*****************************************************************************
* Global Variable Declaration
*****************************************************************************/


/*****************************************************************************
* Function Prototypes
*****************************************************************************/
void DisplayLEDOutput(uint8 gesture);
void ShowPrevLEDColor(void);
void LED_Show(uint8 ledColor, uint16 brightness);
/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
	
	
#endif /* LED_CONTROL_H */

/* [] END OF FILE */
