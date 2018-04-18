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
#define RED							0x11
#define GREEN						0x22
#define BLUE						0x33
#define WHITE						0xFF
	
#define UC_PROBE_LED_DEFAULT_VAL	0xFF000000
#define UC_PROBE_WHITE				0xFFFFFFFF
#define BRIGHTNESS_SHIFT_POS		24
#define RED_SHIFT_POS				16
#define GREEN_SHIFT_POS				8
#define BLUE_SHIFT_POS				0
	
#define UC_PROBE_MAX				0xFF
#define UC_PROBE_MIN				0x00
	
#define MIN_BRIGHTNESS				500
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
void ShowPrevLEDColor(void);
void DisplayLEDOutput(uint8 gesture);

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
	
	
#endif /* LED_CONTROL_H */

/* [] END OF FILE */
