/*****************************************************************************
* File Name		: CapSense_Gestures.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  CapSense_Gestures.c. 
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
#if !defined(CAPSENSE_GESTURES_H) 
#define CAPSENSE_GESTURES_H 

#include "cytypes.h"
#include "CapSense.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define SENSORS_INACTIVE			0x00
#define LEFT						0x01
#define UP							0x02
#define RIGHT						0x03
#define DOWN						0x04
#define CLOCKWISE					0x05
#define COUNTER_CLOCKWISE			0x06
#define NO_GESTURE					0x07

/* State Machine gestures. */
/* #defines are not given values with repeating digits like 
 * 0x11, 0x22, etc. to avoid any conflicts with the #defines 
 * values given for colors in LED_Control.h. */
#define LEFT_START					0x01
#define UP_START					0x02
#define RIGHT_START					0x03
#define DOWN_START					0x04
#define LEFT_SWIPE_BEGIN			0x05
#define DOWN_SWIPE_BEGIN			0x06
#define RIGHT_SWIPE_BEGIN			0x07
#define UP_SWIPE_BEGIN				0x08
#define CLOCK_SWIPE_BEGIN			0x0D
#define CLOCK_SWIPE_MID				0x0E
#define CLOCK_SWIPE_LAST			0x0F
#define CLOCK_SWIPE_END				0x10
#define COUNTER_CLOCK_SWIPE_BEGIN	0x12
#define COUNTER_CLOCK_SWIPE_MID		0x13
#define COUNTER_CLOCK_SWIPE_LAST	0x14
#define COUNTER_CLOCK_SWIPE_END		0x15

#define NONE						0x00
#define LEFT_BUTTON					0x01
#define UP_BUTTON					0x02
#define RIGHT_BUTTON				0x03
#define DOWN_BUTTON					0x04
#define CENTRE_BUTTON				0x05

#define RESET						0x0000
#define INACTIVE					0x00
#define ACTIVE						0x01

#define WITH_CENTRE					0x00	
#define IGNORE_CENTRE				0x01
	
#define ZERO						0x00

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
uint8 DetectGesture(void);

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
	
	
#endif /* CAPSENSE_GESTURES_H */

/* [] END OF FILE */
