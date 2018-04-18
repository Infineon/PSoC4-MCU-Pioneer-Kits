/*****************************************************************************
* File Name		: CapSense_Gestures.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  CapSense_Gestures.c
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

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

/*Macro definition for sensor states. */
#define UP_ACTIVE					0x01
#define UP_END						0x02
#define DOWN_ACTIVE					0x03
#define DOWN_END					0x04
#define ALL_ACTIVE					0x05
#define BRIGHTNESS_CONTROL			0x06

#define RESET						0x00
#define INACTIVE					0x00
#define ACTIVE						0x01

/* Macro definition for gestures. */
#define SENSORS_INACTIVE			0x00
#define UP_SWIPE					0x01
#define DOWN_SWIPE					0x02
#define NO_GESTURE					0x04
	
#define MAX_BRIGHT					65535
	
#define PROX_DETECT_LIMIT			0x01
	
#define PROX_UPPER_LIMIT			60000
#define PROX_LOWER_LIMIT			CapSense_SensorBaseline[CapSense_PROXIMITYSENSORCOMBINED__PROX]
#define SENSOR_SIGNAL				CapSense_SensorRaw[CapSense_PROXIMITYSENSORCOMBINED__PROX]

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
