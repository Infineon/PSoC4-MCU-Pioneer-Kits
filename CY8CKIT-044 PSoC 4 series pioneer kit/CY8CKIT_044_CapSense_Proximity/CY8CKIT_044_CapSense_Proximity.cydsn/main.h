/*****************************************************************************
* File Name		: main.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  main.c
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
#if !defined(MAIN_H) 
#define MAIN_H 

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define FALSE						0x00
#define TRUE						0x01

/* WDT match values for different duration of Deep-Sleep power mode.
 * The ILO is operating at 32 kHz. */
#define WDT_MATCH_VALUE_30MS		(32 * 30) 
#define WDT_MATCH_VALUE_100MS		(32 * 100)

/* 4 ILO periods expressed in microseconds. */
#define ILOX4 						((4 * 1000) / 32)

#define INACTIVE					0
#define ACTIVE						1
#define RESET						0
	
#define UPDATE_30MS					0x01
#define UPDATE_100MS				0x02

/* MAX_VALUE defined to achieve 3 second duration. 
 * See the explanation for variable softCounter in main.c. */
#define MAX_VALUE 					100 

#define ZERO						0x00
#define OFF							0x00
#define ON							0x01
	
#define PROX_UPPER_LIMIT			55000
#define PROX_LOWER_LIMIT			CapSense_SensorBaseline[0]

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
void EnterDeepSleepLowPowerMode(void);
void LED_SetBrightness(uint8 value);

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
	
#endif /* MAIN_H */

/* [] END OF FILE */
