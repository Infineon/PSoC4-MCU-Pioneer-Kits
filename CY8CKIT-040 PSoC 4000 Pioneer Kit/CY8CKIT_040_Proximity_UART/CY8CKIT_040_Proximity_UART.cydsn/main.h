/******************************************************************************
* Project Name		: CY8CKIT_040_Proximity_UART_Design
* File Name			: main.h
* Version 			: 1.0
* Device Used		: CY8C4014LQI-422
* Software Used		: PSoC Creator 3.1
* Compiler    		: ARM GCC 4.8.4, ARM MDK Generic, ARM RVDS Generic
* Related Hardware	: CY8CKIT-040 PSoC 4000 Pioneer Kit 
*
********************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation. All Rights Reserved.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress)
* and is protected by and subject to worldwide patent protection (United
* States and foreign), United States copyright laws and international treaty
* provisions. Cypress hereby grants to licensee a personal, non-exclusive,
* non-transferable license to copy, use, modify, create derivative works of,
* and compile the Cypress Source Code and derivative works for the sole
* purpose of creating custom software in support of licensee product to be
* used only in conjunction with a Cypress integrated circuit as specified in
* the applicable agreement. Any reproduction, modification, translation,
* compilation, or representation of this software except as specified above 
* is prohibited without the express written permission of Cypress.
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
* such use and in doing so indemnifies Cypress against all charges. 
*
* Use of this Software may be limited by and subject to the applicable Cypress
* software license agreement. 
*******************************************************************************/

/******************************************************************************
*                           THEORY OF OPERATION
* Contains all the macro definitions, global variables & function prototypes
*	used in main.c
******************************************************************************/

#ifndef MAIN_H
	#define MAIN_H
#include <project.h>

/* Comment the below line to disable UART debug */
#define TX_ENABLE	

#ifdef TX_ENABLE
	#define TX_PORT		3
	#define TX_PIN		0
#endif

/* Define the proximity sensor number/index - value taken from CapSense.h */
#define PROXIMITY_SENSOR_INDEX			CapSense_SENSOR_PROXIMITYSENSOR0_0__PROX

/* Defines the Proximity signal value for closest hand distance to turn ON the LED fully */
#define PROXIMITY_UPPER_LIMIT		250

/* Defines the Proximity signal value for closest hand distance to turn ON the LED fully */
#define PROXIMITY_LOWER_LIMIT		50
	
/* A copy of PROXIMITY_SENSOR_INDEX */
#define PROXIMITY_SENSOR 			PROXIMITY_SENSOR_INDEX

/* Macros to check CapSense block status */
#define NOT_BUSY					0
#define BUSY						1

/*Macro to enable WDT interrupt in SRSS INTR mask register */
#define SRSS_WDT_ISR_ENABLE				(*(reg32*)CYREG_SRSS_INTR_MASK) |= 1;
	
/* Number of Watchdog ISRs executed before the device starts entering periodic DeepSleep
	If Watchdog rate is 100 ms & this count is 50, then the device takes (50 x 100) = 5000 ms
	before it enters deepsleep-scan mode every 100 ms */
#define ENTER_SLEEP_COUNTS			50

/* Watchdog ISR rate in ms */
#define WATCHDOG_ISR_RATE			100

/* Watchdog clock/ILO frequency */
#define	WATCHDOG_FREQ				32000

/* WatchDog count for generating a 1 ms interrupt */
#define WATCHDOG_1_ms_COUNT			(WATCHDOG_FREQ/1000)

/* Count value to be written to Watchdog counter for generating the interrupt at desired rate */
#define WATCHDOG_TIMER_COUNT		(WATCHDOG_ISR_RATE * WATCHDOG_1_ms_COUNT)

/* variables from CapSense.c & CapSense_CSHL.c files used in the code */
extern uint16 CapSense_sensorRaw[], CapSense_SensorBaseline[];
extern uint8 CapSense_SensorSignal[];
extern uint16 CapSense_rawFilterData1[];

#endif
/* [] END OF FILE */
