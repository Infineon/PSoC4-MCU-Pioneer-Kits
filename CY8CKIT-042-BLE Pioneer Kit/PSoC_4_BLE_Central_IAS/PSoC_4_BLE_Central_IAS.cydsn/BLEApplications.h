/******************************************************************************
* Project Name		: PSoC_4_BLE_Central_IAS
* File Name			: BLEApplications.h
* Version 			: 1.0
* Device Used		: CY8C4247LQI-BL483
* Software Used		: PSoC Creator 4.1
* Compiler    		: ARM GCC 5.4.1, ARM MDK Generic
* Related Hardware	: CY8CKIT-042-BLE Bluetooth Low Energy Pioneer Kit 
* Owner				: ROIT
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
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
*****************************************************************************/

/********************************************************************************
*	Contains all macros and function declaration used in the BLEApplications.c file 
********************************************************************************/
#if !defined (BLEAPPLICATIONS_H)
#define BLEAPPLICATIONS_H
#include <project.h>

/*************************Pre-processor directives****************************/
/* 'ENABLE_LOW_POWER_MODE' pre-processor directive enables the low power mode 
* handling in the firmware, ensuring low power consumption during project usage.
* To disable, comment the following #define. 
* If disabled, prevent usage of the project with coin cell */
#define ENABLE_LOW_POWER_MODE
/****************************************************************************/
	
/* BLE State Macros used for LED status updates*/
#define BLE_DISCONNECTED				0x01
#define BLE_SCANNING					0x02
#define BLE_SERVICE_DISCOVERY			0x03
#define BLE_CONNECTED					0x04

/* Size of IAS Data */
#define IAS_ATTR_SIZE					0x01

/* LED State Macros*/
#define LED_OFF							0x01
#define LED_ON							0x00
	
/* Counter default values used for LED status update during various 
* states BLE */
#ifdef ENABLE_LOW_POWER_MODE
#define LED_SCANNING_COUNTER_VALUE		60
#define LED_DISCOVERY_COUNTER_VALUE		5
#else
#define LED_SCANNING_COUNTER_VALUE		10000
#define LED_DISCOVERY_COUNTER_VALUE		1000	
#endif

/* User button related Macros for Debouncing and Detection*/
#define USER_BUTTON_PUSHED_MASK			0x01
#define	USER_BUTTON_DETECTED_MASK		0x02
#define SWITCH_DEBOUNCE_DELAY			4

/**************************Function Declarations*****************************/
void CheckButtonStatus(void);
void HandleLEDs(uint8 state);
void SetAlertLevel(uint8* alertLevel);
void HandleLowPowerMode(void);
void MyISR(void);
/****************************************************************************/

#endif
/* [] END OF FILE */
