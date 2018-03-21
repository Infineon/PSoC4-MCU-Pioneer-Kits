/******************************************************************************
* Project Name		: PSoC_4_BLE_Central_IAS
* File Name			: BLEclient.h
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
*	Contains all macros and function declaration used in the BLEclient.c file 
********************************************************************************/
#if !defined (BLECLIENT_H)
#define BLECLIENT_H
#include <project.h>

/* Maximum number of devices this project will keep track of */
#define CYBLE_MAX_ADV_DEVICES        10u
	
/* BD Address Length*/
#define ADV_ADDR_LEN				0x06
	
/* Public Address indication used for Scanning */
#define PUBLIC_ADDR_TYPE			0x00
	
/* Random Address indication used for Scanning */
#define RANDOM_ADDR_TYPE			0x01

/**************************Function Declarations*****************************/
void HandleScanDevices(CYBLE_GAPC_ADV_REPORT_T* scanReport);
void LoadPeripheralDeviceData(void);
/****************************************************************************/

#endif
/* [] END OF FILE */
