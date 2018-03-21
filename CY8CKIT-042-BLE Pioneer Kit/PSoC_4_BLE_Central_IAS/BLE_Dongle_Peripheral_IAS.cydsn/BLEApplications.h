/******************************************************************************
* Project Name		: BLE_Dongle_Peripheral_IAS
* File Name			: BLEApplications.h
* Version 			: 1.0
* Device Used		: CYBL10162-56LQXI
* Software Used		: PSoC Creator 4.1
* Compiler    		: ARM GCC 5.4.1, ARM MDK Generic
* Related Hardware	: CySmart USB Dongle (part of CY8CKIT-042-BLE Bluetooth Low 
*                     Energy Pioneer Kit) 
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

#if !defined(BLEAPPLICATIONS_H)
#define BLEAPPLICATIONS_H
#include <project.h>

/***************************************
*        Function Prototypes
***************************************/
void GenericAppEventHandler(uint32 event, void * eventParam);
void iasEventHandler(uint32 event, void * eventParam);
#endif

/* [] END OF FILE */
