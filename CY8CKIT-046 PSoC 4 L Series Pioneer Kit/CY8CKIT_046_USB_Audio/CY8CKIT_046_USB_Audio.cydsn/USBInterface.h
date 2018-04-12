/*******************************************************************************
* File Name: USBInterface.h
*
* Version 1.0
*
*  Description: This file contains the function prototypes and constants used in
*  				USBInterface.c.
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

#if !defined(USBINTERFACE_H)
	#define USBINTERFACE_H

	#include <Config.h>
	#include <project.h>

	#define MINIMUM_FEEDBACK_REQUEST                0x08 
	#define MAX_UINT8_COUNT                         256	
	#define EXPLICIT_FEEDBACK_EP_SCAN_INTERVAL      8

	#define CLEAR_UPDATE_FLAG                       1
	#define DO_NOT_CLEAR_UPDATE_FLAG                0
		
	#define NO_OF_AUDIO_STREAM_INTERFACE            2
	#define AUDIO_OUT_INTERFACE_INDEX               0
	#define AUDIO_IN_INTERFACE_INDEX                1
	#define ALT_SETTING_ZERO_BANDWIDTH              0
	#define ALT_SETTING_ACTIVE_24_BIT               2               
	#define ALT_SETTING_ACTIVE_16_BIT               1
	#define ALT_SETTING_INVALID						0xFF

	#define IsUSBConfigured()                        USBFS_GetConfiguration()
	
	#define USB_INTERFACE_INACTIVE                   0x00
	#define USB_INIT_AFTER_ENUMERATION_REQUIRED      0x01
	#define USB_INIT_AFTER_ENUMERATION_COMPLETED     0x02

	void ServiceUSB(void);
	void UpdateFeedbackCount(void);
	void processAsyncFeedbackTransfer(uint32 clearFlag);

	extern uint8 USBDeviceState;
	extern uint8 altSetting[NO_OF_AUDIO_STREAM_INTERFACE];

#endif

/* [] END OF FILE */
