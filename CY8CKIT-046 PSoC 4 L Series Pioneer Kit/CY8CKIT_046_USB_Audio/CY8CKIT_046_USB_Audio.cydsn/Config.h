/*****************************************************************************
* File Name		: Config.h
* Version		: 1.0 
*
* Description:
*  This file contains macros relevant to configuration of the entire project
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
#ifndef CONFIG_H
	#define CONFIG_H

	#define TXDEBUG	
	#define CAPSENSE_ENABLED	
		
	/* Constants for explicit and implicit async audio */	
	#define EXPLICIT_FEEDBACK                      1
	#define IMPLICIT_FEEDBACK                      0 
	
	/* Selection of OS for Volume control module */
	#define WINDOWS_7_VOLUME_CTL					0
	#define WINDOWS_10_VOLUME_CTL					1
	#define MAC_VOLUME_CTL							2
	#define VOLUME_CTRL								WINDOWS_7_VOLUME_CTL
	
#endif /* #ifndef CONFIG_H */	

/* [] END OF FILE */
