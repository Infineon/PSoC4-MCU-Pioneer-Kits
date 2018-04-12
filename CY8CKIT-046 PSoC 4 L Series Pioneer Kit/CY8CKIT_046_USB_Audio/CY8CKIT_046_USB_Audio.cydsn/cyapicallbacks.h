/*****************************************************************************
* File Name		: cyapicallbacks.h
* Version		: 1.0 
*
* Description:
*  This file contains API callback macros and API mapping of all the callback
*	APIs used across components in the current project.
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
#ifndef CYAPICALLBACKS_H
#define CYAPICALLBACKS_H
    extern void ProcessAudioOut(void);
    extern void ProcessAudioIn(void);
    extern void processAsyncFeedbackTransfer(unsigned long clearFlag);
    extern void UpdateFeedbackCount(void);
	
    /*Define your macro callbacks here */
    /*For more information, refer to the Macro Callbacks topic in the PSoC Creator Help.*/
    #define USBFS_EP_1_ISR_ENTRY_CALLBACK	
	#define USBFS_EP_1_ISR_EntryCallback()	ProcessAudioOut()
	
	#define USBFS_EP_2_ISR_ENTRY_CALLBACK	
	#define USBFS_EP_2_ISR_EntryCallback()	ProcessAudioIn()
	
	#define USBFS_EP_3_ISR_ENTRY_CALLBACK
	#define USBFS_EP_3_ISR_EntryCallback()	processAsyncFeedbackTransfer(1)
	
	#define USBFS_SOF_ISR_ENTRY_CALLBACK	
	#define USBFS_SOF_ISR_EntryCallback()	UpdateFeedbackCount()
	
#endif /* CYAPICALLBACKS_H */   
/* [] */
