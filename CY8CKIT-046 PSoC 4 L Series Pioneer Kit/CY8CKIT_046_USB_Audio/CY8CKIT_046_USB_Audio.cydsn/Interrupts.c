/*******************************************************************************
* File Name: Interrupts.c
*
* Version 1.0
*
*  Description: This file contains interrupt service routines for all the interrupts
*               in the system
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
*******************************************************************************/
#include <AudioControl.h>
#include <AudioOut.h>
#include <Config.h>
#include <project.h>
#include <Application.h>
#include <Interrupts.h>

extern uint16 inCnt;
extern uint16 inLevel;
extern uint16 inUsbCount;
extern uint16 inUsbShadow;
extern uint16 outLevel;
extern uint16 outUsbCount;
extern uint16 outUsbShadow;

/*******************************************************************************
* Function Name: InDMADone_Interrupt
********************************************************************************
* Summary:
*   The Interrupt Service Routine for isr_InDMADone. Arms USB Audio In end-point
* 	for USB host to receive audio data.
*
*
* Parameters:  
*	InDMADone interrupt vector
*
* Return:
*   void.
*
*******************************************************************************/
CY_ISR(InDMADone_Interrupt)
{
    /* Parameters to this API Call CHANGE DEPENDING ON DATA ENDPOINT MEMORY MODE */
    USBFS_LoadInEP(AUDIO_IN_ENDPOINT, USBFS_NULL, inCnt);
}



/*******************************************************************************
* Function Name: RxDMADone_Interrupt
********************************************************************************
* Summary:
*   The Interrupt Service Routine for isr_RxDMADone. This handles the AUDIO In
*	buffer pointers and detects overflow of the buffer to stop AUDIO In. 
*
* Parameters:  
*	RxDMADone interrupt vector
*
* Return:
*   void.
*
*******************************************************************************/
CY_ISR(RxDMADone_Interrupt)
{    
    uint16 removed;
	
	/* Note: Care must be taken in the application code to update inUsbCount atomically */
	removed = inUsbCount - inUsbShadow;
    
	inLevel -= removed;
       
	inUsbShadow = inUsbCount;
		
	if (inLevel >= IN_BUFSIZE) 
    {
		/* Overflow, so disable the DMA, disable the I2S data and tell the main task to reset all its structures */		
		Stop_I2S_Rx();
	}
	else 
    {
		inLevel += IN_TRANS_SIZE;
	} 
}


/*******************************************************************************
* Function Name: TxDMADone_Interrupt
********************************************************************************
* Summary:
*   The Interrupt Service Routine for isr_TxDMADone. This handles the AUDIO Out
*	buffer pointers and detects underflow of the buffer to stop AUDIO out.
*
* Parameters:  
*	TxDMADone interrupt vector
*
* Return:
*   void.
*
*******************************************************************************/
CY_ISR(TxDMADone_Interrupt)
{

	uint16 added;

	/* Note: Care must be taken in the application code to update outUsbCount atomically
	 * Code is currently implemented with both updater of this value in interrupts that are
	 * the same priority so the code will not execute at the same time, but the code
	 * has been written so that this is not a requirement. */
    
	added = outUsbCount - outUsbShadow;
    
	outLevel += added;
            
	outUsbShadow = outUsbCount;
	
	if (outLevel <= AUDIOMAXPKT) 
	{
		/* Underflow, so disable the DMA, disable I2S TX and tell the main task to reset all its structures */
		Stop_I2S_Tx();
	}
	else
	{
		outLevel -= OUT_TRANS_SIZE;
	}
	
	if (outLevel > (OUT_BUFSIZE + MAX_AUDIO_SAMPLE_SIZE)) 
	{		
		/* Stop I2S till the overflow condition is present - this provides USB enough time to correct the overflow
			Disable the DMA, mute and tell the main task to reset all its structures Stop DMA */
		Stop_I2S_Tx();
	}
}


/* [] END OF FILE */
