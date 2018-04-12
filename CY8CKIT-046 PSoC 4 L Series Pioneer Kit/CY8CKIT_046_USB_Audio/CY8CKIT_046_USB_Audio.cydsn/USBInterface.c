/*******************************************************************************
* File Name: USBInterface.c
*
* Version 1.0
*
* Description: This file contains routines for handling PC/Mac/any USB Host
*              USB interface requests.
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
#include <Application.h>
#include <AudioControl.h>
#include <AudioIn.h>
#include <AudioOut.h>
#include <Config.h>
#include <USBInterface.h>
#include <Codec.h>

extern volatile uint8 USBFS_interfaceSetting[];

extern uint8 USBFS_initVar;
extern uint8 playlistControlReport;
extern CYBIT outPlaying;
extern CYBIT inPlaying;

extern CYBIT codecInit;  

uint8 USBDeviceState = USB_INTERFACE_INACTIVE;

#if(USBFS_EP_MM == USBFS__EP_DMAAUTO) 
	uint8 outRam[OUT_AUDIOMAXPKT];
	uint8 inRam[IN_AUDIOMAXPKT];
    uint16 inCnt;    
#endif

uint8 freeRunningLRCLKCount;
uint8 audioSamples;
uint8 lastFreeRunningLRCLKCount = MAX_UINT8_COUNT-1;
uint8 explicitFeedback[3] = {0x00,0x00, 0x0C}; /* 48 bytes explicit feedback */
uint16 explicitFeedbackSamples;
uint8 updated = FALSE;

extern uint16 outLevel;
CYBIT audioAsync = IMPLICIT_FEEDBACK ;
extern uint8 setRate;
uint8 asyncready = FALSE;

uint8 altSetting[NO_OF_AUDIO_STREAM_INTERFACE] = {ALT_SETTING_INVALID, ALT_SETTING_INVALID};

/*******************************************************************************
* Function Name: ServiceUSB
********************************************************************************
* Summary: This routine performs tasks that should be done soon after USB 
*          enumeration is completed (configure DMA, initialize state machine, etc.).
*          When the USB configuration is changed, this routine reinitializes all
*          the USB endpoints as required by the application.       
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void ServiceUSB(void)
{
	audioAsync = EXPLICIT_FEEDBACK;
	
	if(USB_INTERFACE_INACTIVE == USBDeviceState)
    {
        USBDeviceState = USB_INIT_AFTER_ENUMERATION_REQUIRED;
    }
	
	/* Initialization sequence for every USB host enumeration event */
    if(USBDeviceState == USB_INIT_AFTER_ENUMERATION_REQUIRED)
    {
        uint16 index = 0;
        
        USBDeviceState = USB_INIT_AFTER_ENUMERATION_COMPLETED;
        
        #if(USBFS_EP_MM == USBFS__EP_DMAAUTO)
            
			for(index=0; index< AUDIOMAXPKT; index++)
            {
				outRam[index] = 0;
                inRam[index] = 0;                
            }	
            
		  	inCnt = AUDIOMAXPKT;            
        #endif		
    }
    
   
    /* USBFS_IsConfigurationChanged() is a clear on read status update therefore, only one read of 
     * USBFS_IsConfigurationChanged() should ever exist in user code */
    if(USBFS_IsConfigurationChanged())
    {	                    
        if(altSetting[AUDIO_OUT_INTERFACE_INDEX] != USBFS_GetInterfaceSetting(1))
		{
	        /* Get Alternate setting */	        
            altSetting[AUDIO_OUT_INTERFACE_INDEX] = USBFS_GetInterfaceSetting(1); 
			
			/* Set 24-bit mode for I2S */
			/* Clear internal pointers */			
			Stop_I2S_Rx();			  
			Stop_I2S_Tx();
			
			/* Stop component */
			I2S_Stop();

			/* Note: ByteCounter is stopped in Stop_I2S_Tx(); */        
	        ByteCounter_Tx_WritePeriod((OUT_TRANS_SIZE) - 1);
	        ByteCounter_Tx_Enable();

			/* Restart I2S*/
			I2S_Start();
						
	        
	        /* Arming the audio out EP if it is not zero bandwidth alt setting */
	        if (altSetting[AUDIO_OUT_INTERFACE_INDEX]!= ALT_SETTING_ZERO_BANDWIDTH)           
	        {
	            /* Init DMA configurations for audio OUT EP */
	            USBFS_ReadOutEP(AUDIO_OUT_ENDPOINT, &outRam[0], OUT_AUDIOMAXPKT);
	            USBFS_EnableOutEP(AUDIO_OUT_ENDPOINT);                   
                
                if (EXPLICIT_FEEDBACK == audioAsync )
    			{
    	            USBFS_LoadInEP(ASYNC_EXPLICIT_FEEDBACK_ENDPOINT, &explicitFeedback[0], sizeof(explicitFeedback));
    				USBFS_LoadInEP(ASYNC_EXPLICIT_FEEDBACK_ENDPOINT, USBFS_NULL, sizeof(explicitFeedback));
    	        }
	        }
            else
            {                
                /* Trigger USB Audio OUT EP DMA to flush out the data before entering zero bandwidth alt setting */
                USBFS_CyDmaTriggerIn(USBFS_DmaReqOut[AUDIO_OUT_ENDPOINT]);
                USBFS_CyDmaTriggerIn(USBFS_DmaReqOut[ASYNC_EXPLICIT_FEEDBACK_ENDPOINT]);
            }
			
			processAsyncFeedbackTransfer(DO_NOT_CLEAR_UPDATE_FLAG);        
        }        
        
        if(altSetting[AUDIO_IN_INTERFACE_INDEX] != USBFS_GetInterfaceSetting(2))
    	{            
            altSetting[AUDIO_IN_INTERFACE_INDEX] = USBFS_GetInterfaceSetting(2);           
		
			Stop_I2S_Rx();			
			
            ByteCounter_Rx_WritePeriod((IN_TRANS_SIZE) - 1);
            
            /* Arming the audio in EP if it is not zero bandwidth alt setting */
            if (altSetting[AUDIO_IN_INTERFACE_INDEX] != ALT_SETTING_ZERO_BANDWIDTH)               
            {
                /* Init DMA configurations for audio IN EP */  
                inCnt = IN_AUDIOMAXPKT;
                USBFS_LoadInEP(AUDIO_IN_ENDPOINT, &inRam[0], inCnt);
                
                /* Pre-arm first audio IN request */
                USBFS_LoadInEP(AUDIO_IN_ENDPOINT, USBFS_NULL, inCnt);                
            }
            else
            {
                 /* Trigger USB Audio IN EP DMA to flush out the data before entering zero bandwidth alt setting */
                USBFS_CyDmaTriggerIn(USBFS_DmaReqOut[AUDIO_IN_ENDPOINT]);
            }
		}       
	}
}

/*******************************************************************************
* Function Name: UpdateFeedbackCount
********************************************************************************
* Summary: This routine calculates the total number of audio samples consumed
*          by PSoC per SOF interval. The number of audio samples consumed
*          determines the explicit feedback count to be sent to the USB host
*          over the feedback endpoint
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void UpdateFeedbackCount(void)
{	
	/* Feedback calculation logic for both implicit and explicit feedback on every SOF interrupt */
	/* Get the Async_Feedback_Counter current count value */
	freeRunningLRCLKCount = Async_Feedback_Counter_ReadCounter();
	
	/* Calculate the number of audio samples transferred from last SOF interrupt */
    audioSamples = (((uint32)lastFreeRunningLRCLKCount) + MAX_UINT8_COUNT - ((uint32)freeRunningLRCLKCount));
	
	/* Store the read counter value for next cycle */
	lastFreeRunningLRCLKCount = freeRunningLRCLKCount;
	
	/* If clocks and Async endpoint are configured, process Async data */
    if(asyncready)
    {
	    /* Process only for explicit feedback */
		if (EXPLICIT_FEEDBACK == audioAsync )
	    {	 	        
			/* Read the feedback count value */
			explicitFeedbackSamples = ((uint16)explicitFeedback[2]) << 8 | explicitFeedback[1];
			
			/* Increment or decrement the feedback count value depending on whether the audio out buffer is less than half full or more than 3/4th full respectively.
				The update happens only once every Async feedback point read from the host */
	        if((outLevel < OUT_HALF) && (updated == FALSE))
	        {	            
				explicitFeedbackSamples += MINIMUM_FEEDBACK_REQUEST;					
	            updated = TRUE;
	        }
	        else if((outLevel > (OUT_HALF + AUDIOMAXPKT))&& (updated == FALSE))
	        {				
				explicitFeedbackSamples -= MINIMUM_FEEDBACK_REQUEST;			
	            updated = TRUE;				
	        }
			
			/* Update the feedback count back */
			explicitFeedback[2] = HI8(explicitFeedbackSamples);
			explicitFeedback[1] = LO8(explicitFeedbackSamples);
	        	        	        
	        /* Arm the feedback endpoint */
	        if((USBFS_GetEPState(ASYNC_EXPLICIT_FEEDBACK_ENDPOINT) == USBFS_IN_BUFFER_EMPTY))
	        {		            
				USBFS_LoadInEP(ASYNC_EXPLICIT_FEEDBACK_ENDPOINT, USBFS_NULL, sizeof(explicitFeedback));
	        }
	    
	    }
    }

}

/*******************************************************************************
* Function Name: processAsyncFeedbackTransfer
********************************************************************************
* Summary: This routine uses the Async explicit feedback endpoint data transfer
*          completion interrupt to update the explicit feedback count to it's
*          default value corresponding to system sampling rate
    *
* Parameters:
*  None
*
* Return:   
*  None
*
*******************************************************************************/
void processAsyncFeedbackTransfer(uint32 clearFlag)
{
    /* Reset the feedback count every time the host reads the feedback count data */
	
	if (EXPLICIT_FEEDBACK == audioAsync )
	{				
		switch(setRate)
		{
			case RATE_48KHZ:
				explicitFeedback[0] = 0x00;
				explicitFeedback[1] = 0x00;
				explicitFeedback[2] = 0x0C; /* 48 samples explicit feedback */
			break;
				
			case RATE_44KHZ:
				explicitFeedback[0] = 0x66; 
				explicitFeedback[1] = 0x06;
				explicitFeedback[2] = 0x0B; /* 44.1 samples explicit feedback */
			break;
			
			default:
			break;
		}	
		
		/* Clear the flag to indicate the data has been read by the host */
		if(clearFlag)
		{
			updated = FALSE;
		}
	}
}

/* [] END OF FILE */
