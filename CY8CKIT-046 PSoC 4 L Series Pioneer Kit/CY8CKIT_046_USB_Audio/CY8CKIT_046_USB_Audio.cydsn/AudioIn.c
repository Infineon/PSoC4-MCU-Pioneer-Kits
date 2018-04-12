/*******************************************************************************
* File Name: AudioIn.c
*
* Version 1.0
*
*  Description: This file contains the Audio In path configuration and 
*               processing code
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
#include <Interrupts.h>
#include <Application.h>
#include <AudioControl.h>
#include <Config.h>
#include <AudioOut.h>
#include <AudioIn.h>
#include <project.h>
#include <Codec.h>

extern uint8 inBuffer[IN_BUFSIZE];
extern CYBIT audioClkConfigured;

CYBIT resetRx = 0;
CYBIT inPlaying = 0;
uint16 inLevel = IN_BUFSIZE;
uint16 inUsbCount = 0;
uint16 inUsbShadow = 0;
CYBIT clearInBuffer = 0;

/* Digital Audio Receive buffer */
uint8 inBuffer[IN_BUFSIZE];
uint16 inBufIndex = 0;

extern uint8 audioSamples;

#if(USBFS_EP_MM == USBFS__EP_DMAAUTO)   
	extern uint8 outRam[OUT_AUDIOMAXPKT];  
 	extern uint8 inRam[IN_AUDIOMAXPKT];
  	extern uint16 inCnt;  
#endif

extern uint8 setRate;



/*******************************************************************************
* Function Name: InitializeAudioInPath
********************************************************************************
* Summary:
*       This function initializes all the associated DMAs for the audio IN
*       path
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitializeAudioInPath(void)
{
    /* Initialize and setup I2S RX to inBuffer DMA */
	RxDMA_Init();
	RxDMA_SetNumDataElements(0, IN_BUFSIZE);
    RxDMA_SetDstAddress(0, (void *) inBuffer);
	RxDMA_SetSrcAddress(0, (void *) I2S_RX_FIFO_0_PTR);
	
	/* Validate descriptor */
    RxDMA_ValidateDescriptor(0);
        
    /* USB In DMA Config: Entire chain is sent based on a single request from the CPU. TDs are configured  
     	runtime when executing USB Endpoint interrupts */
    USBInDMA_Init();
	/* Enable USBInDMA */
    USBInDMA_ChEnable();
    
    /* Start interrupts */
    isr_RxDMADone_StartEx(RxDMADone_Interrupt);
    USBInDMA_SetInterruptCallback(InDMADone_Interrupt);
	
	/* Enable USB In DMA interrupt */
	CyIntEnable(USBInDMA_CHANNEL);
    
	/* Initialize the RX byte counter - Counter is enabled when IN streaming starts */
    ByteCounter_Rx_Init(); 
}


/*******************************************************************************
* Function Name: ProcessAudioIn
********************************************************************************
* Summary:
*        Handle USB audio in data, setup USB DMA and trigger to transfer samples 
*        from SRAM audio circular buffer to SRAM USB endpoint memory. Start I2S 
*		 receive when USB is active.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/

void ProcessAudioIn(void)
{   
	/* Local variables used for temporary RX received bytes calculation */
	uint32 remain;
    uint32 count = 0;
    
	/* Process IN only if audio clock is configured and running */
    if( audioClkConfigured ==  TRUE)
    {
        /* If there was a reset issued previously, clear the bytes processing variables */
		if (resetRx)
        {
            inBufIndex = 0;
            inUsbCount = 0;
            resetRx = 0;  
        }
		
		/* Disable USB IN channel before reconfiguring the same */
        USBInDMA_ChDisable();
        
		/* Process the number of received bytes from I2S RX based on the rate set */
        switch(setRate)
        {
	       	case RATE_48KHZ:                    
	        	/* count = 48 samples (ideal) per USB transfer (1 ms) */
				count = IN_AUDIOMAXPKT - MAX_AUDIO_SAMPLE_SIZE;        /* Count is number of bytes consumed from the buffer */
				
				/* If received audio from I2S greater than 49, */
	            /* Send an extra packet to avoid overflow */
				if(audioSamples >= AUDIO_SAMPLES_HIGH_48KHZ)
				{
					count = count + SINGLE_STEREO_AUDIO_SAMPLE_SIZE;
				}
	            /* If received audio from I2S less than 47, */
	            /* Send one packet less to avoid underflow */
				else if(audioSamples <= AUDIO_SAMPLES_LOW_48KHZ)
				{
					count = count - SINGLE_STEREO_AUDIO_SAMPLE_SIZE;		
				}
				else
				{
					/* place holder for future */	
				}
	        break;
				
	        case RATE_44KHZ:
	        	/* count = 44 samples (ideal) per USB transfer (1 ms) */
				count = IN_AUDIOMAXPKT - MAX_AUDIO_SAMPLE_SIZE - ((IN_AUDIOMAXPKT - MAX_AUDIO_SAMPLE_SIZE)/12);
					
				if(audioSamples >= AUDIO_SAMPLES_HIGH_44KHZ)
				{
					count = count + SINGLE_STEREO_AUDIO_SAMPLE_SIZE;
				}
				else if(audioSamples <= AUDIO_SAMPLES_LOW_44KHZ)
				{
					count = count - SINGLE_STEREO_AUDIO_SAMPLE_SIZE;		
				}
				else
				{
					/* place holder for future */	
				}
	        break;

	        default:
            break;
        }
        
        inCnt = count;
         
        /* Update of inUsbCount needs to be atomic */
        isr_RxDMADone_Disable();
        inUsbCount += count;
        isr_RxDMADone_Enable();
        
		/* Implement the circular buffer (inBuffer) DMA transfer
			When the number of bytes to transfer from inBuffer to USB inRam wraps around inBuffer,
			reset the buffer pointer and fill the remaining number of bytes from the buffer start. Two DMA TDs are
			used for this purpose - one (TD 0) transferring the inBuffer to inRam till inBuffer hits its last element
			and	second one (TD 1) transferring the remaining number of bytes to the inRam from inBuffer starting locations.
			
			When the number of bytes to transfer from inBufferRam to inRam does not reach inBuffer boundary,
			Use only one TD (TD 0) to transfer the inBuffer contents to inRam */
        if ((inBufIndex + count) > sizeof(inBuffer)) 
        {
            /* Set up TD to wrap around circular buffer */
            remain = sizeof(inBuffer) - inBufIndex;
			
			USBInDMA_SetNumDataElements(0, remain);
			USBInDMA_SetDstAddress(0, (void *) (inRam));
			USBInDMA_SetSrcAddress(0, (void *) (inBuffer + inBufIndex));			
			/* Chain to next descriptor */
            USBInDMA_SetPostCompletionActions(0, CYDMA_CHAIN | CYDMA_INVALIDATE);
			USBInDMA_SetTransferMode(0,CYDMA_ENTIRE_DESCRIPTOR_CHAIN);			
			
			USBInDMA_SetNumDataElements(1, count - remain);
			USBInDMA_SetDstAddress(1, (void *) (inRam + remain));
			USBInDMA_SetSrcAddress(1, (void *) (inBuffer));
			/* Invalidate descriptor 1 after transfer, trigger interrupt */
            USBInDMA_SetPostCompletionActions(1, CYDMA_CHAIN | CYDMA_INVALIDATE | CYDMA_GENERATE_IRQ);
			USBInDMA_SetTransferMode(1, CYDMA_ENTIRE_DESCRIPTOR);
			
			USBInDMA_SetNextDescriptor(0);	
			
			/* Validate descriptors */
            USBInDMA_ValidateDescriptor(0);
            USBInDMA_ValidateDescriptor(1);
           
            inBufIndex = count-remain;
        }
        else 
        {
            USBInDMA_SetNumDataElements(0, count);
			USBInDMA_SetDstAddress(0, (void *) (inRam));
			USBInDMA_SetSrcAddress(0, (void *) (inBuffer + inBufIndex));			
			/* Invalidate descriptor 0 after transfer, trigger interrupt */
            USBInDMA_SetPostCompletionActions(0, CYDMA_INVALIDATE | CYDMA_GENERATE_IRQ);
			USBInDMA_SetTransferMode(0,CYDMA_ENTIRE_DESCRIPTOR);
			
			USBInDMA_SetNextDescriptor(0);	
			
			/* Validate descriptors */
            USBInDMA_ValidateDescriptor(0);
			
            inBufIndex += count;
            if (inBufIndex == sizeof(inBuffer)) 
			{
				inBufIndex = 0;
			}	
        }
    
        /* Enable the USB In DMA, don't update the TD as it progresses */
        USBInDMA_ChEnable();
		
        /* Start the DMA now */
		USBInDMA_Trigger();
        
        
        /* Sending of the data on the USB interface is enabled when the interrupt indicates that the buffer 
         * has been filled. */
        if (!inPlaying && inUsbCount >= IN_HALF) 
        {    
            inPlaying = 1;
			
			I2S_DisableRx();            
            
            /* Enable power to microphone input */
            Codec_PowerOnControl(CODEC_POWER_CTRL_MICPD);
			
            /* Clear the I2S internal FIFO */
            I2S_ClearRxFIFO();            
           
            /* Clear Any potential pending DMA requests before starting the DMA channel to transfer data */
			RxDMA_ChEnable();
			
			/* Restart Byte Counter */            
            ByteCounter_Rx_Enable();
                        
            /* Unmute the RX output */
            I2S_EnableRx();        
        }
    }	
}

/*******************************************************************************
* Function Name: Stop_I2S_Rx
********************************************************************************
* Summary:
*        This function stops the I2S data reception by disabling the I2S and 
*        receive DMA.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/

void Stop_I2S_Rx(void)
{
    if(inPlaying)
    {
        ByteCounter_Rx_Stop(); /* Stop Byte Counter */

        I2S_DisableRx();    /* Stop I2S Receive (Mute), I2S output clocks still active */
        
        /* Terminate TD chain & Stop/Disable DMA - Needed to reset to start of chain */        
		RxDMA_ChDisable();
		
		/* Reset the DMA status for starting a fresh transaction in the next start */
		CYDMA_DESCR_BASE.descriptor[RxDMA_CHANNEL][0].status &= 0xFFFF0000;
                
        resetRx = 1;
        
        clearInBuffer = 1;
        
        inLevel = IN_BUFSIZE;
        inUsbShadow = 0;    
        inPlaying = 0;
        
        /* Disable power to microphone input */
        Codec_PowerOffControl(CODEC_POWER_CTRL_MICPD);
    }
}


/*******************************************************************************
* Function Name: HandleAudioInBuffer
********************************************************************************
* Summary:
*       This routine clears the audio IN stream circular buffer and the audio
*       IN endpoint memory location based on the status of the audio IN stream.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/

void HandleAudioInBuffer(void)
{
    uint16 i;
    
    if(clearInBuffer)
    {
        /* Clear the IN circular buffer - This is slow and hence moving to main loop */
        for (i=0; i < sizeof(inBuffer); i++)
        {
            if(i < AUDIOMAXPKT)
            {
                inRam[i] = 0;
            }
            inBuffer[i] = 0;
        }
        
        clearInBuffer = 0;
    }
}


/* [] END OF FILE */
