/*******************************************************************************
* File Name: AudioOut.c
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
#include <project.h>
#include <Codec.h>

extern uint8 outBuffer[];

CYBIT resetTx = 0;
CYBIT outPlaying = 0;
uint16 outLevel = 0;
uint16 outUsbCount = 0;
uint16 outUsbShadow = 0;
uint8 outBuffer[OUT_BUFSIZE];
uint16 outBufIndex = 0;
extern CYBIT audioClkConfigured;



#if(USBFS_EP_MM == USBFS__EP_DMAAUTO)   
	extern uint8 outRam[];  
#endif

/*******************************************************************************
* Function Name: InitializeAudioOutPath
********************************************************************************
* Summary:
*       This function initializes all the associated DMAs for the Audio OUT
*       path
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitializeAudioOutPath(void)
{
    TxDMA_Init();
	TxDMA_SetNumDataElements(0, OUT_BUFSIZE);
    TxDMA_SetSrcAddress(0, (void *) outBuffer);
	TxDMA_SetDstAddress(0, (void *) I2S_TX_FIFO_0_PTR);
	
	/* Validate descriptor */
    TxDMA_ValidateDescriptor(0);
		        
    /* USB Out DMA Config: Entire chain is sent based on a single request from the CPU TDs are configured  */
    /* runtime when executing USB Endpoint interrupts */	
    USBOutDMA_Init(); 
	USBOutDMA_ChEnable();
	
	ByteCounter_Tx_Init();
    
    /* Start interrupts */
    isr_TxDMADone_StartEx(TxDMADone_Interrupt);

}

/*******************************************************************************
* Function Name: ProcessAudioOut
********************************************************************************
* Summary:
*        Handle audio out data, setup USB DMA and trigger the DMA to transfer 
*        audio samples from SRAM USB endpoint memory to SRAM audio circular 
*        buffer. The API also starts the I2S transmit when USB audio out streaming 
*        is active
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/

void ProcessAudioOut(void) 
{
	uint16 count;
    uint16 remain;
    
    if( TRUE == audioClkConfigured)
    {
        /* Check TX buffer reset condition and clear pointers if required */
        if (resetTx) 
        {
            outBufIndex = 0;
            outUsbCount = 0;
            resetTx = 0;
        }        
		
        /* Disable the USBOut DMA channel */
		USBOutDMA_ChDisable();
		
		/* Obtain the number of bytes received from USB Audio OUT endpoint */
        count = USBFS_GetEPCount(AUDIO_OUT_ENDPOINT);
    
        /* Update of usbCount needs to be atomic */
        isr_TxDMADone_Disable();
        outUsbCount += count;
        isr_TxDMADone_Enable();
        
		/* Implement the circular buffer (outBuffer) DMA transfer
			When the number of bytes to transfer from USB outRam to outBuffer makes the outBuffer to overflow,
			wraparound the buffer and fill the remaining number of bytes from the buffer start. Two DMA TDs are
			used for this purpose - one (TD 0) transferring the outRam to outBuffer till outBuffer is full and
			second one (TD 1) transferring the remaining number of bytes to the outBuffer starting locations.
			
			When the number of bytes to transfer from outRam to outBuffer does not make the outBuffer to overflow,
			Use only one TD (TD 0) to transfer the entire outRam contents to outBuffer */
        if((outBufIndex + count) > sizeof(outBuffer))
        {
            /* Set up TD to wrap around circular buffer */
            remain = sizeof(outBuffer) - outBufIndex;
			
			USBOutDMA_SetNumDataElements(0, remain);
			USBOutDMA_SetDstAddress(0, (void *) (outBuffer + outBufIndex));
			USBOutDMA_SetSrcAddress(0, (void *) (outRam));						
			USBOutDMA_SetPostCompletionActions(0, CYDMA_CHAIN | CYDMA_INVALIDATE);
			USBOutDMA_SetTransferMode(0,CYDMA_ENTIRE_DESCRIPTOR_CHAIN);			
			
			USBOutDMA_SetNumDataElements(1, count - remain);
			USBOutDMA_SetDstAddress(1, (void *) (outBuffer));
			USBOutDMA_SetSrcAddress(1, (void *) (outRam + remain));			
			USBOutDMA_SetPostCompletionActions(1, CYDMA_CHAIN | CYDMA_INVALIDATE);
			USBOutDMA_SetTransferMode(1, CYDMA_ENTIRE_DESCRIPTOR);
			
			USBOutDMA_SetNextDescriptor(0);	
			
			/* Validate descriptors */
            USBOutDMA_ValidateDescriptor(0);
            USBOutDMA_ValidateDescriptor(1); 
                        
            outBufIndex = count-remain;
        }
        else 
        {
            /* Single contiguous TD */
			USBOutDMA_SetNumDataElements(0, count);
			USBOutDMA_SetDstAddress(0, (void *) (outBuffer + outBufIndex));
			USBOutDMA_SetSrcAddress(0, (void *) (outRam));						
			USBOutDMA_SetPostCompletionActions(0, CYDMA_INVALIDATE);
			USBOutDMA_SetTransferMode(0,CYDMA_ENTIRE_DESCRIPTOR);
			
			USBOutDMA_SetNextDescriptor(0);	
			
			USBOutDMA_ValidateDescriptor(0);
			           
            outBufIndex += count;
            if (outBufIndex == sizeof(outBuffer)) 
			{
				outBufIndex = 0;
			}
        }
        
        /* Enable the USB Out DMA, don't update the TD as it progresses */
		USBOutDMA_ChEnable();
        USBOutDMA_Trigger();                
    
        /* Start playing audio only when transmit buffer is more than half full */
        if((!outPlaying) && (outUsbCount >= OUT_HALF))
        {
            outPlaying = TRUE;
            
            I2S_ClearTxFIFO(); /* Clear the I2S internal FIFO */              
            
            /* Enable power to speaker output */
            Codec_PowerOnControl(CODEC_POWER_CTRL_OUTPD);
            
			TxDMA_ChEnable();            
            
            /* Restart Byte Counter */
            ByteCounter_Tx_Enable();            
            
            I2S_EnableTx(); /* Unmute the TX output */           
        }
    }	
}


/*******************************************************************************
* Function Name: Stop_I2S_Tx
********************************************************************************
* Summary:
*        This function stops the I2S data transmission by disabling the I2S and 
*        transmit DMA.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Stop_I2S_Tx(void) CYREENTRANT
{
    if(outPlaying)
    {       
        ByteCounter_Tx_Stop();
		        
        I2S_DisableTx();     /* Stop I2S Transmit (Mute), I2S output clocks still active */
        
        CyDelayUs(20); /* Provide enough time for DMA to transfer the last audio samples completely to I2S TX FIFO */
   
        /* Stop / Disable DMA - Needed to reset to start of chain */
        TxDMA_ChDisable();
		
		/* Make DMA transaction count zero */ 
        CYDMA_DESCR_BASE.descriptor[TxDMA_CHANNEL][0].status &= 0xFFFF0000;
		
        /* Disable power to speaker output */
        Codec_PowerOffControl(CODEC_POWER_CTRL_OUTPD);
        
        resetTx = 1;
        outLevel = 0;
        outUsbShadow = 0;
        outPlaying = 0;
    }    
}

/* [] END OF FILE */
