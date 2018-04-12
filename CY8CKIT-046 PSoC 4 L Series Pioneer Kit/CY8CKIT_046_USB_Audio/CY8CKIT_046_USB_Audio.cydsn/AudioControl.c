/*******************************************************************************
* File Name: AudioControl.c
*
* Version 1.0
*
*  Description: This file contains the Audio signal path configuration and 
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
#include <Application.h>
#include <AudioControl.h>
#include <AudioOut.h>
#include <AudioIn.h>
#include <Codec.h>
#include <Config.h>
#include <project.h>
#include <Interrupts.h>
#include <USBInterface.h>

uint8 newRate = RATE_48KHZ;
extern uint8 asyncready;


extern CYBIT outPlaying;

extern CYBIT inPlaying;



uint8 audioClkConfigured = FALSE;
uint8 setRate = FREQUENCY_NOT_SET; 
uint8 newRate;

/*******************************************************************************
* Function Name: InitAudioPath
********************************************************************************
* Summary:
*       This function sets up the XTAL, DMA and starts USB, I2S and interrupts
*        to get the part configured for the audio signal paths
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitAudioPath(void)
{    
    /* Set the default Audio clock rate to 48 kHz */
	AudioClkSel_Write(RATE_48KHZ);
    
	/* Enable DMA */
	CyDmaEnable();
	
	InitializeAudioInPath();
	InitializeAudioOutPath();

	/* Set TX FIFO trigger to 2 bytes (half-empty) to increase timing margin */
    I2S_TX_AUX_CONTROL_REG = I2S_TX_AUX_CONTROL_REG | FIFO_HALF_EMPTY_MASK;
    
    /* Set RX FIFO trigger to 2 bytes (half-empty) to increase timing margin */
    I2S_RX_AUX_CONTROL_REG = I2S_RX_AUX_CONTROL_REG | FIFO_HALF_EMPTY_MASK;
}



/*******************************************************************************
* Function Name: SetClockRate
********************************************************************************
* Summary:
*        This function changes the audio clocking to generate clocks for a desired 
*        sample rate.
*
* Parameters:
*  newRate: audio sample rate to be set
*
* Return:
*  void
*
*******************************************************************************/
void SetClockRate(uint8 newRate) 
{
	/* Stop I2S before changing PLL clock */
	Stop_I2S_Tx();    
    Stop_I2S_Rx();
    
	  
	/* Configure Codec and Audio clock */
	Codec_Deactivate();
	
	if(newRate == RATE_48KHZ)
	{
		CySysClkPllStart(RATE_48KHZ, 1);
		CySysClkPllStop(RATE_44KHZ);
		AudioClkSel_Write(RATE_48KHZ);
		Codec_SetSamplingRate(CODEC_SRATE_NORMAL_48KHZ_256FS);
		
	}
	else if(newRate == RATE_44KHZ)
	{
		CySysClkPllStart(RATE_44KHZ, 1);
		CySysClkPllStop(RATE_48KHZ);
		AudioClkSel_Write(RATE_44KHZ);
		Codec_SetSamplingRate(CODEC_SRATE_NORMAL_44KHZ_256FS);		
	}
	else
	{
		CySysClkPllStop(RATE_48KHZ);
		CySysClkPllStop(RATE_44KHZ);		
		Codec_SetSamplingRate(CODEC_SRATE_NORMAL_48KHZ_256FS);
	}
	
	Codec_Activate();	 
	
    /* flag to indicate audio clock active */
    audioClkConfigured = TRUE;    
}
 
/*******************************************************************************
* Function Name: HandleSamplingFrequencyChangeRequest
********************************************************************************
* Summary:
*       This function processes the sampling frequency change request from USB
*       host and updates the accessory playback sampling frequency accordingly
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void HandleSamplingFrequencyChangeRequest(void)
{
    /* USBFS_frequencyChanged is set by the USB component when a setup token for Sampling frequency change is received.
     * It takes sometime for the host to actually send the Out token containing the updated sampling frequency.
     * Wait for USBFS_transferState to be equal to USBFS_TRANS_STATE_IDLE to make sure the updated sampling frequency 
     * is used for setting audio clocks */
     
    if((USBFS_TRANS_STATE_IDLE == USBFS_transferState) && USBFS_frequencyChanged)
    {		
        if(((!inPlaying) && (!outPlaying)) && ((AUDIO_OUT_ENDPOINT == USBFS_frequencyChanged)		
             || (AUDIO_IN_ENDPOINT == USBFS_frequencyChanged)))
        {
            uint32 newFrequency;
                        
            newFrequency = (((uint32)USBFS_currentSampleFrequency[USBFS_frequencyChanged][2] << 16) |
                              ((uint32)USBFS_currentSampleFrequency[USBFS_frequencyChanged][1] << 8) |
                              ((uint32)USBFS_currentSampleFrequency[USBFS_frequencyChanged][0]));

			USBFS_frequencyChanged = 0;
			
            if(SAMPLING_RATE_48KHZ == newFrequency)
            {
                newRate = RATE_48KHZ;
				PRINT("Sampling Rate Change Request: 48 ksps \r\n");
            }
            else if(SAMPLING_RATE_44KHZ == newFrequency)
            {
                newRate = RATE_44KHZ;
				PRINT("Sampling Rate Change Request: 44.1 ksps \r\n");
            }
                
            if(setRate != newRate)
            {   
                setRate = newRate;				
				processAsyncFeedbackTransfer(DO_NOT_CLEAR_UPDATE_FLAG);
                
                SetClockRate(setRate);
                                
				asyncready = TRUE;
            }            
        }        
        else
        {
            /* If another frequency change request is received when one of the audio stream is still active, clear the
             * frequency change request */
            USBFS_frequencyChanged = 0;
        }
        
    }
}

/* [] END OF FILE */
