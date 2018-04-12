/*******************************************************************************
* File Name: Application.c
*
* Version 1.0
*
*  Description: This file contains all the application layer code which performs
*               the user interface handling
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
#include <Config.h>
#include <project.h>
#include <Interrupts.h>
#include <stdio.h>
#include <USBInterface.h>
#include <Gesture.h>
#include <Codec.h>

static void HandleUserInputs(uint32 gesture);


extern CYPDATA uint8 audioSource;
extern CYDATA uint8 auxConfigured;

extern uint32 accSliderDelta;

uint8 playlistControlReport;
uint8 prevReport = 0;
uint8 audioControlStatus;
uint8 reportClearFlag;

extern volatile uint8 USBFS_currentVolume[];
extern volatile uint8 USBFS_currentMute;
extern CYPDATA uint8 newRate;


/*******************************************************************************
* Function Name: InitApp
********************************************************************************
* Summary:
*        This function configures the application layer hardware modules and
*        clocks to be in sync with the audio clock updates done in 
*        InitializeAudioPath API
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitApp(void)
{
    /* Configure CPU/DMA to be in round robin mode while accessing memory */
	CY_SET_REG32((void *) 0x40100038, CY_GET_REG32((void *) 0x40100038) | 0x22222222);     
	
	CyGlobalIntEnable;
	
    #ifdef TXDEBUG
		UART_Start();
		if(CySysGetResetReason(CY_SYS_RESET_SW) != CY_SYS_RESET_SW)
		{
			PRINT("\r\nApp Started...\r\n");
		}
		else
		{
			PRINT("\r\nApp Restarted after SW reset...\r\n");
		}
	#endif /* #ifdef TXDEBUG */

    /* Start I2C Master */
    CodecI2CM_Start();	
	
	if(Codec_Init() == 0)
	{
		PRINT("Codec comm works!... \r\n");
	}
	else
	{
		PRINT("Codec comm DOESN'T work!... \r\n");
	}
	
	Update_VolumeAudioOut();
	
	I2S_Start();	
	
	USBFS_Start(PC_MAC_AUDIO_WITH_VOLUME_DEVICE, USBFS_5V_OPERATION);
    USBDeviceState = USB_INIT_AFTER_ENUMERATION_REQUIRED;
	
	/* Enables SOF interrupt source */
    USBFS_INTR_SIE_MASK_REG |= USBFS_INTR_SIE_SOF_INTR;

	while(USBFS_GetConfiguration());
	
	CyIntSetPriority(CYDMA_INTR_NUMBER, 0);
	
	USBFS_LoadInEP(MAC_PC_HID_CONTROL_ENDPOINT, &playlistControlReport, sizeof(playlistControlReport));
	USBFS_LoadInEP(MAC_PC_HID_CONTROL_ENDPOINT, USBFS_NULL, sizeof(playlistControlReport) );	
	
	Async_Feedback_Counter_Start();
	
	CapSense_Start();
	CapSense_InitializeAllBaselines();
	CapSense_ScanEnabledWidgets();
}


/*******************************************************************************
* Function Name: RunApplication
********************************************************************************
* Summary:
*      This function runs the application layer firmware of handling CapSense and
*       volume control.
*   
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void RunApplication(void)
{
    uint32 gesture = 0;
	           
    if(!CapSense_IsBusy()) /* scan all CapSense buttons and sliders */
    {
        CapSense_UpdateEnabledBaselines(); /* update the baseline for the CapSense algorithm */
		gesture = DetectGesture(CapSense_CheckIsAnyWidgetActive());
        CapSense_ScanEnabledWidgets();
		
		if(USBFS_GetConfiguration() == TRUE)
		{
			/* Handle CapSense button and slider user inputs */
	   	 	HandleUserInputs(gesture); 
		}
    }
    
	if(USBFS_GetConfiguration() == TRUE)
	{
		/* Update the volume data */
		Update_VolumeAudioOut();		
	}
}


/*******************************************************************************
* Function Name: HandleUserInputs
********************************************************************************
* Summary:  Handles CapSense button user inputs for next/previous/play track 
*           functions.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void HandleUserInputs(uint32 gesture)
{    
	static uint32 volUpdateThreshold = 0;
	static uint8 prevReport;	
	static bool volumeReport = false;
	#if (UP_BUTTON_FUNCTION == MIC_MUTE)
		static bool micMute;
	#endif
	#ifndef VOLUME_BTN_UPDATE
		int32 volume;
	#endif
	
	switch(capsenseButtonStatus)
	{
		case LEFT_BTN_MASK:		
			audioControlStatus |= MAC_PC_PREVIOUS_TRACK_MASK; 
			PRINT("Prev Track\n\r");
		break; 
		
		case RIGHT_BTN_MASK:
			audioControlStatus |= MAC_PC_NEXT_TRACK_MASK; 
			PRINT("Next Track\n\r");
			break;
			
		case UP_BTN_MASK:
			
			#if (UP_BUTTON_FUNCTION == VOLUME_MUTE)
				audioControlStatus |= MAC_PC_MUTE_MASK; 
				PRINT("Mute\n\r");
			#else
				if(micMute == true)
				{
					Codec_MuteMic(false);
					micMute = false;
					PRINT("MIC Unmute \n\r");
				}
				else
				{
					Codec_MuteMic(true);
					micMute = true;
					PRINT("MIC Mute \n\r");
				}
			#endif
			break; 
			
		case DOWN_BTN_MASK:
			audioControlStatus |= MAC_PC_STOP_MASK; 
			PRINT("Stop \n\r");
			break;
			
		case CENTRE_BTN_MASK:
			audioControlStatus |= MAC_PC_PLAY_PAUSE_MASK; 
			PRINT("Play/Pause\n\r");
			break;
			
		default:
			break;
	}
	
	switch(gesture)
	{	
		case GESTURE_LEFT_SWIPE:
			/* Left swipe sends out Previous Track command to computer */
			PRINT("Left Swipe - Prev Track\n\r");
			audioControlStatus |= MAC_PC_PREVIOUS_TRACK_MASK; 
			break;
			
		case GESTURE_RIGHT_SWIPE:
			/* Right swipe sends out Next Track command to computer */
			PRINT("Right Swipe - Next Track\n\r");
			audioControlStatus |= MAC_PC_NEXT_TRACK_MASK; 
			break;
			
		case GESTURE_UP_SWIPE:
			/* Up swipe sends out Volume up command to computer */
			if(volUpdateThreshold == 0)
			{
				/* Send a Volume up command every time the volUpdateThreshold crosses threshold 
				- This is done to reduce sudden increase in volume because of the gesture */
				PRINT("Up Swipe - Vol Up\n\r");
				audioControlStatus |= MAC_PC_VOL_UP_MASK;
			}
			
			volUpdateThreshold++;
			
			if(volUpdateThreshold >= VOL_CHANGE_UPDATE_FREQ)
			{				
				volUpdateThreshold = 0;				
			}
			
			break;
			
		case GESTURE_DOWN_SWIPE:
			/* Down swipe sends out Volume down command to computer */
			if(volUpdateThreshold == 0)
			{
				/* Send a Volume down command every time the volUpdateThreshold crosses threshold 
				- This is done to reduce sudden decrease in volume because of the gesture */
				PRINT("Down Swipe - Vol Down\n\r");
				audioControlStatus |= MAC_PC_VOL_DOWN_MASK;
			}
			
			volUpdateThreshold++;
			
			if(volUpdateThreshold >= VOL_CHANGE_UPDATE_FREQ)
			{				
				volUpdateThreshold = 0;				
			}
		break;			
		
		case GESTURE_OUTER_CLKWISE:
		case GESTURE_OUTER_COUNTER_CLKWISE:
			if(volumeReport == false)
			{
				volumeReport = true;
				accTheta = 0;
			}
			
		break;
			
			
		default:
			volUpdateThreshold = 0;
			break;
	}	
	
    /* Volume update for circular gestures - Outer clockwise and counter-clockwise */
	/* Check the volume report flag - this flag is set only when a circular gesture is detected on the slider */
	if(volumeReport == true)
	{
		/* Process the gestures only if the slider is active ==> radius > 2 */
		if((radius != INVALID_RADIUS) && (radius > 2))
		{						
			/* Check accumulated theta value
				Accumulated theta is positive for counter-clockwise
				Once accumulated theta crosses a set threshold send a Volume down command */
			if(accTheta > VOL_CHANGE_THRESHOLD)
			{
				accTheta -= VOL_CHANGE_THRESHOLD;
				audioControlStatus |= MAC_PC_VOL_DOWN_MASK;
			}
			
			/* Accumulated theta is negative for clockwise
				Once accumulated theta crosses a set threshold send a Volume up command */
			else if(accTheta < -VOL_CHANGE_THRESHOLD)
			{
				accTheta += VOL_CHANGE_THRESHOLD;
				audioControlStatus |= MAC_PC_VOL_UP_MASK;
			}			
		}
		/* If slider is not active, then clear the volumeReport flag */
		else
		{
			volumeReport = false;
		}
	}
	
	

	/* If USB is configured and there is an event pending on the HID endpoint, send the HID report */
    if((IsUSBConfigured()) && (USBFS_GetEPState(MAC_PC_HID_CONTROL_ENDPOINT) == USBFS_EVENT_PENDING) )
    {
        /* Send report only if there is a change in data or if data is available */
		if((prevReport != audioControlStatus) || (audioControlStatus != 0))
		{			
			/* Update the report buffer with the data */
			playlistControlReport = audioControlStatus;
			
			/* Store the current data for next comparison */
			prevReport = audioControlStatus;				
			
			/* Clear the report once sent */
			audioControlStatus = 0;
			
			/* Arm the endpoint */			
			USBFS_LoadInEP(MAC_PC_HID_CONTROL_ENDPOINT, USBFS_NULL, sizeof(playlistControlReport));
		}
    }		
	
}

/*******************************************************************************
* Function Name: Update_VolumeAudioOut
********************************************************************************
* Summary:  Updates the volume from PC to codec
*
* Parameters:
*  void
*
* Return:
*  uint8 - status of I2C operation between PSoC 4200L and codec
*
*******************************************************************************/
uint8 Update_VolumeAudioOut(void)
{
	uint8 ret = 0;
	static int32 prevVol = 0;
	static uint8 prevMute;
	#ifdef TXDEBUG
		char str[20];
	#endif
	
	/* Get the MSB of the current volume data */
	int32 volume = (((int8)USBFS_currentVolume[1])) + PC_VOLUME_MSB_CODEC_OFFSET;
	
	if(volume > PC_VOLUME_MSB_MAX)
	{
		volume = PC_VOLUME_MSB_MAX;
	}
	else if(volume < PC_VOLUME_MSB_MIN)
	{
		volume = PC_VOLUME_MSB_MIN;
	}
	
	/* Process volume control only when USB bus is idle */
	if(USBFS_TRANS_STATE_IDLE == USBFS_transferState)
	{
		/* If there is a change in volume, update codec */
		if(volume != prevVol)
		{
			/* Store current volume for next comparison */
			prevVol = volume;
			
			/* Filter volume to be in the expected range */
			if((volume > PC_VOLUME_MSB_MIN) && (volume <= PC_VOLUME_MSB_MAX))
			{
				volume = (uint8)(((volume - PC_VOLUME_MSB_MIN) * CODEC_HP_VOLUME_MAX) / (PC_VOLUME_MSB_MAX - PC_VOLUME_MSB_MIN));	 
			}
			else
			{
				/* Set volume to 0 if the volume from PC is not in expected range */
				volume = 0;
			}
							
			
			/* Update the codec volume */
			ret = Codec_AdjustBothHeadphoneVolume((uint8)volume);
			
			#ifdef TXDEBUG
				sprintf(str, "Codec vol set to %d \r\n", (int)volume);
				PRINT(str);
			#endif
		}
		
		/* Process mute data, if changed */
		if(USBFS_currentMute != prevMute)
		{
			/* Store current mute data for next comparison */
			prevMute = USBFS_currentMute;
			
			/* If mute is non-zero, then mute is active and update codec volume to 0 */
			if(prevMute != 0)
			{
				ret = Codec_AdjustBothHeadphoneVolume(0);
				PRINT("Muted\r\n");
			}
			else
			{
				/* If mute is released, then process and update the current volume to codec */
				prevVol = volume;
								
				/* Filter volume to be in the expected range */
				if((volume > PC_VOLUME_MSB_MIN) && (volume <= PC_VOLUME_MSB_MAX))
				{
					volume = (uint8)(((volume - PC_VOLUME_MSB_MIN) * CODEC_HP_VOLUME_MAX) / (PC_VOLUME_MSB_MAX - PC_VOLUME_MSB_MIN));		 
				}
				else
				{
					/* Set volume to 0 if the volume from PC is not in expected range */
					volume = 0;
				}
								
				/* Update the codec volume */
				ret = Codec_AdjustBothHeadphoneVolume((uint8)volume);
				
				#ifdef TXDEBUG
					sprintf(str, "Unmute - Codec vol set to %d \r\n", (int)volume);
					PRINT(str);
				#endif
			}
		}
	}
	
	
	return ret;
}


/* [] END OF FILE */
