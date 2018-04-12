/******************************************************************************
* Project Name		: CY8CKIT_046_USB_Audio
* Version			: 1.0
* Device Used		: CY8C4248BZI-L489     
* Software Used		: PSoC Creator 3.3 Service Pack 1
* Compiler Used		: ARM GCC 4.9.3 
* Related Hardware	: CY8CKIT-046 PSoC 4 L-Series Pioneer Kit 
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
/*******************************************************************************
* Theory of Operation: This project demonstrates the capability of PSoC 4200L to 
* provide a high-quality audio playback and recording interface to a Windows or 
* Mac PC. The example uses the PSoC 4200L USB to implement an asynchronous USB 
* Audio Class v1.0 compliant device. The PSoC 4 L-Series Pioneer Kit has a 
* standard 3.5-mm TRRS audio jack onboard and an audio codec to convert digital 
* audio stream into analog and vice-versa. These components along with PSoC 4200L
* implement the complete USB Audio interface.
*******************************************************************************/

#include <project.h>
#include "Application.h"
#include "Codec.h"
#include "AudioControl.h"
#include "USBInterface.h"

int main()
{
	int32 usbEnumFlag = 0; 
	
	/* Setup the Audio pipe from USB to I2S.*/	
    InitAudioPath();
	
	/* Setup the Application part of the firmware */
    InitApp();
	
	PRINT("InitApp Complete\r\n");
	
    for(;;)
    {
        /* Check for USB enumeration with USB host */
        if(USBFS_VBusPresent() == TRUE)
		{
			if( USBFS_GetConfiguration() != FALSE)
	        {				
	            /* Accessory initialization routine after USB enumeration completion 
				 * Also loads the EP's when audio playback/recording starts */
				ServiceUSB();               
	            
	            /* USB audio sampling frequency change handler. */
	            HandleSamplingFrequencyChangeRequest(); 
				
				if(usbEnumFlag == FALSE)
				{
					usbEnumFlag = TRUE; 
					PRINT("USB Enumeration Done\r\n");
				}
	        }	
		}
		else
		{		
			/* Do not operate the device when VBUS is absent */			
			PRINT("USB Disconnected \r\n");	
			
			/* Wait till the USB is connected again */
			while(USBFS_VBusPresent() == FALSE);
			
			/* Do a software reset once VBUS is connected to properly reinitialize USB, DMA and I2S components */
			PRINT("USB Reconnected - SW reset \r\n");
            #ifdef TXDEBUG
			    while(UART_SpiUartGetTxBufferSize() != 0);
            #endif
            
            /* Do a software reset once data is sent over UART */
			CySoftwareReset();		
		}
        
		/* Clear audio IN buffer when IN stream is stopped */
		HandleAudioInBuffer();             
        
		/* Runs application layer code for CapSense */
		RunApplication(); 
    }
}

/* [] END OF FILE */
