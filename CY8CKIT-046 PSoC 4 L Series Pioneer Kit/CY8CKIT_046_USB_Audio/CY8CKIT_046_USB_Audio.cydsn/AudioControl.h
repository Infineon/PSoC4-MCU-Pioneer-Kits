/*******************************************************************************
* File Name: AudioControl.h
*
* Version 1.0
*
*  Description:  This file contains the Audio signal path configuration routine
*                declarations and constants.
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

#ifndef AUDIOCONTROL_H
	#define AUDIOCONTROL_H

	#include <Config.h>
	#include <cytypes.h>

	#define FREQUENCY_NOT_SET	0xFF

	#if(USBFS_EP_MM != USBFS__EP_DMAAUTO) 
		#define NUM_TDS 8
		#define TRANS_SIZE 96
		#define BUFSIZE (TRANS_SIZE*NUM_TDS)
		#define HALF (BUFSIZE/2)
	#else
		#define NUM_TDS 8
		#define OUT_TRANS_SIZE 144
		#define OUT_BUFSIZE (OUT_TRANS_SIZE*NUM_TDS)
		#define OUT_HALF (OUT_BUFSIZE/2)
		#define IN_TRANS_SIZE 144
		#define IN_BUFSIZE (IN_TRANS_SIZE*NUM_TDS)
		#define IN_HALF (IN_BUFSIZE/2)
	#endif

	#define AUDIOMAXPKT (IN_BUFSIZE/4)
	#define MAX_AUDIO_SAMPLE_SIZE                6	
	#define IN_AUDIOMAXPKT  (AUDIOMAXPKT + MAX_AUDIO_SAMPLE_SIZE)
	#define OUT_AUDIOMAXPKT (AUDIOMAXPKT + MAX_AUDIO_SAMPLE_SIZE)

	/* Clock Rates */
	#define RATE_48KHZ                                   0
	#define RATE_44KHZ                                   1	

	#define BUS_CLOCK_DIVIDE_BY_8                        0x07
	#define BUS_CLOCK_DIVIDE_BY_16                       0x0F
	#define BUS_CLOCK_DIVIDE_BY_2                        0x01

	#define ANALOG_AUX_INPUT                             0

	#define SAMPLING_RATE_48KHZ                          48000	
	#define SAMPLING_RATE_44KHZ                          44100

	#define FIFO_HALF_EMPTY_MASK                         0x0C
	
	extern uint8 audioClkConfigured;
	
	void InitAudioPath(void);
	void ProcessAudioIn(void);
	void SetClockRate(uint8 newRate);
	void Stop_I2S_Rx(void);	
	void Stop_I2S_Tx(void);	
	void HandleSamplingFrequencyChangeRequest(void);	
	void HandleAudioInBuffer(void);
#endif /* #ifndef AUDIOCONTROL_H */

/* [] END OF FILE */
