/*****************************************************************************
* File Name		: Codec.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  Codec.c
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

#ifndef CODEC_H
	#define CODEC_H	

	#include "cytypes.h"
		
	#define CODEC_I2C_ADDR			(0x1A)


	/**************************************************************************************************
	* Register Addresses for Codec I2C Interface
	**************************************************************************************************/

	#define CODEC_REG_LLIN        	(0x00)    /* Left channel line input volume & mute control register */
	#define CODEC_REG_RLIN        	(0x01)    /* Right channel line input volume & mute control register */
	#define CODEC_REG_LHPOUT     	(0x02)    /* Left channel headphone output volume control register */
	#define CODEC_REG_RHPOUT  	   	(0x03)    /* Right channel headphone output volume control register */
	#define CODEC_REG_ANALOG_CTRL  	(0x04)    /* Analog audio path control register */
	#define CODEC_REG_DIGITAL_CTRL 	(0x05)    /* Digital audio path control register */
	#define CODEC_REG_POWER_CTRL   	(0x06)    /* Power down control register */
	#define CODEC_REG_DIGITAL_IF   	(0x07)    /* Digital audio interface format register */
	#define CODEC_REG_SAMPLING_CTRL	(0x08)    /* Sampling control register */
	#define CODEC_REG_ACTIVATE  	(0x09)    /* Activate/Deactivate register */
	#define CODEC_REG_RESET        	(0x0F)    /* Reset register */

	/* Register bit settings for CODEC_REG_LLIN register */	
	#define CODEC_LLIN_BOTH			0x100
	#define CODEC_LLIN_MUTE			0x080
	#define CODEC_LLIN_VOL_MASK		0x01F
	
	/* Register bit settings for CODEC_REG_RLIN register */	
	#define CODEC_RLIN_BOTH			0x100
	#define CODEC_RLIN_MUTE			0x080
	#define CODEC_RLIN_VOL_MASK		0x01F
	
	/* Register bit settings for CODEC_REG_LHPOUT register */	
	#define CODEC_LHPOUT_BOTH		0x100
	#define CODEC_LHPOUT_LZCEN		0x080
	#define CODEC_LHPOUT_VOL_MASK	0x07F
	
	/* Register bit settings for CODEC_REG_RHPOUT register */	
	#define CODEC_RHPOUT_BOTH		0x100
	#define CODEC_RHPOUT_LZCEN		0x080
	#define CODEC_RHPOUT_VOL_MASK	0x07F
	
	/* Register bit settings for CODEC_REG_ANALOG_CTRL register */	
	#define CODEC_ANALOG_CTRL_SIDEATT_MASK		0xC0
	#define CODEC_ANALOG_CTRL_SIDEATT_m15_DB	0xC0
	#define CODEC_ANALOG_CTRL_SIDEATT_m12_DB	0x80
	#define CODEC_ANALOG_CTRL_SIDEATT_m9_DB		0x40
	#define CODEC_ANALOG_CTRL_SIDEATT_m6_DB		0x00
	#define CODEC_ANALOG_CTRL_SIDETONE			0x20
	#define CODEC_ANALOG_CTRL_DACSEL			0x10
	#define CODEC_ANALOG_CTRL_BYPASS			0x08
	#define CODEC_ANALOG_CTRL_INSEL				0x04
	#define CODEC_ANALOG_CTRL_MUTEMIC			0x02
	#define CODEC_ANALOG_CTRL_MICBOOST			0x01
	
	/* Register bit settings for CODEC_REG_DIGITAL_CTRL register */	
	#define CODEC_DIGITAL_CTRL_HPOR			0x10
	#define CODEC_DIGITAL_CTRL_DACMU		0x08
	#define CODEC_DIGITAL_CTRL_DEEMPH_MASK	0x06
	#define CODEC_DIGITAL_CTRL_DEEMPH_48K	0x06
	#define CODEC_DIGITAL_CTRL_DEEMPH_44K	0x04
	#define CODEC_DIGITAL_CTRL_DEEMPH_32K	0x02
	#define CODEC_DIGITAL_CTRL_DEEMPH_OFF	0x00
	#define CODEC_DIGITAL_CTRL_ADCHPD		0x01
	
	/* Register bit settings for CODEC_REG_POWER_CTRL register */	
	#define CODEC_POWER_CTRL_POWEROFF		0x80
	#define CODEC_POWER_CTRL_CLKOUTPD		0x40
	#define CODEC_POWER_CTRL_OSCPD			0x20
	#define CODEC_POWER_CTRL_OUTPD			0x10
	#define CODEC_POWER_CTRL_DACPD			0x08
	#define CODEC_POWER_CTRL_ADCPD			0x04
	#define CODEC_POWER_CTRL_MICPD			0x02
	#define CODEC_POWER_CTRL_LINEINPD		0x01
		
	/* Register bit settings for CODEC_REG_DIGITAL_IF register */	
	#define CODEC_DIGITAL_IF_BCLKINV		0x80
	#define CODEC_DIGITAL_IF_MS				0x40
	#define CODEC_DIGITAL_IF_LRSWAP			0x20
	#define CODEC_DIGITAL_IF_LRP			0x10
	#define CODEC_DIGITAL_IF_IWL_MASK		0x0C
	#define CODEC_DIGITAL_IF_IWL_32_BIT		0x0C
	#define CODEC_DIGITAL_IF_IWL_24_BIT		0x08
	#define CODEC_DIGITAL_IF_IWL_20_BIT		0x04
	#define CODEC_DIGITAL_IF_IWL_16_BIT		0x00
	#define CODEC_DIGITAL_IF_FORMAT_MASK	0x03
	#define CODEC_DIGITAL_IF_FORMAT_DSP		0x03
	#define CODEC_DIGITAL_IF_FORMAT_I2S		0x02
	#define CODEC_DIGITAL_IF_FORMAT_LEFT	0x01
	#define CODEC_DIGITAL_IF_FORMAT_RIGHT	0x00
	
	/* Register bit settings for CODEC_REG_SAMPLING_CTRL register */	
	#define CODEC_SAMPLING_CTRL_CLKODIV2		0x80
	#define CODEC_SAMPLING_CTRL_CLKIDIV2		0x40
	#define CODEC_SAMPLING_CTRL_SR_MASK			0x3C
	#define CODEC_SAMPLING_CTRL_BOSR			0x02
	#define CODEC_SAMPLING_CTRL_MODE			0x01
	
	/* Register bit settings for CODEC_REG_ACTIVE_CTRL register */	
	#define CODEC_ACTIVE_CTRL_ACTIVE			0x01
			
	/* Register settings for the Codec chip. Defined as 9-bit value */
	#define CODEC_CTRL_MUTEMIC_MASK				(0x02)
	#define CODEC_CTRL_ACTIVATE					(0x01)
	#define CODEC_CTRL_DEACTIVATE				(0x00)
	#define CODEC_CTRL_RESET					(0x00)
	#define CODEC_CTRL_LRHPBOTH					(0x0100)
	#define CODEC_CTRL_LZCEN					(0x80)
	#define CODEC_CTRL_MIC_INSEL				(0x04)	

	#define CODEC_RESET_WAIT_DELAY				(10)	 /* in milli seconds */

	#define CODEC_HP_DEFAULT_VOLUME				(0)
	#define CODEC_HP_VOLUME_MAX					(80) 	/* 81 levels including MUTE */	
	#define CODEC_HP_MUTE_VALUE					(0x2F) 	/* Writing <= 0x2F mutes the headphone output */
		
	/* Value format: bit[0] - BOSR, bit[4:1] - SR[3:0] */	
	#define CODEC_SRATE_NORMAL_48KHZ_256FS		(0x00) 	/* BOSR = 0 for 256fs and SRx = 0b0000 for 48 KHz in Normal mode */
	#define CODEC_SRATE_NORMAL_44KHZ_256FS		(0x20) 	/* BOSR = 0 for 256fs and SRx = 0b1000 for 44.1 KHz in Normal mode */
		
	#define CODEC_DEF_SAMPLING_RATE				CODEC_SRATE_NORMAL_48KHZ_256FS
	#define CODEC_DEF_ANALOG_CTRL				(CODEC_ANALOG_CTRL_INSEL | CODEC_ANALOG_CTRL_DACSEL | CODEC_ANALOG_CTRL_SIDETONE)
	#define CODEC_DEF_DIGITAL_CTRL				(CODEC_DIGITAL_CTRL_HPOR)
	#define CODEC_DEF_POWER_CTRL				(CODEC_POWER_CTRL_CLKOUTPD | CODEC_POWER_CTRL_OSCPD | CODEC_POWER_CTRL_LINEINPD)
		
	uint8 Codec_Init(void);
	uint8 Codec_SelectMicInputToADC(void);
	uint8 Codec_AdjustBothHeadphoneVolume(uint8 volume);
	uint8 Codec_MuteMic(_Bool isMuteOrUnmute);
    uint8 Codec_SetMicBoost(_Bool micBoost);
	uint8 Codec_SetSamplingRate(uint8 srCtrlField);
	uint8 Codec_Activate(void);
	uint8 Codec_Deactivate(void);
	uint8 Codec_ResetOverI2C(void);
	uint8 Codec_SendData(uint8 regAddr, uint16 data);
    uint8 Codec_PowerOffControl(uint32 powerOffMask);
    uint8 Codec_PowerOnControl(uint32 powerOnMask);
		
	
#endif /* #ifndef CODEC_H */

/* [] END OF FILE */
