/*******************************************************************************
* File Name: Codec.c
*
* Version 1.0
*
*  Description: This file contains the codec control APIs
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

#include "Codec.h"
#include "cytypes.h"
#include "CodecI2CM_I2C.h"
#include "stdbool.h"

#define I2C_WRITE_OPERATION		(0x00)

static uint16 Codec_analogPathSetting = CODEC_DEF_ANALOG_CTRL;
uint32 Codec_powerControlSetting = CODEC_DEF_POWER_CTRL;

/*******************************************************************************
* Function Name: Codec_Init
********************************************************************************
* Summary:
*   Initializes the codec with default settings.
*
*
* Parameters:  
*	None
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_Init(void)
{
	uint8 ret;
	
	ret = Codec_ResetOverI2C();
	CyDelay(CODEC_RESET_WAIT_DELAY);
	
	ret = Codec_AdjustBothHeadphoneVolume(CODEC_HP_DEFAULT_VOLUME);
	ret = Codec_SendData(CODEC_REG_ANALOG_CTRL, CODEC_DEF_ANALOG_CTRL);
	ret = Codec_SendData(CODEC_REG_DIGITAL_CTRL, CODEC_DEF_DIGITAL_CTRL);
	ret = Codec_SendData(CODEC_REG_POWER_CTRL, CODEC_DEF_POWER_CTRL);
	ret = Codec_SetSamplingRate(CODEC_DEF_SAMPLING_RATE);
	
	return ret;  
}

/*******************************************************************************
* Function Name: Codec_SendData
********************************************************************************
* Summary:
*   Low level API to send data to codec over I2C.
*
*
* Parameters:  
*	regAddr - Address of the codec register to be updated
*	data - 16-bit data to be updated in the register
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_SendData(uint8 regAddr, uint16 data)
{
	uint8 temp;
	
	temp = CodecI2CM_I2CMasterSendStart(CODEC_I2C_ADDR, I2C_WRITE_OPERATION,0); 
	if(temp == CodecI2CM_I2C_MSTR_NO_ERROR)
	{
		temp = CodecI2CM_I2CMasterWriteByte(((regAddr << 0x01) | (HI8(data) & 0x01)),0);
		
		if(temp == CodecI2CM_I2C_MSTR_NO_ERROR)
		{
			temp = CodecI2CM_I2CMasterWriteByte(LO8(data),0);
			
			if(temp == CodecI2CM_I2C_MSTR_NO_ERROR)
			{
				temp = CodecI2CM_I2CMasterSendStop(0);
			}
		}
	}

	return temp;
}

/*******************************************************************************
* Function Name: Codec_SetSamplingRate
********************************************************************************
* Summary:
*   This function sets the sampling rate of the codec. Note that the codec must be deactivated 
*	before configuring the sample rate.
*
*
* Parameters:  
*	srCtrlField - Sampling control register settings 
*					
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_SetSamplingRate(uint8 srCtrlField)
{		
	/* This function modifies the BOSR & SR bits of sampling rate control register.
     * srCtrlField : bit[0] - BOSR, bit[1:4] - SR.
     */
    return Codec_SendData(CODEC_REG_SAMPLING_CTRL, srCtrlField);;
}

/*******************************************************************************
* Function Name: Codec_AdjustBothHeadphoneVolume
********************************************************************************
* Summary:
*   This function updates the volume of both the left and right channels of the
* 	headphone output.
*
*
* Parameters:  
*	volume - 0 - Mute
*			 1 to CODEC_HP_VOLUME_MAX levels
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/

uint8 Codec_AdjustBothHeadphoneVolume(uint8 volume)
{
	if(volume > CODEC_HP_VOLUME_MAX)
	{
		volume = CODEC_HP_VOLUME_MAX;
	}
	
	return Codec_SendData(CODEC_REG_LHPOUT, (volume + (CODEC_LHPOUT_BOTH + CODEC_LHPOUT_LZCEN + CODEC_HP_MUTE_VALUE)));
}

/*******************************************************************************
* Function Name: Codec_MuteMic
********************************************************************************
* Summary:
*   Mutes/unmutes the microphone input
*
*
* Parameters:  
*	isMuteOrUnmute - 0 - Unmute
*					 1 - Mute
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_MuteMic(bool isMuteOrUnmute)
{
	Codec_analogPathSetting &= ~(CODEC_ANALOG_CTRL_MUTEMIC);
	
	if(isMuteOrUnmute)
	{
		Codec_analogPathSetting |= CODEC_ANALOG_CTRL_MUTEMIC;		
	}
	
	return Codec_SendData(CODEC_REG_ANALOG_CTRL, Codec_analogPathSetting);
}

/*******************************************************************************
* Function Name: Codec_SetMicBoost
********************************************************************************
* Summary:
*   Enables/Disables microphone boost (amplification)
*
*
* Parameters:  
*	micBoost -       0 - Disable
*					 1 - Enable
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_SetMicBoost(bool micBoost)
{
	Codec_analogPathSetting &= ~(CODEC_ANALOG_CTRL_MICBOOST);
	
	if(micBoost)
	{
		Codec_analogPathSetting |= CODEC_ANALOG_CTRL_MICBOOST;		
	}
	
	return Codec_SendData(CODEC_REG_ANALOG_CTRL, Codec_analogPathSetting);
}

/*******************************************************************************
* Function Name: Codec_Activate
********************************************************************************
* Summary:
*   Activates the codec - This function is called in conjunction with Codec_Deactivate API
*	after successful configuration update of the codec.
*
*
* Parameters:  
*	None
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_Activate(void)
{
	return Codec_SendData(CODEC_REG_ACTIVATE, CODEC_CTRL_ACTIVATE); 
}

/*******************************************************************************
* Function Name: Codec_Deactivate
********************************************************************************
* Summary:
*   Deactivates the CODEC - the configuration is retained, just the CODEC input/outputs are
*	disabled. The function should be called before changing any setting in the codec over I2C
*
*
* Parameters:  
*	None
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_Deactivate(void)
{
	return Codec_SendData(CODEC_REG_ACTIVATE, CODEC_CTRL_DEACTIVATE); 
}

/*******************************************************************************
* Function Name: Codec_ResetOverI2C
********************************************************************************
* Summary:
*   Resets the codec by sending an I2C command
*
* Parameters:  
*	None
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_ResetOverI2C(void)
{
	return Codec_SendData(CODEC_REG_RESET, CODEC_CTRL_RESET);
}
	
/*******************************************************************************
* Function Name: Codec_PowerOffControl
********************************************************************************
* Summary:
*   Disables power for various blocks in the codec
*
*
* Parameters:  
*	powerOffMask - Bit(s) mask for the power off control 
*                   Refer to bit settings available for CODEC_REG_POWER_CTRL in 
*                   Codec.h
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_PowerOffControl(uint32 powerOffMask)
{
	Codec_powerControlSetting |= powerOffMask;
    return Codec_SendData(CODEC_REG_POWER_CTRL, Codec_powerControlSetting);
}

/*******************************************************************************
* Function Name: Codec_PowerOnControl
********************************************************************************
* Summary:
*   Enables power for various blocks in the codec
*
*
* Parameters:  
*	powerOffMask - Bit(s) mask for the power on control 
*                   Refer to bit settings available for CODEC_REG_POWER_CTRL in 
*                   Codec.h
*
* Return:
*   uint8 - I2C master transaction error status
*				CodecI2CM_I2C_MSTR_NO_ERROR - Function complete without error                       
*				CodecI2CM_I2C_MSTR_ERR_ARB_LOST - Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
*				CodecI2CM_I2C_MSTR_ERR_LB_NAK - Last Byte NACKed: INTR_MASTER_I2C_NACK               
*				CodecI2CM_I2C_MSTR_NOT_READY - Master on the bus or Slave operation is in progress  
*				CodecI2CM_I2C_MSTR_BUS_BUSY - Bus is busy, process not started
*				CodecI2CM_I2C_MSTR_ERR_ABORT_START - Slave was addressed before master begin Start
*				CodecI2CM_I2C_MSTR_ERR_BUS_ERR - Bus error has: INTR_MASTER_I2C_BUS_ERROR 
*
*******************************************************************************/
uint8 Codec_PowerOnControl(uint32 powerOnMask)
{
	Codec_powerControlSetting &= ~powerOnMask;
    return Codec_SendData(CODEC_REG_POWER_CTRL, Codec_powerControlSetting);
}
/* [] END OF FILE */
