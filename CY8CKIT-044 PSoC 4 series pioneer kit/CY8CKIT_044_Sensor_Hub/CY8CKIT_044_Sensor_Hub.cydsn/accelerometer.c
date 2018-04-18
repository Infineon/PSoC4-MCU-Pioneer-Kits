/*****************************************************************************
* File Name: accelerometer.c
*
* Version 1.0
*
* Description:
*  This file provides functions to configure the onboard accelerometer of 
*  PSoC 4 M-Series Pioneer Kit. This file also defines function to read the acceleration
*  data from the accelerometer.
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
#include "accelerometer.h"
#include "project.h"


/*****************************************************************************
* Global Variable Declarations
*****************************************************************************/


/*******************************************************************************
* Function Name: Accelerometer_RegWrite
********************************************************************************
* Summary:
* 	Write a single byte to a specified register location.
*
* Parameters:
*  uint8 reg: Register to which data is to be written.
*  uint8 *value: Variable which holds the data to be written.
*
* Return:
*  uint8: Status of write operation
*
*******************************************************************************/
uint8 Accelerometer_RegWrite(uint8 reg, uint8 value)
{
	uint8 ackStatus = COMM_FAIL;
	ackStatus = I2C_I2CMasterSendStart(ACCELEROMETER_ADDR, ACC_WRITE, I2C_TIMEOUT_MS);
	if(ackStatus == I2C_I2C_MSTR_NO_ERROR)
	{
		ackStatus = I2C_I2CMasterWriteByte(reg, I2C_TIMEOUT_MS);
		ackStatus = I2C_I2CMasterWriteByte(value, I2C_TIMEOUT_MS);
	}
	else
	{
		if(stopDataAccess == FALSE)
		{
			/* Stop accessing the I2C if communication failed. */
			stopDataAccess = TRUE;
			displayErrorOnce = TRUE;
		}
	}
	ackStatus = I2C_I2CMasterSendStop(I2C_TIMEOUT_MS);
	if(ackStatus != I2C_I2C_MSTR_NO_ERROR)
	{
		if(stopDataAccess == FALSE)
		{
			/* Stop accessing the I2C if communication failed. */
			stopDataAccess = TRUE;
			displayErrorOnce = TRUE;
		}
	}
	return(ackStatus);
}

/*******************************************************************************
* Function Name: Accelerometer_RegRead
********************************************************************************
* Summary:
* 	Read a single byte from a specified register location.
*
* Parameters:
*  uint8 reg: Register from where data is to be read.
*  uint8 *value: Variable to which the data from register is stored.
*
* Return:
*  uint8: Status of read operation
*
*******************************************************************************/
uint8 Accelerometer_RegRead(uint8 reg, uint8 *value)
{
	uint8 ackStatus = COMM_FAIL;
	ackStatus = I2C_I2CMasterSendStart(ACCELEROMETER_ADDR, ACC_WRITE, I2C_TIMEOUT_MS);
	if(ackStatus == I2C_I2C_MSTR_NO_ERROR)
	{
		ackStatus = I2C_I2CMasterWriteByte(reg, I2C_TIMEOUT_MS);
		ackStatus = I2C_I2CMasterSendRestart(ACCELEROMETER_ADDR, ACC_READ, I2C_TIMEOUT_MS);
		I2C_I2CMasterReadByte(I2C_I2C_NAK_DATA, value, I2C_TIMEOUT_MS);
	}
	else
	{
		if(stopDataAccess == FALSE)
		{
			/* Stop accessing the I2C if communication failed. */
			stopDataAccess = TRUE;
			displayErrorOnce = TRUE;
		}
	}
	ackStatus = I2C_I2CMasterSendStop(I2C_TIMEOUT_MS);
	if(ackStatus != I2C_I2C_MSTR_NO_ERROR)
	{
		if(stopDataAccess == FALSE)
		{
			/* Stop accessing the I2C if communication failed. */
			stopDataAccess = TRUE;
			displayErrorOnce = TRUE;
		}
	}
	return(ackStatus);
}

/*******************************************************************************
* Function Name: Accelerometer_SelfTest
********************************************************************************
* Summary:
* 	Runs a test routine to make sure that the accelerometer communication is 
*   functional.
*
* Parameters:
*  void
*
* Return:
*  uint8: Communication functional status.
*
*******************************************************************************/
uint8 Accelerometer_SelfTest(void)
{
	uint8 accStatus = ACC_FAIL;
	uint8 accRegValue = ZERO;
	uint8 commStatus = COMM_FAIL;
	
	/* Read the initial value of the DCST register. This must be 0x55 at reset. */
	commStatus = Accelerometer_RegRead(ACC_DCST_RESP, &accRegValue);
	if(commStatus == COMM_PASS && accRegValue == ACC_SELF_TEST_VAL_1)
	{
		/* Set the PC1 bit of the CTRL_1 register of accelerometer to 0. 
		 * This puts the accelerometer in standby mode. */
		commStatus = Accelerometer_RegRead(ACC_CTRL_REG_1, &accRegValue);
		accRegValue = accRegValue & ACC_PC1_BIT_AND_MASK;
		commStatus = Accelerometer_RegWrite(ACC_CTRL_REG_1, accRegValue);
		
		/* Set the DCST bit of CTRL_2 register of the accelerometer to 
		 * 1 for self-test. */
		commStatus = Accelerometer_RegRead(ACC_CTRL_REG_2, &accRegValue);
		accRegValue = accRegValue | ACC_DCST_BIT_OR_MASK;
		commStatus = Accelerometer_RegWrite(ACC_CTRL_REG_2, accRegValue);
		
		/* Read the DCST value again. This must be 0xAA after the DCST bit
		 * is set to 1 in the CTRL_2 register. */
		commStatus = Accelerometer_RegRead(ACC_DCST_RESP, &accRegValue);
		if(commStatus == COMM_PASS && accRegValue == ACC_SELF_TEST_VAL_2)
		{
			/* Read the DCST value again. This must be 0x55 after the 
			 * register is read once. */
			commStatus = Accelerometer_RegRead(ACC_DCST_RESP, &accRegValue);
			if(commStatus == COMM_PASS && accRegValue == ACC_SELF_TEST_VAL_1)
			{
				accStatus = ACC_PASS;
			}
		}
	}
	return(accStatus);
}

/*******************************************************************************
* Function Name: InitializeAccelerometer
********************************************************************************
* Summary:
* 	Initializes the accelerometer control registers for proper operation.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitializeAccelerometer(void)
{
	uint8 accWorkStatus = ACC_FAIL;
	uint8 accRegValue = ZERO;
	uint8 commStatus = COMM_FAIL;
	
	/* Ensure proper functioning of hardware */
	accWorkStatus = Accelerometer_SelfTest();
	
	
	if(accWorkStatus == ACC_PASS)
	{
		/* Initialize the CTRL_REG1 after putting the accelerometer in standby mode. */
		accRegValue = accRegValue & ACC_PC1_BIT_AND_MASK;
		commStatus = Accelerometer_RegWrite(ACC_CTRL_REG_1, accRegValue);
		
		if(commStatus == COMM_PASS)
		{
			/* Modify the current value for the following settings:
			 * RES (Resolution) 								= 12-bit mode
			 * DRDYE (Report acceleration via Interrupt) 		= ENABLED
			 * GSEL1, GSEL0 (Acceleration range of the output) 	= +/-2g
			 * WUFE (Wake Up on Motion Detect) 					= ENABLED
			 */
			accRegValue = accRegValue | ACC_RES_BIT_OR_MASK | ACC_WUFE_BIT_OR_MASK | ACC_DRDYE_BIT_OR_MASK;
			accRegValue = accRegValue & ACC_GSEL0_BIT_AND_MASK & ACC_GSEL1_BIT_AND_MASK;
			
			/* Write back the modified value. */
			commStatus = Accelerometer_RegWrite(ACC_CTRL_REG_1, accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Initialize the CTRL_REG2 */
			/* Read the current value of the CTRL_REG2 register */
			commStatus = Accelerometer_RegRead(ACC_CTRL_REG_2, &accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Modify the current value for the following settings:
			 * SRST (Software Reset)							= FALSE
			 * DCST (Digital Communication Self Test)	 		= DISABLED
			 * OWUFA,OWUFB,OWUFC(Output Data Rate for Wakeup)	= 12.5 Hz
			 */
			accRegValue = accRegValue | ACC_OWUFA_BIT_OR_MASK;
			accRegValue = accRegValue & ACC_SRST_BIT_AND_MASK & ACC_DCST_BIT_AND_MASK & ACC_OWUFB_BIT_AND_MASK & ACC_OWUFC_BIT_AND_MASK;

			/* Write back the modified value. */
			commStatus = Accelerometer_RegWrite(ACC_CTRL_REG_2, accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Read the current value of the INT_CTRL_REG1 register */
			commStatus = Accelerometer_RegRead(ACC_INT_CTRL_REG_1, &accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Modify the current value for the following settings:
			 * IEN (Physical Interrupt Pin)						= ENABLED
			 * IEA (Interrupt Polarity)					 		= ACTIVE HIGH
			 * IEL (Interrupt Latch/Pulse Enable)				= LATCH INTERRUPT
			 */
			accRegValue = accRegValue | ACC_IEN_BIT_OR_MASK | ACC_IEA_BIT_OR_MASK;
			accRegValue = accRegValue & ACC_IEL_BIT_AND_MASK;
			
			/* Write back the modified value. */
			commStatus = Accelerometer_RegWrite(ACC_INT_CTRL_REG_1, accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Read the current value of the INT_CTRL_REG2 register */
			commStatus = Accelerometer_RegRead(ACC_INT_CTRL_REG_2, &accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Modify the current value for the following settings:
			 * XNWU, XPWU, YNWU, YPWU, ZNWU, ZPWU (Direction)	= SENSING_ENABLED
			 */
			accRegValue = accRegValue | ACC_XNWUE_BIT_OR_MASK | ACC_XPWUE_BIT_OR_MASK | \
			ACC_YNWUE_BIT_OR_MASK | ACC_YPWUE_BIT_OR_MASK | ACC_ZNWUE_BIT_OR_MASK | ACC_ZPWUE_BIT_OR_MASK;
			
			/* Write back the modified value. */
			commStatus = Accelerometer_RegWrite(ACC_INT_CTRL_REG_2, accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Read the current value of the DATA_CTRL register */
			commStatus = Accelerometer_RegRead(ACC_DATA_CTRL_REG, &accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Modify the current value for the following settings:
			 * OSAA, OSAB, OSAC, OSAD (Output Data Rate)		= 50 Hz
			 */
			accRegValue = accRegValue | ACC_OSAC_BIT_OR_MASK;
			accRegValue = accRegValue & ACC_OSAA_BIT_AND_MASK & ACC_OSAB_BIT_AND_MASK & ACC_OSAD_BIT_AND_MASK;
			
			/* Write back the modified value. */
			commStatus = Accelerometer_RegWrite(ACC_DATA_CTRL_REG, accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Read the current value of the WAKEUP_TIMER register */
			commStatus = Accelerometer_RegRead(ACC_WAKEUP_TIMER, &accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Modify the current value for the following settings:
			 * Wakeup Timer value								= 1 * 50 = 50 counts
			 * WAKEUP_TIMER (counts) = Desired Delay Time (sec) x OWUF (Hz) 
			 */
			accRegValue = ACC_WAKEUP_TIMER_VALUE;
			
			/* Write back the modified value. */
			commStatus = Accelerometer_RegWrite(ACC_WAKEUP_TIMER, accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Read the current value of the ACC_WAKUP_THRESHOLD register */
			commStatus = Accelerometer_RegRead(ACC_WAKUP_THRESHOLD, &accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Modify the current value for the following settings:
			 * Wakeup Timer value								= 0.5 * 16 = 8counts
			 * WAKEUP_THRESHOLD (counts) = Desired Threshold (g) x 16 (counts/g) 
			 */
			accRegValue = ACC_WAKUP_THRESHOLD_VAL;
			
			/* Write back the modified value. */
			commStatus = Accelerometer_RegWrite(ACC_WAKUP_THRESHOLD, accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Read the current value of the CTRL_REG1 register */
			commStatus = Accelerometer_RegRead(ACC_CTRL_REG_1, &accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Put the Accelerometer in the Operating Mode */
			accRegValue = accRegValue | ACC_PC1_BIT_OR_MASK;
					
			/* Write back the modified value. */
			commStatus = Accelerometer_RegWrite(ACC_CTRL_REG_1, accRegValue);
		}
		else
		{
			commStatus = COMM_FAIL;
		}
		
		if(commStatus == COMM_PASS)
		{
			/* Set the flag to indicate the accelerometer is configured. */
			acclerometer_configured = TRUE;
			
			/* Clear all pending interrupts from accelerometer. */
			Accelerometer_RegRead(ACC_INT_REL, &accRegValue);
		}
	}	
}

/*******************************************************************************
* Function Name: ReadAccelerometer
********************************************************************************
* Summary:
* 	Reads the accelerometer acceleration values in x, y, and z directions.
*
* Parameters:
*  int16 *xData: Pointer to the variable in which the acceleration data 
*                for x-direction is to be stored. 
*
*  int16 *yData: Pointer to the variable in which the acceleration data 
*                for y-direction is to be stored. 
*
*  int16 *zData: Pointer to the variable in which the acceleration data 
*                for z-direction is to be stored. 
*
* Return:
*  void
*
*******************************************************************************/
void ReadAccelerometer(int16 *xData, int16 *yData, int16 *zData)
{
	uint8 tempVal = ZERO, commStatus = COMM_FAIL;
	int16 x_Val = ZERO, y_Val = ZERO, z_Val = ZERO;
	
	commStatus = Accelerometer_RegRead(ACC_XOUT_H, &tempVal);
	
	if(commStatus != COMM_FAIL)
	{
		x_Val = tempVal;
		commStatus = Accelerometer_RegRead(ACC_XOUT_L, &tempVal);
	}
	
	if(commStatus != COMM_FAIL)
	{
		x_Val = (x_Val << 8) | tempVal;
		if((x_Val & 0x8000) == 0x8000)
		{
			x_Val = x_Val >> 4;
			x_Val |= 0xF000;
		}
		else
		{
			x_Val = x_Val >> 4;
		}
		*xData = x_Val;
		commStatus = Accelerometer_RegRead(ACC_YOUT_H, &tempVal);
	}
	
	if(commStatus != COMM_FAIL)
	{
		y_Val = tempVal;
		commStatus = Accelerometer_RegRead(ACC_YOUT_L, &tempVal);
	}
		
	if(commStatus != COMM_FAIL)
	{
		y_Val = (y_Val << 8) | tempVal;
		
		if((y_Val & 0x8000) == 0x8000)
		{
			y_Val = y_Val >> 4;
			y_Val |= 0xF000;
		}
		else
		{
			y_Val = y_Val >> 4;
		}
		*yData = y_Val;
		commStatus = Accelerometer_RegRead(ACC_ZOUT_H, &tempVal);
	}
	
	if(commStatus != COMM_FAIL)
	{
		z_Val = tempVal;
		commStatus = Accelerometer_RegRead(ACC_ZOUT_L, &tempVal);
	}
	if(commStatus != COMM_FAIL)
	{
		z_Val = (z_Val << 8) | tempVal;
		
		if((z_Val & 0x8000) == 0x8000)
		{
			z_Val = z_Val >> 4;
			z_Val |= 0xF000;
		}
		else
		{
			z_Val = z_Val >> 4;
		}
		*zData = z_Val;
	}
}

/* [] END OF FILE */

