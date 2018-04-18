/*****************************************************************************
* File Name		: FRAM_Control.c
* Version		: 1.0 
*
* Description:
*  This file defines the functions to communicate with the onboard F-RAM of 
*  CY8CKIT-044 PSoC 4 M-Series Pioneer Kit.
*
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
#include "project.h"
#include "FRAM_Control.h"
#include "stdio.h"

extern int16 uC_Probe_ambientLight;
extern int8 uC_Probe_temperatureInteger;
extern uint8 temperatureFractional;
extern int16 uC_Probe_x_dirMovement;
extern int16 uC_Probe_y_dirMovement;
extern int16 uC_Probe_z_dirMovement;
extern uint8 uC_Probe_Seconds;
extern uint8 uC_Probe_Minutes;
extern uint8 uC_Probe_Hours;

uint8 stopDataAccess = FALSE;

/*******************************************************************************
* Function Name: FRAM_SaveSensordata
********************************************************************************
* Summary:
*  Saves the most recent sensor data values in F-RAM.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void FRAM_SaveSensorData(void)
{
	uint8 writeBuf[WRITE_BUF_SIZE] = {ZERO};
	uint32 time;
	static uint16 locFRAM = ZERO;
	static uint8 addrFRAM = FRAM_PAGE1_ADDR;
		
	if(stopDataAccess == FALSE)
	{
		/* First two bytes of I2C transfer to F-RAM indicates the location where 
		 * data is to be written. The subsequent bytes of the I2C transfer is the 
		 * actual data to be stored in the F-RAM. */
		writeBuf[0] = HI8(locFRAM);
		writeBuf[1] = LO8(locFRAM);
		
		/* Timestamp is stored in 1st, 2nd and 3rd bytes */
		time = RTC_GetTime();
		uC_Probe_Seconds = RTC_GetSecond(time);
		uC_Probe_Minutes = RTC_GetMinutes(time);
		uC_Probe_Hours = RTC_GetHours(time);
		
		writeBuf[2] = (uint8) uC_Probe_Hours;
		writeBuf[3] = (uint8) uC_Probe_Minutes;
		writeBuf[4] = (uint8) uC_Probe_Seconds;
		
		
		/* Temperature Sensor Data stored in the 4th and 5th
		 * locations with respect to the offset. */
		writeBuf[5] = uC_Probe_temperatureInteger;
		writeBuf[6] = temperatureFractional;
		
		/* Accelerometer x-direction Data Stored in the 6th and
		 * 7th locations with respect to the offset.*/
		writeBuf[7] = HI8(uC_Probe_x_dirMovement);
		writeBuf[8] = LO8(uC_Probe_x_dirMovement);
		
		/* Accelerometer y-direction Data Stored in the 8th and
		 * 9th locations with respect to the offset. */
		writeBuf[9] = HI8(uC_Probe_y_dirMovement);
		writeBuf[10] = LO8(uC_Probe_y_dirMovement);
		
		/* Accelerometer z-direction Data Stored in the 10th and
		 * 11th locations with respect to the offset. */
		writeBuf[11] = HI8(uC_Probe_z_dirMovement);
		writeBuf[12] = LO8(uC_Probe_z_dirMovement);
		
		/* Update the offset value. */
		locFRAM = locFRAM + FRAM_PACKET_SIZE;
		
		/* If complete information including RTC data and sensor data
		 * cannot be stored within the page boundary, switch pages. */
		if(locFRAM >= FRAM_PAGE_BOUNDARY)
		{
			locFRAM = ZERO; 
			if(addrFRAM == FRAM_PAGE1_ADDR)
			{
				/* If first page of F-RAM is full, switch to second page. */
				addrFRAM = FRAM_PAGE2_ADDR;
			}
			else
			{
				/* If second page of F-RAM is full, switch to first page. */
				addrFRAM = FRAM_PAGE1_ADDR;
			}
		}
		
		/* Write data bytes to F-RAM. */
		I2C_I2CMasterWriteBuf(addrFRAM, writeBuf, WRITE_BUF_SIZE, I2C_I2C_MODE_COMPLETE_XFER);
		
		while(0u == (I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT))
	    {
	        /* Wait until master complete write. If the Master lost arbitration, stop waiting. */
			if((I2C_I2CMasterStatus() & I2C_I2C_MSTAT_ERR_ARB_LOST) == I2C_I2C_MSTAT_ERR_ARB_LOST)
			{
				/* Flag to indicate that the data logging will not be 
				 * continued until a device reset happens. */
				stopDataAccess = TRUE;
				displayErrorOnce = TRUE;
				break;
			}
	    }
	}
}

/*******************************************************************************
* Function Name: UART_WriteAsciiData
********************************************************************************
* Summary:
*  Converts an array of data bytes to ASCII and sends through PSoC 4200M UART.
*
* Parameters:
*  uint8 *dataArray: Array of data bytes to be sent through UART.
*  uint8 length: Number of data bytes to be sent.
*  uint8 data_format: Format in which the data should be sent (Hex or Decimal).
*
* Return:
*  void
*
*******************************************************************************/
void UART_WriteAsciiData(uint8 *dataArray, uint8 length, uint8 data_format)
{
	uint8 index = ZERO;
	char firstChar = ' ';
	char secondChar = ' ';
	
	/* Repeat till end of array. */
	for(index = ZERO; index < length; index++)
	{
		/* Extract the first digit. */
		firstChar = dataArray[index]/data_format;
		if(firstChar < TEN)
		{
			/* Add 0x30 if value is less than 10. */
			firstChar = firstChar + ASCII_DEC_OFFSET;
		}
		else
		{
			/* Add 0x37 values greater than 9 and less than 16. */
			firstChar = firstChar + ASCII_HEX_OFFSET;
		}
		
		/* Extract the second digit. */
		secondChar = dataArray[index]%data_format;
		if(secondChar < TEN)
		{
			/* Add 0x30 if value is less than 10. */
			secondChar = secondChar + ASCII_DEC_OFFSET;
		}
		else
		{
			/* Add 0x37 values greater than 9 and less than 16. */
			secondChar = secondChar + ASCII_HEX_OFFSET;
		}
		
		/* Send data bytes through UART. */
		UART_Comm_SpiUartWriteTxData(firstChar);
		UART_Comm_SpiUartWriteTxData(secondChar);
		if(data_format == HEX_FORMAT)
		{
			UART_Comm_UartPutChar(' ');
		}
	}
}

/*******************************************************************************
* Function Name: FRAM_ReadSensordata
********************************************************************************
* Summary:
*  Reads the most recent sensor data values from F-RAM and sends it through 
*  PSoC 4200M SCB UART.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void FRAM_ReadSensorData(void)
{
	uint8 readBuf[READ_BUF_SIZE] = {ZERO};
	uint8 writeBuf[ADDRESS_OFFSET] = {ZERO};
	
	char uartString[MAX_STRING_SIZE] = {' '};
	char firstChar = ' ';
	char secondChar = ' ';
	
	static uint16 locFRAM = ZERO;
	static uint8 addrFRAM = FRAM_PAGE1_ADDR;
	
	if(stopDataAccess == FALSE)
	{
		/* First two bytes of I2C transfer to F-RAM indicates the location from 
		 * where data bytes are to be read. The subsequent I2C transfer read the
		 * actual data stored in the F-RAM. */
		writeBuf[0] = HI8(locFRAM);
		writeBuf[1] = LO8(locFRAM);
		
		/* Write the offset address of F-RAM. */
		I2C_I2CMasterWriteBuf(addrFRAM, writeBuf, ADDRESS_OFFSET, I2C_I2C_MODE_NO_STOP);
		
		while(0u == (I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT))
	    {
	        /* Wait until master complete write. If the Master lost arbitration, stop waiting. */
			if((I2C_I2CMasterStatus() & I2C_I2C_MSTAT_ERR_ARB_LOST) == I2C_I2C_MSTAT_ERR_ARB_LOST)
			{
				/* Flag to indicate that the data logging will not be 
				 * continued until a device reset happens. */
				stopDataAccess = TRUE;
				displayErrorOnce = TRUE;
				break;
			}
	    }
		
		/* Read data bytes from F-RAM. */
		I2C_I2CMasterReadBuf(addrFRAM, readBuf, READ_BUF_SIZE, I2C_I2C_MODE_REPEAT_START);
		
		while(0u == (I2C_I2CMasterStatus() & I2C_I2C_MSTAT_RD_CMPLT))
	    {
	        /* Wait until master complete read. If the Master lost arbitration, stop waiting. */
			if((I2C_I2CMasterStatus() & I2C_I2C_MSTAT_ERR_ARB_LOST) == I2C_I2C_MSTAT_ERR_ARB_LOST)
			{
				/* Flag to indicate that the data logging will not be 
				 * continued until a device reset happens. */
				stopDataAccess = TRUE;
				displayErrorOnce = TRUE;
				break;
			}
	    }
		
		if(stopDataAccess == FALSE)
		{
			sprintf(uartString, "\n\rData Written to F-RAM Page");
			UART_Comm_UartPutString(uartString);
			
			/* Send the F-RAM page number. */
			if(addrFRAM == FRAM_PAGE1_ADDR)
			{
				/* F-RAM Page 1. */
				sprintf(uartString, " 0 ");
				UART_Comm_UartPutString(uartString);
			}
			else
			{
				/* F-RAM Page 2. */
				sprintf(uartString, " 1 ");
				UART_Comm_UartPutString(uartString);
			}
			
			sprintf(uartString, "at Location: 0x");
			UART_Comm_UartPutString(uartString);
			
			/* Send the remaining address data. */
			/* Extract the first digit. */
			firstChar = HI8(locFRAM)/DIVISOR_HEX;
			if(firstChar < TEN)
			{
				/* Add 0x30 if value is less than 10. */
				firstChar = firstChar + ASCII_DEC_OFFSET;
			}
			else
			{
				/* Add 0x37 values greater than 9 and less than 16. */
				firstChar = firstChar + ASCII_HEX_OFFSET;
			}
			
			/* Extract the second digit. */
			secondChar = HI8(locFRAM)%DIVISOR_HEX;
			if(secondChar < TEN)
			{
				/* Add 0x30 if value is less than 10. */
				secondChar = secondChar + ASCII_DEC_OFFSET;
			}
			else
			{
				/* Add 0x37 values greater than 9 and less than 16. */
				secondChar = secondChar + ASCII_HEX_OFFSET;
			}
		
			/* Send data bytes through UART. */
			UART_Comm_SpiUartWriteTxData(firstChar);
			UART_Comm_SpiUartWriteTxData(secondChar);
			
			/* Extract the first digit. */
			firstChar = LO8(locFRAM)/DIVISOR_HEX;
			if(firstChar < TEN)
			{
				/* Add 0x30 if value is less than 10. */
				firstChar = firstChar + ASCII_DEC_OFFSET;
			}
			else
			{
				/* Add 0x37 values greater than 9 and less than 16. */
				firstChar = firstChar + ASCII_HEX_OFFSET;
			}
			
			/* Extract the second digit. */
			secondChar = LO8(locFRAM)%DIVISOR_HEX;
			if(secondChar < TEN)
			{
				/* Add 0x30 if value is less than 10. */
				secondChar = secondChar + ASCII_DEC_OFFSET;
			}
			else
			{
				/* Add 0x37 values greater than 9 and less than 16. */
				secondChar = secondChar + ASCII_HEX_OFFSET;
			}
		
			/* Send data bytes through UART. */
			UART_Comm_SpiUartWriteTxData(firstChar);
			UART_Comm_SpiUartWriteTxData(secondChar);
			
			/* Send the timestamp information. */
			sprintf(uartString, "\n\rTimestamp: ");
			UART_Comm_UartPutString(uartString);
			UART_WriteAsciiData(&readBuf[FRAM_HOUR_OFFSET], TIME_DATA_SIZE, DECIMAL_FORMAT);
			UART_Comm_UartPutChar(':');
			UART_WriteAsciiData(&readBuf[FRAM_MINUTE_OFFSET], TIME_DATA_SIZE, DECIMAL_FORMAT);
			UART_Comm_UartPutChar(':');
			UART_WriteAsciiData(&readBuf[FRAM_SECOND_OFFSET], TIME_DATA_SIZE, DECIMAL_FORMAT);
						
			/* Send the sensor data. */
			sprintf(uartString, " Sensor Data: ");
			UART_Comm_UartPutString(uartString);
			UART_WriteAsciiData(&readBuf[FRAM_DATA_OFFSET], SENSOR_DATA_SIZE, HEX_FORMAT);
			UART_Comm_UartPutString("\n\r");
			
			
			/* Update the offset value. */
			locFRAM = locFRAM + FRAM_PACKET_SIZE;
			
			if(locFRAM >= FRAM_PAGE_BOUNDARY)
			{
				locFRAM = ZERO; 
				if(addrFRAM == FRAM_PAGE1_ADDR)
				{
					/* If first page of F-RAM is full, switch to second page. */
					addrFRAM = FRAM_PAGE2_ADDR;
				}
				else
				{
					/* If second page of F-RAM is full, switch to first page. */
					addrFRAM = FRAM_PAGE1_ADDR;
				}
			}
		}
	}
}

/* [] END OF FILE */
