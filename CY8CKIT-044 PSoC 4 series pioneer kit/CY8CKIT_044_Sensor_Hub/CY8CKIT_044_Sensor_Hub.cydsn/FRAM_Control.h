/*****************************************************************************
* File Name		: FRAM_Control.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  FRAM_Control.c
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

#if !defined(FRAM_CONTROL_H) 
#define FRAM_CONTROL_H 

#include "cytypes.h"
	
/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define FRAM_PAGE1_ADDR				0x50
#define FRAM_PAGE2_ADDR				0x51

#define ZERO						0x00
	
#define FRAM_DATA_LOC_LSB			0x00
#define FRAM_DATA_LOC_MSB			0x00

#define READ_BUF_SIZE				0x0B
#define ADDRESS_OFFSET				0x02	
#define WRITE_BUF_SIZE				0x0D

#define FRAM_HOUR_OFFSET			0x00
#define FRAM_MINUTE_OFFSET			0x01
#define FRAM_SECOND_OFFSET			0x02
#define FRAM_DATA_OFFSET			0x03
	
#define TIME_DATA_SIZE				0x01
#define SENSOR_DATA_SIZE			0x08
#define FRAM_PACKET_SIZE			0x0B
#define FRAM_PAGE_BOUNDARY			0xFFF7

#define WRITE_BUF_LOC_SIZE			0x02

#define MAX_STRING_SIZE				0x40

#define TRUE						0x01
#define FALSE						0x00
	
#define DIVISOR_HEX					0x10
#define	TEN							0x0A
#define ASCII_DEC_OFFSET			0x30
#define ASCII_HEX_OFFSET			0x37

#define ASCII_PAGE_1				0x30
#define ASCII_PAGE_2				0x31

#define HEX_FORMAT					0x10
#define DECIMAL_FORMAT				10
/*****************************************************************************
* Data Type Definition
*****************************************************************************/


/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/


/*****************************************************************************
* Data Structure Definition
*****************************************************************************/


/*****************************************************************************
* Global Variable Declaration
*****************************************************************************/
extern uint8 displayErrorOnce;

/*****************************************************************************
* Function Prototypes
*****************************************************************************/
void FRAM_SaveSensorData(void);
void FRAM_ReadSensorData(void);
/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
	
	
#endif /* FRAM_CONTROL_H */

/* [] END OF FILE */
