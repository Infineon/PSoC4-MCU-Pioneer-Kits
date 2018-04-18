/*****************************************************************************
* File Name: accelerometer.h
* Version 1.0
*
* Description:
*  This file contains the function prototypes and constants used in
*  accelerometer.c
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
#if !defined(ACCELEROMETER_H) 
#define ACCELEROMETER_H 

#include "cytypes.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define G2_MODE					0x00
#define G4_MODE					0x01
#define G8_MODE					0x02

#define G2_MODE_RESOLUTION		0x400
#define G4_MODE_RESOLUTION		0x200
#define G8_MODE_RESOLUTION		0x100

#define ACCELEROMETER_MODE		G2_MODE

#define ACCELEROMETER_ADDR		0x0F

#define ACC_XOUT_L				0x06
#define ACC_XOUT_H				0x07

#define ACC_YOUT_L				0x08
#define ACC_YOUT_H				0x09

#define ACC_ZOUT_L				0x0A
#define ACC_ZOUT_H				0x0B

#define ACC_DCST_RESP			0x0C

#define ACC_WHO_AM_I			0x0F

#define ACC_INT_SRC_1			0x16
#define ACC_INT_SRC_2			0x17

#define ACC_STATUS_REG			0x18

#define ACC_INT_REL 			0x1A

/* PC1 bit in the ACC_CTRL_REG_1 must be set to '0' before changing the 
 * contents of the following registers (ACC_CTRL_REG_1 to ACC_WAKEUP_TIMER). */
#define ACC_CTRL_REG_1			0x1B
#define ACC_CTRL_REG_2			0x1D
#define ACC_INT_CTRL_REG_1		0x1E
#define ACC_INT_CTRL_REG_2		0x1F
#define ACC_DATA_CTRL_REG		0x21
#define ACC_WAKEUP_TIMER		0x29

#define ACC_SELF_TEST			0x3A
#define ACC_WAKUP_THRESHOLD		0x6A

#define ACC_WRITE				0x00
#define ACC_READ				0x01

#define ACC_ACK					0x00
#define ACC_NACK				0x01

#define ACC_FAIL				0x00
#define ACC_PASS				0x01

#define COMM_FAIL				0x01
#define COMM_PASS				0x00

#define ACC_SELF_TEST_VAL_1		0x55
#define ACC_SELF_TEST_VAL_2		0xAA

#define ACC_PC1_BIT_OR_MASK		0x80
#define ACC_RES_BIT_OR_MASK		0x40
#define ACC_DRDYE_BIT_OR_MASK	0x20
#define ACC_GSEL1_BIT_OR_MASK	0x10
#define ACC_GSEL0_BIT_OR_MASK	0x08
#define ACC_WUFE_BIT_OR_MASK	0x02

#define ACC_PC1_BIT_AND_MASK	0x7F
#define ACC_RES_BIT_AND_MASK	0xBF
#define ACC_DRDYE_BIT_AND_MASK	0xDF
#define ACC_GSEL1_BIT_AND_MASK	0xEF
#define ACC_GSEL0_BIT_AND_MASK	0xF7
#define ACC_WUFE_BIT_AND_MASK	0xFD

#define ACC_SRST_BIT_OR_MASK	0x80
#define ACC_DCST_BIT_OR_MASK	0x10
#define ACC_OWUFA_BIT_OR_MASK	0x04
#define ACC_OWUFB_BIT_OR_MASK	0x02
#define ACC_OWUFC_BIT_OR_MASK	0x01

#define ACC_SRST_BIT_AND_MASK	0x7F
#define ACC_DCST_BIT_AND_MASK	0xEF
#define ACC_OWUFA_BIT_AND_MASK	0xFB
#define ACC_OWUFB_BIT_AND_MASK	0xFD
#define ACC_OWUFC_BIT_AND_MASK	0xFE

#define ACC_IEN_BIT_OR_MASK		0x20
#define ACC_IEA_BIT_OR_MASK		0x10
#define ACC_IEL_BIT_OR_MASK		0x08

#define ACC_IEN_BIT_AND_MASK	0xDF
#define ACC_IEA_BIT_AND_MASK	0xEF
#define ACC_IEL_BIT_AND_MASK	0xF7

#define ACC_XNWUE_BIT_OR_MASK	0x20
#define ACC_XPWUE_BIT_OR_MASK	0x10
#define ACC_YNWUE_BIT_OR_MASK	0x08
#define ACC_YPWUE_BIT_OR_MASK	0x04
#define ACC_ZNWUE_BIT_OR_MASK	0x02
#define ACC_ZPWUE_BIT_OR_MASK	0x01

#define ACC_XNWUE_BIT_AND_MASK	0xDF
#define ACC_XPWUE_BIT_AND_MASK	0xEF
#define ACC_YNWUE_BIT_AND_MASK	0xF7
#define ACC_YPWUE_BIT_AND_MASK	0xFB
#define ACC_ZNWUE_BIT_AND_MASK	0xFD
#define ACC_ZPWUE_BIT_AND_MASK	0xFE

#define ACC_OSAA_BIT_OR_MASK	0x08
#define ACC_OSAB_BIT_OR_MASK	0x04
#define ACC_OSAC_BIT_OR_MASK	0x02
#define ACC_OSAD_BIT_OR_MASK	0x01

#define ACC_OSAA_BIT_AND_MASK	0xF7
#define ACC_OSAB_BIT_AND_MASK	0xFB
#define ACC_OSAC_BIT_AND_MASK	0xFD
#define ACC_OSAD_BIT_AND_MASK	0xFE

#define ACC_WAKEUP_TIMER_VALUE	0x01
#define ACC_WAKUP_THRESHOLD_VAL	0x01

#define ACC_NO_MOVEMENT			0x0000

#define ZERO					0x00	
	
#define TRUE					0x01
#define FALSE					0x00
    
#define I2C_TIMEOUT_MS          0x64
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
extern uint8 acclerometer_configured;

/*****************************************************************************
* Function Prototypes
*****************************************************************************/
void InitializeAccelerometer(void);
void ReadAccelerometer(int16 *xData, int16 *yData, int16 *zData);
uint8 Accelerometer_RegRead(uint8 reg, uint8 *value);

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
	
	
#endif /* ACCELEROMETER_H */

/* [] END OF FILE */

