/******************************************************************************
* Project Name		: CY8CKIT_040_CapSense_I2C
* File Name			: main.c
* Version 			: 1.0
* Device Used		: CY8C4014LQI-422
* Software Used		: PSoC Creator 3.1
* Compiler    		: ARM GCC 4.8.4, ARM RVDS Generic , ARM MDK Generic
* Related Hardware	: CY8CKIT-040 PSoC 4000 Pioneer Kit 
*
********************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation. All Rights Reserved.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress)
* and is protected by and subject to worldwide patent protection (United
* States and foreign), United States copyright laws and international treaty
* provisions. Cypress hereby grants to licensee a personal, non-exclusive,
* non-transferable license to copy, use, modify, create derivative works of,
* and compile the Cypress Source Code and derivative works for the sole
* purpose of creating custom software in support of licensee product to be
* used only in conjunction with a Cypress integrated circuit as specified in
* the applicable agreement. Any reproduction, modification, translation,
* compilation, or representation of this software except as specified above 
* is prohibited without the express written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH 
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the 
* materials described herein. Cypress does not assume any liability arising out 
* of the application or use of any product or circuit described herein. Cypress 
* does not authorize its products for use as critical components in life-support 
* systems where a malfunction or failure may reasonably be expected to result in 
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of 
* such use and in doing so indemnifies Cypress against all charges. 
*
* Use of this Software may be limited by and subject to the applicable Cypress
* software license agreement. 
*******************************************************************************/

/******************************************************************************
*                           THEORY OF OPERATION
* The project demonstrates CapSense trackpad with I2C tuner for data viewing.
*	Though I2C tuner can be used to tune the trackpad in manual tuning mode,
*	SmartSense is used in the project to implement the trackpad. Hence data
*	can only be viewed in Tuner GUI for SNR & trackpad position.
*
* Hardware connection required for testing -* 
* Trackpad 	- Refer CYDWR file for details (requires Trackpad shield connection)
* Cmod pin	- P0[4] (connected in the board itself)
* I2C pins	- P1[2]/P1[3] (SCL/SDA lines - hard-wired in the board) 
******************************************************************************/
#include <project.h>

/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary:
*  main() performs following functions:
*  1: Initialize CapSense along with tuner APIs
*  2: Scans the sensors, updates baseline & sends data over I2C tuner comm
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
int main()
{   
	/* Enable Global interrupts */
    CyGlobalIntEnable; 
	
	/* Start the CapSense block, initialize all sensors & the tuner comm interface */
	CapSense_TunerStart();
	
    for(;;)
    {
        /* Scan the sensors, update the baseline & exchange data over tuner comm interface */
		CapSense_TunerComm();		
    }
}

/* [] END OF FILE */
