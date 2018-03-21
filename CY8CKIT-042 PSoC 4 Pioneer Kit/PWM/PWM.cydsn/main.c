/******************************************************************************
* Project Name		: PWM
* File Name			: main.c
* Version 			: 1.0
* Device Used		: CY8C4245AXI-483
* Software Used		: PSoC Creator 3.1
* Compiler    		: ARMGCC, ARM RVDS Generic , ARM MDK Generic
* Related Hardware	: CY8CKIT-042 PSoC 4 Pioneer Kit 
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
* This project demonstrates the use of the PWM component. The project uses
* three PWM outputs to set the color of the tricolor LED on the CY8CKIT-042. 
* The color changes  every one second from Violet to Red in the order Violet - 
* Indigo - Blue - Green - Yellow - Orange - Red. This is achieved by changing
* the pulse width of the PWMs.
*
* Hardware connection on the Kit
* LED_Red   - P1[6]
* LED_Green - P0[2]
* LED_Blue  - P0[3]
******************************************************************************/
#include <device.h>
#define RED 0u
#define GREEN 1u
#define BLUE 2u
#define NUM_OF_COLORS 7u

/* Mask for BYTE shifting */

#define BLUE_POSITION 0
#define GREEN_POSITION 8
#define RED_POSITION 16
#define ALPHA_POSITION 24

/* Color code for VIBGYOR */
CYCODE const uint8 colorMap[][3] = {{0x99,0x00,0x99},  /* Violet */
									{0x4B,0x00,0x82},  /* Indigo */
									{0x00,0x00,0xFF},  /* Blue   */
									{0x00,0xFF,0x00},  /* Green  */
									{0xFF,0xD3,0x00},  /* Yellow */
									{0xFF,0x45,0x00},  /* Orange */
									{0xFF,0x00,0x00}}; /* Red    */

uint8 ucrCompare, ucgCompare, ucbCompare;
uint32 ucARGB;
int main()
{
    uint8 colorIndex = 0u;
	
	/* Start all components */
	PWM_Red_Start();
	PWM_Green_Start();
	PWM_Blue_Start();
	
	/* Clock can be started automatically after reset by enabling “Start on 
	Reset” in Clocks tab of PWM.cydwr. We are doing this manually for instructive 
	purpose */
	Clock_PWM_Start();

	for(;;)
    {
		/* Loop to change the LED color. */
        for(colorIndex = 0; colorIndex < NUM_OF_COLORS; colorIndex++)
		{
			/* Set the compare value of the PWM components to change 
			the PWM duty-cycle. This wll set the tricolor LED color
			to one of the pre-defined colors as given in the colormap */
			PWM_Red_WriteCompare(colorMap[colorIndex][RED]);
			PWM_Green_WriteCompare(colorMap[colorIndex][GREEN]);
			PWM_Blue_WriteCompare(colorMap[colorIndex][BLUE]);
			
			ucrCompare = colorMap[colorIndex][RED];
			ucgCompare = colorMap[colorIndex][GREEN];
			ucbCompare = colorMap[colorIndex][BLUE];			
			
			ucARGB = (ucrCompare << RED_POSITION) | (ucgCompare << GREEN_POSITION) | (ucbCompare << BLUE_POSITION);
			/* Wait for one second before changing the color */
			CyDelay(1000);
		}
    }
}

/* [] END OF FILE */
