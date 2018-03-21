/******************************************************************************
* Project Name		: Deep Sleep
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
* This project demonstrates the Deep Sleep mode of the PSoC 4. The LED on the 
* kit is turned on for one second and then turned off. The device enters the 
* deep sleep mode after that. The device wakes up from the sleep mode when user
* presses the button on the kit. The device wakes up from sleep as a result of
* the PICU interrupt being triggered by the button press. This interrupt is 
* cleared in the ISR of the isr_WakeUp.c file.
*
* Hardware connection on the Kit
* LED_Red 		- P1[6]
* WakeUp_Switch - P0[7]
******************************************************************************/
#include <device.h>

/* Write '0' to Pin_RedLED to turn on the LED. 
 * See the connections on the TopDesign. */

#define LED_ON 0u
#define LED_OFF 1u
#define DELAY_1S 1000
#define HIGH_IMPEDANCE 0x00000000

/* Function declarations */
void Component_ShutDown(void);
void Component_Restore(void);

int main()
{
	/* Enables the interrupt */
    isr_WakeUp_Start();

	/* Enable Global Interrupts */
	CyGlobalIntEnable;
	
    for(;;)
    {
		/* Wake Up for one second in active mode 
		 * Glow the LED to indicate that the device is in active mode */
		Pin_RedLED_Write(LED_ON);		
		CyDelay(DELAY_1S);
		Pin_RedLED_Write(LED_OFF);
		
		/* Prepare Device to Sleep */
		Component_ShutDown();
		
		/* Enter Deep Sleep Mode */
		CySysPmDeepSleep();

		/* Restore the device configuration after coming back from sleep */
		Component_Restore();
    }
}

/*******************************************************************************
* Function Name: Component_ShutDown
********************************************************************************
*
* Summary:
*  Shuts down all the components and puts all the pins to HiZ state to reduce
* power consumption.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Component_ShutDown(void)
{
	/* Call all component stop/sleep functions here . 
	 * This project doesn't have any components that needs shut down */ 
	 
	/* Set all the port pin registers to high impedence mode */
	CY_SET_REG32(CYREG_PRT0_PC , HIGH_IMPEDANCE);
	CY_SET_REG32(CYREG_PRT1_PC , HIGH_IMPEDANCE);
	CY_SET_REG32(CYREG_PRT2_PC , HIGH_IMPEDANCE);
	CY_SET_REG32(CYREG_PRT3_PC , HIGH_IMPEDANCE);
	CY_SET_REG32(CYREG_PRT4_PC , HIGH_IMPEDANCE);
	
	/* Set the WakeUp Switch pin in resistive pull up mode for detecting 
	 * the falling edge interrupt when switch is pressed */
	Pin_WakeUpSwitch_SetDriveMode(Pin_WakeUpSwitch_DM_RES_UP);
}

/*******************************************************************************
* Function Name: Component_Restore
********************************************************************************
*
* Summary:
*  Restore all the components and pin state after coming back from sleep.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Component_Restore(void)
{
	/* Call all component restore functions here . 
	 * This project doesn't have any components that needs to be restored */ 
	 
	/* Restore all the pin states */
	
	/* Set the Red LED output pin in strong drive mode */
	Pin_RedLED_SetDriveMode(Pin_RedLED_DM_STRONG);
}

/* [] END OF FILE */
