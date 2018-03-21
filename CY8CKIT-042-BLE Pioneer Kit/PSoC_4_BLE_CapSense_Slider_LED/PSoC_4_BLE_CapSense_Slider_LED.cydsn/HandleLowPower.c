/******************************************************************************
* Project Name		: PSoC_4_BLE_CapSense_Slider_LED
* File Name			: HandleLowPower.c
* Version 			: 1.0
* Device Used		: CY8C4247LQI-BL483
* Software Used		: PSoC Creator 4.1
* Compiler    		: ARM GCC 5.4.1, ARM MDK Generic
* Related Hardware	: CY8CKIT-042-BLE Bluetooth Low Energy Pioneer Kit 
* Owner             : ROIT
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
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
*****************************************************************************/
#include <main.h>

extern uint8 shut_down_led;
/*******************************************************************************
* Function Name: HandleLowPowerMode
********************************************************************************
* Summary:
*        This function puts the BLESS in deep sleep mode and CPU to sleep mode. 
* System will resume from here when it wakes from user button press.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void HandleLowPowerMode(void)
{
	#ifdef ENABLE_LOW_POWER_MODE
		/* Local variable to store the status of BLESS Hardware block */
		CYBLE_LP_MODE_T sleepMode;
		CYBLE_BLESS_STATE_T blessState;
		
		#ifdef CAPSENSE_ENABLED
		/* Put CapSense to Sleep*/
		CapSense_Sleep();
		#endif
		
		/* Put BLESS into Deep Sleep and check the return status */
		sleepMode = CyBle_EnterLPM(CYBLE_BLESS_DEEPSLEEP);
		
		/* Disable global interrupt to prevent changes from any other interrupt ISR */
		CyGlobalIntDisable;
	
		/* Check the Status of BLESS */
		blessState = CyBle_GetBleSsState();

		if(sleepMode == CYBLE_BLESS_DEEPSLEEP)
		{
		    /* If the ECO has started or the BLESS can go to Deep Sleep, then place CPU 
			* to Deep Sleep */
			if(blessState == CYBLE_BLESS_STATE_ECO_ON || blessState == CYBLE_BLESS_STATE_DEEPSLEEP)
		    {
				if(shut_down_led)
				{
					/* Put PrISM modules to sleep */
					PrISM_1_Sleep();
					PrISM_2_Sleep();
					
					/* Place CPU to Deep sleep only when the RGB PrISM module is not 
					* active (indicated by flag 'shut_down_led'). 
					* If RGB PrISM is active, then the CPU should only be placed in 
					* Sleep to allow the PrISM to function and control the color 
					* and Intensity */
			        CySysPmDeepSleep();
					
					/* After system wakes up, wake up the PrISM modules*/
					PrISM_1_Wakeup();
					PrISM_2_Wakeup();
				}
				else
				{
					/* If the system is controlling RGB LED, then PrISM needs to be running. 
					* Put CPU to sleep only */
					CySysClkWriteImoFreq(IMO_FREQUENCY_3MHZ);
					CySysPmSleep();
					CySysClkWriteImoFreq(IMO_FREQUENCY_12MHZ);

				}
		 	}
		}
		else
		{
		    if(blessState != CYBLE_BLESS_STATE_EVENT_CLOSE)
		    {
				/* If the BLESS hardware block cannot go to Deep Sleep and BLE Event has not 
				* closed yet, then place CPU to Sleep */
		        CySysPmSleep();
		    }
		}
		
		/* Re-enable global interrupt mask after wakeup */
		CyGlobalIntEnable;
		
		#ifdef CAPSENSE_ENABLED
		/* Wakeup CapSense Block */
		CapSense_Wakeup();
		#endif
		
	#endif
}

/*******************************************************************************
* Function Name: MyISR
********************************************************************************
* Summary:
*        ISR routine for isr_button. System enters here after CPU wakeup.
* Clears pending interrupt.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
CY_ISR(MyISR)
{
	/* Clear Pending interrupts */
	isr_button_ClearPending();
	
	User_Button_ClearInterrupt();
}
/* [] END OF FILE */
