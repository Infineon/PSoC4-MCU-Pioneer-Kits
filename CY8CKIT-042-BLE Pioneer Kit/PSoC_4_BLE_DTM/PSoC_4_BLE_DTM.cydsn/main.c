/******************************************************************************
* Project Name		: PSoC_4_BLE_DTM
* File Name			: main.c
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

/*********************************************************************************
*                           THEORY OF OPERATION
**********************************************************************************
* This project operates in BLE specification defined HCI mode and executes the Direct
* Test Mode (DTM) cases. There is no application level activity except processing
* BLE events. This project requires the external CBT tester to operate.
* Refer CY8CKIT-042-BLE Pioneer Kit user guide for more details. 
**********************************************************************************
* Hardware connection required for testing -
* UART RX 	- P0[0] (Connect this pin from J3 header on BLE Pioneer Kit to UART TX
					 line of external RS232 converter)
* UART TX	- P0[1] (Connect this pin from J3 header on BLE Pioneer Kit to UART RX
					 line of external RS232 converter)
* RTS 		- P0[2] (Connect this pin from J3 header on BLE Pioneer Kit to RTS
					 line of external RS232 converter; can be left unconnected)
* CTS		- P0[3] (Connect this pin from J3 header on BLE Pioneer Kit to CTS
					 line of external RS232 converter; can be left unconnected)
**********************************************************************************/
#include <project.h>

#define ENABLE_LOW_POWER_MODE

/* BLE Callback Event Handler Function */
void ApplicationEventHandler(uint32 event, void * eventparam);
/* Handle Low Power Mode Function */
void HandleLowPowerMode(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*        System entrance point. This calls the BLE start and processes BLE Events
*
* Parameters:
*  void
*
* Return:
*  int
*

*******************************************************************************/
int main()
{
	/* Enable global interrupts. */
    CyGlobalIntEnable; 
	
	/* Start BLE Component and register the event callback function */
	CyBle_Start(ApplicationEventHandler);
	
    for(;;)
    {
        /* Process BLE events continuously */
		CyBle_ProcessEvents();
		
		/* Put CPU and BLESS to low power mode */
		HandleLowPowerMode();
    }
}

/*******************************************************************************
* Function Name: ApplicationEventHandler
********************************************************************************
* Summary:
*        Call back event function to handle various events from BLE stack
*
* Parameters:
*  event:		event returned
*  eventParam:	link to value of the events returned
*
* Return:
*  void
*
*******************************************************************************/
void ApplicationEventHandler(uint32 event, void * eventparam)
{
	switch(event)
	{
		/* No application event is generated in HCI Mode. All commands
		* are processed by BLE stack */
	
		default:
			/* To prevent compiler warning */
			eventparam = eventparam;
		break;
	}
}

/*******************************************************************************
* Function Name: HandleLowPowerMode
********************************************************************************
* Summary:
*        This function puts the BLESS in deep sleep mode and CPU to sleep mode. 
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

		/* Put BLESS into Deep Sleep and check the return status */
		sleepMode = CyBle_EnterLPM(CYBLE_BLESS_DEEPSLEEP);
		
		/* Disable global interrupt to prevent changes from any other interrupt ISR */
		CyGlobalIntDisable;
	
		/* Check the Status of BLESS */
		blessState = CyBle_GetBleSsState();

		if(sleepMode == CYBLE_BLESS_DEEPSLEEP)
		{
			if(blessState == CYBLE_BLESS_STATE_ECO_ON || blessState == CYBLE_BLESS_STATE_DEEPSLEEP)
		    {
				/* No action for DTM Project */
		 	}
		}
		else
		{
		    if(blessState != CYBLE_BLESS_STATE_EVENT_CLOSE)
		    {
				/* If the BLESS hardware block didnot go to Deep Sleep and BLE Event has not 
				* closed yet, then place CPU to Sleep */
				CySysPmSleep();
		    }
		}
		
		/* Re-enable global interrupt mask after wakeup */
		CyGlobalIntEnable;
	#endif
}

/* [] END OF FILE */
