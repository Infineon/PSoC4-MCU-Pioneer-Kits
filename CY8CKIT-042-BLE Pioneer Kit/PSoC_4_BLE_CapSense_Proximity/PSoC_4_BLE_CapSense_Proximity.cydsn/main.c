/******************************************************************************
* Project Name		: PSoC_4_BLE_CapSense_Proximity
* File Name			: main.c
* Version 			: 1.0
* Device Used		: CY8C4247LQI-BL483
* Software Used		: PSoC Creator 4.1
* Compiler    		: ARM GCC 5.4.1, ARM MDK Generic
* Related Hardware	: CY8CKIT-042-BLE Bluetooth Low Energy Pioneer Kit 
* Owner				: ROIT
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

/******************************************************************************
*                           THEORY OF OPERATION
*******************************************************************************
* The project will show case the capability of PSoC 4 BLE to communicate with
* a BLE Central device. The data communicated is the CapSense proximity value,
* sent from PSoC 4 BLE over a custom service. This custom service allows
* notifications to be sent to central device after notifications are enabled.
* This project utilizes CapSense component to detect the change in capacitance
* of the proximity sensor and report this to central device over BLE.
* The BLE central device can be either BLE capable smartphone with associated 
* mobile application or CySmart PC tool.
* This project also inludes low power mode operation, idle for battery operated 
* devices. The project utlizes Deep Sleep feature of both BLESS and CPU to remain 
* in low power mode as much as possible, while maintaining the BLE connection and  
* data transfer. This allows the device to run on coin cell battery for long time.
*
* Note:
* The programming pins have been configured as GPIO, and not SWD. This is because 
* when programming pins are configured for SWD, then the silicon consumes extra
* power through the pins. To prevent the leakage of power, the pins have been set 
* to GPIO. With this setting, the kit can still be acquired by PSoC Creator or
* PSoC Programmer software tools for programming, but the project cannot be 
* externally debugged. To re-enable debugging, go to PSoC_4_BLE_CapSense.cydwr from
* Workspace Explorer, go to Systems tab, and set the Debug Select option to 'SWD'.
* Build and program this project to enable external Debugging.
*
* Refer to BLE Pioneer Kit user guide for details.
*******************************************************************************
* Hardware connection required for testing -
* PROX pin 		- P2[0] (requires an external wire-loop connection to PROX sensor)
* Cmod pin		- P4[0] (hard-wired in the PSoC 4 BLE Module)
* User Button	- P2[7] (hard-wired on BLE Pioneer Kit)
* Status LED	- P3[7] (hard-wired on BLE Pioneer Kit)
******************************************************************************/
#include <main.h>

/*'deviceConnected' flag is used by application to know whether a Central device  
* has been connected. This is updated in BLE event callback */
extern uint8 deviceConnected;

/*'startNotification' flag is set when the central device writes to CCC (Client  
* Characteristic Configuration) of the CapSense proximity characteristic to 
* enable notifications */
extern uint8 startNotification;	

/* 'restartAdvertisement' flag is used to restart advertisement */
extern uint8 restartAdvertisement;

/* 'initializeCapSenseBaseline' flag is used to call the function once that initializes 
* all CapSense baseline. The baseline is initialized when the first advertisement 
* is started. This is done so that any external disturbance while powering the kit does 
* not infliuence the baseline value, that may cause wrong readings. */
uint8 initializeCapSenseBaseline = TRUE;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*        System entrance point. This calls the BLE routine functions as well as
* function for handling CapSense Proximity changes
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
    /* This function will initialize the system resources such as BLE and CapSense */
	InitializeSystem();
	
    for(;;)
    {
		/*Process Event callback to handle BLE events. The events generated and 
		* used for this application are inside the 'CustomEventHandler' routine*/
		CyBle_ProcessEvents();
		
		/* Function to handle LED status depending on BLE state */
		HandleStatusLED();
		
		/* Handle proximity data and CCCD value update only if BLE device is connected */
		if(TRUE == deviceConnected) 
		{
			/* After the connection, send new connection parameter to the Client device 
			* to run the BLE communication on desired interval. This affects the data rate 
			* and power consumption. High connection interval will have lower data rate but 
			* lower power consumption. Low connection interval will have higher data rate at
			* expense of higher power. This function is called only once per connection. */
			UpdateConnectionParam();
			
			/* If CapSense Initialize Baseline API has not been called yet, call the
			* API and reset the flag. */
			if(initializeCapSenseBaseline)
			{
				/* Reset 'initializeCapSenseBaseline' flag*/
				initializeCapSenseBaseline = FALSE;
				
				/* Initialize all CapSense Baseline */
				CapSense_InitializeAllBaselines();
			}
			
			/* Send proximity data when notifications are enabled */
			if(startNotification == CCCD_NTF_BIT_MASK)
			{
				if(CYBLE_BLESS_STATE_ECO_STABLE == CyBle_GetBleSsState())
				{
					/*Check for CapSense proximity change and report to BLE central device*/
					HandleCapSenseProximity();
				}
			}
		}

		#ifdef ENABLE_LOW_POWER_MODE
			/* Put system to Deep sleep, including BLESS, and wakeup on interrupt. 
			* The source of the interrupt can be either BLESS Link Layer in case of 
			* BLE advertisement and connection or by User Button press during BLE 
			* disconnection */
			HandleLowPowerMode();
		#endif
		
		if(restartAdvertisement)
		{
			/* Reset 'restartAdvertisement' flag*/
			restartAdvertisement = FALSE;
			
			/* Start Advertisement and enter Discoverable mode*/
			CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);	
		}
    }	/* This is end of for(;;) */
}

/*******************************************************************************
* Function Name: InitializeSystem
********************************************************************************
* Summary:
*        Starts the components and initializes CapSense baselines
*
* Parameters:
*  void
*
* Return:
*  void
*

*******************************************************************************/
void InitializeSystem(void)
{
	/* Enable global interrupt mask */
	CyGlobalIntEnable;

	/* Start BLE component and register the CustomEventHandler function. This 
	* function exposes the events from BLE component for application use */
	CyBle_Start(CustomEventHandler);					
	
	/* Set drive mode of the status LED pin to Strong*/
	Status_LED_SetDriveMode(Status_LED_DM_STRONG);	
	
	#ifdef CAPSENSE_ENABLED
	/* Enable the proximity widget explicitly and start CapSense component. 
	* Initialization of the baseline is done when the first connection 
	* happens  */
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR0__PROX);
	CapSense_Start();
	#endif
		
	/* Start the Button ISR to allow wakeup from sleep */
	isr_button_StartEx(MyISR);
}

/*******************************************************************************
* Function Name: HandleCapSenseProximity
********************************************************************************
* Summary:
*       Read the proximity data from the sensor and update the BLE
*		custom notification value.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void HandleCapSenseProximity(void)
{
	#ifdef CAPSENSE_ENABLED
	/* Static variable to store last proximity value */
	static uint8 lastProximityValue = MAX_PROX_VALUE;
		
	/* 'proximityValue' stores the proximity value read from CapSense component */
	uint8 proximityValue;	

	/* Scan the Proximity Widget */
	CapSense_ScanWidget(CapSense_PROXIMITYSENSOR0__PROX);				
	
	/* Wait for CapSense scanning to be complete */
	while(CapSense_IsBusy())
	{
		#ifdef ENABLE_LOW_POWER_MODE
		/* While CapSense is scanning, put CPU to sleep to conserve power */
		CySysPmSleep();
		#endif
	}
	
	/* Update CapSense Baseline */
	CapSense_UpdateEnabledBaselines();			

	/* Get the Diffcount between proximity raw data and baseline */
	proximityValue = CapSense_GetDiffCountData(CapSense_PROXIMITYSENSOR0__PROX);
	
	/* Check if the proximity data is different from previous */
	if(lastProximityValue != proximityValue)				
	{
		/* Send the proximity data to BLE central device by notifications*/
		SendDataOverCapSenseNotification(proximityValue);
		
		/* Update present proximity value */
		lastProximityValue = proximityValue;		
	}
	#endif
}

/* [] END OF FILE */
