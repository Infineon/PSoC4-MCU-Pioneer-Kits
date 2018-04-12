/*******************************************************************************
* Project Name		: CY8CKIT_046_USB_Mouse
* Version			: 1.0
* Device Used		: CY8C4248BZI-L489     
* Software Used		: PSoC Creator 3.3 Service Pack 1
* Compiler Used		: ARM GCC 4.9.3 
* Related Hardware	: CY8CKIT-046 PSoC 4 L-Series Pioneer Kit 
*******************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation.
*******************************************************************************
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
*******************************************************************************/

/*******************************************************************************
* Theory of Operation: This example demonstrates a simple USB human interface 
* device (HID) implementation (mouse/keyboard) using the CapSense Gesture Pad 
* present in the CY8CKIT-046 PSoC 4 L-Series Pioneer Kit. In addition to 
* emulating mouse/keyboard over USB, the project also controls the RGB LED 
* intensity.
*******************************************************************************/

#include "project.h"
#include "main.h"
#include "Gesture.h"
#include "HidReport.h"
#include "LedControl.h"
#include "stdbool.h"

/*******************************************************************************
* 	External Variable Declarations
*******************************************************************************/

/* Keyboard and Mouse HID report arrays, declared in HidReport.c */
extern uint8 keyboardReport[];
extern int8 mouseReport[];

/*******************************************************************************
* 	Local Function Declarations
*******************************************************************************/

static void InitSystem(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
* Initializes the application, scans the sensors, detects gesture, converts the
* gesture into HID report, and sends it over USB.
*
* Parameters:
* None
*
* Return:
* int
*
*******************************************************************************/
int main()
{
	tGestureId gestureId;
	uint32 isAnySensorActive, usbDeviceConfiguration = 0;
	uint32 usbDataSentFlag = USB_HID_DATA_SENT;
	bool usbDriverInstallationComplete = false;
	
    CyGlobalIntEnable; /* Enable global interrupts. */
	
	#if(TUNER_ENABLE)
		CapSense_TunerStart();	
		
		for(;;)
		{
			CapSense_TunerComm();
		}
	#endif /* #if(TUNER_ENABLE) */
	
	InitSystem();
	
	for(;;)
	{		
		/* Check if VBUS is detected on the USB, if not then reset the USB flags */
		if(USBFS_VBusPresent() == true)
		{
			/* Read the USB device configuration - Non-zero once enumeration completes */
			usbDeviceConfiguration = USBFS_GetConfiguration();
		}
		else
		{
			if(usbDeviceConfiguration == true)
			{
				PRINT("USB Disconnected \n\r");
				
				/* Turn the LED OFF, when USB is disconnected */
				SetLedColor(COLOR_OFF, BRIGHT_LEVEL_9);
			}
			
			/* Clear/reset the USB related flags */
			usbDeviceConfiguration = false;
			USBFS_configuration = false;
			usbDataSentFlag = USB_HID_DATA_SENT;
		}
			
		/* Wait until CapSense scan completes */
		if((CapSense_IsBusy() == 0) && (usbDataSentFlag == USB_HID_DATA_SENT))
		{
			/* Update baseline and check if any sensor is active */
		    CapSense_UpdateEnabledBaselines();	
			isAnySensorActive = CapSense_CheckIsAnyWidgetActive();	
			
			/* Initiate next scan of all the sensors */
			CapSense_ScanEnabledWidgets();
			
			/* Decode gesture and process the gesture */
			gestureId = DetectGesture(isAnySensorActive); 
			UpdateLedColor(gestureId);	
            
			/* Convert the gestures to HID report */
			ConvertGestureToHidReport(gestureId);			
			
			/* Process USB endpoint load tasks, if USB is connected */
			if(usbDeviceConfiguration != 0)
			{
				if((USBFS_GetEPAckState(KEYBOARD_END_POINT) || USBFS_GetEPAckState(MOUSE_END_POINT)) && (usbDriverInstallationComplete == false))
				{
					usbDriverInstallationComplete = true;					
					
					/* Turn the LED with BLUE after USB device drivers (first ack on the USB end points) are installed */
					SetLedColor(COLOR_BLUE, BRIGHT_LEVEL_9);
					PRINT("HID Drivers Installed \n\r");
					
				}
				else if(usbDriverInstallationComplete == false)
				{
					/* Load initial data into the endpoints when device first enumerates */
					USBFS_LoadInEP(KEYBOARD_END_POINT, (uint8 *)keyboardReport, KEY_RPT_SIZE);
					USBFS_LoadInEP(MOUSE_END_POINT, (uint8 *)mouseReport, MOUSE_RPT_SIZE);
				}
				else if(usbDriverInstallationComplete == true)
				{
					usbDataSentFlag = USB_HID_DATA_SENT_FLAG_CLEAR;
				}			
			}
			else
			{
				usbDriverInstallationComplete = false;				
			}
		}					
			
		/* Process USB HID report only when the drivers are installed and device acks on desired endpoints */
		if((usbDriverInstallationComplete == true) && (usbDataSentFlag != USB_HID_DATA_SENT))
		{					
			/* Send Keyboard data to PC, if there is an USB keyboard endpoint event pending */
			if(USBFS_GetEPState(KEYBOARD_END_POINT) == USBFS_EVENT_PENDING)
			{
				USBFS_LoadInEP(KEYBOARD_END_POINT, (uint8 *)keyboardReport, KEY_RPT_SIZE);
				usbDataSentFlag |= KEYBOARD_DATA_SENT;
			}
			
			/* Send Mouse data to PC, if there is an USB mouse endpoint event pending */
			if(USBFS_GetEPState(MOUSE_END_POINT) == USBFS_EVENT_PENDING)
			{
				USBFS_LoadInEP(MOUSE_END_POINT, (uint8 *)mouseReport, MOUSE_RPT_SIZE);
				usbDataSentFlag |= MOUSE_DATA_SENT;
			}
		}
	}
				
}


/*******************************************************************************
* Function Name: InitSystem
********************************************************************************
*
* Summary:
* Initializes the application.
*
* Parameters:
* None
*
* Return:
* None
*
*******************************************************************************/
void InitSystem(void)
{	
	#if(ENABLE_UART_DBG_OUTPUT)
		UART_Start();
		PRINT("Application Starts...\n\r");
	#endif /* #if(ENABLE_UART_DBG_OUTPUT) */
	
	PWM_Red_Start();
	PWM_Green_Start();
	PWM_Blue_Start();
	
	/* Turn off the LED after USB is initialized */
	SetLedColor(INIT_LED_COLOR, INIT_BRIGHT_LEVEL);
	
	/* Setting the drive mode to strong to enable the LED output. This is
    required because the pin is configured as "High impedance digital" in
    the schematic.  The high impedance digital setting is used to prevent
    a glitch on the PWM output from causing the LED to flash each time the
    device powers up. */
	Pin_BlueLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);
	Pin_RedLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);
	Pin_GreenLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);
		
	/* Initialize CapSense */
	CapSense_Start();
	CapSense_InitializeAllBaselines();

	/* Initialize USB */
	USBFS_Start(0, USBFS_5V_OPERATION);		
	
	/* Initiate the first scan of all the sensors */
	CapSense_ScanEnabledWidgets();	
}


/* [] END OF FILE */

