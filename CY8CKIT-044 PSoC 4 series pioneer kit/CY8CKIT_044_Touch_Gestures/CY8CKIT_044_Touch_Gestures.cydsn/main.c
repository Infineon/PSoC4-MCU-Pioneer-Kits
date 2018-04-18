/*******************************************************************************
* Project Name		: CY8CKIT_044_Touch_Gestures
* Version			: 1.0
* Device Used		: CY8C4247AZI-M485     
* Software Used		: PSoC Creator 3.2
* Compiler Used		: ARM GCC 4.8.4 
* Related Hardware	: CY8CKIT-044 PSoC 4 M-Series Pioneer Kit 
*********************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation.
********************************************************************************
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
********************************************************************************/
/********************************************************************************
* Theory of Operation: This project demonstrates the touch gesture detection 
* capability of PSoC 4200M device. Refer to the PSoC 4 M-Series Pioneer Kit User 
* Guide for more details on testing of this project. 
*
* The PSoC 4200M detects a touch gesture by checking the order in which individual 
* sensors on the gesture pad is activated. Six touch gestures are detected by this 
* project: Swipe UP, Swipe DOWN, Swipe LEFT, Swipe RIGHT, Swipe CLOCKWISE, and 
* Swipe COUNTER-CLOCKWISE. The color of the RGB LED is changed in the following 
* fashion for each detected gesture. The color changes from one to another 
* each time a valid gesture is detected.
* _______________________________________________________________________________________________
* GESTURE					| ACTION															|
* ----------------------------------------------------------------------------------------------|
* Swipe LEFT				| RED > GREEN > BLUE > RED											|
* Swipe RIGHT				| RED > BLUE > GREEN > RED											|
* Swipe CLOCKWISE			| VIOLET > INDIGO > BLUE > GREEN > YELLOW > ORANGE > RED > VIOLET	|
* Swipe COUNTER-CLOCKWISE	| RED > ORANGE > YELLOW > GREEN > BLUE> INDIGO > VIOLET > RED		|
* Swipe UP					| RGB LED Brightness Increases										|
* Swipe UP					| RGB LED Brightness Decreases										|
* -----------------------------------------------------------------------------------------------
* 
* If proximity is not detected, the PSoC 4200M gangs the all touch sensors 
* as a combined proximity sensor and scans the combined proximity sensor every 
* 100ms. This is to achieve lower average power consumption. If proximity is 
* detected during sensor scan, PSoC 4200M starts scanning individual sensors
* every 10ms. If the proximity is absent for a period more than 3 seconds, 
* PSoC 4200M scans only the combined proximity sensor and increases the interval
* between sensor scans to 100ms. 
*
* Note: The time periods of 10ms, 100ms, and 3 seconds are dependent on the 
* LFCLK accuracy. These time periods can be inaccurate up to 60% when ILO is used 
* as the clock source for LFCLK. Use the ILO Trim Component as demonstrated in 
* the CY8CKIT_044_Deep_Sleep_Blinky project to achieve higher accuracy with ILO.
*********************************************************************************/
#include "project.h"
#include "main.h"
#include "CapSense_Gestures.h"
#include "LED_Control.h"

/* Change this define to TRUE if you are using uC/Probe with this project. */
#define uCProbeEnabled			FALSE

#if(uCProbeEnabled)
	#include "string.h"
#endif

/* Variables to store touch states and show in uC/Probe. Declared volatile 
 * to avoid compile optimization of the variables. */
volatile uint8 uC_left_btnCurrent = INACTIVE;
volatile uint8 uC_right_btnCurrent  = INACTIVE;
volatile uint8 uC_up_btnCurrent  = INACTIVE;
volatile uint8 uC_down_btnCurrent  = INACTIVE;
volatile uint8 uC_centre_btnCurrent  = INACTIVE;

/* Variable to show the detected gesture on uC/Probe. */
char uC_Probe_Gesture[UC_PROBE_TEXT_SIZE] = UC_PROBE_NO_GESTURE;

int main()
{	
	/* Variable used to store the gesture state */
	uint8 gesture = NO_GESTURE;
	
	/* Variable used to store the status of combined proximity sensor. */	
	uint8 proximity_combined  = INACTIVE;
	
	/* This variable is used to implement a software counter. If the value 
	 * of this counter is greater than 300, the PSoC 4200M proximity sensor
	 * was inactive for more than 3s. */
	uint16 softCounter = RESET;
	
	/* Variable to store the WDT match value. */
	uint16 currentCountVal = ZERO;
	
	/* Variable to determine whether to update the WDT match value. */
	uint8 updateMatch = UPDATE_10MS;
	
	/* Starts all the Components of the project. Enables the global interrupt. */
	Initialize_Project();

	for(;;)
    {	
		/* Scan the proximity sensor only if the previous scans detected no touch. */
		if(proximity_combined == INACTIVE)
		{			
			/* Scan the combined proximity sensor. */						
			CapSense_ScanSensor(CapSense_COMB_PROX__PROX);
			
			while(CapSense_IsBusy())
			{
				/* Put the PSoC 4200M in Sleep power mode while the CapSense is scanning. 
				 * The device wakes up using the interrupt generated by CapSense CSD Component 
				 * after scanning. */
				CySysPmSleep();
			}
					
			/* Update the baseline of the proximity sensor. */
			CapSense_UpdateSensorBaseline(CapSense_COMB_PROX__PROX);

			/* Check if proximity sensor is active. */
			proximity_combined = CapSense_CheckIsWidgetActive(CapSense_COMB_PROX__PROX);			
		}

		if(proximity_combined == ACTIVE)
		{
			/* Update the WDT if required. */
			if(updateMatch == UPDATE_10MS)
			{
				/* Update the WDT match value for a period corresponding
				 * to 5ms if proximity is detected. */
				CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0, WDT_MATCH_VALUE_10MS);
				
				/* Read the current count value of the WDT. */
		        currentCountVal = CySysWdtReadCount(CY_SYS_WDT_COUNTER0);
				
				/* Reset the counter if the count value is greater than
				 * the match value */
		        if (currentCountVal > WDT_MATCH_VALUE_10MS)
		        {
		            CySysWdtResetCounters(CY_SYS_WDT_COUNTER0_RESET);
				}
				
				/* Provide a delay to allow the changes on WDT register to take 
				 * effect. This is approximately 4 LFCLK cycles. */
				CyDelayUs(ILOX4);
				
				/* Next update to the WDT match value is required only when
				 * switching to 100ms scan interval. */
				updateMatch = UPDATE_100MS;
			}

			/* Switch ON the RGB LED and show the previous color. */
			DisplayLEDOutput(prevLEDColor);

			/* Scan all sensors individually. */			
			CapSense_ScanEnabledWidgets();
			
			while(CapSense_IsBusy())
			{	
				/* Put the PSoC 4200M in Sleep power mode while the CapSense is scanning. 
				 * The device wakes up using the interrupt generated by CapSense CSD Component 
				 * after scanning. */
				CySysPmSleep();
			}
		
			/* Update baseline of sensors of gesture pad. */
		    CapSense_UpdateEnabledBaselines();	

			/* Check if a gesture is detected. */
			gesture = DetectGesture();
			
			if(gesture == SENSORS_INACTIVE)
			{
				/* Increment the software counter if sensors are inactive. */
				softCounter++;
				
				if(softCounter >= MAX_VALUE)
				{
					/* Set the maximum limit for the variable softCounter. */
					softCounter = MAX_VALUE;
					
					/* Scan the combined proximity sensor. */						
					CapSense_ScanSensor(CapSense_COMB_PROX__PROX);
					
					while(CapSense_IsBusy())
					{
						/* Put the PSoC 4200M in Sleep power mode while the CapSense is scanning. 
						 * The device wakes up using the interrupt generated by CapSense CSD Component 
						 * after scanning. */
						CySysPmSleep();
					}
							
					/* Update the baseline of the proximity sensor. */
					CapSense_UpdateSensorBaseline(CapSense_COMB_PROX__PROX);
					
					/* Update the WDT if required. */
					if(updateMatch == UPDATE_100MS)
					{
						/* Proximity sensors are inactive for more than 3s. 
						 * Increase Deep-Sleep duration. */ 
						CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0, WDT_MATCH_VALUE_100MS);	
						
						/* Read the current count value of the WDT. */
				        currentCountVal = CySysWdtReadCount(CY_SYS_WDT_COUNTER0);
						
						/* Reset the counter if the count value is greater than
						 * the match value */
				        if (currentCountVal > WDT_MATCH_VALUE_100MS)
				        {
				            CySysWdtResetCounters(CY_SYS_WDT_COUNTER0_RESET);
						}

						/* Provide a delay to allow the changes on WDT register to take 
						 * effect. This is approximately 4 LFCLK cycles. */
						CyDelayUs(ILOX4);
						
						/* Next update to the WDT match value is required only when
				 		 * switching to 10ms scan interval. */
						updateMatch = UPDATE_10MS;
					}
					
					/* Turn off the LED. */
					DisplayLEDOutput(OFF);
					
					/* Clear the combined Sensor status to re-initiate the scan 
					 * of combined sensor. */
					proximity_combined = INACTIVE;
				}
			}
			else /* One of the sensors is active. */
			{
				/* Reset the software counter if any sensor is active. */
				softCounter = RESET;
			}
			
			if((gesture != SENSORS_INACTIVE) && (gesture != NO_GESTURE))
			{
				/* Show the LED color change if a valid gesture is detected. */
				DisplayLEDOutput(gesture);
				
				#if(uCProbeEnabled)
					/* Copy the text to be shown on the uC/Probe to the global variable. */
					strcpy(&uC_Probe_Gesture[ZERO], &gestureList[gesture][ZERO]);
				#endif
			}
			
			if(softCounter < MAX_VALUE)
			{
				/* Put CPU in sleep mode after completing the gesture 
			 	* recognition and displaying output on LED. */
				CySysPmSleep();
			}
		}
		
		if(proximity_combined == INACTIVE) 
		{
			/* Switch OFF the RGB LED if proximity is not detected. */
			DisplayLEDOutput(OFF);
			
			#if(uCProbeEnabled)
				/* Copy the text to be shown on the uC/Probe to the global variable. */
				strcpy(&uC_Probe_Gesture[ZERO], &gestureList[NO_GESTURE][ZERO]);
			#endif
			
			softCounter = RESET;
			
			/* Deep-Sleep low power modes is not available when the uC/Probe tool is active. 
			 * This section is not compiled if the macro uCProbeEnabled is set to TRUE. */
			#if(uCProbeEnabled)
				CySysPmSleep();
			#else
				/* Put CPU in Sleep mode instead of Deep-Sleep if uC/Probe tool is enabled. */
				EnterDeepSleepLowPowerMode();
			#endif
		}
	} 	
}

/*******************************************************************************
* Function Name: Initialize_Project
********************************************************************************
* Summary:
*  Starts and initializes all the Components in the project. Enables 
   global interrupt. 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Initialize_Project(void)
{
	/* Enable global interrupt. */
	CyGlobalIntEnable; 

	/* Enable and start the CapSense block. */	
	CapSense_Start();
	
	/* Initialize the baselines of all CapSense widgets. */ 
	CapSense_InitializeAllBaselines();
	    
 	/* Initialize the Watchdog Timer with a match value corresponding to 100ms. */
    CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0 ,WDT_MATCH_VALUE_100MS);
       	
	/* Enable and start the PWM blocks. */	
	PWM_Red_Start();
	PWM_Green_Start();
	PWM_Blue_Start();

	/* Show white color on RGB LED. The variable prevLEDColor 
	 * is initialized to show white color. */
	DisplayLEDOutput(prevLEDColor);
}

/*******************************************************************************
* Function Name: EnterDeepSleepLowPowerMode
********************************************************************************
* Summary:
*  Puts the device in Deep-Sleep power mode. Reconfigures the Components for 
*  normal operation after wake-up. 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void EnterDeepSleepLowPowerMode(void)
{
	/* Prepare CapSense CSD Component for Deep-Sleep power mode entry. */
	CapSense_Sleep();
	
	/* Make the pins connected to the LEDs analog impedance to save power. */
	Pin_RedLED_SetDriveMode(Pin_RedLED_DM_ALG_HIZ);
	Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_ALG_HIZ);
	Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_ALG_HIZ);
	
	/* Prepare the PWM components for Deep-Sleep power mode entry. */
	PWM_Red_Sleep();
	PWM_Green_Sleep();
	PWM_Blue_Sleep();
	
	/* Enter Deep-Sleep. */
	CySysPmDeepSleep();	
	
	/* The device has woken up from Deep-Sleep power mode. 
	 * Reconfigure PWM components. */
	PWM_Red_Wakeup();
	PWM_Green_Wakeup();
	PWM_Blue_Wakeup();
	
	/* Set the pin connected to the LEDs to be in strong drive
	 * mode for normal operation. */
	Pin_RedLED_SetDriveMode(Pin_RedLED_DM_STRONG);
	Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_STRONG);
	Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_STRONG);
	
	/* Reconfigure CapSense CSD Component after Deep-Sleep */
	CapSense_Wakeup();
}

/* [] END OF FILE */
