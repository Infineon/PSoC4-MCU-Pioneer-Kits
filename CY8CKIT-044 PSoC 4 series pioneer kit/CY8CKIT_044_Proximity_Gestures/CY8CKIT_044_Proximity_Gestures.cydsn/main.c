/******************************************************************************
* Project Name		: CY8CKIT_044_Proximity_Gestures
* Version			: 1.0
* Device Used		: CY8C4247AZI-M485     
* Software Used		: PSoC Creator 3.2
* Compiler Used		: ARM GCC 4.8.4 
* Related Hardware	: CY8CKIT-044 PSoC 4 M-Series Pioneer Kit 
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
* Theory of Operation: This project demonstrates the proximity gesture detection 
* capability of PSoC 4200M device. Note that proximity sensor wires must be 
* connected to P3[7] and P3[6] of the CY8CKIT-044 PSoC 4 M-Series Pioneer Kit for this
* project. Refer to the PSoC 4 M-Series Pioneer Kit User Guide for more details on 
* hardware connection and testing of this project. 
*
* The PSoC 4200M detects a hand gesture using the proximity sensor. If the
* signal level detected by the proximity sensors is greater than a threshold value
* in a particular order, the PSoC 4200M detects if a gesture is made by the 
* approaching hand. 
*
* Two proximity gestures are detected by this project: Wave UP and Wave DOWN. 
* If a Wave UP gesture is detected, the onboard RGB LED changes color in the 
* order RED > GREEN > BLUE > RED. If a Wave DOWN gesture is detected, the 
* onboard RGB LED changes color in the order RED > BLUE > GREEN > RED. The color
* changes from one to another each time a gesture is detected. 
* 
* If proximity is not detected, the PSoC 4200M gangs the two proximity sensors
* and scans the combined proximity sensor every 200ms. This is to achieve lower 
* average power consumption. If proximity is detected during sensor scan, 
* PSoC 4200M starts scanning individual proximity sensors every 20ms. If the
* proximity is absent for a period more than 3 seconds, PSoC 4200M scans only 
* the combined proximity sensor and increases the interval between sensor 
* scans to 200ms. This project also modulates the intensity of the RGB LED 
* based on the distance of hand from the proximity sensors. The LED stays in
* OFF state after the device reset until proximity is detected.
*
* Note: The time periods of 20ms, 200ms, and 3 seconds are dependent on the 
* LFCLK accuracy. These time periods can be inaccurate up to 60% when ILO is used 
* as the clock source for LFCLK. Use the ILO Trim Component as demonstrated in 
* the CY8CKIT_044_Deep_Sleep_Blinky project to achieve higher accuracy with ILO.
*******************************************************************************/

#include "project.h"
#include "main.h"
#include "CapSense_Gestures.h"
#include "LED_Control.h"

/* Change this define to TRUE if you are using uC/Probe with this project. */
#define uCProbeEnabled			FALSE

/* Variables for uC/Probe. */
uint16 uC_Probe_CapSenseSensorSignal_0 = ZERO;
uint16 uC_Probe_CapSenseSensorSignal_1 = ZERO;
uint32 uC_Probe_LED = UC_PROBE_LED_DEFAULT_VAL;

/* Variable to store the LED color*/
uint8 prevLEDColor = WHITE;

int main()
{	
	/* Variable used to store the gesture state */
	uint8 gesture = NO_GESTURE;
	
	/* Variable used to store the status of combined proximity sensor. */	
	uint8 proximity_combined  = INACTIVE;
	
	/* This variable is used to implement a software counter. If the value 
	 * of this counter is greater than 150, the PSoC 4200M proximity sensor
	 * was inactive for more than 3s. */
	uint8 softCounter = RESET;
	
	/* Variable to store the WDT match value. */
	uint16 currentCountVal = ZERO;
	
	/* Variable to determine whether to update the WDT match value. */
	uint8 updateMatch = UPDATE_20MS;
	
	/* Starts all the Components of the project. Enables the global interrupt. */
	Initialize_Project();
	
	for(;;)
    {	
		/* Scan the proximity sensor only if the previous scans detected no proximity. */
		if(proximity_combined == INACTIVE)
		{		
			/* Update the baseline of the proximity sensor. */
			CapSense_UpdateSensorBaseline(CapSense_PROXIMITYSENSORCOMBINED__PROX);
			
			/* Scan the combined proximity sensor. */						
			CapSense_ScanSensor(CapSense_PROXIMITYSENSORCOMBINED__PROX);
			
			/* Wait till the scan is complete. */
			while(CapSense_IsBusy())
			{
				/* Put the PSoC 4200M in Sleep power mode while the CapSense is scanning. 
				 * The device wakes up using the interrupt generated by CapSense CSD Component 
				 * after scanning. */
				CySysPmSleep();
			}
		
			/* Check if the combined proximity sensor is active. */
			proximity_combined = CapSense_CheckIsWidgetActive(CapSense_PROXIMITYSENSORCOMBINED__PROX);
		}

		/* Scan the sensors individually if the combined proximity sensor scan detected a proximity */
		if(proximity_combined == ACTIVE)
		{
			/* Update the WDT if required. */
			if(updateMatch == UPDATE_20MS)
			{
				/* Update the WDT match value for a period corresponding to 
				 * 20ms if proximity is detected. */
				CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0, WDT_MATCH_VALUE_20MS);
				
				/* Read the current count value of the WDT. */
		        currentCountVal = CySysWdtReadCount(CY_SYS_WDT_COUNTER0);
				
				/* Reset the counter if the count value is greater than
				 * the match value */
		        if (currentCountVal > WDT_MATCH_VALUE_20MS)
		        {
		            CySysWdtResetCounters(CY_SYS_WDT_COUNTER0_RESET);
				}
				
				/* Provide a delay to allow the changes on WDT register to take 
				 * effect. This is approximately 4 LFCLK cycles. */
				CyDelayUs(ILOX4);
				
				/* Next update to the WDT match value is required only when
				 * switching to 200ms scan interval. */
				updateMatch = UPDATE_200MS;
				
				/* Switch ON the RGB LED and show the previous color. */
				DisplayLEDOutput(prevLEDColor);
			}
			
			/* Scan the individual proximity sensors */
			CapSense_ScanEnabledWidgets();

			/* Wait till the scan is complete. */
			while(CapSense_IsBusy())
			{
				/* Put the PSoC 4200M in Sleep power mode while the CapSense is scanning. 
				 * The device wakes up using the interrupt generated by CapSense CSD Component 
				 * after scanning. */
				CySysPmSleep();
			}

			/* Update the baseline of the proximity sensor. */
			CapSense_UpdateEnabledBaselines();
			
			#if(uCProbeEnabled)
				/* Variables to store the signal data from the proximity sensors. 
				 * This data is used by the uC/Probe tool to show the sensor status visually. */
				uC_Probe_CapSenseSensorSignal_0 = CapSense_SensorSignal[CapSense_PROXIMITYSENSOR0__PROX];
				uC_Probe_CapSenseSensorSignal_1 = CapSense_SensorSignal[CapSense_PROXIMITYSENSOR1__PROX];
			#endif

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
					
					/* Update the WDT if required. */
					if(updateMatch == UPDATE_200MS)
					{
						/* Proximity sensors are inactive for more than 3s. 
						 * Increase Deep-Sleep duration. */ 
						CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0, WDT_MATCH_VALUE_200MS);	
						
						/* Read the current count value of the WDT. */
				        currentCountVal = CySysWdtReadCount(CY_SYS_WDT_COUNTER0);
						
						/* Reset the counter if the count value is greater than
						 * the match value */
				        if (currentCountVal > WDT_MATCH_VALUE_200MS)
				        {
				            CySysWdtResetCounters(CY_SYS_WDT_COUNTER0_RESET);
						}
						
						/* Provide a delay to allow the changes on WDT register to take 
						 * effect. This is approximately 4 LFCLK cycles. */
						CyDelayUs(ILOX4);

						/* Next update to the WDT match value is required only when
				 		* switching to 20ms scan interval. */
						updateMatch = UPDATE_20MS;
					}
					
					/* Turn off the LED. */
					DisplayLEDOutput(OFF);
					
					/* Clear the combined Sensor status to re-initiate the scan 
					 * of combined sensor. */
					proximity_combined = INACTIVE;
					
					/* Reset the software counter. */
					softCounter = RESET;
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
			}

			/* Put CPU in sleep mode after completing the gesture 
			 * recognition and displaying output on LED. */
			CySysPmSleep();
		}
		else /* Combined proximity sensor is inactive.*/
		{
			/* Turn off the LED if proximity is not detected. */
			DisplayLEDOutput(OFF);
			
			/* Deep-Sleep low power mode is not available when the uC/Probe tool is active. 
			 * This section is not compiled if the macro uCProbeEnabled is set to TRUE. */
			#if(!uCProbeEnabled)
				EnterDeepSleepLowPowerMode();
			#else
				/* Put CPU in Sleep mode instead of Deep-Sleep if uC/Probe tool is enabled. */
				CySysPmSleep();
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
	
	/* Enable the two individual proximity sensors. */	
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR0__PROX);
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR1__PROX);
	CapSense_EnableWidget(CapSense_PROXIMITYSENSORCOMBINED__PROX);
	
 	/* Initialize the Watchdog Timer with a match value corresponding to 200ms. */
    CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0 ,WDT_MATCH_VALUE_200MS);
       	
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
