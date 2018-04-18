/******************************************************************************
* Project Name		: CY8CKIT_044_CapSense_Proximity
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
* Theory of Operation: This project demonstrates the proximity sensing capability
* of PSoC 4200M device. Note that a proximity sensor wire must be connected to 
* P3[7] of the CY8CKIT-044 PSoC 4 M-Series Pioneer Kit for this project. Refer to the
* PSoC 4 M-Series Pioneer Kit User Guide for more details on hardware connection and 
* testing of this project. 
*
* The PSoC 4200M detects an approaching hand using the proximity sensor. If the
* signal level detected by the proximity sensor is greater than a threshold value,
* the PSoC 4200M switches on the Green LED on the PSoC 4 M-Series Pioneer Kit. The 
* brightness of the LED is increased as the hand approaches the proximity sensor.
* To achieve lower average power consumption, the PSoC 4200M scans the proximity
* sensor every 100ms when proximity is not detected. If proximity is detected 
* during sensor scan, PSoC 4200M starts scanning the sensor every 30ms. If the
* proximity is absent for a period more than 3 seconds, PSoC 4200M increases the
* interval between sensor scans and sensor scan is repeated every 100ms. 
*
* Note: The time periods of 30ms and 100ms are dependent on the LFCLK accuracy. 
* These time periods can be inaccurate up to 60% when ILO is used as the clock 
* source for LFCLK. Use the ILO Trim Component as demonstrated in the 
* CY8CKIT_044_Deep_Sleep_Blinky project to achieve higher accuracy with ILO.
*******************************************************************************/

#include "project.h"
#include "main.h"

/* Change this define to TRUE if you are using uC/Probe with this project. */
#define uCProbeEnabled 		FALSE

/* Proximity status for uC/Probe. */
uint32 uC_Probe_Prox_Indicator = ZERO;

int main()
{	
	/* Proximity sensor state. */
	uint8 proximity  = INACTIVE;
	
	/* This variable is used to implement a software counter. If the value 
	 * of this counter is greater than 100, the proximity sensor was inactive
	 * for more than 3s. */
	uint8 softCounter = RESET;
	
	/* Variable to store the WDT match value. */
	uint16 currentCountVal = ZERO;
	
	/* Variable to determine whether to update the WDT match value. */
	uint8 updateMatch = UPDATE_100MS;
	
	/* Enable global interrupt. */
	CyGlobalIntEnable; 

	/* Enable and start the CapSense block. */	
	CapSense_Start();
	
	/* Initialize the baselines of all CapSense widgets. */ 
	CapSense_InitializeSensorBaseline(CapSense_PROXIMITYSENSOR__PROX);
	
	/* Enable and start PWM block. */	
	PWM_Green_Start();
	
	/* Switch off the Green LED. */
	LED_SetBrightness(ZERO);
	
	for(;;)
    {	
		/* Update the baseline of the proximity sensor. */
		CapSense_UpdateSensorBaseline(CapSense_PROXIMITYSENSOR__PROX);
		
		/* Scan the proximity sensor. */						
		CapSense_ScanSensor(CapSense_PROXIMITYSENSOR__PROX);
		
		/* Wait till the scan is complete*/
		while(CapSense_IsBusy())
		{
			/* Put the PSoC 4200M in Sleep power mode while the CapSense is scanning. 
			 * The device wakes up using the interrupt generated by CapSense CSD Component 
			 * after scanning. */
			CySysPmSleep();
		}
	
		/* Check if proximity sensor is active. */
		proximity = CapSense_CheckIsSensorActive(CapSense_PROXIMITYSENSOR__PROX);

		if(proximity == ACTIVE)
		{
			/* Set the LED at a brightness level corresponding 
			 * to the proximity distance. */
			LED_SetBrightness(ON);
			
			/* Update the WDT if required. */
			if(updateMatch == UPDATE_30MS)
			{
				/* Configure the wake up period to 30ms and update WDT match value. */
		    	CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0, WDT_MATCH_VALUE_30MS);
				
				/* Read the current count value of the WDT. */
			    currentCountVal = CySysWdtReadCount(CY_SYS_WDT_COUNTER0);
				
				/* Reset the counter if the count value is greater than
				 * the match value */
			    if (currentCountVal > WDT_MATCH_VALUE_30MS)
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
			
			/* Reset the software counter if proximity is detected. */
			softCounter = RESET;
			
			/* Put CPU in sleep mode after displaying output on LED. */
			CySysPmSleep();
		}
		else /* Proximity sensor is inactive. */
		{
			/* Switch off the LED if proximity is not detected. */
			LED_SetBrightness(OFF);
			
			/* Increment the software counter every 30ms if proximity is not detected. */
			softCounter++;
			
			/* Set the maximum limit for the soft counter. */
			if(softCounter >= MAX_VALUE)
			{
				/* Set the maximum limit for the softCounter. */
				softCounter = MAX_VALUE;
				
				/* Update the WDT if required. */
				if(updateMatch == UPDATE_100MS)
				{
					/* Proximity sensor is inactive for more than 3s. 
					 * Increase Deep-Sleep duration to 100ms and update WDT match value. */
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
					 * switching to 30ms scan interval. */
					updateMatch = UPDATE_30MS;
				}
			}

			/* Deep-Sleep low power mode is not available when the uC/Probe tool is active. 
			 *  We use the Sleep low power mode if the macro uCProbeEnabled is set to TRUE. */
			#if(uCProbeEnabled)
				CySysPmSleep();
			#else
				EnterDeepSleepLowPowerMode();
			#endif
		}
	} 	
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
	
	/* Make the pin connected to Green LED high impedance to save power. 
	 * This is also required to prevent a glitch on the PWM output which 
	 * causes the LED to flash each time the device wakes up from the 
	 * Deep-Sleep low power mode. */
	Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_ALG_HIZ);
	
	/* Prepare the PWM component for Deep-Sleep power mode entry. */
	PWM_Green_Sleep();
	
	/* Enter Deep-Sleep. */
	CySysPmDeepSleep();	
	
	/* The device has woken up from Deep-Sleep power mode. 
	 * Reconfigure PWM component. */
	PWM_Green_Wakeup();
	
	/* Set the pin connected to Green LED to be in strong drive
	 * mode for normal operation. */
	Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_STRONG);
	
	/* Reconfigure CapSense CSD Component after Deep-Sleep */
	CapSense_Wakeup();
}

/*******************************************************************************
* Function Name: LED_SetBrightness
********************************************************************************
* Summary:
*  Sets the Green LED on PSoC 4 M-Series Pioneer Kit at a brightness level 
*  corresponding value passed to this function.
*
* Parameters:
*  signal: The proximity signal value corresponding to which the LED brightness
           has to be changed.
*
* Return:
*  void
*
*******************************************************************************/
void LED_SetBrightness(uint8 mode)
{
	uint16 ledBrighness = ZERO;
	
	/* The variable signal holds a value between 0 and 255.
	 * Scale the value for a visible brightness change. */
	if(mode != OFF)
	{
		if(CapSense_SensorRaw[0] < PROX_UPPER_LIMIT)
		{
			ledBrighness = ((PWM_Green_PWM_PERIOD_VALUE * (CapSense_SensorRaw[0] - PROX_LOWER_LIMIT))/(PROX_UPPER_LIMIT - PROX_LOWER_LIMIT));
		}
		else
		{
			ledBrighness = PWM_Green_PWM_PERIOD_VALUE;
		}
	}
	else
	{
		ledBrighness = OFF;
	}
	
	/* Update the PWM compare value to change the LED brightness. */
	PWM_Green_WriteCompare(ledBrighness);
	
	/* This section of code is used to update the LED brightness on the
	 * uC/Probe GUI corresponding to the proximity distance. */
	#if(uCProbeEnabled)
		/* Variable to update the uC/Probe. */
		uC_Probe_Prox_Indicator = ledBrighness;
	#endif
}

/* [] END OF FILE */
