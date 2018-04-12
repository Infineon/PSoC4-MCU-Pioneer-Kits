/******************************************************************************
* Project Name		: CY8CKIT_046_CapSense_Proximity
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
* Theory of Operation: This project demonstrates the proximity sensing capability
* of PSoC 4200L device. Note that a proximity sensor wire must be connected to 
* P9[4] of the CY8CKIT-046 PSoC 4 L-Series Pioneer Kit for this project. Refer 
* to the PSoC 4 L-Series Pioneer Kit User Guide for more details on hardware  
* connection and testing of this project. 
*
* The PSoC 4200L detects an approaching hand using the proximity sensor. If the
* signal level detected by the proximity sensor is greater than a threshold value,
* the PSoC 4200L turns on the Blue LED on the PSoC 4 L-Series Pioneer Kit. The 
* brightness of the LED is increased as the hand approaches the proximity sensor.
* To achieve lower average power consumption, the PSoC 4200L scans the proximity
* sensor every 100ms when proximity is not detected. If proximity is detected 
* during sensor scan, PSoC 4200L starts scanning the sensor every 30ms. If the
* proximity is absent for a period more than 3 seconds, PSoC 4200L increases the
* interval between sensor scans and sensor scan is repeated every 100ms. 
*
* Note: The time of 30 ms and 100 ms are dependent on LFCLK accuracy. This 
* example uses the 32.768-kHz WCO in the kit as the LFCLK source. This offers a 
* worst-case 250 ppm accuracy on the time. If WCO is not used or available, ILO can be 
* used to source LFCLK. In the case of ILO, the accuracy of the time can vary as 
* much as 60 percent. Use the ILO trim method provided in the 
* CY8CKIT_046_Deep_Sleep_Blinky example to improve the ILO accuracy to 10 percent.
*******************************************************************************/

#include <project.h>
#include <stdbool.h>
#include "main.h"

int main()
{	
#if (TUNER_ENABLE == ENABLED)
    /* Enable global interrupts */
    CyGlobalIntEnable;
    
    /* Enable the proximity sensors. */	
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR__PROX);
        
    /* Start CapSense Tuner */
    CapSense_TunerStart();
    
    /* Run CapSense Tuner */
    while(1)
    {
        CapSense_TunerComm();
    }
#else
    /* Initialize current and previous Proximity sensor state to inactive. */
	bool currentProxSensorState  = false;
	bool prevProxSensorState  = false;
	
	/* This variable is used to implement a software counter. If the value 
	 * of this counter is greater than 100, the proximity sensor was inactive
	 * for more than 3s. */
	uint8 softCounter = RESET;
	
	/* Enable global interrupt. */
	CyGlobalIntEnable; 

	/* Start the CapSense block and initialize the sensor baselines. */	
	CapSense_Start();	
	CapSense_InitializeSensorBaseline(CapSense_PROXIMITYSENSOR__PROX);	
		
	PWM_Blue_Start();
	
	/* Setting the drive mode to strong to enable the LED output. This is
    required because the pin is configured as "High impedance digital" in
    the schematic.  The high impedance digital setting is used to prevent
    a glitch on the PWM output from causing the LED to flash each time the
    device powers up. */
	Pin_BlueLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);	
	
	/* Switch off the Blue LED. */
	LED_SetBrightness(OFF, ZERO);
    
    /* Initialize WDT match value for a scan interval of 100ms*/
    UpdateWdtMatchValue(WDT_MATCH_VALUE_100MS);
	
	for(;;)
    {	
		/* Update the baseline of the proximity sensor. */
		CapSense_UpdateSensorBaseline(CapSense_PROXIMITYSENSOR__PROX);
		
		/* Scan the proximity sensor. */						
		CapSense_ScanSensor(CapSense_PROXIMITYSENSOR__PROX);
		
		/* Wait till the scan is complete*/
		while(CapSense_IsBusy())
		{
			/* Put the PSoC 4200L in Sleep power mode while the CapSense is scanning. 
			 * The device wakes up using the interrupt generated by CapSense CSD Component 
			 * after scanning. */
			CySysPmSleep();
		}
	
		/* Check if proximity sensor is active. */
		currentProxSensorState = (bool)CapSense_CheckIsSensorActive(
                                                CapSense_PROXIMITYSENSOR__PROX);

		if(currentProxSensorState)
		{
			/* Set the LED at a brightness level corresponding 
			 * to the proximity distance. 
             */
			LED_SetBrightness(
                            ON, 
                    CapSense_GetDiffCountData(CapSense_PROXIMITYSENSOR__PROX)
                              );
			
			/* Update the WDT match value to set lower scan interval if 
             * proximity sensor state just changed from inactive to active.  
             */
			if(prevProxSensorState == false)
			{
				UpdateWdtMatchValue(WDT_MATCH_VALUE_30MS);
			}
			
			/* Reset the software counter if proximity is detected. */
			softCounter = RESET;
			
			/* Put CPU in sleep mode after displaying output on LED. 
             * Note that CPU sleep mode is used instead of DeepSleep as the 
             * IMO needs to be ON for LED brightness control by PRS PWM.
             */
			CySysPmSleep();
		}
		else /* Proximity sensor is inactive. */
		{
			/* Switch off the LED if proximity is not detected. */
			LED_SetBrightness(OFF, ZERO);
			
			/* Increment the software counter every 30ms if proximity is not 
             * detected. 
             */
			softCounter++;
			
			/* 3 seconds after the proximity is removed, update the WDT match 
             * value to increase the scan interval 
             */
            if(softCounter == SOFT_COUNTER_3SEC_VALUE)
            {
                /* Proximity sensor is inactive for more than 3s. 
				 * Increase DeepSleep duration to 100ms 
                 */
				UpdateWdtMatchValue(WDT_MATCH_VALUE_100MS);
            }
            
            /* Limit the softCounter maximum value to prevent overflow due to 
             * increment in every loop
             */
			if(softCounter > SOFT_COUNTER_3SEC_VALUE)
			{
				/* Set the maximum limit for the softCounter. */
				softCounter = SOFT_COUNTER_3SEC_VALUE;
			}

			/* Put the device to sleep */
			EnterDeepSleepLowPowerMode();
		
		}
	} 	
#endif    
}

/*******************************************************************************
* Function Name: EnterDeepSleepLowPowerMode
********************************************************************************
* Summary:
*  Puts the device in DeepSleep power mode. Reconfigures the Components for 
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
	/* Prepare CapSense CSD Component for DeepSleep power mode entry. */
	CapSense_Sleep();
	
	/* Make the pin connected to Blue LED high impedance to save power. 
	 * This is also required to prevent a glitch on the PWM output which 
	 * causes the LED to flash each time the device wakes up from the 
	 * DeepSleep low power mode. */
	Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_ALG_HIZ);
	
	/* Prepare the PWM component for DeepSleep power mode entry. */
	PWM_Blue_Sleep();
	
	/* Enter DeepSleep. */
	CySysPmDeepSleep();	
	
	/* The device has woken up from DeepSleep power mode. 
	 * Reconfigure PWM component. */
	PWM_Blue_Wakeup();
	
	/* Set the pin connected to Blue LED to be in strong drive
	 * mode for normal operation. */
	Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_STRONG);
	
	/* Reconfigure CapSense CSD Component after DeepSleep */
	CapSense_Wakeup();
}

/*******************************************************************************
* Function Name: LED_SetBrightness
********************************************************************************
* Summary:
*  Sets the Blue LED on PSoC 4 L-Series Pioneer Kit at a brightness level 
*  corresponding value passed to this function.
*
* Parameters:
* 	ledState: LED ON/OFF state 
*	signal: The proximity signal value corresponding to which the LED brightness
           has to be changed.
*
* Return:
*  void
*
*******************************************************************************/

void LED_SetBrightness(bool ledState, uint16 signal)
{
    uint32 ledBrighness = ZERO;	
	
	/* Scale the signal value for a visible brightness change. */
	if(ledState)
	{
		ledBrighness = (
                        MIN_BRIGHTNESS_COMPARE +
                        ((BRIGHTNESS_STEP * (signal - MIN_PROXIMITY_SIGNAL))/
                        SIGNAL_CHANGE_STEP)
                        );
        
		/* Limit the brightness compare value to 16 bit */
        if(ledBrighness > MAX_16_BIT_VALUE)
		{
            ledBrighness = MAX_16_BIT_VALUE;
        }
	}
	else
	{
		ledBrighness = ZERO;
	}
	
	/* Update the PWM compare value to change the LED brightness. */
	PWM_Blue_WriteCompare((uint16)ledBrighness);
}

/*******************************************************************************
* Function Name: UpdateWdtMatchValue
********************************************************************************
* Summary: 
*  This function updates the match value of the Watchdog Timer equal to the 
*  input parameter.
*  It also checks the current count value of Watchdog Timer and resets the 
*  timer if current count value is greater than the new match value.
*
* Parameters:
*  wdtMatchValue: Watchdog timer match value to be set.
*
* Return:
*  void
*
*******************************************************************************/
void UpdateWdtMatchValue(uint32 wdtMatchValue)
{
	/* Variable to store current count value of Watchdog Timer */
	uint16 currentCountVal = ZERO;
    
    /* Update WDT match value as equal to the input parameter */
	CySysWdtSetMatch(CY_SYS_WDT_COUNTER0, wdtMatchValue);
	
	/* Read the current count value of the WDT. */
    currentCountVal = CySysWdtReadCount(CY_SYS_WDT_COUNTER0);
	
	/* Reset the counter if the current count value is greater than
	 * the new match value */
    if (currentCountVal > wdtMatchValue)
    {
        CySysWdtResetCounters(CY_SYS_WDT_COUNTER0_RESET);
	}
	
	/* Provide a delay to allow the changes on WDT register to take 
	 * effect. This is approximately 4 LFCLK cycles. */
	CyDelayUs(ILOX4);
}

/* [] END OF FILE */
