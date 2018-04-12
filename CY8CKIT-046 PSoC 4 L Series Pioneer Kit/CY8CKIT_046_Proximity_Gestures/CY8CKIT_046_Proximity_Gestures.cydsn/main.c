/******************************************************************************
* Project Name		: CY8CKIT_046_Proximity_Gestures
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
* Theory of Operation: This example demonstrates the proximity gesture detection 
* capability of PSoC 4200L device. Note that proximity sensor wires must be 
* connected to P9[4] and P9[5] of the CY8CKIT-046 PSoC 4 L-Series Pioneer Kit 
* for this project. Refer to the PSoC 4 L-Series Pioneer Kit User Guide for more
* details on hardware connection and testing of this project. 
*
* The PSoC 4200L detects a hand gesture using the proximity sensor by detecting
* the order in which the two proximity sensors get activated. 
*
* Two proximity gestures are detected by this project: Wave UP and Wave DOWN. 
* If a Wave UP gesture is detected, the on-board RGB LED changes color in the 
* order RED > GREEN > BLUE > RED. If a Wave DOWN gesture is detected, the 
* on-board RGB LED changes color in the order RED > BLUE > GREEN > RED. The color
* changes from one to another each time a gesture is detected. 
* 
* If proximity is not detected, the PSoC 4200L gangs the two proximity sensors
* and scans the combined proximity sensor every 200ms. This is to achieve lower 
* average power consumption. If proximity is detected during sensor scan, 
* PSoC 4200L starts scanning individual proximity sensors every 20ms. If the
* proximity is absent for a period more than 3 seconds, PSoC 4200L scans only 
* the combined proximity sensor and increases the interval between sensor 
* scans to 200ms. This project also modulates the intensity of the RGB LED 
* based on the distance of hand from the proximity sensors. The LED stays in
* OFF state after the device reset until proximity is detected.
*
* Note: The time of 30 ms, 200 ms and 3 s are dependent on LFCLK accuracy. This 
* example uses the 32.768-kHz WCO in the kit as the LFCLK source. This offers a 
* 250 ppm accuracy on the time. If WCO is not used or available, ILO can be 
* used to source LFCLK. In the case of ILO, the accuracy of the time can vary as 
* much as 60 percent. Use the ILO trim method provided in the 
* CY8CKIT_046_Deep_Sleep_Blinky example to improve the ILO accuracy to 10 percent.
*******************************************************************************/

#include "main.h"

extern uint16 brightness;   

int main()
{	
#if (TUNER_ENABLE == ENABLED)
    /* Enable global interrupts */
    CyGlobalIntEnable;
    
    /* Enable the two individual proximity sensors. */	
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR0__PROX);
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR1__PROX);
	CapSense_EnableWidget(CapSense_PROXIMITYSENSORCOMBINED__PROX);
    
    /* Make the RGB LED pins analog impedance to turn the LED Off */
	Pin_RedLED_SetDriveMode(Pin_RedLED_DM_ALG_HIZ);
	Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_ALG_HIZ);
	Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_ALG_HIZ);
    
    /* Start CapSense Tuner */
    CapSense_TunerStart();
    
    /* Run CapSense Tuner */
    while(1)
    {
        CapSense_TunerComm();
    }
#else    
    /* Variable used to store the gesture state */
	GESTURE gesture = GESTURE_NO_GESTURE;
    
    /* Local Variable to calculate led brightness */
    uint32 ledBrightness = 0; 
	
	/* Variable for storing filtered value of combined proximity sensor signal */
	uint32 filteredSignal = 0;
    
	/* Variables used to store current and previous state of combined proximity
     * sensor.
     */
	bool currentCombinedProxState  = false;
	bool prevCombinedProxState  = false;
    
    /* Variable used to store state of combined proximity sensor for 3 seconds 
     * after the combined proximity sensor turns OFF.
     */
    bool combinedProxStateTemp;
	
	/* This variable is used to implement a software counter. If the value 
	 * of this counter is greater than 150, the PSoC 4200L proximity sensor
	 * was inactive for more than 3s. */
	uint32 softCounter = RESET;
	
	/* Starts all the Components of the project. Enables the global interrupt. */
	Initialize_Project();
	
	for(;;)
    {	
		/* Scan the proximity sensor individually if the previous scans did not detect
         * proximity. 
         */
		if(prevCombinedProxState == false)
		{		
			/* Scan the combined proximity sensor. */						
			CapSense_ScanSensor(CapSense_PROXIMITYSENSORCOMBINED__PROX);
			
			/* Wait till the scan is complete. */
			while(CapSense_IsBusy())
			{
				/* Put the PSoC 4200L in Sleep power mode while the CapSense is scanning. 
				 * The device wakes up using the interrupt generated by CapSense CSD Component 
				 * after scanning. */
				CySysPmSleep();
			}
		
            /* Update the baseline of the proximity sensor. */
			CapSense_UpdateSensorBaseline(CapSense_PROXIMITYSENSORCOMBINED__PROX);
            
			/* Check if the combined proximity sensor is active. */
			currentCombinedProxState = CapSense_CheckIsWidgetActive(CapSense_PROXIMITYSENSORCOMBINED__PROX); 
		}

		/* Scan the sensors individually if the combined proximity sensor scan 
         * detected proximity 
         */
		if(currentCombinedProxState == true)
		{
            /* Update the WDT match value to set lower scan interval if 
             * proximity sensor state just changed from inactive to active.  
             */
			if(prevCombinedProxState == false)
			{
				UpdateWdtMatchValue(WDT_MATCH_VALUE_30MS);				
			}
			
			/* Scan the individual proximity sensors */
			CapSense_ScanEnabledWidgets();

			/* Wait till the scan is complete. */
			while(CapSense_IsBusy())
			{
				/* Put the PSoC 4200L in Sleep power mode while the CapSense is scanning. 
				 * The device wakes up using the interrupt generated by CapSense CSD Component 
				 * after scanning. */
				CySysPmSleep();
			}

			/* Update the baseline of the proximity sensor. */
			CapSense_UpdateEnabledBaselines();

            /* Control LED brightness based on proximity sensor signal present
             * on combined proximity sensor
             */
            combinedProxStateTemp = CapSense_CheckIsWidgetActive(
                                        CapSense_PROXIMITYSENSORCOMBINED__PROX
                                                                );
            if(combinedProxStateTemp == true)
            {
                /* Temporary variable for filtering signal */
				uint32 tmp;

		        /* IIR = 1/4 Current Value + 3/4 Previous Value */
		        tmp = (uint32)SIGNAL + (uint32)filteredSignal;
		        tmp += ((uint32)filteredSignal << 1u);
		        tmp >>= 2u;
				
				/* Update the filteredSignal variable */
				filteredSignal = tmp;
				
				ledBrightness = (
                            MIN_BRIGHTNESS_COMPARE +
                            ((BRIGHTNESS_STEP * filteredSignal)/
                            SIGNAL_CHANGE_STEP)
                                    );
                /* Limit the brightness compare value to 16 bit */
                if(ledBrightness > MAX_16_BIT_VALUE)
        		{
                    brightness = MAX_16_BIT_VALUE;
                }
                else
    			{
    				brightness = (uint16)ledBrightness;
    			}
            }
            
			/* Check if a gesture is detected. */
			gesture = DetectGesture();
            
            /* Show the LED color change if a valid gesture is detected. */
			switch(gesture)
            {
            case GESTURE_UP_SWIPE:
                DisplayLEDOutput(DISPLAY_UP_SWIPE);
                /* Reset soft counter since sensors are active */
                softCounter = RESET;
                break;
                
            case GESTURE_DOWN_SWIPE:
                DisplayLEDOutput(DISPLAY_DOWN_SWIPE);
                /* Reset soft counter since sensors are active */
                softCounter = RESET;
                break; 
                
            case GESTURE_NO_GESTURE:
                DisplayLEDOutput(DISPLAY_LED_PREV_COLOR);
                /* Reset soft counter since sensors are active */
                softCounter = RESET;
                break;
                
            case GESTURE_SENSORS_INACTIVE:    
                if(combinedProxStateTemp == false)
                {
                    /* Increment the software counter if sensors are inactive. */
    				softCounter++;
    				/* 3 seconds after the proximity becomes inactive, update the WDT match 
                     * value to increase the scan interval 
                     */
                    if(softCounter == SOFT_COUNTER_3SEC_VALUE)
                    {
                        /* Proximity sensor is inactive for more than 3s. 
        				 * Increase DeepSleep duration to 200ms 
                         */
        				UpdateWdtMatchValue(WDT_MATCH_VALUE_200MS);
                        
                        /* Clear the combined Sensor status to re-initiate the scan 
        				 * of combined sensor. */
        				currentCombinedProxState = false;
                        
                        /* Turn off the LED if proximity is not detected for 3 secs */
        			    DisplayLEDOutput(DISPLAY_LED_OFF);
                        
                        /* Reset the software counter if any sensor is active. */
    				    softCounter = RESET;      
                    }
                }
                break;
                
            default:
                break;
            }
            
			/* Put CPU in sleep mode after completing the gesture 
			 * recognition and displaying output on LED. */
			CySysPmSleep();
		}
		else /* Combined proximity sensor is inactive.*/
		{					
			/* Put device to deep sleep mode */
			EnterDeepSleepLowPowerMode();
		}
        
         /* Store the previous state of combined proximity sensor */
        prevCombinedProxState = currentCombinedProxState;
	}
    
#endif    
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

	/* Start the CapSense block and initialize the baselines of all CapSense sensors */	
	CapSense_Start();	
	CapSense_InitializeAllBaselines();
	
	/* Enable the all the proximity sensors in the project */	
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR0__PROX);
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR1__PROX);
	CapSense_EnableWidget(CapSense_PROXIMITYSENSORCOMBINED__PROX);
	
 	/* Initialize the Watchdog Timer with a match value corresponding to 200ms. */
    CySysWdtSetMatch(CY_SYS_WDT_COUNTER0 ,WDT_MATCH_VALUE_200MS);
       	
	/* Enable and start the PWM blocks. */	
	PWM_Red_Start();
	PWM_Green_Start();
	PWM_Blue_Start();
	
	/* Setting the drive mode to strong to enable the LED output. This is
    required because the pin is configured as "High impedance digital" in
    the schematic.  The high impedance digital setting is used to prevent
    a glitch on the PWM output from causing the LED to flash each time the
    device powers up. */
	Pin_BlueLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);
	Pin_RedLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);
	Pin_GreenLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);

	/* Set red color as default color. The variable prevLEDColor 
	 * is initialized to red color. */
	DisplayLEDOutput(DISPLAY_LED_PREV_COLOR);
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
	
	/* Make the pins connected to the LEDs Hi-Z to save power. */
	Pin_RedLED_SetDriveMode(Pin_RedLED_DM_ALG_HIZ);
	Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_ALG_HIZ);
	Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_ALG_HIZ);
	
	/* Prepare the PWM components for DeepSleep power mode entry. */
	PWM_Red_Sleep();
	PWM_Green_Sleep();
	PWM_Blue_Sleep();
	
	/* Enter DeepSleep. */
	CySysPmDeepSleep();	
	
	/* The device has woken up from DeepSleep power mode. 
	 * Reconfigure PWM components. */
	PWM_Red_Wakeup();
	PWM_Green_Wakeup();
	PWM_Blue_Wakeup();
	
	/* Set the pin connected to the LEDs to be in strong drive
	 * mode for normal operation. */
	Pin_RedLED_SetDriveMode(Pin_RedLED_DM_STRONG);
	Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_STRONG);
	Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_STRONG);
	
	/* Reconfigure CapSense CSD Component after DeepSleep */
	CapSense_Wakeup();
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
    currentCountVal = CySysWdtGetCount(CY_SYS_WDT_COUNTER0);
	
	/* Reset the counter if the current count value is greater than
	 * the new match value */
    if (currentCountVal > wdtMatchValue)
    {
        CySysWdtResetCounters(CY_SYS_WDT_COUNTER0_RESET);
	}
}

/* [] END OF FILE */
