/******************************************************************************
* Project Name		: CY8CKIT_046_CapSense_Buttons
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
* Theory of Operation: This example demonstrates a simple CapSense button
* implementation with LED color and intensity control. The example also 
* demonstrates dual-channel CapSense available in the device. The CapSense
* Gesture Pad buttons are connected to CapSense block 0(CSD0) and the proximity 
* sensor is connected to the CapSense block 1(CSD1). CapSense buttons select the
* color and controls intensity of the RGB LED. Proximity provides a scaling factor
* to the intensity control. As hand gets closer to the proximity loop, the scaling
* factor increases and intensity of the LED changes relatively.
*******************************************************************************/
#include "main.h"

int main()
{		
	#if(TUNER_ENABLE == ENABLED)
		/* Process tuner code if Tuner is enabled */
		CyGlobalIntEnable; /* Enable global interrupts. */
        
		/* Based on the type of tuner enabled, process appropriate code */
		#if(TUNER_TYPE == TUNER_TYPE_PROXIMITY)
			CapSense_Proximity_EnableWidget(CapSense_Proximity_PROXIMITYSENSOR__PROX);
            CapSense_Proximity_TunerStart();
			
			while(TRUE)
			{
				CapSense_Proximity_TunerComm();
			}
		#else
			CapSense_Buttons_TunerStart();
			
			while(TRUE)
			{
				CapSense_Buttons_TunerComm();
			}
		#endif
	#else
	/* Process example code if tuner is disabled */
		        
		/* Variables to hold current and previous button status */
		uint32 buttonStatus = OFF, prevButtonStatus = OFF;		
		
		/* Variable to hold scaled proximity signal for brightness scaling */
		int32 proxSignal = BRIGHTNESS_STEP_MIN;
		
		/* Variable to store status of CSD0/CSD1 scan */
		uint32 scanStatus = 0;
				
		/* Variable for led color, brightness and ON/OFF status */
		int32 ledColor = COLOR_RED, ledBrightness = BRIGHTNESS_MAX, ledOn = OFF;
        
        CyGlobalIntEnable; /* Enable global interrupts. */
        
	    /* Initialize the CapSense buttons CSD block (CSD0) */
		CapSense_Buttons_Start();	
		CapSense_Buttons_InitializeAllBaselines();
		
		/* Initialize the CapSense proximity CSD block (CSD1) */
		CapSense_Proximity_Start();
		CapSense_Proximity_EnableWidget(CapSense_Proximity_PROXIMITYSENSOR__PROX);
		CapSense_Proximity_InitializeAllBaselines();
		
		/* Start and initialize the PWM blocks */
		PWM_Red_Start();
		PWM_Green_Start();
		PWM_Blue_Start();
        
		PWM_Red_WriteCompare(OFF);
		PWM_Green_WriteCompare(OFF);
		PWM_Blue_WriteCompare(OFF);
			
		/* Setting the drive mode to strong to enable the LED output. This is
        required because the pin is configured as "High impedance digital" in
        the schematic.  The high impedance digital setting is used to prevent
        a glitch on the PWM output from causing the LED to flash each time the
        device powers up. */
		Pin_BlueLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);
		Pin_RedLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);
		Pin_GreenLED_SetDriveMode(CY_SYS_PINS_DM_STRONG);
		
		/* Put the device to CPU sleep, WDT0 will wakeup the device every 20 ms
			This first sleep is executed to make sure all the scanning happens 
			synchronously every 20 ms */
		CySysPmSleep();
		
		/* Trigger the initial scan of the CapSense buttons and proximity sensor */
		CapSense_Buttons_ScanEnabledWidgets();
		CapSense_Proximity_ScanEnabledWidgets();	
		
	    for(;;)
	    {
			/* Check if CSD0 (CapSense buttons) completed scanning and the scan complete has not been processed */
			if((CapSense_Buttons_IsBusy() != TRUE) && ((scanStatus & CAPSENSE_BUTTON_SCAN_COMPLETE) != CAPSENSE_BUTTON_SCAN_COMPLETE))
			{				
				/* Update CapSense buttons baselines */
				CapSense_Buttons_UpdateEnabledBaselines();
				
				/* If any widget is reported active, process the button active status */
				if(CapSense_Buttons_CheckIsAnyWidgetActive())
				{
					/* If more than one button is active, suppress the unwanted sensors */
					/* If only one sensor is active, the sensor ON mask will be a power of 2 (only one bit set in the mask)
						The 'if' condition exploits a simple property of 2^n number to check if the number is 2^n or not 
						If it is 2^n, then only one sensor is active. Otherwise more than one sensors are active 
						
						Property of a 2^n number used below - 
							2^n AND (Two's Complement(2^n)) = 2^n
						The above property is ONLY valid for 2^n numbers */
					if(IsNotPowerOfTwo(CapSense_Buttons_sensorOnMask[0]))
					{					
						/* Call the CapSense_ValidateButtons() API to suppress unwanted buttons */
						buttonStatus = CapSense_ValidateButtons(buttonStatus);
					}
					else
					{
						/* If only one button is active, then directly use the CapSense ON mask */
						buttonStatus = CapSense_Buttons_sensorOnMask[0];
					}
				}
				else
				{
					/* Clear the button status if no sensor is active */
					buttonStatus = CLEAR;
				}
				
				/* Set the CapSense scan complete flag for CSD0 */
				scanStatus |= CAPSENSE_BUTTON_SCAN_COMPLETE;
				
				/* If any button is active, then process LED control activities */
				if(buttonStatus != OFF)
				{
					/* Detect the rising edge on buttonStatus */
					switch(RISING_EDGE_DET(buttonStatus, prevButtonStatus))
					{
						/* Left button activation, change LED color from R > G > B > R */
						case (LEFT_BUTTON_MASK):
                            if(ledOn == ON)
                            {
    							ledColor++;
    							if(ledColor > COLOR_BLUE)
    							{
    								ledColor = COLOR_RED;
    							}
                            }
						break;
						
						/* Right button activation, change LED color from R > B > G > R */
						case (RIGHT_BUTTON_MASK):
							if(ledOn == ON)
                            {
                                ledColor--;
    							if(ledColor < COLOR_RED)
    							{
    								ledColor = COLOR_BLUE;
    							}
                            }
						break;
						
						/* Up button activation, change LED brightness from min to max */
						case (UP_BUTTON_MASK):
							if(ledOn == ON)
                            {
                                ledBrightness++;
    							if(ledBrightness > BRIGHTNESS_MAX)
    							{
    								ledBrightness = BRIGHTNESS_MAX;
    							}
                            }
						break;
						
						/* Down button activation, change LED brightness from max to min */
						case (DOWN_BUTTON_MASK):
							if(ledOn == ON)
                            {
                                ledBrightness--;
    							if(ledBrightness < BRIGHTNESS_MIN)
    							{
    								ledBrightness = BRIGHTNESS_MIN;
    							}
                            }
						break;
						
						/* Centre button activation, toggle LED ON/OFF state */
						case (CENTRE_BUTTON_MASK):
							ledOn ^= ON;
						break;
						
						default:
						break;
					}
				}
			}
			
			/* Check if CSD1 (CapSense proximity) completed scanning and the scan complete has not been processed */
			if((CapSense_Proximity_IsBusy() != TRUE) && ((scanStatus & CAPSENSE_PROX_SCAN_COMPLETE) != CAPSENSE_PROX_SCAN_COMPLETE))
			{				
				/* Update CapSense proximity baseline */
				CapSense_Proximity_UpdateEnabledBaselines();
				
				/* Check if Proximity sensor is active or not */
				if((CapSense_Proximity_CheckIsAnyWidgetActive()) && (ledOn == ON))
				{
					/* Get the proximity signal if the proximity sensor is active */
					proxSignal = CapSense_Proximity_GetDiffCountData(CapSense_Proximity_SENSOR_PROXIMITYSENSOR_0__PROX);
					
					/* Shift right the proximity signal for proper brightness step control */
					proxSignal >>= PROXIMITY_SIGNAL_SHIFT_FACTOR;				
					
					/* Apply step min/max cap on the proximity signal */
					if(proxSignal < BRIGHTNESS_STEP_MIN)
					{					
						proxSignal = BRIGHTNESS_STEP_MIN;
					}
					else if(proxSignal > BRIGHTNESS_STEP_MAX)
					{					
						proxSignal = BRIGHTNESS_STEP_MAX;
					}
				}
				else
				{
					/* If proximity is not active, set the step size to minimum */
					proxSignal = BRIGHTNESS_STEP_MIN;
				}			
				
				/* Set the CapSense scan complete flag for CSD1 */
				scanStatus |= CAPSENSE_PROX_SCAN_COMPLETE;				
			}		
			
			/* Check if both CSD0 and CSD1 have completed their scan */
			if(scanStatus == CAPSENSE_SCAN_COMPLETE)
			{	
				/* Store the current button status to previous status */
				prevButtonStatus = buttonStatus;			
				
				/* If LED is ON, then process color/brightness control. Turn OFF LEDs otherwise */
				if(ledOn == ON)
				{
					/* Depending on the color of the LED selected turn on that PWM alone 
						Brightness of the PWM = ledBrightness (from Up/Down buttons) * proxSignal (scaled step from proximity) */
					if(ledColor == COLOR_RED)
					{
						PWM_Red_WriteCompare(ledBrightness * proxSignal);
						PWM_Green_WriteCompare(OFF);
						PWM_Blue_WriteCompare(OFF);					
					}
					else if(ledColor == COLOR_GREEN)
					{
						PWM_Red_WriteCompare(OFF);
						PWM_Green_WriteCompare(ledBrightness * proxSignal);	
						PWM_Blue_WriteCompare(OFF);
					}
					else if(ledColor == COLOR_BLUE)
					{
						PWM_Red_WriteCompare(OFF);
						PWM_Green_WriteCompare(OFF);					
						PWM_Blue_WriteCompare(ledBrightness * proxSignal);
					}
					
				}
				else
				{
					PWM_Red_WriteCompare(OFF);
					PWM_Green_WriteCompare(OFF);
					PWM_Blue_WriteCompare(OFF);
				}
				
				/* Put the device to CPU sleep as PWMs are running and we cannot enter DeepSleep */
                /* WDT0 is configured in the Clocks tab of .cydwr file to generate an interrupt every 20 ms */
				CySysPmSleep();
				
				/* Clear the scan status flag and trigger the next scan */
				scanStatus = CLEAR;
				CapSense_Buttons_ScanEnabledWidgets();
				CapSense_Proximity_ScanEnabledWidgets();
			}
	    }
	
	#endif
}
/*******************************************************************************
* Function Name: CapSense_ValidateButtons
********************************************************************************
* Summary:
*  Validates the active buttons and suppresses the unwanted button triggers 
*
* Parameters:
*  prevButtonStatus - Previous button status
*
* Return:
*  uint32 - validated button status 
*
*******************************************************************************/
uint32 CapSense_ValidateButtons(uint32 prevButtonStatus)
{	
	/* For loop variables */
	uint32 indexCount, indexButton;
	
	/* Variable to store current button status */
	uint32 currButtonStatus = CapSense_Buttons_sensorOnMask[0];
    
	
	/* Loop through all buttons */
	for(indexButton = 0; indexButton < (CapSense_Buttons_LEFT__BTN + CapSense_Buttons_TOTAL_BUTTONS_COUNT - 1); indexButton++)
	{
		/* Check if the current button is active */
		if((1<<indexButton) & currButtonStatus)
		{
			/* If the current button is active, then compare against only the buttons that are not yet validated in the loop */
			for(indexCount = indexButton+1; indexCount < (CapSense_Buttons_LEFT__BTN + CapSense_Buttons_TOTAL_BUTTONS_COUNT); indexCount++)
			{
				/* Check if the button in comparison is active */
				if((1<<indexCount) & currButtonStatus)
				{
					/* Priority is for the button that was active in the previous scan - so suppress the newly active button  */                    
					if(prevButtonStatus & (0x01<<indexCount))
					{						
						currButtonStatus &= ~(0x01<<indexButton);
						break;
					}                    
					else if(prevButtonStatus & (0x01<<indexButton))
					{
						currButtonStatus &= ~(0x01<<indexCount);
					}
                    /* If the button in comparison is active and neither of them were active in the previous scan, 
                        then compare the signal and declare the button with stronger signal as active and suppress the other */
					else if(CapSense_Buttons_GetDiffCountData(indexButton) < CapSense_Buttons_GetDiffCountData(indexCount))
					{
						/* If current button has a weaker signal, suppress the current button and break out of the comparison loop */
						currButtonStatus &= ~(0x01<<indexButton);						
						break;
					}
					else 
					{
						/* If current button has a stronger signal, suppress the button in comparison and move to next button for comparison */
						currButtonStatus &= ~(0x01<<indexCount);
					}					
				}
			}
		}
	}
	
	/* Return the updated button mask */
	return currButtonStatus;
}

/* [] END OF FILE */
