/******************************************************************************
* Project Name		: CY8CKIT_040_Color_Palette
* File Name			: main.c
* Version 			: 1.0
* Device Used		: CY8C4014LQI-422
* Software Used		: PSoC Creator 3.1
* Compiler    		: ARM GCC 4.8.4, ARM RVDS Generic, ARM MDK Generic
* Related Hardware	: CY8CKIT-040 PSoC 4000 Pioneer Kit 
*
********************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation. All Rights Reserved.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress)
* and is protected by and subject to worldwide patent protection (United
* States and foreign), United States copyright laws and international treaty
* provisions. Cypress hereby grants to licensee a personal, non-exclusive,
* non-transferable license to copy, use, modify, create derivative works of,
* and compile the Cypress Source Code and derivative works for the sole
* purpose of creating custom software in support of licensee product to be
* used only in conjunction with a Cypress integrated circuit as specified in
* the applicable agreement. Any reproduction, modification, translation,
* compilation, or representation of this software except as specified above 
* is prohibited without the express written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH 
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the 
* materials described herein. Cypress does not assume any liability arising out 
* of the application or use of any product or circuit described herein. Cypress 
* does not authorize its products for use as critical components in life-support 
* systems where a malfunction or failure may reasonably be expected to result in 
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of 
* such use and in doing so indemnifies Cypress against all charges. 
*
* Use of this Software may be limited by and subject to the applicable Cypress
* software license agreement. 
*******************************************************************************/

/******************************************************************************
*                           THEORY OF OPERATION
*******************************************************************************
* The project will show case the capability of PSoC 4000 device to interface 
* with a capacitive trackpad and control a RGB LED based on the color touched 
* in the sticker used on top of the trackpad. The sticker will also include a 
* slider area (part of trackpad), which will control the color saturation level 
* of the RGB LED. The project will also demonstrate the Proximity sensing 
* capability of the device with a wire. The firmware provided will be in a 
* sleep-scan_proximity-sleep loop, till the proximity sensor is detected. 
* Once the hand is in the proximity range, the firmware will turn on the PWMs 
* with previously touched/selected color and the intensity of the same will 
* depend on the hands distance from the proximity. Once the Touchpad is touched, 
* the firmware moves into an active scan mode, where it reads the trackpad & 
* updates the LED intensity levels. If, in active mode, no finger is present 
* for more than ~3 seconds on the trackpad, RGB LED will slowly ramp down the 
* brightness and at ~5 secs, the LED will turn OFF and device will enter 
* proximity scan-sleep mode. LED intensity is controlled by a software 
* Pseudo-Random Sequence (PRS) modulator.
*
*******************************************************************************
* Hardware connection required for testing -
* Trackpad	- See cydwr file (requires trackpad shield connection)
* PROX pin 	- P2[0] (requires an external wire/wire-loop connection) - Optional
* Cmod pin	- P0[4] (hard-wired in the board itself)
* TX pin	- P3[0] (hard-wired in the board itself) - Optional/selected through
*					TX_PORT/TX_PIN macros in 'main.h'
*
* Note: TX is disabled by default in the project and SWD port is enabled 
*		on SWD IO line (P3[0]). If TX output is desired, the same can be enabled 
*		by uncommenting TX_ENABLE macro definition in main.h file. If SWD debug is 
*		enabled, TX should be either disabled or routed to another pin 
*		(by modifying TX_PORT and TX_PIN macros) and resistor R57 in the board
* 		should be removed for simultaneous operation of both TX and SWD.
******************************************************************************/

/* Contains all macros used in the main.c file */
#include "main.h"

/* Mask for BYTE shifting */

#define BLUE_POSITION 0
#define GREEN_POSITION 8
#define RED_POSITION 16
#define ALPHA_POSITION 24

/* Global variables used */
uint16 sleepPeriod;
uint8 wdtFlag;
uint8 centroid;
uint8 xPos, yPos;
uint8 rIntensity, gIntensity, bIntensity, sliderOutput = DEFULT_BRIGHTNESS;
uint8 rDuty, gDuty, bDuty;
uint8 rDutySaved = DEFAULT_RED, gDutySaved = DEFAULT_GREEN, bDutySaved = DEFAULT_BLUE;
uint32 ucARGB;

#if (ENABLE_PROXIMITY || ENABLE_GANG)
	uint8 mode = SLEEP_SCAN;
#else
	uint8 mode = ACTIVE_SCAN;
#endif
	
uint16 pos[2];
uint8  prevTouchState, touchPadStatus;

/* Exponential duty table for LED dimming - 7-bit Duty */
const uint8 EXP_LOOKUP[]={0,1,1,2,2,3,4,4,5,5,6,7,7,8,9,9,10,11,11,12,13,14,14,15,16,17,17,18,19,20,20,21,22,23,24,24,25,26,27,28,29,29,30,31,32,33,
34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,56,57,58,59,60,61,63,64,65,66,67,69,70,71,73,74,75,77,78,79,
81,82,84,85,86,88,89,91,92,94,95,97,98,100};

/* Function prototypes */
uint8 CapSense_ScanSleepSensor(void);
void Multichart_SendTxData(void);
void Process_TrackPad_Output(void);
void Process_ColorOutput(uint8 saturation);
void Process_ProximityBrightness(uint8 brightness);
void Process_TrackPadCoordinates(void);

void ActiveMode_Process(void);
void ProximityMode_Process(void);
void SleepMode_Process(void);
void System_Init(void);

void RGB_AssignIntensity(uint8 rVal, uint8 gVal, uint8 bVal);

CY_ISR_PROTO(WDT_ISR);


/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary:
*  main() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
int main()
{
   /* Initialize the system - CapSense, WDT, software PRS */	
	System_Init();
		
    for(;;)
    {	
		/* Process based on the current system mode */
		switch (mode)
		{
			case ACTIVE_SCAN:						
			
				/* Active mode process - does scanning of all Touchpad elements and LED color update at a predefined active scan rate */
				ActiveMode_Process();
				
				#ifdef TX_ENABLE
					#ifdef MINIMAL_TX
						Multichart_SendTxData();
					#endif
				#endif
					
			break;
				
		#if (ENABLE_PROXIMITY || ENABLE_GANG)
			case SLEEP_SCAN:
			
				/* Sleep mode process - Includes proximity sensor scanning alone at a predefined sleep rate */
				SleepMode_Process();
			
			break;
		#endif /* #if (ENABLE_PROXIMITY || ENABLE_GANG) */
		
		#ifdef ENABLE_PROXIMITY
			case PROX_SCAN:
			
				/* Proximity mode process - includes scanning of all sensors & updating LED brightness depending on proximity distance */
				ProximityMode_Process();
			
			break;			
		#endif /* #ifdef ENABLE_PROXIMITY */	
		
			default:
				/* Default switch case */
			break;
		}
		
		#ifdef TX_ENABLE
			#ifndef MINIMAL_TX
				Multichart_SendTxData();
			#endif
		#endif
    }
}

/******************************************************************************
* Function Name: System_Init
*******************************************************************************
*
* Summary:
*  The function performs the following function - 
*	1. Sets up WDT for generating periodic ISR mapped to WDT_ISR routine
*	2. Initializes CapSense block
*	3. Initializes PRSm & TX line if enabled
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*	None
*
******************************************************************************/
void System_Init(void)
{
	/* Initialize WDT */	
	CySysWdtWriteMatch(ACTIVE_SCAN_PERIOD);
	CySysWdtUnmaskInterrupt();
 	WDT_ISR_StartEx(WDT_ISR);			
	
	/* Initialize sleep period */
	sleepPeriod = ACTIVE_SCAN_PERIOD;
	
	/* Enable PRSm for LED intensity control */
	RGB_PRSm_Start(RGB_PRSm_Red_DM_STRONG);
	RGB_AssignIntensity(0,0,0);
	
	/* Enable global interrupts */
    CyGlobalIntEnable; 
	
	/* Enable CapSense component along with proximity sensor */
	#ifdef ENABLE_PROXIMITY
		CapSense_EnableWidget(CapSense_PROXIMITYSENSOR0__PROX);	
	#endif /* #ifdef ENABLE_PROXIMITY */
	
	#ifdef ENABLE_GANG
		CapSense_EnableWidget(CapSense_PROXIMITYSENSOR1__PROX);	
	#endif /* #ifdef ENABLE_GANG */
	
	CapSense_Start();		
	CapSense_InitializeAllBaselines();
	
	/* Enable UART debug interface if enabled */
	#ifdef TX_ENABLE
	TX_StartEx(TX_PORT, TX_PIN);
	#endif /* #ifdef TX_ENABLE */
}

/******************************************************************************
* Function Name: ActiveMode_Process
*******************************************************************************
*
* Summary:
*  The function performs the following function - 
*	1. Scans all the sensors (including proximity)
*	2. Checks if any sensor is active & processes trackpad & proximity output
*		else increments a counter everytime a WDT ISR occurs
*	3. If any sensor was active, the inactivity counter is cleared
*	4. If no sensor was active for LED_DIM_THRESHOLD counts, LED Dimming
*		algorithm is initiated
*	5. LED dimming algorithm decreases the Color brightness slowly to 0
*	6. Once the inactivity counter hits ENTER_SLEEP_COUNTS, the device
*		enters slower sleep scan where only proximity is scanned at a
*		slower rate.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*	Many low level functions are called from here.
*
******************************************************************************/

void ActiveMode_Process(void)
{
	uint8 index;
	uint8 sensorActive = 0;	
	static uint16  activeScans = 0;
	/* Commenting out variables if not used */
	#if (ENABLE_PROXIMITY || ENABLE_GANG)
		static uint16 prevDimmerMult = 0;
		uint16 dimmerMultiplier;	
		uint8 tempBrightness =0;
	#endif /* #if (ENABLE_PROXIMITY || ENABLE_GANG) */
	
	/* Enter Sleep scan mode if activeScans count reaches ENTER_SLEEP_COUNTS */
	if(activeScans >= ENTER_SLEEP_COUNTS)
	{			
		#if (ENABLE_PROXIMITY || ENABLE_GANG)
			/* Prepare device to enter Sleep */
			CapSense_Sleep();								
			
			/* Reset the sleep period to SLEEP_TIMER_PERIOD */
			sleepPeriod = SLEEP_TIMER_PERIOD;
			
			/* Change mode to SLEEP_SCAN */
			mode = SLEEP_SCAN;	
			
			/* Clear Dimmer multipliers */		
			prevDimmerMult = 0;
			
			/* Clear the active scan counter */
			activeScans = 0;
			
			/* Clear & stop the RGB LED PRSm */
			RGB_AssignIntensity(0, 0, 0);
			RGB_PRSm_Stop(0);
		#endif /* #if (ENABLE_PROXIMITY || ENABLE_GANG) */
	}
	else
	{
		/* Scan all sensors */
		/* Based on whether proximity is enabled or not decide the number of sensors to scan */
		#ifdef ENABLE_PROXIMITY
			for(index = 0; index < (CapSense_TOTAL_SENSOR_COUNT-1); index++)
		#else
			for(index = 0; index < (CapSense_TOTAL_SENSOR_COUNT-2); index++)
		#endif
		{
			/* Call ScanSensor API for the current sensor */
			CapSense_ScanSensor(index);	
			
			/* Wait till the scan is complete */
			while(CapSense_IsBusy())
			{}
			
			/* Update the baseline of the current sensor */
			CapSense_UpdateSensorBaseline(index);
			
			/* Check if the sensor is active, set sensoractive flag if it is */
			if(CapSense_CheckIsSensorActive(index))
			{				
				/* Set a different bit for proximity which is used for clearing activeScans counter */
				#ifdef ENABLE_PROXIMITY
					if(index == PROXIMITY_SENSOR_INDEX)
					{
						sensorActive |= PROXIMITY_ACTIVE;
					}
					else
				#endif
					{
						sensorActive = 1;
					}
			}
			
		}
		
		/* Even if one of the sensor is ON, Clear the active scan counter & Do sensor active processing */
		if(sensorActive)
		{
			#ifdef ENABLE_PROXIMITY
			/* Neither increment nor clear the activeScans counter, if only proximity is active */
			if((sensorActive != PROXIMITY_ACTIVE))
			#endif
			{
				/* Clear active scan counter */
				activeScans = 0;
			}
			
			/* Clear the sensoractive flag */
			sensorActive = 0;							
			
			/* Obtain X, Y coordinates */
			touchPadStatus = CapSense_GetTouchCentroidPos(CapSense_TOUCHPAD0__TP, pos);
			xPos = pos[0];
			yPos = pos[1];
			
			/* Process coordinates only if Touchpad was active */
			if(touchPadStatus)
			{
				Process_TrackPad_Output();
				
				if((xPos > SLIDER_START) && (prevTouchState == NO_TOUCH))
				{
					touchPadStatus = NO_TOUCH;
				}				
			}
			prevTouchState = touchPadStatus;
		}
		else
		{
			/* If no sensor is active, increment the active scan counter */
			if(wdtFlag)
			{
				wdtFlag = 0;
				/* If neither of the wakeup mode is selected,
				Do not increment the activeScans counter as the device should not enter sleep scan mode.
				No source is present to wakeup the device */
				#if (ENABLE_PROXIMITY || ENABLE_GANG)
					activeScans++;					
				#endif
			}
		}
		
		/* Remove the LED dimming code if no wakeup source is selected */
		#if (ENABLE_PROXIMITY || ENABLE_GANG)
			/* Dim the LEDs if the inactive period crosses LED_DIM_THRESHOLD */
			if(activeScans > LED_DIM_THRESHOLD)
			{
				/* Calculates if the activeScan count crossed DIM_RATE step */
				dimmerMultiplier = ((activeScans - LED_DIM_THRESHOLD)/LED_DIM_RATE);					
				
				/* Only if the dimmerMultiplier has changed, process the saturation change
				during LED dimming phase */
				if((dimmerMultiplier != prevDimmerMult) && (dimmerMultiplier <= LED_DIM_STEPS))
				{
					/* Reduce the brightness in LED_DIM_STEPS steps every LED_DIM_RATE period */
					tempBrightness = sliderOutput - (sliderOutput * dimmerMultiplier)/LED_DIM_STEPS;
					Process_ProximityBrightness(tempBrightness);						
				}					
				
				prevDimmerMult = dimmerMultiplier;
			}
			/* Save the duty before the LED starts dimming */
			else if(activeScans == LED_DIM_THRESHOLD)
			{
				rDutySaved = rDuty;
				gDutySaved = gDuty;
				bDutySaved = bDuty;
			}
		#endif /* #if (ENABLE_PROXIMITY || ENABLE_GANG) */
	}
		
		/* Set the intensity of the LEDs */
		RGB_AssignIntensity(rDuty, gDuty, bDuty);
		
		ucARGB = (rDuty << RED_POSITION) | (gDuty << GREEN_POSITION) | (bDuty << BLUE_POSITION);
}
#if (ENABLE_PROXIMITY || ENABLE_GANG)
/******************************************************************************
* Function Name: SleepMode_Process
*******************************************************************************
*
* Summary:
*  The function performs the following function - 
*	1. Scans either the proximity sensor or the Gang sensor or both depending on
*		ENABLE_PROXIMITY & ENABLE_GANG sensor macro setting
*	2. Checks if the sensor is active. Puts the device in proximity mode if 
*		proximity is active. If gang sensor is active, puts the device in active
*		scan mode.
*	3. If baseline update number of counts are reached, baseline of all sensors
*		except gang & proximity are updated.
*	4. Device sleeps for the rest of the duration till a WDT ISR wakes the device	
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*	Many low level functions are called from here.
*
******************************************************************************/
void SleepMode_Process(void)
{
	uint8 index;
	uint16  updateBaselineCounts =0;
	
	#ifdef ENABLE_GANG
		uint16 diff = 0;
	#endif
	/* Wakeup the CapSense module after device wakeup */
		CapSense_Wakeup();
		
		/* Check if baseline update counter has reached the baseline update threshold */
		if(updateBaselineCounts == SLEEP_UPDATE_BASELINE_COUNT)
		{
			/* Clear the counter & update all baselines */
			updateBaselineCounts = 0;				
			
			for(index = 0; index < CapSense_TOTAL_SENSOR_COUNT-2; index++)
			{
				
				/* Scan the sensor */
				CapSense_ScanSensor(index);
				
				/* Wait till scan is not complete */
				while(CapSense_IsBusy())
				{}
				
				/* update the baseline */
				CapSense_UpdateSensorBaseline(index);					
			}				
		}
		else
		{
			/* Increment baseline update counter */
			updateBaselineCounts++;
			
			/* Scan the proximity sensor and see if it is active - wake the device if so*/
			#ifdef ENABLE_PROXIMITY
				if(CapSense_ScanSleepSensor())
				{
					updateBaselineCounts = 0;
					
					/* Turn on the PRSm with strong drive for the pins */
					RGB_PRSm_Start(RGB_PRSm_Red_DM_STRONG);
					
					/* Change the mode & scan period */
					mode = PROX_SCAN;
					sleepPeriod = ACTIVE_SCAN_PERIOD;
				}
			#endif /* #ifdef ENABLE_PROXIMITY */
			
			#ifdef ENABLE_GANG
			/* If proximity sensor is also enabled, check if proximity was active during the above prox scan
				else check if touchpad is active */
			#ifdef ENABLE_PROXIMITY	
				if(mode != PROX_SCAN)
			#endif /* #ifdef ENABLE_PROXIMITY */
			{
				/* Scan the gang sensor */
				CapSense_ScanSensor(GANG_SENSOR_INDEX);
				
				/* Wait till scan is complete */
				while(CapSense_IsBusy())
				{}
				/* update the gang sensor baseline */
				CapSense_UpdateSensorBaseline(GANG_SENSOR_INDEX);
				
				/* Check if the change in raw count is positive and above GANG_SENSOR threshold,
					to wake the device in ACTIVE_MODE*/
				if(CapSense_SensorRaw[GANG_SENSOR_INDEX] > CapSense_SensorBaseline[GANG_SENSOR_INDEX])
				{
					diff = CapSense_SensorRaw[GANG_SENSOR_INDEX] - CapSense_SensorBaseline[GANG_SENSOR_INDEX];
				
					/* Check if any activity is present on the gang sensor */
					if(diff > GANG_THRESHOLD)
					{
						/* Wake the device in ACTIVE_SCAN if the gang sensor is active */
						updateBaselineCounts = 0;
					
						/* Turn on the PRSm with strong drive for the pins */
						RGB_PRSm_Start(RGB_PRSm_Red_DM_STRONG);
						
						/* Change the mode & scan period */
						mode = ACTIVE_SCAN;
						sleepPeriod = ACTIVE_SCAN_PERIOD;
					}
				}
			}
				
			#endif /* #ifdef ENABLE_GANG */
			
		}
		
		/* Put the device to sleep, if the mode is still sleep scan */
		if(mode == SLEEP_SCAN)
		{
			/* Put CapSense component to sleep */
			CapSense_Sleep();
			
			/* Put device to deep sleep */
			CySysPmDeepSleep();
		}
}
#endif /* #if (ENABLE_PROXIMITY || ENABLE_GANG) */

#ifdef ENABLE_PROXIMITY
/******************************************************************************
* Function Name: ProximityMode_Process
*******************************************************************************
*
* Summary:
*  The function performs the following function - 
*	1. Scans all the sensors (including proximity)
*	2. Checks if trackpad is active, puts the device to active mode if so
*	3. Checks if Proximity is active, Puts the device to sleep mode if not
*	4. Based on Proximity signal level, Color saturation level is controlled
*
* Parameters:
*  None.
*
* Return:
*  none
*
* Side Effects:
*	Many low level functions are called from here.
*
******************************************************************************/
void ProximityMode_Process(void)
{
	uint8 index;
	uint8 tempBrightness = 0;
	static uint8 prevBrightness;

	for(index = 0; index < (CapSense_TOTAL_SENSOR_COUNT-1); index++)
	{
		/* Call ScanSensor API for the current sensor */
		CapSense_ScanSensor(index);	
		
		/* Wait till Scan is complete */
		while(CapSense_IsBusy())
		{}
		
		/* Update the baseline of the current sensor */
		CapSense_UpdateSensorBaseline(index);
		
		/* Check if any touchpad sensor is active, set mode to active scan mode if so */
		if(CapSense_CheckIsSensorActive(index))
		{					
			if(index != PROXIMITY_SENSOR_INDEX)
			{
				mode = ACTIVE_SCAN;
			}
		}				
	}
	
	/* Based on proximity signal value, update the LED intensity */
	if(CapSense_SensorSignal[PROXIMITY_SENSOR_INDEX] >= PROXIMITY_LOWER_LIMIT)
	{
		if(CapSense_SensorSignal[PROXIMITY_SENSOR_INDEX] >= PROXIMITY_UPPER_LIMIT)
		{
			tempBrightness = sliderOutput;
		}
		else
		{
			tempBrightness = (uint8)((((uint32)sliderOutput) * (CapSense_SensorSignal[PROXIMITY_SENSOR_INDEX] - PROXIMITY_LOWER_LIMIT)) /(PROXIMITY_UPPER_LIMIT - PROXIMITY_LOWER_LIMIT));
		}
		
		/* Update saturation & intensity only if there is a change in saturation */
		if(prevBrightness != tempBrightness)
		{
			Process_ProximityBrightness(tempBrightness);
			
			RGB_AssignIntensity(rDuty, gDuty, bDuty);
		}
		
		prevBrightness = tempBrightness;
	}
	else
	{
		prevBrightness = 0;
		/* Put CapSense component to sleep */
		CapSense_Sleep();
		
		/* Clear LED lines & stop */
		RGB_AssignIntensity(0, 0, 0);
		RGB_PRSm_Stop(0);
		
		/* Change the mode to sleep scan */
		mode = SLEEP_SCAN;
		
		/* Change the sleep period */
		sleepPeriod = SLEEP_TIMER_PERIOD;				
	}
}
#endif /* #ifdef ENABLE_PROXIMITY */

/******************************************************************************
* Function Name: Process_TrackPad_Output
*******************************************************************************
*
* Summary:
*  The function processes trackpad coordinates & calculates the RGB intensities
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
void Process_TrackPad_Output(void)
{
	/* Based on the Touchpad coordinates found, process the color/saturation selected */
	Process_TrackPadCoordinates();
	
	/* Process the color based on saturation level set & update the LED */
	Process_ColorOutput(sliderOutput);
}

/******************************************************************************
* Function Name: Process_TrackPadCoordinates
*******************************************************************************
*
* Summary:
*  The function processes trackpad coordinates & calculates color or saturation
*	level based on the same.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*	Updates rIntensity, gIntensity & bIntensity variables
*
******************************************************************************/
void Process_TrackPadCoordinates(void)
{	
	static uint8 prevXPos, prevYPos;
	
	/* Process coordinates only if there is a change in coordinates */
	if((xPos != prevXPos) || (yPos != prevYPos))
	{
		/* Check if the touch is in slider area - if so obtain the color saturation selected */
		if((xPos > SLIDER_START))
		{
			if(prevTouchState == NO_TOUCH)
			{
				if(yPos <= SLIDER_MIN)
				{
					sliderOutput = MAX_SATURATION;
				}
				else if (yPos >= SLIDER_MAX)
				{
					sliderOutput = MIN_SATURATION;
				}
				else
				{
					sliderOutput = MAX_SATURATION - ((uint16)(MAX_SATURATION - MIN_SATURATION) * yPos)/SLIDER_RESOLUTION;
				}
				
				touchPadStatus = NO_TOUCH;
			}
		}
		/* Process color coordinates only if saturation is above minimum,
		to avoid color switching when RGB LED is off - which user will not see  */
		else if((sliderOutput != MIN_SATURATION))
		{
			/* Check if the touch is in the grey area of the color palette - if so then set the color to grey */
			if(yPos >= GREY_AREA_START)
			{
				rIntensity = Y_AXIS_BRIGHTNESS;
				gIntensity = Y_AXIS_BRIGHTNESS;
				bIntensity = Y_AXIS_BRIGHTNESS;
			}
			/* If none of the above, then the touch is on color selection area - obtain the color selected by the user */
			else
			{
				/* Process X co-ordinate
				The sticker is designed in such a way that there are a total of 6 color windows along the X-axis
				In each window, one color will be max another will be min and one will vary from min to max (thus giving rise to 6 windows)
				to figure out the color values along the x-axis
				1. Check if X Position for its color window based on the boundaries defined in main.h - this will change depending on sticker design
				2. Once the window is found, assign the max & min values to the max/min colors & calculate the value of the 3rd color 
				 	with the help of relative x-pos in the color window */
				
				/* Check for Red Max windows - windows where Red is max (total 2 for each color) */
				if((xPos < RED_LIMIT_RIGHT) || ((xPos > RED_LIMIT_LEFT)))
				{
					rIntensity = MAX_BRIGHTNESS;
					
					/* Find the min color & calculate the other color */
					if(xPos <= RED_LIMIT_RIGHT)
					{
						bIntensity = MIN_BRIGHTNESS;
						gIntensity = (xPos * MAX_BRIGHTNESS)/COLOR_WINDOW_SIZE;
					}
					else
					{
						if(xPos > RED_AREA_CENTER)
						{
							xPos = RED_AREA_CENTER;
						}
						gIntensity = MIN_BRIGHTNESS;
						bIntensity = ((RED_AREA_CENTER - xPos) * MAX_BRIGHTNESS)/COLOR_WINDOW_SIZE;
					}
				}
				/* Check for Green max windows */
				else if((xPos < GREEN_LIMIT_RIGHT) && (xPos > GREEN_LIMIT_LEFT))
				{
					gIntensity = MAX_BRIGHTNESS;
					/* Find the min color & calculate the other color */
					if(xPos > GREEN_AREA_CENTER)
					{
						rIntensity = MIN_BRIGHTNESS;
						bIntensity = ((xPos - GREEN_AREA_CENTER) * MAX_BRIGHTNESS)/COLOR_WINDOW_SIZE;
					}
					else
					{
						bIntensity = MIN_BRIGHTNESS;
						rIntensity = ((GREEN_AREA_CENTER - xPos) * MAX_BRIGHTNESS)/COLOR_WINDOW_SIZE;
					}
				}
				/* Check for Blue max windows */
				else if((xPos < BLUE_LIMIT_RIGHT) && (xPos > BLUE_LIMIT_LEFT))
				{
					bIntensity = MAX_BRIGHTNESS;
					
					/* Find the min color & calculate the other color */
					if(xPos > BLUE_AREA_CENTER)
					{
						gIntensity = MIN_BRIGHTNESS;
						rIntensity = ((xPos - BLUE_AREA_CENTER) * MAX_BRIGHTNESS)/COLOR_WINDOW_SIZE;
					}
					else
					{
						rIntensity = MIN_BRIGHTNESS;
						gIntensity = ((BLUE_AREA_CENTER - xPos) * MAX_BRIGHTNESS)/COLOR_WINDOW_SIZE;
					}
				}
				
				/* Process Y coordinates
				Y axis moves the colors selected along x-axis to Half brightness 
				i.e. if a color is 100%, it will move it to 50% through the y-axis position 
				if a color is 75%, then it will reduce the color to 50% along the y-axis 
				if a color is 10%, then it will increase the color to 50% along the y-axis */
				if(rIntensity > Y_AXIS_BRIGHTNESS)
				{
					rIntensity = rIntensity - (EXP_LOOKUP[yPos] * (rIntensity - Y_AXIS_BRIGHTNESS))/Y_POS_SIZE;
				}
				else
				{
					rIntensity = rIntensity + (EXP_LOOKUP[yPos] * (Y_AXIS_BRIGHTNESS - rIntensity ))/Y_POS_SIZE;
				}
				
				if(gIntensity > Y_AXIS_BRIGHTNESS)
				{
					gIntensity = gIntensity - (EXP_LOOKUP[yPos] * (gIntensity - Y_AXIS_BRIGHTNESS))/Y_POS_SIZE;
				}
				else
				{
					gIntensity = gIntensity + (EXP_LOOKUP[yPos] * (Y_AXIS_BRIGHTNESS - gIntensity ))/Y_POS_SIZE;
				}
				
				if(bIntensity > Y_AXIS_BRIGHTNESS)
				{
					bIntensity = bIntensity - (EXP_LOOKUP[yPos] * (bIntensity - Y_AXIS_BRIGHTNESS))/Y_POS_SIZE;
				}
				else
				{
					bIntensity = bIntensity + (EXP_LOOKUP[yPos] * (Y_AXIS_BRIGHTNESS - bIntensity ))/Y_POS_SIZE;
				}
				
				/* Capping the colors to Max brightness if any overflow occur in the above calculations */
				if(rIntensity > MAX_BRIGHTNESS)
				{
					rIntensity = MAX_BRIGHTNESS;
				}
				
				if(gIntensity > MAX_BRIGHTNESS)
				{
					gIntensity = MAX_BRIGHTNESS;
				}
				
				if(bIntensity > MAX_BRIGHTNESS)
				{
					bIntensity = MAX_BRIGHTNESS;
				}
			}
		}
	}
	
	prevXPos = xPos;
	prevYPos = yPos;
}

/******************************************************************************
* Function Name: Process_ColorOutput
*******************************************************************************
*
* Summary:
*  The function processes saturation level & outputs the final PWM duty value
*	to be written to the PWM Compare registers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*	Updates rDuty, gDuty & bDuty variables
*
******************************************************************************/
void Process_ColorOutput(uint8 sliderOutput)
{
	#ifndef DO_SATURATION
	/* Process Brightness 	
	Adjust the color brightness proportional to the max saturation level */
	rDuty = ((sliderOutput) * (uint16)rIntensity)/MAX_SATURATION;
	gDuty = ((sliderOutput) * (uint16)gIntensity)/MAX_SATURATION;
	bDuty = ((sliderOutput) * (uint16)bIntensity)/MAX_SATURATION;
	#else
	/* Process Saturation 
	Moves the color selected towards all 100% or all 0% 
	i.e. if a color is at 30%, then it will move to 100% at 100% saturation, 0% at 0% saturation & 
		30% at 50% saturation. Thus forming a relative scale slider */
	if(sliderOutput == MIN_SATURATION)
	{
		rDuty = MIN_BRIGHTNESS;
		gDuty = MIN_BRIGHTNESS;
		bDuty = MIN_BRIGHTNESS;
	}
	else if(sliderOutput == MAX_SATURATION)
	{
		rDuty = MAX_BRIGHTNESS;
		gDuty = MAX_BRIGHTNESS;
		bDuty = MAX_BRIGHTNESS;
	}
	else if (sliderOutput <= HALF_SATURATION)
	{
		rDuty = rIntensity - ((HALF_SATURATION - sliderOutput) * (uint16)rIntensity)/HALF_SATURATION;
		gDuty = gIntensity - ((HALF_SATURATION - sliderOutput) * (uint16)gIntensity)/HALF_SATURATION;
		bDuty = bIntensity - ((HALF_SATURATION - sliderOutput) * (uint16)bIntensity)/HALF_SATURATION;
	}
	else if (sliderOutput > HALF_SATURATION)
	{
		rDuty = rIntensity + (((uint16)sliderOutput - HALF_SATURATION) * ((uint16)MAX_BRIGHTNESS - rIntensity))/HALF_SATURATION;
		gDuty = gIntensity + (((uint16)sliderOutput - HALF_SATURATION) * ((uint16)MAX_BRIGHTNESS - gIntensity))/HALF_SATURATION;
		bDuty = bIntensity + (((uint16)sliderOutput - HALF_SATURATION) * ((uint16)MAX_BRIGHTNESS - bIntensity))/HALF_SATURATION;
	}
	#endif
}

/******************************************************************************
* Function Name: Process_ProximityBrightness
*******************************************************************************
*
* Summary:
*  The function processes brightness level for Proximity mode & LED dimming phase
*	& outputs the final PWM duty value to be written to the PWM Compare registers.
*
* Parameters:
*  uint8 brightness - brightness level.
*
* Return:
*  None.
*
* Side Effects:
*	Updates rDuty, gDuty & bDuty variables
*
******************************************************************************/
void Process_ProximityBrightness(uint8 brightness)
{	
	/* Process Brightness 	
	Adjust the color brightness proportional to the slider output level */
	rDuty = ((brightness) * (uint16)rDutySaved)/sliderOutput;
	gDuty = ((brightness) * (uint16)gDutySaved)/sliderOutput;
	bDuty = ((brightness) * (uint16)bDutySaved)/sliderOutput;
}

/******************************************************************************
* Function Name: CapSense_ScanSleepSensor
*******************************************************************************
*
* Summary:
*  The function scans the proximity sensor during sleep & returns 1 if the
*	sensor is active.
*
* Parameters:
*  None.
*
* Return:
*  0 - Proximity not active
*  1 - Proximity above PROXIMITY_LOWER_LIMIT
*
* Side Effects:
*	None
*
******************************************************************************/
uint8 CapSense_ScanSleepSensor(void)
{
	static uint16 prevRawCount;

	uint16 rawCount, diff = 0;		
	
	/* Scan the proximity sensor alone */
	CapSense_ScanSensor(PROXIMITY_SENSOR_INDEX);
	
	/* Sleep till the scan gets complete */
	while(CapSense_IsBusy())
	{}
	
	/* Update baseline */
	CapSense_UpdateSensorBaseline(PROXIMITY_SENSOR_INDEX);
	
	/* Obtain the rawcount & baseline for calculating difference */
	rawCount = CapSense_SensorRaw[PROXIMITY_SENSOR_INDEX];
	prevRawCount = CapSense_SensorBaseline[PROXIMITY_SENSOR_INDEX];
	
	/* calculate the difference only if it is positive */
	if(rawCount > prevRawCount)
	{
		diff = rawCount - prevRawCount;
	}

	/* return non-zero value if the diff is above sleep threshold */
	if(diff > SLEEP_THRESHOLD)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
* Function Name: RGB_AssignIntensity
*******************************************************************************
*
* Summary:
*  The function performs the following function - 
*	1.Updates the PWM compare registers based on the passed values
*
* Parameters:
*  rVal - Red LED intensity
*	gVal - Green LED intensity.
*	bVal - Blue LED intensity.
*
* Return:
*  none
*
* Side Effects:
*	None
*
******************************************************************************/
void RGB_AssignIntensity(uint8 rVal, uint8 gVal, uint8 bVal)
{
	/* Because of unequal max intensity levels of the LEDs on the board (RED being
		more bright) the RED LED intensity is scaled to 87.5% of the calculated 
		intensity. */
	/* Scale down red only if SCALE_DOWN_RED macro is enabled */
	#ifdef SCALE_DOWN_RED
		if(rVal < (((uint16)MAX_BRIGHTNESS*MAX_PERCENT) >> BRIGHTNESS_SHIFT))
		{
			RGB_PRSm_WriteCompareBuffer(((uint16)rVal*MAX_PERCENT)>>BRIGHTNESS_SHIFT, 0);
		}
		else
		{
			RGB_PRSm_WriteCompareBuffer((((uint16)MAX_BRIGHTNESS*MAX_PERCENT) >> BRIGHTNESS_SHIFT), 0);
		}
	#else
		if(rVal < (MAX_BRIGHTNESS))
		{
			RGB_PRSm_WriteCompareBuffer(rVal, 0);
		}
		else
		{
			RGB_PRSm_WriteCompareBuffer(MAX_BRIGHTNESS, 0);
		}
	#endif
		
	if(gVal < MAX_BRIGHTNESS)
	{
		RGB_PRSm_WriteCompareBuffer(gVal, 1);
	}
	else
	{
		RGB_PRSm_WriteCompareBuffer(MAX_BRIGHTNESS, 1);
	}
	
	if(bVal < MAX_BRIGHTNESS)
	{
		RGB_PRSm_WriteCompareBuffer(bVal, 2);
	}
	else
	{
		RGB_PRSm_WriteCompareBuffer(MAX_BRIGHTNESS, 2);
	}
}

/******************************************************************************
* Function Name: WDT_ISR
*******************************************************************************
*
* Summary:
*  Watchdog Timer match ISR - Used for timing & sleep wake-up source
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
CY_ISR(WDT_ISR)
{
	/* update match for generating proper periodic WDT interrupt */
	CySysWdtWriteMatch((uint16)CySysWdtReadMatch()+sleepPeriod);
	
	/* Clear WDT ISR */
	CySysWdtClearInterrupt();
	
	/* Set WDT flag for inactive period counter in active mode */
	wdtFlag = 1;
}

#ifdef TX_ENABLE
/******************************************************************************
* Function Name: Multichart_SendTxData
*******************************************************************************
*
* Summary:
*  The function sends sensor data over UART Tx line
*
* Parameters:
*  None.
*
* Return:
*  none
*
* Side Effects:
*	None
*
******************************************************************************/
void Multichart_SendTxData(void)
{
	#ifndef MINIMAL_TX
		uint8 index;
	#endif	
	
	/* If MINIMAL_TX macro is not defined, send the whole TX data -
	Sensor raw data, baseline & signal 
	Else send the xPos, yPos, saturation & RGB duty values only */
	#ifndef MINIMAL_TX
		TX_PutCRLF();
		
		for(index = 0; index < CapSense_TOTAL_SENSOR_COUNT; index++)
		{
			TX_PutChar((uint8)(CapSense_SensorRaw[index]>>8));
			TX_PutChar((uint8)(CapSense_SensorRaw[index]));		
		}
		
		TX_PutChar((uint8)(0));
		TX_PutChar((uint8)(rDuty));
		
		for(index = 0; index < CapSense_TOTAL_SENSOR_COUNT; index++)
		{
			TX_PutChar((uint8)(CapSense_SensorBaseline[index]>>8));
			TX_PutChar((uint8)(CapSense_SensorBaseline[index]));
		}	
		
		TX_PutChar((uint8)(0));
		TX_PutChar((uint8)(gDuty));
		
		for(index = 0; index < CapSense_TOTAL_SENSOR_COUNT; index++)
		{
			TX_PutChar((uint8)(CapSense_SensorSignal[index]>>8));
			TX_PutChar((uint8)(CapSense_SensorSignal[index]));
		}
		
		
		TX_PutChar((uint8)(0));
		TX_PutChar((uint8)(bDuty));
		
		TX_PutChar(0);
		TX_PutChar(0xFF);
		TX_PutChar(0xFF);
	#else
		TX_PutChar((uint8)('-'));
		TX_PutChar((uint8)(xPos));		
		TX_PutChar((uint8)(yPos));		
	#endif	
}
#endif /* #ifdef TX_ENABLE */

/* [] END OF FILE */
