/******************************************************************************
* Project Name		: CY8CKIT_040_Color_Palette
* File Name			: main.h
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

/********************************************************************************
*	Contains all macros used in the main.c file 
********************************************************************************/

#ifndef MAIN_H
	#define MAIN_H
	#include <project.h>
	#include "RGB_PRSm.h"
/* Comment the line to disable the software TX data out interface */
//#define TX_ENABLE						1

	#ifdef TX_ENABLE
/* Uncomment/Comment the below line to enable/disable minimal data (only coordinates) transfer over TX respectively 
	Note: The below macro is ineffective if TX_ENABLE is not enabled */
#define MINIMAL_TX						1

/* TX Port number and pin number macro */
#define TX_PORT								3
#define TX_PIN								0
	#endif
/***********************************************************************************
* Below macros (ENABLE_GANG & ENABLE_PROXIMITY) help define how the device needs to 
* wakeup into active mode  
************************************************************************************	
1. If the board has a proximity loop around the trackpad, then just keeping the
	ENABLE_PROXIMITY macro should be sufficient to wake the device & control the 
	color brightness with approaching hand.
2. If the board doesn't have a proximity loop around it, then keeping the 
	ENABLE_GANG macro makes sure the device wakes up on a touch activity on
	the trackpad/touchpad. But the brightness control will not be effective
	as the GANG sensor does not have good proximity range.
3. If the board needs brightness control using proximity but doesn't have a loop
	then both the macros can be enabled. This will make sure, the device wakes on 
	touch on the trackpad & proximity wire can control the brightness of the color. 
4. If both are commented out, the LED will display the selected color
	as long as power is present.
***********************************************************************************/
/* Comment the below line to disable Gang-sensor based wakeup detection */
#define ENABLE_GANG						1
	
/* Comment the below line to disable Proximity sensor based wakeup detection/brightness control */
#define ENABLE_PROXIMITY				1
	
/* Define the minimum, Maximum & half saturation level macros for the RGB colors */
#define	MAX_SATURATION					100
#define MIN_SATURATION					0
#define HALF_SATURATION					(MIN_SATURATION + (MAX_SATURATION - MIN_SATURATION )/2)

/* Define the Max, Min & half Brightness of the LED in terms PWM_Period counts */
#define MAX_BRIGHTNESS					127
#define MIN_BRIGHTNESS					0
#define Y_AXIS_BRIGHTNESS				32

/* Define the slider related parameters like Slider boundary, Resolution, lower & upper saturation values */
#define SLIDER_START					118
#define SLIDER_MAX						100
#define SLIDER_MIN						0
#define SLIDER_RESOLUTION				100

/* If the below macro is defined/enabled, slider output controls saturation level of the color
	else slider output controls brightness of the color */
//#define DO_SATURATION					1
	
/* If Saturation is enabled, then default saturation is 50% */
#ifdef DO_SATURATION
	#define DEFULT_BRIGHTNESS				MAX_SATURATION>>1
#else
	#define DEFULT_BRIGHTNESS				MAX_SATURATION
#endif

/* Define the limit on the Y axis from where grey area starts (All RGB duty value equals Half brightness with 50% saturation) */
#define GREY_AREA_START					99

/* Macro to scale down Red intensity compared to others - enabling this macro will scale down red to 87.5% of calculated intensity */
#define SCALE_DOWN_RED					1
/* Number of bit shifts done on the Red brightness output intensity */
#define BRIGHTNESS_SHIFT				3

/* Multiplier applied on the numerator for calculating scaled Red brightness
	MAX Output Red brightness = (MAX_BRIGHTNESS * MAX_PERCENT)>>BRIGHTNESS_SHIFT */
#define MAX_PERCENT						7
	
/* Define the boundary & center for each color in the slider X axis */
#define RED_LIMIT_RIGHT					19
#define RED_LIMIT_LEFT					95
#define RED_AREA_CENTER					116

#define GREEN_LIMIT_RIGHT				57
#define GREEN_LIMIT_LEFT				19
#define GREEN_AREA_CENTER				38

#define BLUE_LIMIT_RIGHT				95
#define BLUE_LIMIT_LEFT					57
#define BLUE_AREA_CENTER				76

/* Defines the window size (in X axis counts) for which a color remains max in its region */ 
#define COLOR_WINDOW_SIZE				((BLUE_AREA_CENTER - GREEN_AREA_CENTER)/2)

/* Y Axis size */
#define Y_POS_SIZE						SLIDER_RESOLUTION

/* Idle active scan count at which the LED needs to be dimmed - Active scan period is defined below
	Time at which LED starts dimming from last touch = LED_DIM_THRESHOLD * ACTIVE_SCAN_TIME */
#define LED_DIM_THRESHOLD				120

/* Defines the number of active scan counts, at which rate the LED brightness is dimmed slowly */
#define LED_DIM_RATE					1

/* Number of steps after which the LED brightness will hit 0 */
#define LED_DIM_STEPS					64

/* defines the thresholds for proximity sensor during sleep for waking the device to perform normal scan */
#define SLEEP_THRESHOLD					(PROXIMITY_LOWER_LIMIT + 10)

/* Threshold for GANG sensor - not effective if GANG sensor is disabled */
#define GANG_THRESHOLD					100

/* Rate at which baseline needs to updated. 
	Baseline Update rate = SLEEP_UPDATE_BASELINE_COUNT * SLEEP_SCAN_PERIOD */
#define SLEEP_UPDATE_BASELINE_COUNT		25

/* Defines the Proximity sensor limits & index */
#define PROXIMITY_SENSOR_INDEX			CapSense_SENSOR_PROXIMITYSENSOR0_0__PROX
#define PROXIMITY_LOWER_LIMIT			60
#define PROXIMITY_UPPER_LIMIT			200
#define PROXIMITY_ACTIVE				0x02
/* Index of the Gang sensor */
#define GANG_SENSOR_INDEX				CapSense_SENSOR_PROXIMITYSENSOR1__PROX
	
/* Defines the WDT counts for calculation sleep & active scan period
	SCAN_FREQ = 32000/counts (Hz) 
	SLEEP_FREQ = 32000/SLEEP_TIMER_PERIOD (Hz)
	ACTIVE_FREQ = 32000/ACTIVE_SCAN_PERIOD (Hz) */
#define SLEEP_TIMER_PERIOD				4200
#define ACTIVE_SCAN_PERIOD				(SLEEP_TIMER_PERIOD/4)

/* Operating mode macros */
#define ACTIVE_SCAN						0
#define SLEEP_SCAN						1
#define PROX_SCAN						2

/* Number of Idle active scan periods after which the device enters sleep scan mode */
#define ENTER_SLEEP_COUNTS				183
	
#define NO_TOUCH						0
	
/* Default color and Brightness */
#define DEFAULT_RED						MIN_BRIGHTNESS
#define DEFAULT_BLUE					MAX_BRIGHTNESS
#define DEFAULT_GREEN					MIN_BRIGHTNESS


/* External variables from CapSense component */
extern uint16 CapSense_sensorRaw[], CapSense_SensorBaseline[], CapSense_rawFilterData1[];
extern uint8 CapSense_SensorOnMask[], CapSense_SensorSignal[];

#endif
/* [] END OF FILE */
