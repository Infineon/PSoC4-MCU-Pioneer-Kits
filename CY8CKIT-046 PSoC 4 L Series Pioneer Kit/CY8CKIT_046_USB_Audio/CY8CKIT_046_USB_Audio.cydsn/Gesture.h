/*****************************************************************************
* File Name		: Gesture.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  Gesture.c
*
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
* Cypress reserves the Right to make changes to the Software without notice. 
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

#ifndef GESTURE_H		/* Guard to prevent multiple inclusions */
	#define GESTURE_H 

	#include "cytypes.h"
	#include "CapSense.h"
	#include "stdbool.h"


	/*******************************************************************************
	* 	Macro Definitions
	*******************************************************************************/

	/* Defines the resolution of the radial slider */
	#define SLIDER_RESOLUTION				(180)
		
	/* These macros indicate the sensor number or index to be used with CapSense
	 * APIs.
	 */
	#define LEFT_BTN_ID					CapSense_SENSOR_LEFT__BTN
	#define RIGHT_BTN_ID				CapSense_SENSOR_RIGHT__BTN
	#define UP_BTN_ID					CapSense_SENSOR_UP__BTN
	#define DOWN_BTN_ID					CapSense_SENSOR_DOWN__BTN
	#define CENTRE_BTN_ID				CapSense_SENSOR_CENTRE__BTN	
	#define SLIDER_ID					CapSense_SENSOR_RADIALSLIDER0_E0__RS

	/* These macros defines the mask for each sensor to be used with CapSense APIs
	 * or global variables. 
	 */
	#define LEFT_BTN_MASK				(1 << LEFT_BTN_ID)
	#define RIGHT_BTN_MASK				(1 << RIGHT_BTN_ID)
	#define UP_BTN_MASK					(1 << UP_BTN_ID)
	#define DOWN_BTN_MASK				(1 << DOWN_BTN_ID)
	#define CENTRE_BTN_MASK				(1 << CENTRE_BTN_ID)
	#define SLIDER_ACT_MASK				(1 << SLIDER_ID)
		
	#define SLIDER_4_SECTOR_MASK		0x06
	#define SECTOR_LEFT					0x04
	#define SECTOR_UP					0x06
	#define SECTOR_RIGHT				0x00
	#define SECTOR_DOWN					0x02
		
	#define GetSignal(x)				CapSense_GetDiffCountData(x)
		
	#define SLIDER_MID_POS				SLIDER_RESOLUTION/2

	#define ANGLE_0						0
	#define ANGLE_45					(SLIDER_RESOLUTION/8)
	#define ANGLE_90					(SLIDER_RESOLUTION/4)
	#define ANGLE_135					(ANGLE_45 + ANGLE_90)
	#define ANGLE_180					(SLIDER_RESOLUTION/2)
	#define INVALID_ANGLE				250
		
	#define INVALID_SECTOR				255
	#define INVALID_SINE				1000
	#define INVALID_RADIUS				0xFF
		
	#define MAX_X_Y						2
	#define MAX_R						4
	#define MAX_SINE					256
		
	#define GESTURE_STATE_IDLE				1
	#define GESTURE_STATE_1ST_SECTOR		2
	#define GESTURE_STATE_1ST_SECTOR_MOVED	3
	#define GESTURE_STATE_2ND_SECTOR		4
	#define GESTURE_STATE_3RD_SECTOR		5
	#define GESTURE_STATE_OUTWARD_SWIPE		6
	#define GESTURE_STATE_SECTOR_RELEASE	7
	#define GESTURE_STATE_ERROR				8

	#define SECTOR_0						0
	#define SECTOR_1						1
	#define SECTOR_2						2
	#define SECTOR_3						3
	#define SECTOR_4						4

	#define SECTOR_0_WEIGHT						2
	#define SECTOR_1_WEIGHT						3
	#define SECTOR_2_WEIGHT						5
	#define SECTOR_3_WEIGHT						7
	#define SECTOR_4_WEIGHT						11

	#define FIRST_SECTOR_WEIGHT				13
	#define SECOND_SECTOR_WEIGHT			17
	#define THIRD_SECTOR_WEIGHT				19

	#define SECTOR_1_0_3					(SECTOR_1_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_0_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_3_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_3_0_1					(SECTOR_3_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_0_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_1_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_2_0_4					(SECTOR_2_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_0_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_4_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_4_0_2					(SECTOR_4_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_0_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_2_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_1_2_3					(SECTOR_1_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_2_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_3_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_2_3_4					(SECTOR_2_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_3_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_4_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_3_4_1					(SECTOR_3_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_4_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_1_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_4_1_2					(SECTOR_4_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_1_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_2_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_1_4_3					(SECTOR_1_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_4_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_3_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_4_3_2					(SECTOR_4_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_3_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_2_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_3_2_1					(SECTOR_3_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_2_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_1_WEIGHT*THIRD_SECTOR_WEIGHT)
	#define SECTOR_2_1_4					(SECTOR_2_WEIGHT*FIRST_SECTOR_WEIGHT + SECTOR_1_WEIGHT*SECOND_SECTOR_WEIGHT + SECTOR_4_WEIGHT*THIRD_SECTOR_WEIGHT)

	#define FIRST_SECTOR_INDEX				0
	#define SECOND_SECTOR_INDEX				1
	#define THIRD_SECTOR_INDEX				2

	#define TOTAL_SECTORS					5

	#define TAP_TIME_1_SEC					100

	#define THETA_CHANGE_THRESHOLD			ANGLE_45
	#define RADIUS_CHANGE_THRESHOLD			2

	#define SECTOR_DEBOUNCE					3

	/*******************************************************************************
	* 	Data Type Definitions
	*******************************************************************************/

	/* Defines the various gesture Ids */
	typedef enum
	{
		GESTURE_NONE,
		GESTURE_RELEASE,
		GESTURE_LEFT_CLICK,
		GESTURE_RIGHT_CLICK,
		GESTURE_MIDDLE_CLICK,
		GESTURE_LEFT_SWIPE, 
		GESTURE_RIGHT_SWIPE,
		GESTURE_UP_SWIPE,
		GESTURE_DOWN_SWIPE,
		GESTURE_OUTWARD_SWIPE,
		GESTURE_INNER_CLKWISE,
		GESTURE_INNER_COUNTER_CLKWISE, 
		GESTURE_OUTER_CLKWISE,
		GESTURE_OUTER_COUNTER_CLKWISE,
		GESTURE_UP_CLICK,
		GESTURE_DOWN_CLICK,
		GESTURE_INVALID
	}tGestureId;

	/*****************************************************************************
	* 	Function Prototypes
	*****************************************************************************/

	tGestureId DetectGesture(uint8 isAnySensorActive);

	extern uint32 capsenseButtonStatus;
	
	#ifdef CALCULATE_X_Y
		extern int32 xPos, yPos;
	#endif
	/* Accumulated theta and radius during CapSense activity on the Gesture Pad*/
	extern int32 accRadius, accTheta;

	extern uint32 radius;
	extern int32 theta;
		
#endif /* #ifndef GESTURES_H */

/* [] END OF FILE */
