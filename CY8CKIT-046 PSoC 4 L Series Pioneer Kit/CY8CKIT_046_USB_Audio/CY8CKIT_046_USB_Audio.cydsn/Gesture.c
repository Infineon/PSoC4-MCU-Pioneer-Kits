/*****************************************************************************
* File Name		: Gesture.c
* Version		: 1.0 
*
* Description:
*  This file contains the CapSense gesture detection logic APIs
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

#include "Gesture.h"
#include "CapSense_CSHL.h"
#include "stdbool.h"
#include "stdlib.h"

/*******************************************************************************
* 	Global Variable Declarations
*******************************************************************************/

/* The centroid of the radial slider from the current scan is stored in this 
 * variable. 
 */
uint16 sliderCentroid;

/* Accumulated theta and radius during CapSense activity on the Gesture Pad*/
int32 accRadius, accTheta;

/* Status of various segments */
uint32 capsenseButtonStatus;

/* X and Y coordinates from Gesture Pad radius and theta */
int32 xPos, yPos;

/* Polar coordinates - Radius and theta */
uint32 radius;
int32 theta;

/*******************************************************************************
* 	Static Variable Declarations
*******************************************************************************/

/* Constants used in various functions for lookup */
const int16 sinLookUp[] = {0,
9,
18,
27,
36,
44,
53,
62,
71,
79,
88,
96,
104,
112,
120,
128,
136,
143,
150,
158,
165,
171,
178,
184,
190,
196,
202,
207,
212,
217,
222,
226,
230,
234,
237,
241,
243,
246,
248,
250,
252,
254,
255,
255,
256,
256,
};
const uint32 sectorLookup[] = {SECTOR_0_WEIGHT, SECTOR_1_WEIGHT, SECTOR_2_WEIGHT, SECTOR_3_WEIGHT, SECTOR_4_WEIGHT};
const uint32 sectorToButtonLookup[] = {CENTRE_BTN_MASK, UP_BTN_MASK, LEFT_BTN_MASK, DOWN_BTN_MASK, RIGHT_BTN_MASK};

/*******************************************************************************
* 	Global Function Prototypes
*******************************************************************************/

int32 Gestures_CalculateY(uint32 radius, int32 theta);
int32 Gestures_CalculateX(uint32 radius, int32 theta);
tGestureId DetectGesture(uint8 isAnySensorActive);

/*******************************************************************************
* 	Local Function Prototypes
*******************************************************************************/

static uint32 Gestures_ValidateButtons(uint32 buttonStatus, uint32 sliderSector);
static int32 Gestures_CalculateTheta(uint32 buttonStatus, int32 sliderPos);
static uint32 Gestures_CalculateRadius(uint32 buttonStatus);
static tGestureId Gestures_Process(uint32 radius, int32 theta);

/*******************************************************************************
* Function Name: DetectGesture
********************************************************************************
*
* Summary:
* This function implements the gesture detection logic. 
*
* Parameters:
* isAnySensorActive - Indicates if user touch is present or not.
* 1 - if any of the sensors (5 buttons and 8 slider elements) is active.
* 0 - If none of the sensors is active.
*
* Return:
* tGestureId - Id of the gesture detected. 
*
* Theory:
* This function waits until MAX_SCAN_COUNT number of unique sensor touches are
* found or the touch is released before identifying the gesture. 
*
*******************************************************************************/
tGestureId DetectGesture(uint8 isAnySensorActive)
{	
	tGestureId gestureId;
	uint32 buttonStatus = 0, sliderSector = 0, sliderPos = 0;	
	
	/* If any sensor is active, get the status of various CapSense elements */
	if(isAnySensorActive)
	{
		/* Read the status of LEFT, RIGHT, UP, DOWN and CENTRE buttons  */
		buttonStatus = CapSense_sensorOnMask[0] & (LEFT_BTN_MASK | RIGHT_BTN_MASK | UP_BTN_MASK | DOWN_BTN_MASK | CENTRE_BTN_MASK);
	}
	
	/* Read the slider status */
	sliderPos = CapSense_GetRadialCentroidPos(CapSense_RADIALSLIDER0__RS);
	
	/* If slider is active, process slider position */
	if(sliderPos != 0xFFFF)
	{
		/* Update slider active status to the button status */
		buttonStatus |= SLIDER_ACT_MASK;
		
		/* Find the sector in which finger is present */
		sliderSector = sliderPos/ANGLE_45;	
		
		/* Radial slider offset correction to provide a more accurate theta */
		if(sliderPos < ANGLE_90)
		{
			sliderPos = sliderPos + 10;		
		}
	}
	else
	{
		/* Clear slider active status */
		buttonStatus &= ~SLIDER_ACT_MASK;
	}
	
	/* Validate the buttons pressed and update the status with valid button presses */
	buttonStatus = Gestures_ValidateButtons(buttonStatus, sliderSector);
	
	/* Get radius and theta from active buttons and slider position */
	radius = Gestures_CalculateRadius(buttonStatus);
	theta = Gestures_CalculateTheta(buttonStatus, sliderPos);
	
	/* Obtain the gesture from theta and radius */
	gestureId = Gestures_Process(radius, theta);
	
	#ifdef CALCULATE_X_Y_IN_GES
		/* Calculate the Cartesian coordinates from Polar coordinates - provided a valid angle is detected/present */
		if(theta != INVALID_ANGLE)
		{
			xPos = Gestures_CalculateX(radius, theta);
			yPos = Gestures_CalculateY(radius, theta);
		}
		else
		{
			xPos = 0;
			yPos = 0;
		}
	#endif
	
	return gestureId;		
}

/*******************************************************************************
* Function Name: Gestures_ValidateButtons
********************************************************************************
*
* Summary:
* This function validates the button and slider active status and returns the 
*	validated output. 
*
* Parameters:
*  buttonStatus - status of various buttons, refer to Gesture.h for various
*					masks
*  sliderSector - sector of the slider which is active (slider divided into 4 
*                   sectors)
*
* Return:
* uint32 - validated button status
*
*******************************************************************************/
uint32 Gestures_ValidateButtons(uint32 buttonStatus, uint32 sliderSector)
{
	static uint32 prevButtonStatus;
	
	/* If both slider and centre button are active, slider gets preference
		Only one of these two can be activated in the design */
	if((buttonStatus & SLIDER_ACT_MASK) && (buttonStatus & CENTRE_BTN_MASK))
	{
		buttonStatus &= ~ CENTRE_BTN_MASK;		
	}
	
	/* If both left and right button are active, analyze and report only the
		more probable button */
	if((buttonStatus & LEFT_BTN_MASK) && (buttonStatus & RIGHT_BTN_MASK))
	{
		/* If slider is active, then check the sector of slider is referred to 
			decide the button to be reported */
		if(buttonStatus & SLIDER_ACT_MASK)
		{
			if((sliderSector & SLIDER_4_SECTOR_MASK) == SECTOR_LEFT) 
			/* Clear Right button mask if slider sector is left */
			{
				buttonStatus &= ~RIGHT_BTN_MASK;
			}
			else if((sliderSector & SLIDER_4_SECTOR_MASK) == SECTOR_RIGHT)
			/* Clear Left button mask if slider sector is Right */
			{
				buttonStatus &= ~LEFT_BTN_MASK;
			}
			else
			/* Retain the previous button status if slider sector is neither left nor right */
			{
				buttonStatus &= ~(LEFT_BTN_MASK | RIGHT_BTN_MASK);
				buttonStatus |= prevButtonStatus & (LEFT_BTN_MASK | RIGHT_BTN_MASK);
			}
		}
		else
		/* If slider is not active, use button signal to decide the button */
		{
			if(GetSignal(LEFT_BTN_ID) > GetSignal(RIGHT_BTN_ID))
			{
				buttonStatus &= ~RIGHT_BTN_MASK;
			}
			else if(GetSignal(LEFT_BTN_ID) < GetSignal(RIGHT_BTN_ID))
			{
				buttonStatus &= ~LEFT_BTN_MASK;
			}
			else
			{
				buttonStatus &= ~(LEFT_BTN_MASK | RIGHT_BTN_MASK);
				buttonStatus |= prevButtonStatus & (LEFT_BTN_MASK | RIGHT_BTN_MASK);
			}
		}
	}
	
	/* Same logic as Left/Right button for Up/Down button as well */
	if((buttonStatus & UP_BTN_MASK) && (buttonStatus & DOWN_BTN_MASK))
	{
		if(buttonStatus & SLIDER_ACT_MASK)
		{
			if((sliderSector & SLIDER_4_SECTOR_MASK) == SECTOR_UP)
			{
				buttonStatus &= ~DOWN_BTN_MASK;
			}
			else if((sliderSector & SLIDER_4_SECTOR_MASK) == SECTOR_DOWN)
			{
				buttonStatus &= ~UP_BTN_MASK;
			}
			else
			{
				buttonStatus &= ~(UP_BTN_MASK | DOWN_BTN_MASK);
				buttonStatus |= prevButtonStatus & (UP_BTN_MASK | DOWN_BTN_MASK);
			}
		}
		else
		{
			if(GetSignal(UP_BTN_ID) > GetSignal(DOWN_BTN_ID))
			{
				buttonStatus &= ~DOWN_BTN_MASK;
			}
			else if(GetSignal(UP_BTN_ID) < GetSignal(DOWN_BTN_ID))
			{
				buttonStatus &= ~UP_BTN_MASK;
			}
			else
			{
				buttonStatus &= ~(UP_BTN_MASK | DOWN_BTN_MASK);
				buttonStatus |= prevButtonStatus & (UP_BTN_MASK | DOWN_BTN_MASK);
			}
		}
	}
	
	/* Store current buttonStatus to previous button status for next cycle */
	prevButtonStatus = buttonStatus;
	
	/* Return the validated button status */
	return	buttonStatus;
}


/*******************************************************************************
* Function Name: Gestures_CalculateRadius
********************************************************************************
*
* Summary:
* This function calculates the radius of active point on the Gesture Pad 
*
* Parameters:
*  buttonStatus - status of various buttons, refer to Gesture.h for various
*					masks
*
* Return:
* uint32 - calculated radius (valid range 0 to 4)
*
*******************************************************************************/
uint32 Gestures_CalculateRadius(uint32 buttonStatus)
{
	uint32 radius = INVALID_RADIUS;
	
	/* Check if slider is active */
	if(buttonStatus & SLIDER_ACT_MASK)
	{
		/* If slider is active then 
            1. radius = 3, if any of the LEFT/RIGHT/UP/DOWN buttons are active
            2. radius = 4, if ONLY slider is active */
		if(buttonStatus & (LEFT_BTN_MASK | RIGHT_BTN_MASK | UP_BTN_MASK | DOWN_BTN_MASK))
		{
			radius = 3;
		}
		else
		{
			radius = 4;
		}
	}
	/* Check if centre button is active */
	else if(buttonStatus & CENTRE_BTN_MASK)
	{
		/* If centre button is active then 
            1. radius = 1, if any of the LEFT/RIGHT/UP/DOWN buttons are active
		    2. radius = 0, if ONLY centre button is active */
		if(buttonStatus & (LEFT_BTN_MASK | RIGHT_BTN_MASK | UP_BTN_MASK | DOWN_BTN_MASK))
		{
			radius = 1;
		}
		else
		{
			radius = 0;
		}
	}
	else if(buttonStatus != 0)
	{
		/* radius = 2, if neither slider nor centre button is active */
		radius = 2;
	}
	
	return radius;
}

/*******************************************************************************
* Function Name: Gestures_CalculateTheta
********************************************************************************
*
* Summary:
* This function calculates the theta of active point on the Gesture Pad 
*
* Parameters:
*  buttonStatus - status of various buttons, refer to Gesture.h for various
*					masks
*  sliderPos - position of finger on slider
*
* Return:
* int32 - calculated theta (valid range -90 to 90); 1 count = 2 degrees;
*
*******************************************************************************/
int32 Gestures_CalculateTheta(uint32 buttonStatus, int32 sliderPos)
{
	int32 theta = INVALID_ANGLE;
	
	/* If slider is active, theta is calculated from the slider centroid */
	if(buttonStatus & SLIDER_ACT_MASK)
	{
		/* theta is in the range of -90 to 90 (-180 degrees to +180 degrees), hence adjust slider output 
			with respect to mid position to get -90 to 90 output for theta */
		if(sliderPos > SLIDER_MID_POS)
		{
			theta = sliderPos - SLIDER_RESOLUTION;
		}
		else
		{
			theta = sliderPos;
		}
	}
	/* If slider is not active, check any of LEFT/RIGHT/UP/DOWN is active */
	else if(buttonStatus & (LEFT_BTN_MASK | RIGHT_BTN_MASK | UP_BTN_MASK | DOWN_BTN_MASK))
	{
		/* Based on the combination of active buttons between LEFT/RIGHT/UP/DOWN, report theta */
		switch(buttonStatus & (LEFT_BTN_MASK | RIGHT_BTN_MASK | UP_BTN_MASK | DOWN_BTN_MASK))
		{
			/* Only LEFT active, theta = 135 degree */
			case (LEFT_BTN_MASK):
				theta = ANGLE_135;
			break;
			
			/* LEFT and DOWN active, theta = 180 degree */
			case (LEFT_BTN_MASK | DOWN_BTN_MASK):
				theta = ANGLE_180;
			break;
			
			/* LEFT and UP active, theta = 90 degree */
			case (LEFT_BTN_MASK | UP_BTN_MASK):
				theta = ANGLE_90;
			break;
			
			/* Only UP active, theta = 45 degree */
			case (UP_BTN_MASK):
				theta = ANGLE_45;
			break;
			
			/* RIGHT and UP active, theta = 0 degree */
			case (RIGHT_BTN_MASK | UP_BTN_MASK ):
				theta = ANGLE_0;
			break;
			
			/* Only RIGHT active, theta = -45 degree */
			case (RIGHT_BTN_MASK):
				theta = -ANGLE_45;
			break;
			
			/* RIGHT and DOWN active, theta = -90 degree */
			case (RIGHT_BTN_MASK | DOWN_BTN_MASK):
				theta = -ANGLE_90;
			break;
			
			/* Only DOWN active, theta = -135 degree */
			case (DOWN_BTN_MASK):
				theta = -ANGLE_135;
			break;
			
			/* For any other combination, theta is not valid */
			default:
				theta = INVALID_ANGLE;
			break;
		}
		
	}
	/* If only CENTRE button is active, theta = 0 */
	else if(buttonStatus != 0)
	{
		theta = 0;
	}
	
	return theta;
}

/*******************************************************************************
* Function Name: GetSinTheta
********************************************************************************
*
* Summary:
* This function calculates the sine theta of a given angle 
*
* Parameters:
*  theta - angle for which sine needs to be calculated (-90 to 90);
*			1 count = 2 degrees;
*
* Return:
* int32 - sine theta value scaled to 256 (i.e. 1 = 256, 0.5 = 128 etc)
*
*******************************************************************************/
int32 GetSinTheta(int32 theta)
{
	int32 sinTheta = INVALID_SINE;
	
	/* Return sine theta, only if angle/theta is valid */
	if(theta != INVALID_ANGLE)
	{
		/* Based on theta, lookup the sine table appropriately and return the value */
		/* sin(-theta) = -sin(theta) */
		if(theta < 0)
		{			
			theta = -theta;
			
			/* sin(180-theta) = sin(theta) */
			if(theta > ANGLE_90)
			{
				sinTheta = -1 * sinLookUp[ANGLE_180 - theta];
			}
			else
			{
				sinTheta = -1 * sinLookUp[theta];
			}
		}
		else
		{
			/* sin(180-theta) = sin(theta) */
			if(theta > ANGLE_90)
			{
				sinTheta = sinLookUp[ANGLE_180 - theta];
			}
			else
			{
				sinTheta = sinLookUp[theta];
			}
		}
	}
	
	
	return sinTheta;
}

/*******************************************************************************
* Function Name: GetCosTheta
********************************************************************************
*
* Summary:
* This function calculates the cosine theta of a given angle 
*
* Parameters:
*  theta - angle for which cosine needs to be calculated (-90 to 90);
*			1 count = 2 degrees;
*
* Return:
* int32 - cosine theta value scaled to 256 (i.e. 1 = 256, 0.5 = 128 etc)
*
*******************************************************************************/
int32 GetCosTheta(int32 theta)
{
	int32 cosTheta = INVALID_SINE;
	
	/* Return cosine theta, only if angle/theta is valid */
	if(theta != INVALID_ANGLE)
	{
		/* Trigonometric relations used in the calculation
		cos(theta) = sin(90 - theta) 
		cos(-theta) = cos(theta) 
		cos(180 - theta) = -cos(theta) */
		if(theta < 0)
		{
			theta = -theta;
		}
		
		if(theta > ANGLE_90)
		{
			cosTheta = -1 * sinLookUp[theta - ANGLE_90];
		}
		else
		{
			cosTheta = sinLookUp[ANGLE_90 - theta];
		}
	}
	
	return cosTheta;
}

/*******************************************************************************
* Function Name: Gestures_CalculateX
********************************************************************************
*
* Summary:
* This function calculates the Cartesian coordinate x from Polar co-ordinates 
*
* Parameters:
* radius - Polar co-ordinate radius  (0 to 4)
* theta - Polar co-ordinate theta (-90 to 90);
*			1 count = 2 degrees;
*
* Return:
* int32 - Cartesian co-ordinate x (-8 to +8)
*
*******************************************************************************/
int32 Gestures_CalculateX(uint32 radius, int32 theta)
{
	int32 x;
	
	/* Shifting theta by 45 degree to provide a proper x offset */
	theta += ANGLE_45;
	
	/* Rollover theta overflow because of the 45 degree addition */
	if(theta > ANGLE_180)
	{
		/* To bring back theta to -180 to 180 degree range, subtract 360 degree from theta */
		theta = theta - (ANGLE_180<<1);
	}
	
	/* x = radius * cos(theta)
		Multiplication by MAX_X_Y is for scaling 
		Shift right by 8 is to remove the cos(theta) normalization to 256 */
	x = (((int32)radius) * GetCosTheta(theta) * MAX_X_Y )>> 8;

	
	return x;
}

/*******************************************************************************
* Function Name: Gestures_CalculateY
********************************************************************************
*
* Summary:
* This function calculates the Cartesian coordinate y from Polar co-ordinates 
*
* Parameters:
* radius - Polar co-ordinate radius  (0 to 4)
* theta - Polar co-ordinate theta (-90 to 90);
*			1 count = 2 degrees;
*
* Return:
* int32 - Cartesian co-ordinate y (-8 to +8)
*
*******************************************************************************/
int32 Gestures_CalculateY(uint32 radius, int32 theta)
{
	int32 y;
	
	theta += ANGLE_45;
	
	if(theta > ANGLE_180)
	{
		theta = theta - (ANGLE_180<<1);
	}
	
	/* y = radius * sin(theta)
		Multiplication by MAX_X_Y is for scaling 
		Shift right by 8 is to remove the sin(theta) normalization to 256 */
	y = (((int32)radius) * GetSinTheta(theta) * MAX_X_Y )>> 8;
	
	return y;
}

/*******************************************************************************
* Function Name: Gestures_GetSector
********************************************************************************
*
* Summary:
* This function calculates the sector in which finger is present. There are a 
*	total of 5 sectors. 
*
* Parameters:
* radius - Polar co-ordinate radius  (0 to 4)
* theta - Polar co-ordinate theta (-90 to 90);
*			1 count = 2 degrees;
*
* Return:
* uint32 - sector number
*	0 - middle sector (radius = 0) 
*	1 - up sector (radius > 0, 0 < theta < 45)
*	2 - left sector (radius > 0, 45 < theta < 90)
*	3 - down sector (radius > 0, -45 > theta > -90)
*	4 - right sector (radius > 0, 0 > theta > -45)
*
*******************************************************************************/
uint32 Gestures_GetSector(uint32 radius, int32 theta)
{
	uint32 sector = INVALID_SECTOR;
	
	/* Process only for valid theta */
	if(theta != INVALID_ANGLE)
	{
		/* If radius = 0, sector is 0 (CENTRE button) */
		if(radius == 0)
		{
			sector = 0;
		}
		else
		{
			/* Calculate sector based on theta if radius != 0 */
			if(theta >= 0)
			{
				sector = 1 + (theta/ANGLE_90);
			}
			else
			{
				sector = (uint32)((int32)4 + (theta/ANGLE_90));
			}			
		}
	}
	
	return sector;
}

/*******************************************************************************
* Function Name: Gestures_LookUpGesture
********************************************************************************
*
* Summary:
* This function returns the gesture based on radius detected at first touch and
*	a unique sum of prime products term
*
* Parameters:
* sectorHistorySum - unique sum term for each gesture (combination of 3 previous 
*						sector states and sector weights)
* firstRadius - radius value when the sector was first touched during the activity
*
* Return:
* tGestureId - gesture ID (refer to Gestures.h for details)
*
*******************************************************************************/
static tGestureId Gestures_LookUpGesture(uint32 sectorHistorySum, uint32 firstRadius)
{
	tGestureId gesture = GESTURE_INVALID;
	
	/* Lookup gestures based on the unique sum term for each gesture.
		The unique sum term is calculated from the last/recent three 
		sector changes recorded from the time finger is active
	
		The following logic is applied to determine the sum and make it unique - 
		1. Assign a unique prime number as weights to each sector
		2. Assign a unique prime number as weights for each of the past 3 states
		3. The weights should be unique for sector and past states 
		4. When two unique primes are multipled and added, they provide a unique sum
		5. The sector weights are multiplied with the state weight and added together 
			with other sector-state product recorded (total three). This sum will be unique
		6. This unique sum can then be used to identify the finger movement between sectors
			with a history of three sectors */
	switch(sectorHistorySum)
	{
		case SECTOR_1_0_3:
			gesture = GESTURE_DOWN_SWIPE;
		break;
		
		case SECTOR_3_0_1:
			gesture = GESTURE_UP_SWIPE;
		break;
		
		case SECTOR_2_0_4:
			gesture = GESTURE_RIGHT_SWIPE;
		break;
		
		case SECTOR_4_0_2:
			gesture = GESTURE_LEFT_SWIPE;
		break;
		
		case SECTOR_1_2_3:
		case SECTOR_2_3_4:
		case SECTOR_3_4_1:
		case SECTOR_4_1_2:
			if(firstRadius != INVALID_RADIUS)
			{
				if(firstRadius < 3)
				{
					gesture = GESTURE_INNER_COUNTER_CLKWISE;
				}
				else
				{
					gesture = GESTURE_OUTER_COUNTER_CLKWISE;
				}
			}
		break;
		
		case SECTOR_1_4_3:
		case SECTOR_4_3_2:
		case SECTOR_3_2_1:
		case SECTOR_2_1_4:
			if(firstRadius != INVALID_RADIUS)
			{
				if(firstRadius < 3)
				{
					gesture = GESTURE_INNER_CLKWISE;
				}
				else
				{
					gesture = GESTURE_OUTER_CLKWISE;
				}
			}
		break;
		
		default:
		break;		
	}
	
	return gesture;
}

/*******************************************************************************
* Function Name: Gestures_Process
********************************************************************************
*
* Summary:
* This function processes the current activity on Gesture Pad and returns the
*	gesture detected
*
* Parameters:
* radius - Polar co-ordinate radius  (0 to 4)
* theta - Polar co-ordinate theta (-90 to 90);
*			1 count = 2 degrees;
*
* Return:
* tGestureId - gesture ID (refer to Gestures.h for details)
*
*******************************************************************************/
tGestureId Gestures_Process(uint32 radius, int32 theta)
{
	tGestureId gesture = GESTURE_NONE;
	static tGestureId stickyGesture = GESTURE_NONE;
	static uint32 tapCounter, firstRadius, prevRadius;
	static int32 firstTheta, prevTheta;
	static uint32 processState = GESTURE_STATE_IDLE, debounceCounter;
	static uint32 Gesture_sectorHistory[3] = {INVALID_SECTOR, INVALID_SECTOR, INVALID_SECTOR};
	
	int32 radiusChange, thetaChange;
	uint32 sector, sectorHistorySum;
	
	/* Run the gesture recognition */
	switch(processState)
	{
		/* Stay in IDLE state if no finger is present or theta is invalid */
		case GESTURE_STATE_IDLE:
			if(theta != INVALID_ANGLE)
			{
				/* Store the first radius, theta and sector details, when first valid theta is detected ==> finger touch */
				firstRadius = radius;
				firstTheta = theta;				
				Gesture_sectorHistory[FIRST_SECTOR_INDEX] = Gestures_GetSector(radius, theta);
				
				/* Move the state to 1st sector process */
				processState = GESTURE_STATE_1ST_SECTOR;
			}
		break;
		
			/* 1st sector process - Buttons are processed in this state */
		case GESTURE_STATE_1ST_SECTOR:
			/* Check for valid theta */			
			if(theta != INVALID_ANGLE)
			{
				/* If tap counter has crossed the long press time, report the button active status */
				if(tapCounter > (TAP_TIME_1_SEC*10))
				{
					capsenseButtonStatus = sectorToButtonLookup[Gesture_sectorHistory[FIRST_SECTOR_INDEX]];
				}
				else
				{
					tapCounter++;
				}
				
				/* Monitor the radius and theta change from first touch, if either of them change by a predefined threshold
					Move to 1st sector moved state as this state processes buttons */
				radiusChange = firstRadius - radius;
				if(radiusChange < 0)
				{
					radiusChange = -radiusChange;
				}
				
				thetaChange = firstTheta - theta;
				if(thetaChange < 0)
				{
					thetaChange = -thetaChange;
				}
				
				if((thetaChange >= THETA_CHANGE_THRESHOLD) || (radiusChange >= RADIUS_CHANGE_THRESHOLD))
				{
					processState = GESTURE_STATE_1ST_SECTOR_MOVED;
					tapCounter = 0;
				}
				
				/* Accumulate radius change */				
				radiusChange = radius - prevRadius;
				accRadius += radiusChange;
			}
			/* If theta becomes invalid i.e. touch is released, report tap on the particular sector/button */
			else
			{
				if(tapCounter < TAP_TIME_1_SEC)
				{
					capsenseButtonStatus = sectorToButtonLookup[Gesture_sectorHistory[FIRST_SECTOR_INDEX]];
				}
				
				/* Move to the gesture release state to clear the state machine variables for next cycle */
				processState = GESTURE_STATE_SECTOR_RELEASE;
			}
		break;
		
		/* 1st sector moved state - monitors sector change */
		case GESTURE_STATE_1ST_SECTOR_MOVED:
			if(theta != INVALID_ANGLE)
			{
				/* accumulate radius and theta for calculations */
				thetaChange = theta - prevTheta;
				
				/* Logic to avoid theta change overflows at theta cross over junctions (180 and 0 degrees) */
				if(thetaChange > ANGLE_90)
				{
					thetaChange = thetaChange - 2*ANGLE_180;
				}
				else if(thetaChange < -ANGLE_90)
				{
					thetaChange = 2*ANGLE_180 + thetaChange;
				}
				
				accTheta += thetaChange;
				
				radiusChange = radius - prevRadius;
				accRadius += radiusChange;
				
				/* Monitor the sector */
				sector = Gestures_GetSector(radius, theta);
				
				/* If there is a change in sector, move to 2nd sector state or outward swipe state depending on 
					first active sector */
				if(sector != Gesture_sectorHistory[FIRST_SECTOR_INDEX])
				{
					/* sector change is accepted only after a debounce */
					debounceCounter++;
					if(debounceCounter > SECTOR_DEBOUNCE)
					{
						/* If first sector was 0 and there is a sector change, report outward swipe */
						if(Gesture_sectorHistory[FIRST_SECTOR_INDEX] == SECTOR_0)
						{
							processState = GESTURE_STATE_OUTWARD_SWIPE;
						}
						else
						{
							processState = GESTURE_STATE_2ND_SECTOR;
						}
						
						/* Store the new sector */
						Gesture_sectorHistory[SECOND_SECTOR_INDEX] = sector;
						debounceCounter = 0;
					}
				}
				else
				{
					debounceCounter = 0;
				}
			}
			else
			{
				/* If the finger is released (theta invalid), move to gesture release state */
				processState = GESTURE_STATE_SECTOR_RELEASE;
			}
		break;
		
		/* 2nd sector state, the state machine will remain in this state as long as finger is present and the sector does not change */
		case GESTURE_STATE_2ND_SECTOR:
			/* Stay in state till a valid theta is present */
			if(theta != INVALID_ANGLE)
			{				
				/* Accumulate theta and radius changes */
				thetaChange = theta - prevTheta;
				
				/* Theta overflow/underflow correction */
				if(thetaChange > ANGLE_90)
				{
					thetaChange = thetaChange - 2*ANGLE_180;
				}
				else if(thetaChange < -ANGLE_90)
				{
					thetaChange = 2*ANGLE_180 + thetaChange;
				}
				
				accTheta += thetaChange;
				
				radiusChange = radius - prevRadius;
				accRadius += radiusChange;
				
				/* Read the current sector */
				sector = Gestures_GetSector(radius, theta);
				
				/* Check if the sector has changed */
				if(sector != Gesture_sectorHistory[SECOND_SECTOR_INDEX])
				{
					debounceCounter++;
					
					/* Debounce the sector change before processing the sector change related activities */
					if(debounceCounter > SECTOR_DEBOUNCE)
					{
						/* If a sector change occurs, then this would be the third sector change.
							Hence update the sector history and process gestures (we require a minimum 
							three sectors to process a gesture). */
						processState = GESTURE_STATE_3RD_SECTOR;
						Gesture_sectorHistory[THIRD_SECTOR_INDEX] = sector;
						
						/* Calculate the unique sector sum gesture detection */
						sectorHistorySum = sectorLookup[Gesture_sectorHistory[THIRD_SECTOR_INDEX]] * THIRD_SECTOR_WEIGHT
									+ sectorLookup[Gesture_sectorHistory[SECOND_SECTOR_INDEX]] * SECOND_SECTOR_WEIGHT
									+ sectorLookup[Gesture_sectorHistory[FIRST_SECTOR_INDEX]] * FIRST_SECTOR_WEIGHT;
						
						gesture = Gestures_LookUpGesture(sectorHistorySum, firstRadius);
						
						/* Make the gesture as sticky for UP/DOWN swipe
							sticky means the gesture will be reported as long as the finger is held in the sector */
						if((gesture == GESTURE_DOWN_SWIPE) || (gesture == GESTURE_UP_SWIPE))
						{
							stickyGesture = gesture;
						}
						
						debounceCounter = 0;
					}
				}
				else
				{
					debounceCounter = 0;
				}
			}
			else
			{
				processState = GESTURE_STATE_SECTOR_RELEASE;
			}
		break;
		
		/* 3rd sector state - This state is similar to 2nd sector state.
			Gesture will be processed every time a sector change occurs */
		case GESTURE_STATE_3RD_SECTOR:
			if(theta != INVALID_ANGLE)
			{
				thetaChange = theta - prevTheta;
				
				if(thetaChange > ANGLE_90)
				{
					thetaChange = thetaChange - 2*ANGLE_180;
				}
				else if(thetaChange < -ANGLE_90)
				{
					thetaChange = 2*ANGLE_180 + thetaChange;
				}
				
				accTheta += thetaChange;
				
				radiusChange = radius - prevRadius;
				accRadius += radiusChange;
				
				sector = Gestures_GetSector(radius, theta);
				
				if(sector != Gesture_sectorHistory[THIRD_SECTOR_INDEX])
				{
					debounceCounter++;
					if(debounceCounter > SECTOR_DEBOUNCE)
					{
						Gesture_sectorHistory[FIRST_SECTOR_INDEX] = Gesture_sectorHistory[SECOND_SECTOR_INDEX];
						Gesture_sectorHistory[SECOND_SECTOR_INDEX] = Gesture_sectorHistory[THIRD_SECTOR_INDEX];						
						Gesture_sectorHistory[THIRD_SECTOR_INDEX] = sector;
						
						sectorHistorySum = sectorLookup[Gesture_sectorHistory[THIRD_SECTOR_INDEX]] * THIRD_SECTOR_WEIGHT
									+ sectorLookup[Gesture_sectorHistory[SECOND_SECTOR_INDEX]] * SECOND_SECTOR_WEIGHT
									+ sectorLookup[Gesture_sectorHistory[FIRST_SECTOR_INDEX]] * FIRST_SECTOR_WEIGHT;
						
						gesture = Gestures_LookUpGesture(sectorHistorySum, firstRadius);
						
						if((gesture == GESTURE_DOWN_SWIPE) || (gesture == GESTURE_UP_SWIPE))
						{
							stickyGesture = gesture;
						}
						else
						{
							stickyGesture = GESTURE_NONE;
						}
						
						debounceCounter = 0;
					}
				}
				else
				{
					debounceCounter = 0;
					
					if(stickyGesture != GESTURE_NONE)
					{
						gesture = stickyGesture;
					}
				}
			}
			else
			{				
				processState = GESTURE_STATE_SECTOR_RELEASE;
			}
		break;
		
		/* Outward swipe state, this state is triggered when first sector is 0.
			In this state, outward swipe gesture is reported till finger is removed */
		case GESTURE_STATE_OUTWARD_SWIPE:
			if(theta != INVALID_ANGLE)
			{
				gesture = GESTURE_OUTWARD_SWIPE;
				
				thetaChange = theta - prevTheta;
				
				if(thetaChange > ANGLE_90)
				{
					thetaChange = thetaChange - 2*ANGLE_180;
				}
				else if(thetaChange < -ANGLE_90)
				{
					thetaChange = 2*ANGLE_180 + thetaChange;
				}
				
				accTheta += thetaChange;
			}
			else
			{
				processState = GESTURE_STATE_SECTOR_RELEASE;
			}
		break;
		
		/* Gesture release state - this state clears all state machine variables for the next activation */
		case GESTURE_STATE_SECTOR_RELEASE:			
			debounceCounter = 0;
			tapCounter = 0;
			
			Gesture_sectorHistory[SECOND_SECTOR_INDEX] = INVALID_SECTOR;
			Gesture_sectorHistory[FIRST_SECTOR_INDEX] = INVALID_SECTOR;
			Gesture_sectorHistory[THIRD_SECTOR_INDEX] = INVALID_SECTOR;
			
			firstRadius = INVALID_RADIUS;
			firstTheta = INVALID_ANGLE;
			
			accRadius = 0;
			accTheta = 0;
			
			capsenseButtonStatus = 0;
			
			stickyGesture = GESTURE_NONE;
			
			processState = GESTURE_STATE_IDLE;			
		break;
					
		default:
		break;
	}
	
	/* Invalidate first radius if there is significant variation in accumulated radius
		This is done for invalidating circular gestures which uses first touch radius as the base for detecting circular gestures */
	if(((accRadius > 1) || (accRadius < -1)))
	{
		firstRadius = INVALID_RADIUS;						
	}
	
	prevRadius = radius;
	prevTheta = theta;
	
	return gesture;
}
/* [] END OF FILE */
