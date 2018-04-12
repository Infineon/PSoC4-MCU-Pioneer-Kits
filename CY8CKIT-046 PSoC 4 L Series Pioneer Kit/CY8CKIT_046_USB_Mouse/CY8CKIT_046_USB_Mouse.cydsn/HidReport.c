/*******************************************************************************
* File Name: HidReport.c
*
* Version 1.0
*
* Description: This file contains routines for converting CapSense Gestures to
*               HID report.
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

#include "HidReport.h"
#include "main.h"
#include "Gesture.h"
#include "cytypes.h"
#include "UART_SPI_UART.h"


/*******************************************************************************
*   Global Variable Declarations
*******************************************************************************/

/* HID report arrays for keyboard & mouse. */
/* Mouse data report array. The report format is as follows. 
 * Byte0: Bit0 - Left button, Bit1 - Right button, Bit2 - Middle button, 
 * 		  Bit3 to Bit7 - Not used.
 * Byte1: X position relative to the current position of the cursor.
 * Byte2: Y position relative to the current position of the cursor.
 * Byte3: Vertical scroll bar position relative to the current position of the
 * 		  scroll bar.
 */
int8 mouseReport[MOUSE_RPT_SIZE] = {0, 0, 0, 0};

/* Keyboard data report array. The report format is as follows. 
 * Byte0: Modifier keys
 * Byte1: Reserved (Not used).
 * Byte2 to Byte7: Codes of keys 1 to 6.
 * Refer to AN58726 to learn more about keyboard report format.
 */
uint8 keyboardReport[KEY_RPT_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};


/*******************************************************************************
*   External Variable Declarations
*******************************************************************************/

/* Indicates the centroid value of the radial slider. */
extern uint16 sliderCentroid;


/*******************************************************************************
* Function Name: ConvertGestureToHidReport
********************************************************************************
*
* Summary:
* This function updates the keyboard and mouse report arrays based on the given
* gesture id. 
*
* Parameters:
* gestureId - Id of the detected gesture.
*
* Return:
* None
*
*******************************************************************************/
void ConvertGestureToHidReport(tGestureId gestureId)
{
	/* This variable is used to control the speed of the scroll bar movement. Refer
	 * to the usage in the switch-case statement below. 
	 */
	static uint32 scrollUpdateThreshold = 0;	
	
	static tGestureId prevGestureId = GESTURE_NONE;
	
	uint32 i;

	/* Initialize keyboard & mouse report to zero before updating with valid
	 * values.
	 */	
	keyboardReport[KEY_RPT_MODIF_POS] = 0;
	keyboardReport[KEY_RPT_KEY1_POS] = 0;
	
	for(i = 0; i < MOUSE_RPT_SIZE; i++)
	{
		mouseReport[i] = 0;
	}
	
	/* Process Mouse buttons */
	switch (capsenseButtonStatus)
	{
		case LEFT_BTN_MASK:
			mouseReport[MOUSE_RPT_BTN_POS] |= MOUSE_RPT_LEFT_BTN_MASK; 
			PRINT("Left Click\n\r");
		break; 
		
		case RIGHT_BTN_MASK:
			mouseReport[MOUSE_RPT_BTN_POS] |= MOUSE_RPT_RIGHT_BTN_MASK; 
			PRINT("Right Click\n\r");
		break;
			
		case CENTRE_BTN_MASK:
			mouseReport[MOUSE_RPT_BTN_POS] |= MOUSE_RPT_MIDDLE_BTN_MASK; 
			PRINT("Middle Click\n\r");
		break;
			
		default:
			break;
	}
	
	/* Refer to the DetectGesture() function in Gesture.c file to learn more on how
	 * these gestures are detected. 
	 */
	switch(gestureId)
	{		
	case GESTURE_LEFT_SWIPE:
		/* Left swipe sends out Ctrl + Shift + TAB keys to computer */
		PRINT("Left Swipe\n\r");
		keyboardReport[KEY_RPT_MODIF_POS] |= (KEY_RPT_LEFT_CTRL_MASK | KEY_RPT_LEFT_SHIFT_MASK); 
		keyboardReport[KEY_RPT_KEY1_POS] = KEY_RPT_TAB_CODE;
		break;
		
	case GESTURE_RIGHT_SWIPE:
		/* Right swipe sends out Ctrl + TAB keys to computer */
		PRINT("Right Swipe\n\r");
		keyboardReport[KEY_RPT_MODIF_POS] |= KEY_RPT_LEFT_CTRL_MASK; 
		keyboardReport[KEY_RPT_KEY1_POS] = KEY_RPT_TAB_CODE;
		break;
		
	case GESTURE_UP_SWIPE:
		/* Up swipe moves the vertical scroll bar up. User can release the touch after
		 * the gesture that would move the scroll bar once or user can continue 
		 * touching the Up button which would move the scroll bar up continuously. 
		 */
		if(prevGestureId != GESTURE_UP_SWIPE)
		{
			/* Start with higher speed the first time scroll gesture occurs. */
			scrollUpdateThreshold = 0;
			mouseReport[MOUSE_RPT_V_POS] = START_SCROLL_SPEED;
			PRINT("Up Swipe\n\r");
		}
		else if(++scrollUpdateThreshold >= CONT_SCROLL_UPDATE_FREQ)
		{
			/* Control scroll speed by limiting the update rate */
			scrollUpdateThreshold = 0;
			PRINT("Up Swipe\n\r");
			mouseReport[MOUSE_RPT_V_POS] = CONT_SCROLL_SPEED;
		}
		break;
		
	case GESTURE_DOWN_SWIPE:
		/* Down swipe moves the vertical scroll bar down. User can release the touch 
		 * after the gesture that would move the scroll bar once or user can continue 
		 * touching the Down button which would move the scroll bar down continuously. 
		 */
		if(prevGestureId != GESTURE_DOWN_SWIPE)
		{
			/* Start with higher speed the first time scroll gesture occurs. */
			scrollUpdateThreshold = 0;
			mouseReport[MOUSE_RPT_V_POS] = -START_SCROLL_SPEED;
			PRINT("Down Swipe\n\r");
		}
		else if(++scrollUpdateThreshold >= CONT_SCROLL_UPDATE_FREQ)
		{
			/* Control scroll speed by limiting the update rate */
			scrollUpdateThreshold = 0;
			PRINT("Down Swipe\n\r");
			mouseReport[MOUSE_RPT_V_POS] = -CONT_SCROLL_SPEED;
		}
		break;
		
	case GESTURE_OUTWARD_SWIPE:
		if(prevGestureId != GESTURE_OUTWARD_SWIPE)
		{				
			PRINT("Outward Swipe\n\r");
		}
		mouseReport[MOUSE_RPT_X_POS] = (int8)xPos/2;
		mouseReport[MOUSE_RPT_Y_POS] = -(int8)yPos/2;
		break;

	case GESTURE_INNER_CLKWISE:
		PRINT("Inner Clockwise\n\r");
		break;
		
	case GESTURE_INNER_COUNTER_CLKWISE:
		PRINT("Inner Counter-Clockwise\n\r");
		break;
		
	case GESTURE_OUTER_CLKWISE:
		if(prevGestureId != GESTURE_OUTER_CLKWISE)
		{
			PRINT("Outer Clockwise\n\r");
		}
		break;
		
	case GESTURE_OUTER_COUNTER_CLKWISE:
		if(prevGestureId != GESTURE_OUTER_COUNTER_CLKWISE)
		{
			PRINT("Outer Counter-Clockwise\n\r");
		}
		break;
		
	default:
		break;
	}	
	
	prevGestureId = gestureId;
}


/* [] END OF FILE */
