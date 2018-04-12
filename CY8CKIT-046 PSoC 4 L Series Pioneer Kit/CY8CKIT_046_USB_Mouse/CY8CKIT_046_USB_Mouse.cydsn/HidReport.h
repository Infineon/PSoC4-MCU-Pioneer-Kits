/*****************************************************************************
* File Name		: HidReport.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  HidReport.c
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

#ifndef HID_REPORT_H		/* Guard to prevent multiple inclusions */
#define HID_REPORT_H
	
#include "Gesture.h"


/*******************************************************************************
* Macro Definitions
*******************************************************************************/
	
/* Defines the relative cursor movement to report when the user issues a mouse
 * gesture for the first time.
 */
#define START_MOUSE_SPEED				(15)

/* Defines the relative cursor movement to report when the user intends to move
 * the cursor continuously.
 */
#define CONT_MOUSE_SPEED				(1)

/* Defines the relative horizontal scroll bar movement to report when the user
 * issues the scroll gesture for the first time. 
 */
#define START_SCROLL_SPEED				(4)

/* This macro defines the relative horizontal scroll bar movement to report when
 * the user intends to scroll continuously.
*/
#define CONT_SCROLL_SPEED				(1)

/* When the user intends to move the scroll bar continuously, the relative
 * position is not continuously sent to the computer. In order to control the
 * speed, the position is sent periodically with zero in between. The position
 * is sent whenever the value of an incrementing variable reaches this macro 
 * value.
 */
#define CONT_SCROLL_UPDATE_FREQ			(10)

/* Macros to be used with keyboard HID report. Do not modify these macros. */

/* Indicates the position in the keyboard report array at which the status of
 * the modifier keys are indicated. 
 */
#define KEY_RPT_MODIF_POS				(0)

/* Indicates the position in the keyboard report array at which the status of
 * the 1st non-modifier key is indicated. 
 */
#define KEY_RPT_KEY1_POS				(2)

/* Mask for Left CTRL key */
#define KEY_RPT_LEFT_CTRL_MASK			(1)

/* Mask for Left shift key */
#define KEY_RPT_LEFT_SHIFT_MASK			(2)

/* Code for TAB key */
#define KEY_RPT_TAB_CODE				(0x2B)

/* Macros to be used with Mouse HID report. Do not modify these macros. */

/* Indicates the position in the mouse report array at which the status of the
 * buttons (Left, Right, middle) are indicated.
 */
#define MOUSE_RPT_BTN_POS				(0)

/* Indicates the position in the mouse report array at which the relative X
 * movement of the mouse cursor is reported.
 */
#define MOUSE_RPT_X_POS			  		(1)

/* Indicates the position in the mouse report array at which the relative Y
 * movement of the mouse cursor is reported.
 */
#define MOUSE_RPT_Y_POS					(2)

/* Indicates the position in the mouse report array at which the relative
 * movement of the vertical scroll bar is reported.
 */
#define MOUSE_RPT_V_POS					(3)

/* Mask for Left button in the mouse */
#define MOUSE_RPT_LEFT_BTN_MASK			(1)

/* Mask for Right button in the mouse */
#define MOUSE_RPT_RIGHT_BTN_MASK		(2)

/* Mask for Middle button in the mouse */
#define MOUSE_RPT_MIDDLE_BTN_MASK		(4)

/* These macros indicate the sector of the radial slider at which finger is at.
 */
#define MOUSE_DIR_SOUTH_WEST			(0)
#define MOUSE_DIR_WEST					(1)
#define MOUSE_DIR_NORTH_WEST			(2)
#define MOUSE_DIR_NORTH					(3)
#define MOUSE_DIR_NORTH_EAST			(4)
#define MOUSE_DIR_EAST					(5)
#define MOUSE_DIR_SOUTH_EAST			(6)
#define MOUSE_DIR_SOUTH					(7)
	
/* Defines the number of sectors the radial slider is divided into. The
 * direction of the mouse pointer is controlled based on which sector the finger
 * is at.
 */
#define RADIAL_SLIDER_SECTORS			(MOUSE_DIR_SOUTH + 1)	


/*******************************************************************************
*   Function Prototypes
*******************************************************************************/
	
void ConvertGestureToHidReport(tGestureId gestureId);


#endif /* #ifndef HID_REPORT_H */


/* [] END OF FILE */

