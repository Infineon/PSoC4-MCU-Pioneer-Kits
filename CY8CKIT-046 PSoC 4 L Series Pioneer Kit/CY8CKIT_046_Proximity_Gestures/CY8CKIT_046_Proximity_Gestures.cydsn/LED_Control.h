/*****************************************************************************
* File Name		: LED_Control.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  LED_Control.c
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
#if !defined(LED_CONTROL_H) 
#define LED_CONTROL_H 

#include "cytypes.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define OFF						    0x00
#define MIN_BRIGHTNESS				500
/*****************************************************************************
* Data Type Definition
*****************************************************************************/


/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/
typedef enum
{
    DISPLAY_LED_OFF,
    DISPLAY_LED_WHITE,
    DISPLAY_LED_BLUE,
    DISPLAY_LED_GREEN,
    DISPLAY_LED_RED,
    DISPLAY_LED_PREV_COLOR,
    DISPLAY_UP_SWIPE,
    DISPLAY_DOWN_SWIPE
}DISPLAY_COMMAND;  

typedef enum
{
    LED_COLOR_WHITE,
    LED_COLOR_BLUE,
    LED_COLOR_GREEN,
    LED_COLOR_RED
}LED_COLOR;  

/*****************************************************************************
* Data Structure Definition
*****************************************************************************/


/*****************************************************************************
* Global Variable Declaration
*****************************************************************************/


/*****************************************************************************
* Function Prototypes
*****************************************************************************/
void ShowPrevLEDColor(void);
void DisplayLEDOutput(DISPLAY_COMMAND command);

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
	
	
#endif /* LED_CONTROL_H */

/* [] END OF FILE */
