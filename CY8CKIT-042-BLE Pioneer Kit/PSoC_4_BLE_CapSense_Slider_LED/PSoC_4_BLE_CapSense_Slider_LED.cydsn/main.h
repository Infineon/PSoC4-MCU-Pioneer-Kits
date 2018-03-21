/******************************************************************************
* Project Name		: PSoC_4_BLE_CapSense_Slider_LED
* File Name			: main.h
* Version 			: 1.0
* Device Used		: CY8C4247LQI-BL483
* Software Used		: PSoC Creator 4.1
* Compiler    		: ARM GCC 5.4.1, ARM MDK Generic
* Related Hardware	: CY8CKIT-042-BLE Bluetooth Low Energy Pioneer Kit 
* Owner             : ROIT
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
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
*****************************************************************************/

/********************************************************************************
*	Contains all macros and function declaration used in the main.c file 
********************************************************************************/
#if !defined(MAIN_H)
#define MAIN_H

#include <project.h>
#include <BLEApplications.h>
#include <HandleLowPower.h>

/***************************Macro Declarations*******************************/
/* Respective indexes of color coordiantes in the 4-byte data received
* over RGB LED control characteristic */
#define RED_INDEX						0
#define GREEN_INDEX						1
#define BLUE_INDEX						2
#define INTENSITY_INDEX					3
	
/* Slider position value received from CapSense Component when no finger is 
* placed on the slider */
#define NO_FINGER 						0xFFFFu
	
/* Range of CapSense Slider centroid with finger placed on slider is 0-100 */
#define SLIDER_MAX_VALUE				0x0064

/* PrISM Density value for Max intensity. Note that the LED on the BLE Pioneer 
* kit is active Low, so the correct value to drive to maximum intensity is 0.
* This is taken care by the UpdateRGBled() function */
#define RGB_LED_MAX_VAL					255

/* PrISM density value for LED OFF. Note that LED on BLE Pioneer kit is
* active low */
#define RGB_LED_OFF						255
	
/* PrISM density value for LED ON. Note that LED on BLE Pioneer kit is
* active low */
#define RGB_LED_ON						0

/* Firmware thresholds for the two extremes of LED intensity */
#define LED_FULL_INTENSITY				0xF0
#define LED_NO_COLOR_THRESHOLD			0x04

/* General Macros */
#define TRUE							1
#define FALSE							0
#define ZERO							0
/****************************************************************************/
	
/**************************Function Declarations*****************************/
CY_ISR_PROTO(MyISR);
void InitializeSystem(void);
void UpdateRGBled(void);
void HandleCapSenseSlider(void);
void SendDataOverCapSenseNotification(uint8 CapSenseSliderData);
/****************************************************************************/
#endif
/* [] END OF FILE */
