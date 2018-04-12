/*****************************************************************************
* File Name		: main.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  main.c
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
#if !defined(MAIN_H) 
#define MAIN_H 

#include <project.h>
#include <stdbool.h>

/*****************************************************************************
* MACRO Definitions
*****************************************************************************/   
/* Constants used to enable/disable tuner */
#define ENABLED                     (1)
#define DISABLED                    (0)    
#define TUNER_ENABLE                (DISABLED)  
    
/* General Constants */    
#define ZERO					(0x00u)
#define OFF						(false)
#define ON						(true)

/* WDT match values for different duration of low power modes.
 * The ILO is operating at 32 kHz. */
#define WDT_MATCH_VALUE_30MS	(32u * 30u) 
#define WDT_MATCH_VALUE_100MS	(32u * 100u)

/* 4 ILO periods expressed in microseconds. */
#define ILOX4 					((4u * 1000u) / 32u)

/* Reset value of softCounter */    
#define RESET					(0u)
	
/* SOFT_COUNTER_3SEC_VALUE defined to achieve 3 second duration. 
 * See the explanation for variable softCounter in main.c. 
 */
#define SOFT_COUNTER_3SEC_VALUE (100u) 

/* LED Brightness Control Constants */  
#define MIN_BRIGHTNESS_COMPARE  (500)
#define MAX_BRIGHTNESS_COMPARE  (PWM_Blue_PWM_PERIOD_VALUE)  
#define BRIGHTNESS_STEP         (50u)
#define MIN_PROXIMITY_SIGNAL    (CapSense_GetFingerThreshold\
                                   (CapSense_PROXIMITYSENSOR__PROX) - \
                                 CapSense_GetFingerHysteresis(CapSense_PROXIMITYSENSOR__PROX)\
                                )    
#define SIGNAL_CHANGE_STEP      (1)
#define MAX_16_BIT_VALUE        (0x7FFFu)    
    
/*****************************************************************************
* Function Prototypes
*****************************************************************************/
void EnterDeepSleepLowPowerMode(void);
void LED_SetBrightness(bool ledState, uint16 signal);
void UpdateWdtMatchValue(uint32 wdtMatchValue);
	
#endif /* MAIN_H */

/* [] END OF FILE */
