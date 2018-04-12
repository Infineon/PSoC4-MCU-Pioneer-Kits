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
#include "CapSense_Gestures.h"
#include "LED_Control.h"
#include <stdbool.h>


/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define ENABLED                     (1u)
#define DISABLED                    (0u)    
#define TUNER_ENABLE                (DISABLED)  
    
#define RESET						(0x00)
#define ZERO						(0x00)
    
/* WDT match values for different duration of DeepSleep power mode.
 * The ILO is operating at 32 kHz. */
#define WDT_MATCH_VALUE_30MS		(32u * 30u)
#define WDT_MATCH_VALUE_200MS		(32u * 200u)

/* SOFT_COUNTER_3SEC_VALUE defined to achieve 3 second duration. 
 * See the explanation for variable softCounter in main.c. */
#define SOFT_COUNTER_3SEC_VALUE		(100u)

/*****************************************************************************
* Function Prototypes
*****************************************************************************/
void EnterDeepSleepLowPowerMode(void);
void Initialize_Project(void);
void UpdateWdtMatchValue(uint32 wdtMatchValue);

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/


#endif /* MAIN_H */

/* [] END OF FILE */
