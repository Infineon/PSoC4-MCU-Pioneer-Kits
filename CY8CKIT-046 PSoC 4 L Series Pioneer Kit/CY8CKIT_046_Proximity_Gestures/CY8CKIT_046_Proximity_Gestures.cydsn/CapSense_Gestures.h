/*****************************************************************************
* File Name		: CapSense_Gestures.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  CapSense_Gestures.c
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
#if !defined(CAPSENSE_GESTURES_H) 
#define CAPSENSE_GESTURES_H 

#include "cytypes.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
	
#define PROX_DETECT_LIMIT			0x01
	
/* LED Brightness Control Constants */  
#define MIN_BRIGHTNESS_COMPARE  (500u)
#define MAX_BRIGHTNESS_COMPARE  (PWM_Green_PWM_PERIOD_VALUE)  
#define BRIGHTNESS_STEP         (30u)
#define MIN_PROXIMITY_SIGNAL    (CapSense_GetFingerThreshold\
                                   (CapSense_PROXIMITYSENSORCOMBINED__PROX) - \
                                 CapSense_GetFingerHysteresis(CapSense_PROXIMITYSENSORCOMBINED__PROX)\
                                )    
#define SIGNAL_CHANGE_STEP      (1u)
#define MAX_16_BIT_VALUE        (0x7FFFu) 
#define SIGNAL                  (CapSense_GetDiffCountData(CapSense_PROXIMITYSENSORCOMBINED__PROX))    

/*****************************************************************************
* Data Type Definition
*****************************************************************************/
typedef enum
{		
    GESTURE_UP_SWIPE,			
    GESTURE_DOWN_SWIPE,					
    GESTURE_NO_GESTURE,
    GESTURE_SENSORS_INACTIVE
}GESTURE;

typedef enum
{
    GESTURE_ZONE_OUT,
    GESTURE_ZONE_SENSOR_0,										
    GESTURE_ZONE_SENSOR_1,
    GESTURE_ZONE_SENSOR_0_1
}GESTURE_ZONE;

typedef enum
{
    GESTURE_STATE_UP_START,
    GESTURE_STATE_DOWN_START,
    GESTURE_STATE_INACTIVE
}GESTURE_STATE;

/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/


/*****************************************************************************
* Data Structure Definition
*****************************************************************************/


/*****************************************************************************
* Global Variable Declaration
*****************************************************************************/


/*****************************************************************************
* Function Prototypes
*****************************************************************************/
GESTURE DetectGesture(void);

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
	
	
#endif /* CAPSENSE_GESTURES_H */

/* [] END OF FILE */
