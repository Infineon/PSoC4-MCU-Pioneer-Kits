/******************************************************************************
* Project Name      : PSoC_4_BLE_Eddystone
* File Name         : main.h
* Version           : 1.0
* Device Used       : CY8C4247LQI-BL483
* Software Used     : PSoC Creator 4.1
* Compiler          : ARM GCC 5.4.1, ARM MDK Generic
* Related Hardware  : CY8CKIT-042-BLE Bluetooth Low Energy Pioneer Kit
* Owner             : DEJO
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

#ifndef __MAIN_H__
#define __MAIN_H__

#include <project.h>
#include <stdbool.h>

/*******************************************************************************
* Macros and Constants
*******************************************************************************/
#define YES                             (1)
#define NO                              (0)

#define LED_ON                          (0)
#define LED_OFF                         (1)

#define TEMPERATURE_SENSOR_ENABLE       (YES)
#define BATTERY_MEASUREMENT_ENABLE      (YES)

#define WCO_STARTUP_PERIOD              ((uint32)(32767 / 2))  /* 500 msec */
#define ECO_STARTUP_PERIOD              ((uint32)(32 * 3.5))   /* 3.5 msec */

/******************************************************************************
* Function prototype
*******************************************************************************/
void LowPower(void);
void SetAdvPacketCount(uint32 value);
uint32 GetAdvPacketCount(void);
bool IsConnAdvStart(void);

#endif  /* __MAIN_H__   */
/* [] END OF FILE */
