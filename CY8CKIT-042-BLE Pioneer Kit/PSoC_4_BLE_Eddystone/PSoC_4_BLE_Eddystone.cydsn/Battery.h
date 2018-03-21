/******************************************************************************
* Project Name      : PSoC_4_BLE_Eddystone
* File Name         : Battery.h
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

#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <project.h>
#include <stdbool.h>

/*******************************************************************************
*          Constants
*******************************************************************************/
/* 25 msec for reference cap to charge */
#define REFERENCE_CAPACITOR_CHARGE_TIME     ((uint32)(32.767 * 25))
/* 1 msec for VDD reference switch */
#define VDD_REFERENCE_SWITCH_PERIOD         ((uint32)(32.767 * 1))

#define ADC_VREF_MASK                       (0x000000F0Lu)
#define ADC_BATTERY_CHANNEL                 (0x00u)

/*******************************************************************************
*       Function Prototypes
*******************************************************************************/
void MeasureBattery(void);
uint16 GetMeasuredBatteryVoltage(void);

#endif  /*  __BATTERY_H__   */
/* [] END OF FILE */
