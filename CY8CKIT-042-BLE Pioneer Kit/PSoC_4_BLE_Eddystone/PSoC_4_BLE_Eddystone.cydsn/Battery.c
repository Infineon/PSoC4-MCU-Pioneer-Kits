/******************************************************************************
* Project Name      : PSoC_4_BLE_Eddystone
* File Name         : Battery.c
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

#include "main.h"
#include "Battery.h"
#include "WatchdogTimer.h"

/* Stores measured battery voltage in mV */
static uint16 batteryLevelmVolts;

/*******************************************************************************
* Function Name: MeasureBattery
********************************************************************************
*
* Summary:
*   This function measures the battery voltage and stores it.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MeasureBattery(void)
{
    int16   adcResult;
    uint32  sarControlReg;

    /* Wakeup ADC from sleep for measurements. */
    ADC_Wakeup();

    /* Set the reference to 1.024V and enable reference bypass */
    sarControlReg = ADC_SAR_CTRL_REG & ~ADC_VREF_MASK;
    ADC_SAR_CTRL_REG = sarControlReg | ADC_VREF_INTERNAL1024BYPASSED;

    /* 25 ms delay for reference capacitor to charge */
    LowPowerWait(REFERENCE_CAPACITOR_CHARGE_TIME);

    /* Set the reference to VDD and disable reference bypass */
    sarControlReg = ADC_SAR_CTRL_REG & ~ADC_VREF_MASK;
    ADC_SAR_CTRL_REG = sarControlReg | ADC_VREF_VDDA;

    /* Perform a measurement. Store this value in Vref. */
    LowPowerWait(VDD_REFERENCE_SWITCH_PERIOD);
    ADC_StartConvert();
    ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);

    /* Set the reference to 1.024V and enable reference bypass */
    sarControlReg = ADC_SAR_CTRL_REG & ~ADC_VREF_MASK;
    ADC_SAR_CTRL_REG = sarControlReg | ADC_VREF_INTERNAL1024BYPASSED;

    adcResult = ADC_GetResult16(ADC_BATTERY_CHANNEL);
    /* Calculate input voltage by using ratio of ADC counts from reference
    *  and ADC Full Scale counts. */
    batteryLevelmVolts = ((1024 * 2048) / adcResult) ;

    /* Put ADC to sleep after measurements are done. */
    ADC_Sleep();
}

/*******************************************************************************
* Function Name: GetMeasuredBatteryVoltage
********************************************************************************
*
* Summary:
*   This function returns currently measured voltage.
*
* Parameters:
*  None
*
* Return:
*  uint16: currently measured battery voltage
*
*******************************************************************************/
uint16 GetMeasuredBatteryVoltage(void)
{
    return batteryLevelmVolts;
}

/* [] END OF FILE */
