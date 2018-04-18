/******************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description: This code example demonstrates how to measure input voltage using
*              Sequencing SAR ADC component on PSoC 4100S device. The ADC output
*              is used to control the breathing rate of a RGB LED using
*              Smart IO component.
*
* Related Document: CE216873 ADC with Breathing LED.pdf
*
* Hardware Dependency: See code example CE216873
*
******************************************************************************
* Copyright (2016), Cypress Semiconductor Corporation.
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
*                           THEORY OF OPERATION
* This example demonstrates how to measure input voltage using Sequencing SAR ADC
* on PSoC 4100S device. It also shows how to use SmartIO component to generate 
* breathing LED effect that doesn't consume CPU bandwidth. 
* The ADC data is displayed using the RGB LED by controlling its breathing rate
* depending on the input voltage value.
*
* Hardware connection required for testing -
* ADC pin 	- P2_4 (connected to potentiometer on CY8CKIT-041-41XX)
* LED	 	- P2_6, P3_4, P3_6 (connected to RGB on CY8CKIT-041-41XX)
* I2C	    - P3_0, P3_1 (connected to KitProg2 on CY8CKIT-041-41XX)
*
********************************************************************************/

/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <project.h>
#include <math.h>

/*****************************************************************************
* MACRO Definitions
*****************************************************************************/  
/* LED related macros */
#define     OFF                 (1u)
#define     ON                  (0u)

/* Kit operating voltages */
/* At 1.9V the Green and Blue LEDs do not work as their forward voltage is 3V.
*  In this case, the ADC output can be viewed on Bridge Control Panel. */

/* ADC Channel */
#define ADC_CHANNEL0            (0u)

/* Full scale value of the ADC */
#define ADC_FULL_SCALE          (int)(pow(2u, ADC_SAR_Seq_MAX_RESOLUTION))

/* Range 1 is between 0v to 1/3 VDD */
#define ADC_RANGE1              (ADC_FULL_SCALE/3)

/* Range 2 is between 1/3 VDD to 2/3 VDD */
#define ADC_RANGE2              ((ADC_FULL_SCALE/3) << 1)

/* Range 3 is between 2/3 VDD to VDD */
#define ADC_RANGE3              (ADC_FULL_SCALE)

/* 0V to VDD is divided into 12 steps */
#define ADC_TOTAL_STEPS         (12u)

#define ADC_STEP_SIZE           (ADC_FULL_SCALE/ADC_TOTAL_STEPS)

/* EZI2C buffer size is 2 bytes to store ADC result */
#define BUFFER_SIZE             (2u)

/* The RAM area (00) where ADC result is stored is read only */
#define READ_ONLY_OFFSET        (0u)

/* Number of values in the period[] array */
#define PERIOD_NUM              (12u)

/* Clock input to all TCPWM components are 2 MHz, 
 * Two PWM signals are XORed to generate a varying duty cycle PWM signal that is used   
 * for LED breathing effect. 
 * The first PWM input to the XOR gate will have 2 MHz/20000 =  100 Hz average frequency 
 * The second PWM input to the XOR gate will have varying frequencies to generate
 * breathing effect. 
 * 2 MHz/19950 = 100.25 Hz 
 * 2 MHz/19800 = 101.01 Hz
 * 2 MHz/19230 = 104.00 Hz
 * 2 MHz/17857 = 112.00 Hz   */
/* Period values for PWM. First four values are for Red LED, next four for Green LED, last four for Blue LED */
const uint16 period[PERIOD_NUM] = { 19950u, 19800u, 19230u, 17857u, 
                                    19950u, 19800u, 19230u, 17857u, 
                                    19950u, 19800u, 19230u, 17857u };

/* Variable to store ADC data. I2C master reads this value for displaying on PC */
volatile uint8 i2cBuffer[BUFFER_SIZE] = {0u, 0u};

/*****************************************************************************
* Function Prototypes
*****************************************************************************/ 

/* API used for initializing PWM and Smart IO component */
void ledInit(void);

/* API used to update I2C buffer with ADC result */
void updateI2cBuffer(uint16);

/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary:
*  main() performs following functions:
*  1: Initialize the ADC, SmartIO and EZI2C components
*  2: Measures voltage across potentiometer using ADC 
*  3: Updates the PWM period and compare value based on ADC result and implements
*     breathing effect on corresponding LEDs
*  4: Sends data over I2C
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: ADC status is checked and if it is idle, then ADC scan is performed and
*         ADC value is read. LEDs are updated, and XORed PWM output is driven on 
*         LEDs in every loop in this function. I2C buffer is also updated every 
*         loop.
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
int main()
{
    /* Variable to store the period and compare value of PWM components */
    uint16 currentPeriod = 0u; 
    uint16 currentCompare = 0u; 
    
    /* Variable to store input voltage measured using CapSense_ADC */
    uint16  adcResult = 0u;
    
    /* Variable to store the offset of period[] array */
    uint8 pwmArrayIndex = 0u; 
    
    /* Enable Global interrupts for CapSense operation */
    CyGlobalIntEnable;
    
    /* Initialize and start Sequencing SAR ADC */
    ADC_SAR_Seq_Start();
    ADC_SAR_Seq_StartConvert();
    
    /* Setup buffer and start EZI2C slave (SCB mode) */
    EZI2C_EzI2CSetBuffer1(BUFFER_SIZE, READ_ONLY_OFFSET, i2cBuffer);
    EZI2C_Start();
    
    /* Initialize all PWM components and Smart IO component */
    ledInit();
    
    for(;;)
    {
        /* Check if the ADC data is ready */
        if(ADC_SAR_Seq_IsEndConversion(ADC_SAR_Seq_RETURN_STATUS))
        { 
            /* Read ADC measurement result */
            adcResult = ADC_SAR_Seq_GetResult16(ADC_CHANNEL0);
            
            /* Get the index for compare value of the PWM depending on the input voltage value */
            pwmArrayIndex = (adcResult/ADC_STEP_SIZE);
            
            /* Limit the array index values */
            if(pwmArrayIndex > (PERIOD_NUM-1))
            {
                pwmArrayIndex  = (PERIOD_NUM-1);
            }
            
            /* Get the period values from the array */
            currentPeriod = period[pwmArrayIndex];
            
            /* Compare value is half of period value */
            currentCompare = currentPeriod >> 1u;
            
            /* Depending on the range of ADC output, drive the respective LED. 
            *  Change the drive mode of the pin(LED) to strong if breathing effect has to be driven. 
            *  Change it to High-Z, if the LED should not be driven.
            */
            /* If input voltage is between 0V to 1/3 VDD drive RED LED */
            if(adcResult < ADC_RANGE1)
            {
                /* Configure period and compare value of PWM_4 */
                PWM_4_WritePeriod(currentPeriod);
                PWM_4_WriteCompare(currentCompare);
                
                /* Drive RED LED */
                RED_LED_SetDriveMode(RED_LED_DM_STRONG);
                GREEN_LED_SetDriveMode(GREEN_LED_DM_DIG_HIZ); 
                BLUE_LED_SetDriveMode(BLUE_LED_DM_DIG_HIZ);
            }
            /* If input voltage is between 1/3 VDD to 2/3 VDD drive Green LED */
            else if(adcResult < ADC_RANGE2)
            {
                /* Configure period and compare value of PWM_2 */
                PWM_2_WritePeriod(currentPeriod);
                PWM_2_WriteCompare(currentCompare);
                
                /* Drive Green LED */
                RED_LED_SetDriveMode(RED_LED_DM_DIG_HIZ);
                GREEN_LED_SetDriveMode(GREEN_LED_DM_STRONG);
                BLUE_LED_SetDriveMode(BLUE_LED_DM_DIG_HIZ);
            }
            /* If input voltage is between 2/3 VDD to VDD drive BLUE LED */
            else if(adcResult > ADC_RANGE2)
            {
                /* Configure period and compare value of PWM_4 */
                PWM_4_WritePeriod(currentPeriod);
                PWM_4_WriteCompare(currentCompare);
                
                /* Drive Blue LED */
                RED_LED_SetDriveMode(RED_LED_DM_DIG_HIZ);
                GREEN_LED_SetDriveMode(GREEN_LED_DM_DIG_HIZ);  
                BLUE_LED_SetDriveMode(BLUE_LED_DM_STRONG);
            }
        }
        
        /* Update I2C buffer with ADC result */
        updateI2cBuffer(adcResult);
    }
}
/*******************************************************************************
* Function Name: updateI2cBuffer
****************************************************************************//**
*  Summary:
*   This function updates the I2C buffer
*
* Parameters:
*  adcResult: ADC result in milli volts
*
* Return:
*  None
*
* Theory: The function checks if the I2C block is free. If yes, the I2C buffer is
*         updated.
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
void updateI2cBuffer(uint16 adcResult)
{
    /* Variable to store interrupt state */
    uint32 interruptState = 0u;
    
    /* Wait until I2C is not busy before updating the I2C buffer */
    /* Prevent new interrupts from occurring */
    interruptState = CyEnterCriticalSection();
    
    /* Check if I2C is busy. If I2C is free, update buffer */
    if(!(EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY))
    {
        /* Update the buffer with ADC result */
        i2cBuffer[0] = HI8(adcResult);
        i2cBuffer[1] = LO8(adcResult);
    }
    /* Exit critical section to continue slave operation */
    CyExitCriticalSection(interruptState);
}

/*******************************************************************************
* Function Name: ledInit
****************************************************************************//**
*  Summary:
*   This function initializes PWM and Smart IO block
*
* Parameters:
*  None
*
* Return:
*  None
*
* Theory: This function initializes PWM and Smart IO block
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
void ledInit(void)
{    
    /* Set drive mode to High-Z to cutoff PWM signal drive */
    RED_LED_SetDriveMode(RED_LED_DM_DIG_HIZ);
    BLUE_LED_SetDriveMode(RED_LED_DM_DIG_HIZ);
    GREEN_LED_SetDriveMode(RED_LED_DM_DIG_HIZ);
    
    /* Initializes the PWM with default customizer values and enables the PWM. */ 
    PWM_1_Start();
    PWM_2_Start();
    PWM_3_Start();
    PWM_4_Start();

    /* Smart IO component is initialized to specified values
    *  in the Configuration dialog and is then enabled 
    */
    XOR_Gate_1_Start();
    XOR_Gate_2_Start();    
}

/* [] END OF FILE */

