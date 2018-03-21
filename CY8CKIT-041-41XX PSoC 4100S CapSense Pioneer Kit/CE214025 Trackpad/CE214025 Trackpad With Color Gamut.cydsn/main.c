/******************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description:  This code example demonstrates how to use CapSense trackpad 
*               to input RGB color code for color mixing with PSoC 4 S-Series device.
*
* Related Document: CE214025 Trackpad with Color Gamut.pdf
*
* Hardware Dependency: See code example document CE214025 Trackpad with Color Gamut.pdf
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
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <project.h>

/* Include boolean function definition */
#include <stdbool.h>

/* Include color mixing API declarations */
#include "colormixing.h"

/* Include sprintf API definitions */
#include <stdio.h>

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/
    
/* Intensity control button status */    
#define BUTTON_ON                   (0x01u)
#define BUTTON_OFF                  (0x00u)

/* Multiplier value for color mixing to avoid floating point math */    
#define POS_MULT_100                (100u)    

/* Number of active sensors when palm is placed on trackpad */
#define ACTIVE_SENSORS_PALM          (4u)

/* Minimum and maximum intensity values. This value is divided by 10 and is multiplied
*  by MAX_COMPAREVALUE     
*/    
#define MAX_INTENSITY_LEVEL         (16u)

/* Default intensity value */    
#define DEFAULT_LED_INTENSITY       (MAX_COMPAREVALUE/2)

/*LED wil be driven for the specified number of loops after no touch*/
#define MAX_LOOPS_TO_DRIVE_LED      (75u)

/*Index for brightness control array*/
#define MAX_INTENSITY_INDEX         (3u)
#define MIN_INTENSITY_INDEX         (0u)

/*LED brightness macros*/
#define LED_BRIGHTNESS_12_5         (2u)
#define LED_BRIGHTNESS_25_0         (4u)
#define LED_BRIGHTNESS_37_5         (6u)
#define LED_BRIGHTNESS_100_0        (16u)

/*Offset correction macros*/
#define MIN_X_COORDINATE            (15u)
#define MAX_X_COORDINATE            (75u)
#define MIN_Y_COORDINATE            (8u)
#define MAX_Y_COORDINATE            (77u)
#define Y_COORDINATE_OFFSET         (7u)
#define Y_COORDINATE_NEED_OFFSET    (29u)
#define MAX_X_COORDINATE_FOR_COLOR_MIXING   (64u)

/*******************************************************************************
*   Module Variable and Constant Declarations with Applicable Initializations
*******************************************************************************/
    
/* Finite state machine states for device operation */
typedef enum
{
    SENSOR_SCAN = 0x01u,    /* Sensor is scanned in this state */
    WAIT_FOR_SCAN_COMPLETE = 0x02u, /* CPU is put to sleep in this state */
    PROCESS_DATA = 0x03u,   /* Sensor data is processed */
} DEVICE_STATE;

const uint8 brightnessLevels[] = {LED_BRIGHTNESS_12_5,LED_BRIGHTNESS_25_0
                                    ,LED_BRIGHTNESS_37_5 ,LED_BRIGHTNESS_100_0};

uint8 brightnessIndex = MAX_INTENSITY_INDEX;

/*******************************************************************************
*  Function Declarations
*******************************************************************************/

/* Function prototype for CapSense parameter initialization */
void capSenseInit(void);

/* Function prototype to scan CapSense sensors */
void capSenseProcess(void); 

/* Function prototype to initialize TCPWM components */
void prismInit(void);

/* Function that checks if trackpad or button sensors are ON */
bool anyWidgetActive(void);

/* Function that performs color mixing */
void colorMixingProcess(void);

/*******************************************************************************
*   Module Variable and Constant Declarations with Applicable Initializations
*******************************************************************************/

/* Contains current x, y coordinates of trackpad */
uint16 trackpadXPos = CapSense_SLIDER_POS_NONE;
uint16 trackpadYPos = CapSense_SLIDER_POS_NONE;    

/* Variable to store color mixing error status */
uint8 colorMixingError;

/* Variable to store interrupt state */
uint32 interruptState = 0u;

/* Variable to store the current trackpad XY coordinates. 
*  The XY coordinate is multiplied with 100 to avoid floating point math
*/
XY_COORDINATE currentXYCoordinate;

/* Variable to store last valid touch coordinates */
XY_COORDINATE lastValidXYCoordinate;

/* Variable to store RGB led dimming values */
uint16 rgbLEDDimmingValue[NUM_LEDS];

uint16 timeoutCounter;

/* Variable to store RGB LED coordinates on the color gamut 
*  The coordinates are multiplied by 10000 to avoid floating point math
*/
LED_COORDINATE rgbLEDXYPosition[NUM_LEDS];

/* Variable to store On/Off condition of trackpad */
uint8 trackpadStatus = 0;

/* Variable to control the maximum intensity of RGB LED 
*  Variable takes the following values 2, 4, 8 and 16
*  The resulting PWM compare value is multiplied with this value
*  and normalized to achieve duty cycle of 100%, 50%, 25% and 12.5%
*/
uint8 intensityCtrl = MAX_INTENSITY_LEVEL;

/* Variable to store the status of intensity control buttons */  
uint8 button0CurrState = BUTTON_OFF;
uint8 button0PrevState = BUTTON_OFF;
uint8 button1CurrState = BUTTON_OFF;
uint8 button1PrevState = BUTTON_OFF;

/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary: This function implements the state machine for device operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: 
*   main() performs following functions:
*  1: Initialize the CapSense, EZI2C and PWM Components
*  2: Scans trackpad, buttons and performs color mixing and brightness control using
*     RGB LED
* 
* Side Effects: None
*
* Note:None
*
*******************************************************************************/

int main()
{    
    /* Start the firmware state machine with sensor scan */
    DEVICE_STATE currentState = SENSOR_SCAN;
    
    /* Variable to hold active sensors count */
    uint16 sensorCount = 0u;
    
    /* Variable to check active sensor bits */
    uint16 mask = 0u;
    
    /* Temporary variable */
    uint16 tempVar = 0u;    
   
    /* Enable interrupts. This is required for CapSense and I2C operation */ 
    CyGlobalIntEnable; 
    
    /* Initialize I2C component for CapSense tuner */
    EZI2C_Start();
    
    /* Set up communication data buffer to CapSense data structure to 
    * expose to I2C master at primary slave address request
    */
    EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam),\
                         (uint8 *)&CapSense_dsRam);
    
    /* Start the CapSense component and autocalibrate IDAC values */
    capSenseInit();  
    
    /* Initialize PWM components */
    prismInit();
    
    /* Load XY coordinates of RGB LED on color gamut into local variables */
    initializeLedCoordinates(rgbLEDXYPosition);
    
    for(;;)
    {        
        switch(currentState)
        {
            case SENSOR_SCAN:
			{
                /* Initiate new scan only if the CapSense hardware is idle */
                if(CapSense_NOT_BUSY == CapSense_IsBusy())
                {
                    /* Update CapSense parameters set via CapSense tuner */
                    CapSense_RunTuner();      
                    
                    /*Scan trackpad and buttons*/
                    CapSense_ScanAllWidgets();
                    
                    /* Set the state machine to wait state until the scan is complete */
                    currentState = WAIT_FOR_SCAN_COMPLETE;   
                }
            	break;
			}
            
			case WAIT_FOR_SCAN_COMPLETE:
			{
                /* Device is in CPU Sleep until CapSense scanning is complete or
                *  device is woken-up by either CapSense interrupt or I2C interrupt 
                */
                /* Disable interrupts, so that ISR is not serviced while
                *  checking for CapSense scan status. Otherwise, interrupt might
                *  get serviced after checking for IsBusy condition and device
                *  might not wakeup since CapSense interrupt is already serviced
                */
                interruptState = CyEnterCriticalSection();
                            
                /* Check if CapSense scanning is complete */
                if(CapSense_NOT_BUSY != CapSense_IsBusy())
                {
                    /* If CapSense scannning is in progress, put CPU to sleep 
                    *  Device wakesup because of CapSense scan complete interrupt
                    */
                    CySysPmSleep();
                }
                /* If CapSense scanning is complete, process the CapSense data */
                else
                {
                    /* If current widget is trackpad or mode is slow scan mode, process the sensor data */
                    currentState = PROCESS_DATA;
                }
                /* Enable interrupts for servicing ISR */
                CyExitCriticalSection(interruptState);
            	break;
            }           
            
			case PROCESS_DATA:
            {            
                /* Set next state to Sensor Scan */
                currentState = SENSOR_SCAN;
                CapSense_ProcessAllWidgets();
                /* The below code resets trackpad baseline if a palm is detected on the trackpad */
                if(CapSense_IsWidgetActive(CapSense_TRACKPAD_WDGT_ID))
                {
                    /* Initialize mask variable */
                    mask = 1u;
                    
                    /* Set active sensor count to zero */
                    sensorCount = 0u;
                    
                    /* Loop through all the row and column sensors */
                    for(tempVar = CapSense_TRACKPAD_COL0_ID; tempVar <= CapSense_TRACKPAD_ROW6_ID; tempVar++)
                    {
                        /* Check each bit for active sensor condition */
                        if(CapSense_SNS_STATUS0_VALUE & mask)
                        {
                            /* Increment sensor count for each active sensor */
                           sensorCount++;
                        }
                        /* If all the column sensors are searched and active sensorCount is not greater than threshold
                        *  reset the sensorCount variable to detect active row sensors
                        */
                        if((tempVar == CapSense_TRACKPAD_COL6_ID) && (sensorCount <= ACTIVE_SENSORS_PALM))
                        {
                            sensorCount = 0u;
                        }
                        
                        /* If active sensor count in either a row or column has exceed the threshold
                        *  reset all the trackpad sensor baselines
                        */
                        if(sensorCount > ACTIVE_SENSORS_PALM)
                        {
                            CapSense_InitializeWidgetBaseline(CapSense_TRACKPAD_WDGT_ID);
                            break;
                        }
                        /* Update the mask variable until all the bits are scanned for active status */
                        mask = mask << 1u;
                    }                           
                }
                /* Check if IntensityUp button is active */
                if(CapSense_IsWidgetActive(CapSense_INTENSITYUP_WDGT_ID))
                    button0CurrState = BUTTON_ON;
                else 
                    button0CurrState = BUTTON_OFF;

                /* Check if IntensityDown button is active */
                if(CapSense_IsWidgetActive(CapSense_INTENSITYDOWN_WDGT_ID))
                    button1CurrState = BUTTON_ON;
                else
                    button1CurrState = BUTTON_OFF;

                /* Check for rising edge of IntensityUp button status  and calculate the LED
                    brightness control array index*/
                if((button0CurrState == BUTTON_ON) && (button0PrevState == BUTTON_OFF))
                {                    
                    if(brightnessIndex >= MAX_INTENSITY_INDEX)
                    {
                        brightnessIndex = MAX_INTENSITY_INDEX; 
                    }
                    else
                    {
                        brightnessIndex++;
                    }
                }
                /* Check for rising edge of IntensityDown button status  and calculate the LED
                    brightness control array index*/
                else if((button1CurrState == BUTTON_ON) && (button1PrevState == BUTTON_OFF))
                {
                    if(brightnessIndex <= MIN_INTENSITY_INDEX)
                    {
                        brightnessIndex = MIN_INTENSITY_INDEX;
                    }
                    else
                    {
                        brightnessIndex--;
                    }                    
                }
                
                intensityCtrl = brightnessLevels[brightnessIndex];

                /* Initialize previous button state to current button state */
                button0PrevState = button0CurrState;
                button1PrevState = button1CurrState;                
               
                /* If trackpad or button sensor is not active, increment the LED timeout counter */
               if(!anyWidgetActive())
                 {
                    timeoutCounter++;

                    /* Check if sensor is inactive for a duration greater than MAX_LOOPS_TO_DRIVE_LED */
                    if(timeoutCounter >= MAX_LOOPS_TO_DRIVE_LED)
                    {                        
                        /* Set LED pin drive mode to high-z to stop driving LEDs */
                        Red_LED_SetDriveMode(Red_LED_DM_ALG_HIZ);
                        Green_LED_SetDriveMode(Green_LED_DM_ALG_HIZ);
                        Blue_LED_SetDriveMode(Blue_LED_DM_ALG_HIZ);                        
                    }
                }  
                /* If either trackpad or button sensors are active, perform color mixing */
                else
                {                    
                    /* Because sensor is active, reset the counter */
                    timeoutCounter = 0;

                    /* If widget is active, perform color mixing */
                    colorMixingProcess();
                }                               
            	break;
            }
               
            default:
			{
            /*******************************************************************
             * Unknown state. Unexpected situation.
             ******************************************************************/
            	CYASSERT(0);
            	break;
			}
        }
    }
}

/******************************************************************************
* Function Name: capSenseInit
*******************************************************************************
*
* Summary: This API initializes CapSense block
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*   capSenseInit() performs following functions:
*  1: Starts the CapSense block
*  2: Scan the trackpad widget and initialize the previous touch coordinate values
* 
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
void capSenseInit(void)
{
    /* Variable to store the XY coordinates */
    uint32 tempCoordinates;
    
    /* Initialize CapSense block */
    CapSense_Start();
    
    CapSense_InitializeWidgetBaseline(CapSense_INTENSITYUP_WDGT_ID);
    CapSense_InitializeWidgetBaseline(CapSense_INTENSITYDOWN_WDGT_ID);
    
    /* Scan the trackpad widget and initialize the previous touch coordinate values*/
    trackpadStatus =  CapSense_IsWidgetActive(CapSense_TRACKPAD_WDGT_ID);
    
    /* Get XY Coordinates */
    tempCoordinates = CapSense_GetXYCoordinates(CapSense_TRACKPAD_WDGT_ID);
    
    /* Load XY position to variable. Note: the XY position is interchanged as the 
    *  columns and rows in the PCB layout is inverse of X, Y coordinate of color gamut 
    */
    trackpadYPos = LO16(tempCoordinates);
    trackpadXPos = HI16 (tempCoordinates);
    
    /* Initialize last valid coordinates so that RGB LED glows
    *  when button is touched before trackpad after device power-up or reset.
    */
    lastValidXYCoordinate.currentX = RED_BASE_COLOR_X;
    lastValidXYCoordinate.currentY = RED_BASE_COLOR_Y;
    
    /* Set default led intensity value */
    currentXYCoordinate.ledIntensity = DEFAULT_LED_INTENSITY;
}

/******************************************************************************
* Function Name: prismInit
*******************************************************************************
*
* Summary: This API initializes PWM components
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  prismInit() performs following functions:
*  1: Starts the TCPWM block
*  2: Initializes the TCPWM compare value to minimum value
* 
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
void prismInit(void)
{
    /* Start TCPWM Blocks */
    PrISM_Red_Start();
    PrISM_Green_Start();
    PrISM_Blue_Start();
    
    /* Set TCPWM compare value to zero initially */
    PrISM_Red_WriteCompare(MIN_COMPAREVALUE);
    PrISM_Green_WriteCompare(MIN_COMPAREVALUE);
    PrISM_Blue_WriteCompare(MIN_COMPAREVALUE);

}
/*******************************************************************************
* Function Name: anyWidgetActive
********************************************************************************
*
* Summary:
*  This API checks if any widget is active
*
* Parameters:
*  None.
*
* Theory: This API checks if any of the IntensityUp, IntensityDown or trackpad sensors
*         are active and returns true if any one of them is active.
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/

bool anyWidgetActive(void)
{
    /* Check if either trackpad or any of the two button sensors are active */
    if(CapSense_IsWidgetActive(CapSense_TRACKPAD_WDGT_ID) || \
       CapSense_IsWidgetActive(CapSense_INTENSITYUP_WDGT_ID) || \
       CapSense_IsWidgetActive(CapSense_INTENSITYDOWN_WDGT_ID))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/*******************************************************************************
* Function Name: colorMixingProcess
********************************************************************************
*
* Summary:
*  This API checks if any widget is active and computes the TCPWM compare value
*
* Parameters:
*  None.
*
* Theory: This API checks if any of the IntensityUp, IntensityDown or trackpad sensors
*         are active. Based on the status, the color mixing algorithm is executed to
*         compute the compare
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
void colorMixingProcess(void)
{
    /* Temporary variable */
    uint32 tempVar;
    
    /* Variable to store XY coordinates of trackpad */
    uint32 tempCoordinates;

    /* Obtain the maximum compare value depending on the intensity level set by user */
    tempVar = (uint32) (intensityCtrl * MAX_COMPAREVALUE);
    tempVar /= MAX_INTENSITY_LEVEL;
    currentXYCoordinate.ledIntensity = (uint16)tempVar;
    lastValidXYCoordinate.ledIntensity = currentXYCoordinate.ledIntensity;    
    
    /* Check if trackpad is active */
    trackpadStatus =  CapSense_IsWidgetActive(CapSense_TRACKPAD_WDGT_ID);
    
    /* Get trackpad XY coordinates */
    tempCoordinates = CapSense_GetXYCoordinates(CapSense_TRACKPAD_WDGT_ID);
    
    /* XY position values and store in local variable 
    *  Note: The XY position is interchanged because the rows and columns are 
    *  interchanged in the kit
    */
    trackpadYPos = LO16(tempCoordinates);
    trackpadXPos = HI16(tempCoordinates);
    
    /* If trackpad is active, load the XY position for computing dimming values */
    if(trackpadStatus)
    {
        /* Normalization is done because row0 to row6 and column0 to column 6 
        *  are inverted in the layout       
        */
        trackpadXPos = CapSense_TRACKPAD_Y_RESOLUTION - trackpadXPos;
        trackpadYPos = CapSense_TRACKPAD_X_RESOLUTION - trackpadYPos;
        
        /*Offset correction to get smooth response near color gamut edges*/
        if(trackpadXPos <= MIN_X_COORDINATE)
        {
            trackpadXPos = MIN_X_COORDINATE;
        }        
        else if (trackpadXPos >= MAX_X_COORDINATE_FOR_COLOR_MIXING && trackpadXPos <= MAX_X_COORDINATE)
        {
            trackpadXPos = MAX_X_COORDINATE_FOR_COLOR_MIXING;
        }        
        
        if(trackpadYPos <= MIN_Y_COORDINATE)
        {
            trackpadYPos = MIN_Y_COORDINATE;
        }
        else if (trackpadYPos <= Y_COORDINATE_NEED_OFFSET)
        {
            trackpadYPos = trackpadYPos + Y_COORDINATE_OFFSET;
        }
        else if(trackpadYPos >= MAX_Y_COORDINATE)
        {
            trackpadYPos = MAX_Y_COORDINATE;
        }
        
    }
    else
    {
        trackpadXPos = 0u;
        trackpadYPos = 0u;
    }
    
    /* Multiply the coordinate value by 100 to avoid floating point math */
    currentXYCoordinate.currentX = (int16)(trackpadXPos * POS_MULT_100);
    currentXYCoordinate.currentY = (int16)(trackpadYPos * POS_MULT_100);
    
    /* If finger is on trackpad, use current touch coordinates to compute dimming values */
    if(trackpadStatus)
    {
        /* Compute RGB LED dimming values for a given XY coordinate */
        colorMixingError = rgbColorMix(currentXYCoordinate, rgbLEDXYPosition, rgbLEDDimmingValue);
        
        /* If color mixing was successful, save the current coordinates */
        if(colorMixingError != INVALID_COLOR)
        {
            lastValidXYCoordinate.currentX = currentXYCoordinate.currentX;
            lastValidXYCoordinate.currentY = currentXYCoordinate.currentY;
        }
    }
    /* If IntensityUp or IntensityDown button is pressed, compute dimming value using previous saved touch coordinates */
    else
    {
        colorMixingError = rgbColorMix(lastValidXYCoordinate, rgbLEDXYPosition, rgbLEDDimmingValue);
    }

    /* Update the LED dimming value only when color mixing process is successful */
    if(colorMixingError != INVALID_COLOR)
    {
        /* Set LED pin drive mode to strong to save power */
        Red_LED_SetDriveMode(Red_LED_DM_STRONG);
        Green_LED_SetDriveMode(Green_LED_DM_STRONG);
        Blue_LED_SetDriveMode(Blue_LED_DM_STRONG);
                        
        /* Load the computed dimming value to the PrISM component */
        PrISM_Red_WriteCompare(rgbLEDDimmingValue[LED_RED]);
        PrISM_Green_WriteCompare(rgbLEDDimmingValue[LED_GREEN]);
        PrISM_Blue_WriteCompare(rgbLEDDimmingValue[LED_BLUE]); 
    }
}

/* [] END OF FILE */

