/******************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description: This code example demonstrates how to implement a low-power 
*              CapSense button with an average current consumption of 5uA
*              per button. 
*
* Related Document: CE214022 LP CapSense Buttons.pdf
*
* Hardware Dependency: See code example CE214022
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
* Theory of Operation: This example code demonstrates low power CapSense button 
* functionality. It will help user to learn how to set parameters in their design 
* for low power CapSense button application. Left button controls the RGB LED color 
* in the order R->G->B->R. Right button controls the brightness of the RGB LED
* in the order Min->Mid->Max. 

* Note: The time of LOOP_TIME_FASTSCANMODE and LOOP_TIME_SLOWSCANMODE are dependent   
* on ILO accuracy. With ILO, the accuracy of the time can vary as much as 60 percent.  
* ILO trim method is used to improve the ILO accuracy to 10 percent.
*******************************************************************************/

/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"

/*****************************************************************************
* MACRO Definitions
*****************************************************************************/   

/* Boolean constants */
#define TRUE						    (1u)
#define FALSE					        (0u)
#define ON						        (TRUE)
#define OFF							    (FALSE)
#define SET							    (ON)
#define CLEAR						    (OFF)

/* General Constants */    
#define ZERO					        (0u)

/* Macros to find rising and falling edge on the data passed */
#define RISING_EDGE_DET(curr, prev)		((curr ^ prev) & curr)
#define FALLING_EDGE_DET(curr, prev)	((curr ^ prev) & prev)

/* Button mask registers */
#define LFT_BUTTON_MASK					(0x01 << CapSense_BUTTON_SNS0_ID)
#define RGT_BUTTON_MASK					(0x01 << CapSense_BUTTON_SNS1_ID)

/* Device power mode related macros */
#define LOOP_TIME_FASTSCANMODE          (20u)
#define LOOP_TIME_SLOWSCANMODE          (100u)
#define MILLI_SEC_TO_MICRO_SEC          (1000u)
#define ILO_CLOCK_FACTOR                (40u)
    
/* Refresh rate control parameters */    
#define WDT_TIMEOUT_FAST_SCAN           (ILO_CLOCK_FACTOR * LOOP_TIME_FASTSCANMODE)   
#define WDT_TIMEOUT_SLOW_SCAN           (ILO_CLOCK_FACTOR * LOOP_TIME_SLOWSCANMODE)

/* Interrupt number for watchdog */    
#define WDT_IRQ_NUMBER                  (6u)
    
/* Reset value of softCounter */    
#define RESET					        (0u)

/* This timeout is for changing the refresh interval from fast to slow rate
*  The timeout value is WDT_TIMEOUT_FAST_SCAN * SCANMODE_TIMEOUT_VALUE
*/
#define SCANMODE_TIMEOUT_VALUE          (150u)

/* Firmware implements two refresh rates for reducing average power consumption */
typedef enum
{
    SLOW_SCAN_MODE = 0u,
    FAST_SCAN_MODE = 1u
} SCAN_MODE;

/* RGB LED color selection */
typedef enum
{
    COLOR_RED = 1u,
    COLOR_GREEN = 2u,
    COLOR_BLUE = 3u
}LED_RGB;

/* Finite state machine states for device operating states */
typedef enum
{
    SENSOR_SCAN = 0x01u, /* Sensor is scanned in this state */
    WAIT_FOR_SCAN_COMPLETE = 0x02u, /* CPU is put to sleep in this state */
    PROCESS_DATA = 0x03u, /* Sensor data is processed */
    SLEEP = 0x04u /* Device is put to deep sleep */
} DEVICE_STATE;

/* Three levels of brightness is used for RGB LED */
typedef enum 
{
    BRIGHTNESS_MIN = 0u,
    BRIGHTNESS_MID = 1u,
    BRIGHTNESS_MAX = 2u        
}BRIGHTNESS;

/* Definition that combines all possible processing tasks other than applying filters */
#define CapSense_PROCESS_CUSTOM    ( CapSense_PROCESS_BASELINE      | \
                                         CapSense_PROCESS_DIFFCOUNTS    | \
                                         CapSense_PROCESS_CALC_NOISE    | \
                                         CapSense_PROCESS_THRESHOLDS    | \
                                         CapSense_PROCESS_DECONVOLUTION | \
                                         CapSense_PROCESS_STATUS )
/*******************************************************************************
*   Module Variable and Constant Declarations with Applicable Initializations
*******************************************************************************/

/* Variable to store interrupt state */
uint32 interruptState = 0u;

/* Variable to check the WDT interrupt state */
volatile uint8 wdtInterruptOccured = FALSE;

/* Contains watchdog match value to generate periodic interrupt */
volatile uint32 watchdogMatchValue = WDT_TIMEOUT_FAST_SCAN;

/* Compare value for different LEDs */
/* Since the brightness of each LED is different for a given current rating
*  the duty cycle of the LED is varied for each LED to ensure same brightness 
*  is obtained when driven with a PRS PWM signal.
*/
const uint16 ledBrightness_R[] = {6553u, 16384u, 65535u};
const uint16 ledBrightness_G[] = {6553u, 16384u, 65535u};
const uint16 ledBrightness_B[] = {17000u, 23000u, 65535u};

/*****************************************************************************
* Function Prototypes
*****************************************************************************/ 
/* API to control the RGB LED color and brightness level */
void LED_Control(uint32 buttonStatus);

/* API to get WDT match value to generate precise scan intervals */
void CalibrateWdtMatchValue(void);

/* API to prepare the device for deep sleep */
void EnterDeepSleepLowPowerMode(void);

/* API to configure the WDT timer for controlling scan intervals */
void WDT_Start(void);

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
*  main() performs following functions:
*  1: Initialize the CapSense, I2C and TCPWM Components
*  2: Configures watchdog interrupt and the corresponding ISR
*  3: There are two modes of operation
*       Slow scan mode : Both the button sensors(left button and right button) are ganged and scanned at LOOP_TIME_SLOWSCANMODE 
*                        refresh interval. LEDs are turned off in this state. Upon touch detection, device
*                        switches to fast scan mode.
*       Fast scan mode : Both the button sensors are scanned individually at LOOP_TIME_FASTSCANMODE refresh interval.
*                        When left button (CapSense_BUTTON_SNS0) is pressed, LED color is switched between red, green and blue.
*                        When right button (CapSense_BUTTON_SNS1) is pressed, RGB LED brightness is varied between 3 levels.
*       By default, device is in fast scan mode. In fast scan mode, if no CapSense widgets are active
*       for SCANMODE_TIMEOUT_VALUE, the operating mode is changed to slow scan mode to reduce 
*       device power consumption. If touch is detected in slow scan mode, the device switches
*       to fast scan mode.
* 
* Side Effects: None
*
* Note:None
*
*******************************************************************************/
int main()
{	
    /* Variable to hold current button status */
	uint32 buttonStatus = OFF;		

    /* Variable to hold the current device state 
    *  State machine starts with sensor scan state after power-up
    */
    DEVICE_STATE currentState = SENSOR_SCAN; 
	
    /* This variable is used to indicate the current power mode */
    SCAN_MODE deviceScanMode = FAST_SCAN_MODE;
    
    /* This variable is used to implement a software counter. If the value 
    *  of this counter is greater than SCANMODE_TIMEOUT_VALUE, it indicates that the button sensor 
    *  was inactive for more than 3s. 
    */
    uint16 softCounter = RESET;

    /* Enable global interrupts. */ 
    CyGlobalIntEnable;    
    
    /* Initialize I2C component for CapSense tuner */
    EZI2C_Start();
    
    /* Set up communication data buffer to CapSense data structure to 
    *  expose to I2C master at primary slave address request		
    */
    EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam),\
                         (uint8 *)&CapSense_dsRam);
    
    /* Initialize the CapSense CSD block */
	CapSense_Start();
    
    /* Configure CapSense parameters for button widget to scan them individually */
    CapSense_SetupWidget(CapSense_BUTTON_WDGT_ID);
                        
    /* Start and initialize the TCPWM block for RGB LED drive */
	PrISM_Red_Start();
	PrISM_Green_Start();
	PrISM_Blue_Start();
                            
    /* Watchdog is used to control the loop time in this project and watchdog
    *  is set to generate interrupt at every LOOP_TIME_FASTSCANMODE in fast scan mode  
    *  and at LOOP_TIME_SLOWSCANMODE in slow scan mode
    */
    WDT_Start();

    for(;;)
    {  
        /* Switch between sensor-scan -> wait-for-scan -> process -> sleep states */
        switch(currentState)
        {
            case SENSOR_SCAN:                 
                /* Initiate new scan only if the CapSense hardware is idle */
                if(CapSense_NOT_BUSY == CapSense_IsBusy())
                {   
                    /* Update CapSense parameters set via CapSense tuner before the 
                    *  beginning of CapSense scan. This check if required if we are using 
                    *  CapSense Extension (Ext) APIs in the project.
                    */
                    if(CapSense_STATUS_RESTART_DONE == CapSense_RunTuner())
                    {      
                        if(deviceScanMode == FAST_SCAN_MODE)
                        {
                            /* Configure button widget after parameter is updated via tuner */
                            CapSense_CSDSetupWidget(CapSense_BUTTON_WDGT_ID);
                        }
                        else
                        {
                            /* Configure ganged widget after parameter is updated via tuner */
                            CapSense_CSDSetupWidgetExt(CapSense_GANGEDBUTTON_WDGT_ID, CapSense_GANGEDBUTTON_SNS0_ID);
                        }
                    }
                    /* In fast scan mode, left button and right button is scanned individually */
                    if(deviceScanMode == FAST_SCAN_MODE)
                    {
                        /* Scan the sensors which are in the button widget i.e left button and right button */
                        CapSense_CSDScan();
                    }
                    /* In slow scan mode, only ganged sensor is scanned. Since we have only one ganged sensor,
                    *  the sensor is always connected to AMUXBUS in this mode.
                    *  This avoids time taken to connect the sensor to AMUXBUS every time the ganged sensor is scanned. 
                    */
                    else
                    {
                        CapSense_CSDScanExt();
                    } 
                    /* Put CPU to sleep while sensor scanning is in progress */
                    currentState = WAIT_FOR_SCAN_COMPLETE;                    
                }
            break;

            case WAIT_FOR_SCAN_COMPLETE:
                /* Device is in CPU Sleep until CapSense scanning is complete or
                *  device is woken-up by either CapSense interrupt or I2C interrupt 
                */
                /* Disable interrupts, so that ISR is not serviced while
                *  checking for CapSense scan status.
                */
                interruptState = CyEnterCriticalSection();
                            
                /* Check if CapSense scanning is complete */
                if(CapSense_NOT_BUSY != CapSense_IsBusy())
                {
                    /* If CapSense scannning is in progress, put CPU to sleep */
                    CySysPmSleep();
                }
                /* If CapSense scanning is complete, process the CapSense data */
                else
                {
                    currentState = PROCESS_DATA;
                }
                /* Enable interrupts for servicing ISR */
                CyExitCriticalSection(interruptState);
            break;
        
            case PROCESS_DATA:
                /* Set next state to SLEEP */
                currentState = SLEEP;
                
                /* Clear the variable that is used for both the button status */
                buttonStatus = CLEAR;
                
                if(deviceScanMode == FAST_SCAN_MODE)
                {
                    /* Update baseline and sensor status for both the button sensors */
                    CapSense_ProcessWidgetExt(CapSense_BUTTON_WDGT_ID, CapSense_PROCESS_CUSTOM);
                    
                    /* If button widget is reported active, process the 
                    *  button active status 
                    */
                    if(CapSense_IsWidgetActive(CapSense_BUTTON_WDGT_ID))
        			{	
                        /* Set left button state */
                        if(CapSense_IsSensorActive(CapSense_BUTTON_WDGT_ID, CapSense_BUTTON_SNS0_ID))
                        {
                            buttonStatus |= LFT_BUTTON_MASK;
                        }
                        /* Set right button state */
                        if(CapSense_IsSensorActive(CapSense_BUTTON_WDGT_ID, CapSense_BUTTON_SNS1_ID))
                        {
                            buttonStatus |= RGT_BUTTON_MASK;
                        }
                        /* Control RGB LED based on button status */
                        LED_Control(buttonStatus);
                        
                        /* Reset the software counter if any button is active. */
        			    softCounter = RESET;
        			}
                    /* If none of the buttons are active, execute below code */
        			else
        			{
                        /* Increment the software counter every LOOP_TIME_FASTSCANMODE if button 
                        *  touch is not detected. 
                        */
        			    softCounter++;  
                        
                        /* Update previous button status which is saved in LED_Control API */
                        LED_Control(buttonStatus);
                        
        			    /* If finger is not on sensor for SCANMODE_TIMEOUT_VALUE, switch off the 
                        *  LEDs and switch mode to slow scan mode to reduce power consumption 
                        */
                        if(softCounter >= SCANMODE_TIMEOUT_VALUE)
                        {   
                            /* Watchdog is configured to generate interrupt at LOOP_TIME_SLOWSCANMODE */
        			        watchdogMatchValue = WDT_TIMEOUT_SLOW_SCAN;
                            
                            /* Set mode to slow scan mode to scan sensors at LOOP_TIME_SLOWSCANMODE */
                            deviceScanMode = SLOW_SCAN_MODE;
                            
                            /* Configure ganged sensor and connect it to AMUXBUS */
                            CapSense_CSDSetupWidgetExt(CapSense_GANGEDBUTTON_WDGT_ID, CapSense_GANGEDBUTTON_SNS0_ID);

                            /* Set LED pin drive mode to high-z to save power */
                            Pin_RedLED_SetDriveMode(Pin_RedLED_DM_ALG_HIZ);
                            Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_ALG_HIZ);
                            Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_ALG_HIZ);
    
                            /* TCPWM component is put to sleep in slow scan mode to save power */
                            PrISM_Red_Sleep();
                            PrISM_Green_Sleep();
                            PrISM_Blue_Sleep();
                        }  
                    }
                }   
                /* If deviceScanMode is SLOW_SCAN_MODE, perform the following tasks */    
                else
                {   
                    /* Update baselines of ganged button widget */
                    CapSense_ProcessWidget(CapSense_GANGEDBUTTON_WDGT_ID);
                    
                    /* If ganged widget is active in slow scan mode move to 
                    *  fast scan mode where sensors are scanned individually at LOOP_TIME_FASTSCANMODE
                    */
                    if(CapSense_IsWidgetActive(CapSense_GANGEDBUTTON_WDGT_ID))
                    {
                        /* If sensor is active in slow-scan mode, skip sleep
                        *  and perform sensor scan
                        */
                        currentState = SENSOR_SCAN;
                        
                        /* Set watchdog match value to fast scan mode */
    			        watchdogMatchValue = WDT_TIMEOUT_FAST_SCAN;      
                        
                        /* Change the device mode to fast scan mode to provide fast touch response */
                        deviceScanMode = FAST_SCAN_MODE; 
                        
                        /* Configure button widgets parameters for scanning in fast scan mode */
                        CapSense_SetupWidget(CapSense_BUTTON_WDGT_ID);
                        
                        /* Turn-off LEDs initially. LEDs will be driven based on the sensor status in fast scan mode */
                        PrISM_Red_WriteCompare(OFF);
                        PrISM_Green_WriteCompare(OFF);
                        PrISM_Blue_WriteCompare(OFF);    
                        
                        /* Set pin drive mode to strong drive for driving RGB LED */
                        Pin_RedLED_SetDriveMode(Pin_RedLED_DM_STRONG);
                        Pin_GreenLED_SetDriveMode(Pin_GreenLED_DM_STRONG);
                        Pin_BlueLED_SetDriveMode(Pin_BlueLED_DM_STRONG);
    
                        /* Turn-on PWM block to drive RGB LED */
                        PrISM_Red_Wakeup();
                        PrISM_Green_Wakeup();
                        PrISM_Blue_Wakeup();
                    }
                }
            break;
            
            case SLEEP:             
                /* In fast scan mode, put the device to only CPU sleep as TCPMW block is ON
                *  to drive RGB LEDs and it cannot work in deep-sleep mode
                */
                if(deviceScanMode == FAST_SCAN_MODE)
                {   
                    CySysPmSleep();
                }
                /* In slow scan mode, device can be put to deep-sleep as TCPWM is off */
                else
                {
                    EnterDeepSleepLowPowerMode();
                }
                /* Start scanning the sensors only if interrupt occured due to WDT.
                   Interrupt can also occur due to I2C interrupt while tuner is running.
                   In such cases, sensor is not scanned until WDT interrupt has occured
                */
                if(wdtInterruptOccured)
                {
                    /* Set state to scan sensor after device wakes up from sleep */
                    currentState = SENSOR_SCAN;
                    
                    wdtInterruptOccured = FALSE;  
                    
                    /* Calibrate the ILO to ensure accurate scan intervals */
                    CalibrateWdtMatchValue();
                }
            break;
            
            default:
                /*******************************************************************
                 * Unknown power mode state. Unexpected situation.
                 ******************************************************************/
                CYASSERT(0);
            break;
        } 
    }
    
}

/*******************************************************************************
* Function Name: LED_Control
********************************************************************************
* Summary:
* This API turns-on red, green or blue LED depending on the left button status.
* It also control the brightness of the LED depending on the right button status.
*
* Parameters:
* 	buttonStatus: Variable indicates the button status of left-button and right-button
*
* Return:
*  void
*
* Theory: LED color is changed on the rising edge of left-button ON status, LED brightness 
*         is changed on the rising edge of right-button ON status. If there is no rising 
*         edge detected, and the buttons are still touched or if the sensors are off, the 
*         previous state is maintained.
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/

void LED_Control(uint32 buttonStatus)
{
    /* Flag to initialise the LED color and intensity after */
    static uint8 initialiseFlag = FALSE;
    
	/* Variable to store previous button status */
	static uint32 prevButtonStatus = OFF;		
			
	/* Variable for led color and brightness control */
	static LED_RGB ledColor;
	static BRIGHTNESS ledBrightnessLevel;
    
    /* If any button is active, then process LED control activities */
	if(buttonStatus != OFF)
	{
        /* If button is pressed for first time, set default LED color and brightness level */
        if(initialiseFlag == FALSE)
        {
            ledColor = COLOR_RED;
            ledBrightnessLevel = BRIGHTNESS_MIN;  
            
            /* Reset the flag after initialization is done */
            initialiseFlag = TRUE;
        }
        else
        {
    		/* Detect the rising edge on buttonStatus */
    		switch(RISING_EDGE_DET(buttonStatus, prevButtonStatus))
    		{
    			/* Upon left button activation, change LED color from Red->Green->Blue->Red */
    			case (LFT_BUTTON_MASK):
                {
                    /* Switch to next color */
    				ledColor++;
                    
                    /* If color # is greater than blue, reset to red */
    				if(ledColor > COLOR_BLUE)
    				{
    					ledColor = COLOR_RED;
    				}
                }
    			break;
                                                                            
    			/* Right button activation, change LED brightness from min->mid->max->min */
    			case (RGT_BUTTON_MASK):
                {
                    switch(ledBrightnessLevel)
                    {
                        case BRIGHTNESS_MIN:                                    
                            ledBrightnessLevel = BRIGHTNESS_MID;
                            break;
                        case BRIGHTNESS_MID:                                    
                            ledBrightnessLevel = BRIGHTNESS_MAX;
                            break;
                        case BRIGHTNESS_MAX:                                    
                            ledBrightnessLevel = BRIGHTNESS_MIN;
                            break;
                        default:
                        	ledBrightnessLevel = BRIGHTNESS_MAX;
                            break;
                    } 
                }    
    			break;
    									
    			default:
                    break;
    		}
    		/* Depending on the color of the LED selected turn on that PWM alone 
    		*  Brightness of the PWM = ledBrightness
            */
            switch(ledColor)
            {
                case COLOR_RED:
                {
                    PrISM_Red_WriteCompare((uint32)ledBrightness_R[ledBrightnessLevel]);
    			    PrISM_Green_WriteCompare(OFF);
    			    PrISM_Blue_WriteCompare(OFF);
                }
                break;
                case COLOR_GREEN:
                {
                    PrISM_Red_WriteCompare(OFF);
    			    PrISM_Green_WriteCompare((uint32)ledBrightness_G[ledBrightnessLevel]);
    			    PrISM_Blue_WriteCompare(OFF);    
                }
                break;
                case COLOR_BLUE:
                {
                    PrISM_Red_WriteCompare(OFF);
    			    PrISM_Green_WriteCompare(OFF);					
    			    PrISM_Blue_WriteCompare((uint32)ledBrightness_B[ledBrightnessLevel]);    
                }
                break;
                default:
                {
                    PrISM_Red_WriteCompare(OFF);
        	        PrISM_Green_WriteCompare(OFF);
        	        PrISM_Blue_WriteCompare(OFF);    
                }
                break;
            }
        }
    }
    /* Store the current button status to previous status */
	prevButtonStatus = buttonStatus;

}

/******************************************************************************
* Function Name: Timer_Interrupt
*******************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the WDT timer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: The interrupt is cleared on the ISR as watchdog in this project is 
*         used for timing maintenance only. Match value is updated to maintain 
*         the loop time. 
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
CY_ISR(Timer_Interrupt)
{
    /* Clear the watchdog interrupt */
    CySysWdtClearInterrupt();    
    
    /* WDT match value is updated in order to obtain periodic interrupts */
    CySysWdtWriteMatch(CySysWdtReadMatch() + watchdogMatchValue); 
    
    /* Set to variable that indicates that WDT interrupt had triggered*/
    wdtInterruptOccured = TRUE;   
}

/******************************************************************************
* Function Name: WDT_Start
*******************************************************************************
*
* Summary:
*  Configures WDT.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: This API unmasks the WDT interrupt to route the interrupt to CPU and 
*         configures the ISR.
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
void WDT_Start(void)
{
    /* Setup ISR for watchdog interrupt */
    CyIntSetVector(WDT_IRQ_NUMBER, Timer_Interrupt);
   
    /* Get the actual match value required to generate a given delay */
    CalibrateWdtMatchValue();
    
     /* WDT match value is updated in order to obtain periodic interrupts */
    CySysWdtWriteMatch(CySysWdtReadMatch() + watchdogMatchValue); 
    
    /* Pass the WDT interrupt from SRSS to NVIC */
    CySysWdtUnmaskInterrupt();
    
    /* Enable WDT interrupt in NVIC to pass it to CPU */
    CyIntEnable(WDT_IRQ_NUMBER);
}

/*******************************************************************************
* Function Name: CalibrateWdtMatchValue
********************************************************************************
* Summary: 
*  This function calibrates the match value of the Watchdog Timer 
*
* Parameter:
*  None
*
* Return:
*  void
*
* Theory: The ILO is calibrated using IMO to improve the accuracy of ILO.
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
void CalibrateWdtMatchValue()
{    
    /* Contains ILO Trimmed value */
    uint32 tempIloCounts = 0u;
    
    /* Desired delay in microseconds for ILO Trimming */
    uint32 desiredDelay = ((watchdogMatchValue / ILO_CLOCK_FACTOR) * MILLI_SEC_TO_MICRO_SEC);  
    
    /* Get the ILO compensated counts i.e. the actual counts for the desired ILO frequency 
	*  Trimming is done to improve ILO accuracy using IMO; ILO default accuracy is +/- 60% 
    */
    if(CYRET_SUCCESS == CySysClkIloCompensate(desiredDelay, &tempIloCounts))
    {    
        watchdogMatchValue = (uint32)tempIloCounts;
    }    
}

/*******************************************************************************
* Function Name: EnterDeepSleepLowPowerMode
********************************************************************************
* Summary:
*  Put the device to DeepSleep power mode. Reconfigures the Components for 
*  normal operation after wake-up. 
*
* Parameters:
*  void
*
* Return:
*  void
*
* Theory: Before going to deep sleep, the API checks for any
*         I2C activity and waits till the I2C transaction is complete before 
*         the device is put to deep sleep. 
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
void EnterDeepSleepLowPowerMode(void)
{
    /* EZI2C_Sleep routine should be called only after on-going I2C 
    *  transaction is complete
    *  Enter critical section to lock slave state 
    */
    interruptState = CyEnterCriticalSection();
    
    /* Check if I2C is busy. If so, skip deep sleep until I2C is free */
    if(!(EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY))
    {
        /* Prepare for deep sleep - stop ILO calibration */
        CySysClkIloStopMeasurement();
        
        /* Configure slave to be wakeup source */
        EZI2C_Sleep();
        
    	/* Enter DeepSleep. */
    	CySysPmDeepSleep();	
       
        /* WDT or any I2C communication wakes up device from deep sleep. */

        /* Configure slave for active mode operation */
        EZI2C_Wakeup();
        
        /* Start the ILO accuracy measurement for accurate scan interval generation */
        CySysClkIloStartMeasurement();
    }
    /* Enable interrupts */
    CyExitCriticalSection(interruptState);
}

/* [] END OF FILE */
