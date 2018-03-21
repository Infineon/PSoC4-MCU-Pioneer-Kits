/******************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description: This code example demonstrates how to implement low-power CapSense 
*              proximity sensing with a proximity sensing distance of 5 cm and an 
*              average current consumption of 25 uA.
*
* Related Document: CE214023 Proximity Sensing.pdf
*
* Hardware Dependency: See code example CE214023
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
/******************************************************************************
* Theory of Operation: This project demonstrates the proximity sensing 
* capability of PSoC 4100S device.  
*
* The PSoC 4100S device detects an approaching hand using the proximity sensor. 
* If the signal level detected by the proximity sensor is greater than a 
* threshold value, the PSoC 4100S turns-on the Red LED on the Pioneer 
* Kit. The brightness of the LED is increased as the hand approaches the 
* proximity sensor. To achieve lower average power consumption, the PSoC 
* 4100S scans the proximity sensor at a low refresh rate when proximity is not 
* detected. If proximity is detected during sensor scan, PSoC 4100S 
* starts scanning the sensor at a fast refresh rate. If there is no proximity for a period 
* more than timeout value, PSoC 4100S increases the interval between two 
* consecutive sensor scans to slow refresh rate. 
*
* Note: The refresh rate is dependent on LFCLK accuracy. ILO is used 
* as the source to LFCLK. With ILO, the accuracy of the time can vary as much as 
* 60 percent. ILO trim method is used to improve the ILO accuracy to 10 percent.
*******************************************************************************/

/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <project.h>

/*****************************************************************************
* MACRO Definitions
*****************************************************************************/   
    
/* Boolean constants */
#define TRUE                            (1u)
#define FALSE                           (0u)
#define OFF						        (FALSE)
#define ON						        (TRUE)

/* General Constants */    
#define ZERO					        (0u)

/* Device power mode related macros */
#define LOOP_TIME_FASTSCANMODE          (20u)
#define LOOP_TIME_SLOWSCANMODE          (200u)
#define MILLI_SEC_TO_MICRO_SEC          (1000u)
#define ILO_CLOCK_FACTOR                (40u)
    
/* Refresh rate control parameters */    
/* Scan period is LOOP_TIME_FASTSCANMODE in FAST_SCAN_MODE */    
#define WDT_TIMEOUT_FAST_SCAN           (ILO_CLOCK_FACTOR * LOOP_TIME_FASTSCANMODE) 
    
 /* Scan period is LOOP_TIME_SLOWSCANMODE in SLOW_SCAN_MODE */    
#define WDT_TIMEOUT_SLOW_SCAN           (ILO_CLOCK_FACTOR * LOOP_TIME_SLOWSCANMODE)
    
/* Interrupt number for watchdog */    
#define WDT_IRQ_NUMBER                  (6u)    
    
/* Reset value of softCounter */    
#define RESET					        (0u)
	
/* This timeout is for changing the refresh interval from fast to slow rate
*  The timeout value is WDT_TIMEOUT_FAST_SCAN * SCANMODE_TIMEOUT_VALUE
*/
#define SCANMODE_TIMEOUT_VALUE          (150u)
 
/* LED Brightness Control Constants */  
#define MIN_BRIGHTNESS_COMPARE                  (500u)
#define MAX_BRIGHTNESS_COMPARE                  (65535u)  
#define BRIGHTNESS_STEP                         (50u)
#define SIGNAL_CHANGE_STEP                      (1u)
#define MAX_16_BIT_VALUE                        (0x7FFFu)    
#define CapSense_TOTAL_PROX_SENSOR_COUNT        (CapSense_PROXIMITY0_NUM_SENSORS)
#define MIN_PROXIMITY_SIGNAL                    (CapSense_PROXIMITYSENSOR_FINGER_TH - CapSense_PROXIMITYSENSOR_HYSTERESIS) 

/* Finite state machine states for device operating states */
typedef enum
{
    SENSOR_SCAN = 0x01u,           /* Sensor is scanned in this state */
    WAIT_FOR_SCAN_COMPLETE = 0x02u, /* CPU is put to sleep in this state */
    PROCESS_DATA = 0x03u,   /* Sensor data is processed */
    SLEEP = 0x04u           /* Device is put to deep sleep */
} DEVICE_STATE;  

/* Firmware implements two refresh rates for reducing average power consumption */
typedef enum
{
    SLOW_SCAN_MODE = 0u,
    FAST_SCAN_MODE = 1u
} SCAN_MODE;

/*******************************************************************************
*   Module Variable and Constant Declarations with Applicable Initializations
*******************************************************************************/

/* Variable to store interrupt state */
uint32 interruptState = 0u;

/* Variable to check the WDT interrupt state */
volatile uint8 wdtInterruptOccured = FALSE;

/* Contains watchdog match value to generate period interrupt */
volatile uint32 watchdogMatchValue = WDT_TIMEOUT_FAST_SCAN;

/*****************************************************************************
* Function Prototypes
*****************************************************************************/ 
/* API to prepare the device for deep sleep */
void EnterDeepSleepLowPowerMode(void);

/* API to update PWM compare value to control LED brightness */
void LED_SetBrightness(uint8 ledState, uint16 signal);

/* API to get compensated WDT match value to generate precise scan intervals */
void CalibrateWdtMatchValue(void);

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
*  1: Initialize the CapSense, Watchdog, I2C and PWM Components
*  2: There are two modes of operation
*       Slow scan mode : Proximity sensor is scanned at LOOP_TIME_SLOWSCANMODE refresh interval.
*                        LEDs are turned off in this state. Upon proximity detection, device
*                        switches to fast scan mode.
*       Fast scan mode : Proximity sensor is scanned at LOOP_TIME_FASTSCANMODE refresh interval.
*                        When proximity is detected, Red LED brightness is varied depending on the 
*                        proximity of the user hand. 
*       By default, device is in fast scan mode. In the fast scan mode, if proximity sensor
*       is not active for SCANMODE_TIMEOUT_VALUE, the operating mode is changed to slow scan mode
*       to reduce device power consumption. If proximity is detected in slow scan mode,
*       the device switches to fast scan mode.
* 
* Side Effects: None
*
* Note:None
*
*******************************************************************************/
int main()
{
    /* Initialize current and previous Proximity sensor state to inactive. */
	uint8 currentProxSensorState  = FALSE;
	
	/* This variable is used to implement a software counter. If the value 
	*  of this counter is greater than SCANMODE_TIMEOUT_VALUE, the proximity sensor was inactive
	*  for more than 3s. 
    */
	uint16 softCounter = RESET;
        
    /* Variable to hold the current device state. 
    *  State machine starts with sensor scan state after power-up
    */
    DEVICE_STATE currentState = SENSOR_SCAN; 
	
    /* This variable is used to indicate the current device scan mode */
    SCAN_MODE deviceScanMode = FAST_SCAN_MODE;
    
    /* Enable global interrupts */
    CyGlobalIntEnable; 
  
    /* Initialize EZI2C block */
    EZI2C_Start();
    
    /* Set up communication data buffer to CapSense data structure to 
    *  expose to I2C master at primary slave address request 
    */		
    EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam),\
                     (uint8 *)&CapSense_dsRam);
 
    /* Start and initialize the PWM block */
	PrISM_LED_Start();
    
	/* Switch off the LED after power-up */
	LED_SetBrightness(OFF, ZERO);
    
    /* Delay CapSense calibration, until the power supply is stable */
    CyDelay(300);
    
    /* Start the CapSense block and initialize the sensor baselines */	
	CapSense_Start();
    
    CapSense_InitializeAllBaselines();
    
    /* Watchdog is used to control the loop time in this project and watchdog
    *  is set to generate interrupt at every LOOP_TIME_FASTSCANMODE in fast scan mode  
    *  and at LOOP_TIME_SLOWSCANMODE in slow scan mode
    */
    WDT_Start();
    
    /* CapSense_CSDSetupWidgetExt() configures CapSense parameters for proximity sensor and connects  
    *  sensor to AMUXBUS.
    *  Since we have only one sensor in this project, calling an extension API (CapSense_CSDSetupWidgetExt)
    *  avoids connecting and disconnecting the sensor from AMUXBUS each time it is scanned.
    *  This eliminates the time taken to connect/disconnect sensor and helps in reducing the device power consumption
    */
    CapSense_CSDSetupWidgetExt(CapSense_PROXIMITYSENSOR_WDGT_ID, CapSense_PROXIMITYSENSOR_SNS0_ID);
   
    for(;;)
    {
        /* Switch between scan->wait-for-scan->process->sleep states */
        switch(currentState)
        {
            /* If currentState is SENSOR_SCAN, initiate sensor scan if the CapSense hardware is idle */
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
                        CapSense_CSDSetupWidgetExt(CapSense_PROXIMITYSENSOR_WDGT_ID, CapSense_PROXIMITYSENSOR_SNS0_ID);
                    }
                    /* Scan widget configured by CSDSetupWidgetExt API 
                    *  This API scans the sensor which is already connected to AMUXBUS
                    *  and does not disconnect it from AMUXBUS after scan is completed
                    */
                    CapSense_CSDScanExt();
                                        
                    /* Set next state to WAIT_FOR_SCAN_COMPLETE */
                    currentState = WAIT_FOR_SCAN_COMPLETE;      
                }
                break;
                
            case WAIT_FOR_SCAN_COMPLETE:
                /* Device is in CPU Sleep until CapSense scanning is complete or
                *  device is woken-up by either CapSense interrupt or I2C interrupt 
                *  Critical section is entered to disable interrupts, so that ISR is not serviced while
                *  checking for CapSense scan status.
                *  If interrupts are not disabled, ISR will be serviced and might cause system hang
                *  in the following scenario:
                *  When CapSense_IsBusy() is checked, lets assume hardware is busy. But, immediately
                *  after checking this, scan might be completed and ISR is serviced. If CPU is put to sleep
                *  there is no interrupt to wakeup the device from sleep as it is already serviced.
                *  To avoid this, interrupts are disabled so that ISR is not serviced. 
                *  If an interrupt is triggered and device is in critical section, CPU will not be put to sleep
                */
                interruptState = CyEnterCriticalSection();
                            
                /* Check if CapSense scanning is still running */
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
                
                /* Update the baseline, sensor status for proximity sensor */
        		CapSense_ProcessWidget(CapSense_PROXIMITYSENSOR_WDGT_ID);           
                
        		/* Check if proximity sensor is active */
        		currentProxSensorState = CapSense_IsWidgetActive(CapSense_PROXIMITYSENSOR_WDGT_ID);
                
                if(deviceScanMode == FAST_SCAN_MODE)
                {
                    /* If proximity widget is reported active, drive LED */
            		if(currentProxSensorState)
                    {
            			/* Set the LED at a brightness level corresponding 
            			*  to the proximity distance 
                        */
            			LED_SetBrightness(ON, CapSense_PROXIMITYSENSOR_SNS0_DIFF_VALUE);

            			/* Reset the software counter if proximity is detected */
            			softCounter = RESET;
                    }
                    /* If proximity is not detected, perform below functions */
                    else
                    {
                        /* Increment the software counter if proximity sensor is not active */
        			    softCounter++;
                        
                        /* Switch off the LED if proximity is not detected */
    			        LED_SetBrightness(OFF, ZERO);
                        
        			    /* If no proximity is detected for a time equal to SCANMODE_TIMEOUT_VALUE, 
                        *  switch to slow scan mode to reduce power
                        */
                        if(softCounter >= SCANMODE_TIMEOUT_VALUE)
                        {                            
                            /* Set refresh interval to WDT_TIMEOUT_SLOW_SCAN */
        			        watchdogMatchValue = WDT_TIMEOUT_SLOW_SCAN;
                            
                            /* Set mode to slow scan mode to scan sensors at LOOP_TIME_SLOWSCANMODE */
                            deviceScanMode = SLOW_SCAN_MODE;
                            
                            /* Set drive mode of LED pin to high-z to avoid leakage */
                            Red_LED_SetDriveMode(Red_LED_DM_ALG_HIZ);
                            
                            /* Put PWM component to sleep since it is not required in slow scan mode */
                            PrISM_LED_Sleep();
                        }  
                    }
                }
                /* If mode is slow scan mode, check for proximity widget status */
                else
                {
                    /* If proximity sensor is active in slow scan mode, immediately switch to 
                    *  fast scan mode
                    */
                    if(CapSense_IsWidgetActive(CapSense_PROXIMITYSENSOR_WDGT_ID))
                    {
                        /* If sensor is active in slow-scan mode, skip sleep
                        *  and perform sensor scan in the next loop
                        */
                        currentState = SENSOR_SCAN;
                        
                        /* Set watchdog match value to fast scan mode */
    			        watchdogMatchValue = WDT_TIMEOUT_FAST_SCAN;      
                        
                        /* Change the device mode to fast scan mode to provide fast touch response */
                        deviceScanMode = FAST_SCAN_MODE; 
                        
                        /* Restore PWM component values */
                        PrISM_LED_Wakeup();
                        
                        /* Set drive mode to strong to drive LEDs */
                        Red_LED_SetDriveMode(Red_LED_DM_STRONG);
                    }
                }
                break;
                     
            case SLEEP:
                /* In the fast scan mode, put the device to only CPU sleep as TCPMW block is ON
                *  to drive LED and TCPWM cannot work in deep-sleep mode
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
                /* Start scanning the proximity sensor only if interrupt occured due to WDT.
                *  Interrupt can also occur due to I2C interrupt while tuner is running.
                *  In such cases, sensor is not scanned until WDT interrupt has occured
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

/*******************************************************************************
* Function Name: LED_SetBrightness
********************************************************************************
* Summary:
*  Sets the LED at a brightness level corresponding to the value passed to this function.
*
* Parameters:
* 	ledState: LED ON/OFF state 
*	signal: The proximity signal value corresponding to which the LED brightness
*           has to be changed.
* Return:
*  void
*
* Theory: LED brightness level is set proportional to the difference counts 
*         of the sensors.
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
void LED_SetBrightness(uint8 ledState, uint16 signal)
{
    uint32 ledBrightness = ZERO;	
	
	/* The relation between proximity signal vs PWM compare value is a straight line 
    *  of the form y = mx + c 
    *  Where y is the compare value and C is the minimum compare value when signal = MIN_PROXIMITY_SIGNAL
    *  The slope of the curve 'm' is given by BRIGHTNESS_STEP/SIGNAL_CHANGE_STEP
    *  Scale the signal value for a visible brightness change. 
    */
	if(ledState)
	{
		ledBrightness = ( MIN_BRIGHTNESS_COMPARE +
                        ((BRIGHTNESS_STEP * (signal - MIN_PROXIMITY_SIGNAL))/
                        SIGNAL_CHANGE_STEP)
                        );
        
		/* Limit the maximum brightness compare value to 16 bit */
        if(ledBrightness > MAX_16_BIT_VALUE)
		{
            ledBrightness = MAX_16_BIT_VALUE;
        }
	}
    /* If proximity sensor is not active, turnoff the PWM */
	else
	{
		ledBrightness = ZERO;
	}
	/* Update the PWM compare value to change the LED brightness. */
	PrISM_LED_WriteCompare((uint16)ledBrightness);
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
    
    /* Set to variable that indicates that WDT interrupt had triggered */
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
*         enables WDT.
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
    
    /* Enable WDT interrupt */
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
* Parameters:
*  watchdogMatchValue: Watchdog timer match value to be set.
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
			Trimming is done to improve ILO accuracy using IMO; ILO default accuracy is +/- 60% */
    if(CYRET_SUCCESS == CySysClkIloCompensate(desiredDelay, &tempIloCounts))
    {    
        watchdogMatchValue = (uint32)tempIloCounts;
    }    
}

/* [] END OF FILE */

