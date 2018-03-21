/******************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description:  This code example demonstrates how to implement a USB-HID trackpad
*               using PSoC 4100S CapSense.
*
* Related Document: CE216892 USB-HID Trackpad.pdf
*
* Hardware Dependency: CY8CKIT-041-41XX PSoC 4100S Pioneer Kit
*                     (See code example CE216892 for details)
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
#include <cytypes.h>

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

/* Enable tuner for CapSense tuning. 
*  When tuner is enabled, I2C slave is used to transmit 
*  CapSense tuning data and hence mouse functionality is disabled.
*/
#define TUNER_ENABLE                (0u)
#define BUFFER_SIZE                 (16u)
#define CHECKSUM_BYTES              (2u)
#define READ_ONLY_OFFSET            (2u)
#define DUT_ID_CHECK                (0x55u)

/* I2C buffer index macros */
#define DUT_ID_INDEX                (0u)
#define HID_DATA_VALID_INDEX        (1u)   
#define BYTES_IN_BUFFER_INDEX       (2u)
#define HID_TYPE_INDEX              (3u)
#define DATA_BYTE1_INDEX            (4u)
#define DATA_BYTE2_INDEX            (5u)
#define DATA_BYTE3_INDEX            (6u)
#define DATA_BYTE4_INDEX            (7u)
#define DATA_BYTE5_INDEX            (8u)
#define DATA_BYTE6_INDEX            (9u)
#define DATA_BYTE7_INDEX            (10u)
#define DATA_BYTE8_INDEX            (11u)
#define DATA_BYTE9_INDEX            (12u)
#define DATA_BYTE10_INDEX           (13u)
#define CHECKSUM_MSB_INDEX          (14u)
#define CHECKSUM_LSB_INDEX          (15u)

/* HID macros */
#define MOUSE_FUNCTION				(1u)
#define MOUSE_DATA_BYTES_COUNT      (5u)
#define HID_DATA_INVALID            (0u)
#define HID_DATA_VALID              (1u)

/* Mouse click function macros */
#define MOUSE_FN_NO_CLICK			(0u)
#define MOUSE_FN_LEFT_CLICK			(1u)
#define MOUSE_FN_RIGHT_CLICK		(2u)

/* Trackpad X and Y coordinate related macros */
#define X_POSITION                  (0u)
#define Y_POSITION                  (1u)
#define X_POSITION_NEGATIVE         (2u)
#define Y_POSITION_NEGATIVE         (1u)

/* Threshold for the delta position after which 
*  position is scaled by a factor of 2.
*/
#define DELTA_POS_MULT_THRESHOLD    (10u)

/* Device power mode related macros */
#define LOOP_TIME_FASTSCANMODE      (20u)
#define LOOP_TIME_SLOWSCANMODE      (150u)
#define ILO_CLOCK_FACTOR            (40u)
    
/* Refresh rate control parameters */    
/* Scan period is LOOP_TIME_FASTSCANMODE in FAST_SCAN_MODE */    
#define WDT_TIMEOUT_FAST_SCAN       (ILO_CLOCK_FACTOR * LOOP_TIME_FASTSCANMODE) 
    
 /* Scan period is LOOP_TIME_SLOWSCANMODE in SLOW_SCAN_MODE */    
#define WDT_TIMEOUT_SLOW_SCAN       (ILO_CLOCK_FACTOR * LOOP_TIME_SLOWSCANMODE)

/* This timeout is for device scan mode shift from fast scan to slow scan if there is
   no proximity  detected. 2
*/
#define SCANMODE_TIMEOUT_VALUE      (150u)

#define FALSE                       (false)
#define TRUE                        (true)

#define LED_ON                      (0u)
#define LED_OFF                     (1u)
    
/* Interrupt number for watchdog */    
#define WDT_IRQ_NUMBER              (6u)
    
/* Software timer reset value */
#define SW_TIMER_RESET              (0u)  

/*******************************************************************************
*   Module Variable and Constant Declarations with Applicable Initializations
*******************************************************************************/
/* Finite state machine states for device operation */
typedef enum
{
    SENSOR_SCAN =            1u,   /* Sensor is scanned in this state */
    WAIT_FOR_SCAN_COMPLETE = 2u,   /* CPU is put to sleep in this state */
    PROCESS_DATA =           3u,   /* Sensor data is processed */
    DEEP_SLEEP =             4u    /* Device is put to deep sleep */
} DEVICE_STATE;

/* Firmware implements two refresh rates for reducing average power consumption */
typedef enum
{
    SLOW_SCAN_MODE = 0u,
    FAST_SCAN_MODE = 1u
} SCAN_MODE;

/* This variable is used to indicate the power mode */
SCAN_MODE deviceScanMode;

/* Variable indicates if trackpad is touched or not */
uint16 fingerStatus;

/* Variable to store X, Y touch coordinates */
uint16 touchPosition[2];

/* Contains previous active X, Y coordinates of trackpad */
uint16 prevTouchPosition[2];

/* Contains DeltaX and DeltaY trackpad coordinates, used for HID (mouse) */
uint16 diffPosition[2];

/* Contains information of whether DeltaX and DeltaY is negative */
uint8 xyDirectionData;

/* Contains mouse left and right click data */
uint8 mouseClickData;

/* I2C buffer to store HID mouse related information */
volatile uint8 ezI2cBuffer[BUFFER_SIZE];

/* Variable to check if device woke up because of WDT interrupt or other sources */
volatile uint8 wdtInterruptOccured = FALSE;

/* Watchdog timer match value. Device starts with fast scan mode after power-up */
volatile uint32 wdtMatchValue = WDT_TIMEOUT_FAST_SCAN;

/*******************************************************************************
*  Function Declarations
*******************************************************************************/

/* Function prototype for CapSense parameter initialization */
void capSenseInit(void);

/* Function prototype to process trackpad data and update I2C buffer */
void mouseDataProcess(void);

/* Function prototype to initialize EZI2C component */
void i2cInit(void);

/* Function prototype to update I2C buffer with trackpad and button sensor status*/
void updateI2cBuffer(void);

/* Function prototype to start watchdog timer */
void WDT_Start(void);

/* Function prototype to put the device to deep sleep */
void enterDeepSleepLowPowerMode(void);

/* Function that checks if trackpad or button sensors are ON */
bool anyWidgetActive(void);

/* API to get WDT match value to generate accurate scan intervals */
void CalibrateWdtMatchValue(void);

/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary: This funcation implements the state machine for device operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: 
*   main() performs following functions:
*  1: Initialize the CapSense, EZI2C Components
*  2: There are two modes of operation
*       Slow scan mode : Only gang sensor (all rows & columns of trackpad, left and right 
*                       buttons are ganged) is scanned at LOOP_TIME_SLOWSCANMODE loop time
*       Fast scan mode : Trackpad and button widgets are are scanned at LOOP_TIME_FASTSCANMODE loop time
*       By default, device is in fast scan mode. In fast scan mode, all the sensors are scanned
*       individually. If no CapSense widgets are active for SCANMODE_TIMEOUT_VALUE, the operating 
*       mode is changed to slow scan mode to reduce device power consumption.
*  3: Once all the CapSense widgets are scanned, the I2C buffer is updated with the 
*       following information
*       a) LeftClick and RightClick button information is updated
*       b) DeltaX and DeltaY of touch coordinates with sign information is updated
*      All the above data is read by the Host (PSoC 5LP) via I2C interface. The host interfaces 
*      to PC as a USB-HID device.
*  4: After CapSense data is processed, the device is put to deep sleep mode
* 
* Side Effects: None
*
* Note:None
*
*******************************************************************************/

int main()
{   
    /* This code implements two application level power modes: 
    *  Fast Scan mode and Slow Scan mode */
    
    /* Start the firmware state machine with sensor scan */
    DEVICE_STATE currentState = SENSOR_SCAN;
    
    /* Variable to check for timeout condition of sensor inactive condition */
    uint32 timeoutCounter = 0u;
    
    /* Variable to hold the widget# that should be scanned in FAST_SCAN_MODE */
    uint8 currentWidget = CapSense_TRACKPAD_WDGT_ID;
    
    /* Variable to store the XY touch coordinates */
    uint32 tempCoordinates;
    
    /* Variable to store interrupt state */
    uint32 interruptState = 0u;
	
    /* Set the current power mode. */
    deviceScanMode = FAST_SCAN_MODE;
    
    /* Enable interrupts. This is required for CapSense and I2C operation */ 
    CyGlobalIntEnable;
    
    /* Start the I2C block and initialize the I2C buffer data */
    i2cInit();
    
    /* Delay CapSense calibration, until the power supply is stable */
    CyDelay(100);
    
    /* Initialize all the CapSense widgets used in the project */
    capSenseInit();  

    /* Watchdog is used to control the loop time in this project */
    WDT_Start();
   
	for(;;)
	{        
		switch(currentState)
		{
            case SENSOR_SCAN:
                /* Initiate new scan only if the CapSense hardware is idle */
                if(CapSense_NOT_BUSY == CapSense_IsBusy())
                {
                    #if (TUNER_ENABLE)
                        /* Update CapSense parameters set via CapSense tuner */
                        CapSense_RunTuner();
                    #endif
                    
                    /* If device is in fast scan mode, set the parameters for trackpad or button widget scanning */
                    if(deviceScanMode == FAST_SCAN_MODE)
                    {
                        /* Check which widget has to be scanned and configure the parameters for scanning 
                        *  Each of the three widgets (trackpad, and 2 buttons) are scanned one at a time
                        *  in fast scan mode. The currentWidget variable is used to switch between the three
                        *  widgets in a round robin fashion and to configure the parameters for scanning.
                        *  Ganged sensor widget is scanned in this state to reset its state to OFF state.
                        */
                        if(currentWidget == CapSense_TRACKPAD_WDGT_ID)
                        {
                            /* Configure trackpad before scanning */
                            CapSense_CSDSetupWidget(CapSense_TRACKPAD_WDGT_ID);
                            
                            /* Scan LeftClick button after trackpad */
                            currentWidget = CapSense_LEFTCLICK_WDGT_ID;
                        }
                        else if(currentWidget == CapSense_LEFTCLICK_WDGT_ID)
                        {
                            /* Configure LeftClick button widget before scanning */
                            CapSense_CSDSetupWidget(CapSense_LEFTCLICK_WDGT_ID);
                            
                            /* Scan RightClick button after scanning LeftClick */
                            currentWidget = CapSense_RIGHTCLICK_WDGT_ID;
                        }
                        else if (currentWidget == CapSense_RIGHTCLICK_WDGT_ID)
                        {
                            /* Configure RightClick button widget before scanning */
                            CapSense_CSDSetupWidget(CapSense_RIGHTCLICK_WDGT_ID);
                            
                            /* If gangsensor status is ON and trackpad, buttons are inactive, 
                            *  scan ganged sensor until its status is OFF */
                            if(CapSense_IsWidgetActive(CapSense_GANGEDSENSOR_WDGT_ID) && (!anyWidgetActive()))
                            {
                                currentWidget = CapSense_GANGEDSENSOR_WDGT_ID;
                            }
                            /* If gangsensor status is OFF, scan trackpad after RightClick button */
                            else
                            {
                                /* Scan trackpad after scanning RightClick */
                                currentWidget = CapSense_TRACKPAD_WDGT_ID;
                            }
                        }
                        /* If currentWidget is gangsensor, configure it for scanning */
                        else
                        {
                            /* Configure gangsensor before scanning */
                            CapSense_CSDSetupWidget(CapSense_GANGEDSENSOR_WDGT_ID);
                            
                            /* Scan trackpad after scanning gangsensor */
                            currentWidget = CapSense_TRACKPAD_WDGT_ID;
                        }
                    }
                    /* If scan mode is slow scan mode, configure ganged sensor widget for scanning */
                    else
                    {
                        /* Set up ganged sensor for scanning */
                        CapSense_CSDSetupWidget(CapSense_GANGEDSENSOR_WDGT_ID);   
                    }
                    
                    /* Scan the widget which was configured in the previous step */
                    CapSense_Scan();
                    
                    /* Set the state machine to wait state until the scan is complete */
                    currentState = WAIT_FOR_SCAN_COMPLETE;   
                }
            break;
                
            case WAIT_FOR_SCAN_COMPLETE:
                /* Device is in sleep state until CapSense scanning is complete or
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
                    /* If CapSense scanning is in progress, put CPU to sleep */
                    CySysPmSleep();
                }
                /* If CapSense scanning is complete, process the CapSense data */
                else
                {
                    /* In fast scan mode, keep scanning the widgets until the widget is trackpad */
                    if((deviceScanMode == FAST_SCAN_MODE) && (currentWidget != CapSense_TRACKPAD_WDGT_ID))
                    {
                        currentState = SENSOR_SCAN;
                    }
                    /* In slow scan mode, process data directly as only gangsensor is scanned 
                    *  In fast scan mode, data is processed after all the sensors are scanned
                    */
                    else
                    {
                        /* If current widget is trackpad or mode is slow scan mode, process the sensor data */
                        currentState = PROCESS_DATA;
                    }
                }
                /* Enable interrupts for servicing ISR */
                CyExitCriticalSection(interruptState);
            break;
            
			case PROCESS_DATA:
                /* Set next state to SLEEP */
                currentState = DEEP_SLEEP;

                if(deviceScanMode == FAST_SCAN_MODE)
                {
                    /* If mode is fast scan mode, process trackpad and button widget data. 
                    *  Filters are applied only for ganged sensor.
                    *  For trackpad, filters are not applied to get best response time
                    */
                    CapSense_ProcessWidget(CapSense_TRACKPAD_WDGT_ID);
                    CapSense_ProcessWidget(CapSense_LEFTCLICK_WDGT_ID);
                    CapSense_ProcessWidget(CapSense_RIGHTCLICK_WDGT_ID);
                    
                    /* If trackpad and button sensor are inactive and gangsensor status is ON
                    *  process the gang sensor data until its status is OFF
                    */
                    if((!anyWidgetActive()) && CapSense_IsWidgetActive(CapSense_GANGEDSENSOR_WDGT_ID))
                       CapSense_ProcessWidget(CapSense_GANGEDSENSOR_WDGT_ID);
                    
                    /* Check if trackpad is active */
					fingerStatus =  CapSense_IsWidgetActive(CapSense_TRACKPAD_WDGT_ID);
                    
                    /* If trackpad is active, obtain XY coordinates */
					if(fingerStatus == TRUE)
					{
    					tempCoordinates = CapSense_GetXYCoordinates(CapSense_TRACKPAD_WDGT_ID);
                        
                        /* Extract XY positions separately */
    					touchPosition[X_POSITION] = LO16(tempCoordinates);
    					touchPosition[Y_POSITION] = HI16 (tempCoordinates);
					}
                    /* If trackpad is inactive, set touch position to 0. This will reset all the 
                    *  previous history
                    */
                    else
                    {
						touchPosition[X_POSITION] = 0u;
						touchPosition[Y_POSITION] = 0u;
                    }
                    
                    /* Compute deltaX and deltaY touch position */
					mouseDataProcess();	
                    
                    /* Update the I2C buffer with HID data */
                    updateI2cBuffer();
                        
                    /* If trackpad or button sensor is not active, increment the timeout counter */
                    if(!CapSense_IsAnyWidgetActive())
                    {
                        /* Turn-off LED */
                        Red_LED_Write(LED_OFF);
                        
                        /* Increment timeout counter variable */
                        timeoutCounter++;
                        
                        /* Check if sensor is inactive for a duration greater than SCANMODE_TIMEOUT_VALUE */
                        if(timeoutCounter >= SCANMODE_TIMEOUT_VALUE)
                        {
                            /* Configure refresh interval to slow rate to reduce device power */
                            wdtMatchValue = WDT_TIMEOUT_SLOW_SCAN; 

                            /* Set mode to slow scan mode to scan only ganged sensor */
                            deviceScanMode = SLOW_SCAN_MODE;
                        }
                    }  
                    /* If either trackpad or button sensors are active, turn-on LED and reset the timeout counter */
                    else
                    {
                        Red_LED_Write(LED_ON);
                        
                        /* Because sensor is active, reset the counter */
                        timeoutCounter = 0u;
                    }
                }
                /* If device is in SLOW_SCAN_MODE, perform the below operations */
                else
                {
                    /* Update baseline, sensor status for ganged sensor */
                    CapSense_ProcessWidget(CapSense_GANGEDSENSOR_WDGT_ID);

                    /* Check ganged sensor status */
                    if(CapSense_IsWidgetActive(CapSense_GANGEDSENSOR_WDGT_ID))
                    {
                        /* Set watchdog match value to fast scan mode */
                        wdtMatchValue = WDT_TIMEOUT_FAST_SCAN;         

                        /* Reset counter */
                        timeoutCounter = SW_TIMER_RESET;

                        /* Change the device mode to fast because touch is detected on gang sensor */
                        deviceScanMode = FAST_SCAN_MODE; 

                        /* Scan trackpad widget first */
                        currentWidget = CapSense_TRACKPAD_WDGT_ID;

                        /* Skip sleep state and set next state to sensor scan */
                        currentState = SENSOR_SCAN;
                    }
                }
            break;

			case DEEP_SLEEP:
                
                /* Enter deep sleep after CapSense scan is completed */
			    enterDeepSleepLowPowerMode();
                
                /* Start scanning the sensors only if interrupt occurred due to WDT.
                *  Interrupt can also occur due to I2C interrupt either when tuner is running
                *  or when PSoC 5LP is reading HID data from slave device.
                *  In such cases, sensor is not scanned until WDT interrupt has occurred
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
    /* Temporary variable to store trackpad coordinates */
    uint32 tempCoordinates;

    /* Starts the CapSense block and performs baseline initialization */
    CapSense_Start();
    
    /* Set the default finger presence status and touch coordinates */
    fingerStatus =  0x0u;
    tempCoordinates = 0xFFFFu;
    
    /* Get the touch position */
    touchPosition[X_POSITION] = LO16(tempCoordinates);
    touchPosition[Y_POSITION] = HI16 (tempCoordinates);
    
    /* Initialize the previous touch position */
    prevTouchPosition[X_POSITION] = touchPosition[X_POSITION];
    prevTouchPosition[Y_POSITION] = touchPosition[Y_POSITION];
}

/******************************************************************************
* Function Name: mouseDataProcess
*******************************************************************************
*
* Summary:
*  This API processes the rawdata from the CapSense block and prepares the data
*  to be sent over I2C to host
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: This API computes the deltaX and deltaY position for trackpad. It also checks 
*         the falling edge on the leftClick and rightClick buttons.
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/

void mouseDataProcess(void)
{
    /* Variable to detect raising edge of left button status */
    static uint8 prevLeftClick = 0u;
    uint8 currentLeftClick = 0u;
    
    /* Variable to detect raising edge of right button status */
    static uint8 prevRightClick = 0u;
    uint8 currentRightClick = 0u;
    
    /* Variable to detect rising edge/falling edge of trackpad status */
    static uint8 prevfingerStatus = 0u;
    uint8 currentfingerStatus = 0u;
            
    /* Initialize the sign indication variable to '0' */
    xyDirectionData = 0u;
    
    /* If trackpad is active, perform delta position computation */
	currentfingerStatus =  CapSense_IsWidgetActive(CapSense_TRACKPAD_WDGT_ID);
    
    /* Compute delta positions only when the finger is placed on the trackpad
    *  Delta position is not computed when there is a touchdown or liftoff condition is detected
    */
    if((currentfingerStatus && prevfingerStatus))
    {
        /* Get the DeltaX and DeltaY coordinates and sign information */
        if(prevTouchPosition[X_POSITION] > touchPosition[X_POSITION])
        {
            diffPosition[X_POSITION] = prevTouchPosition[X_POSITION] - touchPosition[X_POSITION];
            xyDirectionData = xyDirectionData | X_POSITION_NEGATIVE;
        }
        /* If previous X position is less than current X position */
        else
        {
            diffPosition[X_POSITION] = touchPosition[X_POSITION] - prevTouchPosition[X_POSITION];
        }
        
        if(prevTouchPosition[Y_POSITION] > touchPosition[Y_POSITION])
        {
            diffPosition[Y_POSITION] = prevTouchPosition[Y_POSITION] - touchPosition[Y_POSITION];
        }
        /* If previous Y position is less than current Y position */
        else
        {
            diffPosition[Y_POSITION] = touchPosition[Y_POSITION] - prevTouchPosition[Y_POSITION];
            xyDirectionData = xyDirectionData | Y_POSITION_NEGATIVE;
        }
    }
    /* If either trackpad is not active, or a touchdown or liftoff condition is detected
    *  delta position is initialized to 0.
    */
    else
    {
        diffPosition[X_POSITION] = 0u;
        diffPosition[Y_POSITION] = 0u;
    }
    
    /* If delta position is greater than the threshold limit, scale the 
    *  delta position by a factor of 2.
    */
    if(diffPosition[X_POSITION] > DELTA_POS_MULT_THRESHOLD)
        diffPosition[X_POSITION] = diffPosition[X_POSITION] << 1u;
    
    if(diffPosition[Y_POSITION] > DELTA_POS_MULT_THRESHOLD)
        diffPosition[Y_POSITION] = diffPosition[Y_POSITION] << 1u;

    /* Update previous touch position variables once delta position is computed */
    prevTouchPosition[X_POSITION] = touchPosition[X_POSITION];
    prevTouchPosition[Y_POSITION] = touchPosition[Y_POSITION];
    
    /* Get the current button status */
    currentLeftClick = CapSense_IsWidgetActive(CapSense_LEFTCLICK_WDGT_ID);
    currentRightClick = CapSense_IsWidgetActive(CapSense_RIGHTCLICK_WDGT_ID);
    
    /* Get the mouse left click and right click data */
    mouseClickData = 0;
    
    /* Send left click only on the raising edge of left-click button status */
    if((prevLeftClick == 0) && (currentLeftClick !=0))
    {
        mouseClickData |= MOUSE_FN_LEFT_CLICK; 
    }
    else
    {
        mouseClickData &= ~(MOUSE_FN_LEFT_CLICK); 
    }
    
    /* Send right click only on the raising edge of right-click button status */
    if((prevRightClick == 0) && (currentRightClick !=0))
    {
        mouseClickData |= MOUSE_FN_RIGHT_CLICK; 
    }
    else
    {
        mouseClickData &= ~(MOUSE_FN_RIGHT_CLICK); 
    }
    
    /* Update previous trackpad and button status */
    prevfingerStatus = currentfingerStatus;
    prevLeftClick = currentLeftClick;
    prevRightClick = currentRightClick;
}

/******************************************************************************
* Function Name: i2cInit
*******************************************************************************
*
* Summary:
*  This function starts the I2C block and assigns the I2C buffer that is exposed to the master
*  If tuner communication is enabled, the I2C buffer will be the CapSense data structure
*  If tuner communication is disabled, the I2C buffer will be the HID data structure
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: DUT index and HID type are one time updates and hence they are 
*         updated on power up.
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
void i2cInit(void)
{
    /* Configure the SCB clock for EZI2C operation */
    EZI2C_Start();
    
    /* If tuner is enabled, expose CapSense data structure to I2C master */
    #if (TUNER_ENABLE)
        EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam),\
                         (uint8 *)&CapSense_dsRam);
    #else
        /* Fill in the buffer with default values */
        ezI2cBuffer[DUT_ID_INDEX] = DUT_ID_CHECK;
        
        /* Trackpad acts as a HID mouse in this example */
        ezI2cBuffer[HID_TYPE_INDEX] = MOUSE_FUNCTION;
        
        /* Setup I2C buffer */
        EZI2C_EzI2CSetBuffer1(BUFFER_SIZE, READ_ONLY_OFFSET, ezI2cBuffer);
    #endif
}

/******************************************************************************
* Function Name: updateI2cBuffer
*******************************************************************************
*
* Summary:
*  This API updates the I2C buffer with the HID related data
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: This API updates the buffer with trackpad and button sensor status. 
*         Once the host reads the data, the entire buffer is cleared.
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
void updateI2cBuffer(void)
{
    uint16 checksum = 0u;
    uint8 index;
    static uint8 resetDone = 0u;
    
    /* If either trackpad or button sensor is touched, update the I2C buffer */
    if(fingerStatus || mouseClickData)
    {
        /* Set the data valid byte to invalid before updating the buffer */
        ezI2cBuffer[HID_DATA_VALID_INDEX] = HID_DATA_INVALID;
                
        /* Reset all the data */
        for(index = DATA_BYTE1_INDEX; index <= DATA_BYTE5_INDEX;index++)
        {
            ezI2cBuffer[index] = 0u;
        }

        /* 5 bytes of data is sent to host 
        BYTE1 - Mouse click data
        BYTE2 - Delta x data
        BYTE3 - Delta y data
        BYTE4 - 0 (Scroll data, not used in this code example)
        BYTE5 - delta x and delta y sign information
        */
        ezI2cBuffer[BYTES_IN_BUFFER_INDEX] = MOUSE_DATA_BYTES_COUNT;
        
        ezI2cBuffer[DATA_BYTE1_INDEX] = mouseClickData;
        ezI2cBuffer[DATA_BYTE2_INDEX] = diffPosition[Y_POSITION];
        ezI2cBuffer[DATA_BYTE3_INDEX] = diffPosition[X_POSITION];
        ezI2cBuffer[DATA_BYTE4_INDEX] = 0u;
        ezI2cBuffer[DATA_BYTE5_INDEX] = xyDirectionData;

        /* Compute the checksum, sum of all register data
           Checksum is set to 1 because data valid byte is set to '0'
           and once checksum is computed and updated to buffer the data
           valid byte will be set to '1'
        */
        checksum = 1u;         
                    
        for(index=0; index < (BUFFER_SIZE- CHECKSUM_BYTES); index++)
        {
            checksum += ezI2cBuffer[index];        
        }
        
        /* Store checksum */
        ezI2cBuffer[CHECKSUM_MSB_INDEX] = HI8(checksum);
        ezI2cBuffer[CHECKSUM_LSB_INDEX] = LO8(checksum);
        
        /* Set HID_DATA_VALID_INDEX to '1' to indicate buffer is updated */
        ezI2cBuffer[HID_DATA_VALID_INDEX] = HID_DATA_VALID;
        
        /* Set variable to indicate that buffer data is not reset */
        resetDone = 0;
    }
    
    /* If host has read the data and reset the HID_DATA_VALID_INDEX byte, reset all the data */
    if(ezI2cBuffer[HID_DATA_VALID_INDEX] == HID_DATA_INVALID)
    {
        /* Check if reset is already done */
        if(resetDone == 0)
        {
            /* Initialize all the buffer data bytes to zero */
            for(index = DATA_BYTE1_INDEX; index <= DATA_BYTE5_INDEX; index++)
            {
                ezI2cBuffer[index] = 0;
            }
            
            /* Set variable to indicate that reset is done */
            resetDone = 1u;
        }
    }    
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
* Note: None
*
*******************************************************************************/

CY_ISR(Timer_Interrupt)
{
    /* Clear the watchdog interrupt */
    CySysWdtClearInterrupt();  
    
    /* Update the watchdog match value */
    CySysWdtWriteMatch(CySysWdtReadMatch() + wdtMatchValue);
    
    /* Set flag to true to indicate main loop that interrupt is because of WDT */
    wdtInterruptOccured = TRUE;
}

/******************************************************************************
* Function Name: WDT_Start
*******************************************************************************
*
* Summary:
*  Configures the Watchdog Timer (WDT).
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: This API configures the WDT interrupt and loads the WDT match value.
*         The WDT interrupt is masked to route the interrupt to CPU for periodic
*         interrupt generation.
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
    CySysWdtWriteMatch(CySysWdtReadCount() + wdtMatchValue);
    
    /* Pass the WDT interrupt from SRSS to NVIC */
    CySysWdtUnmaskInterrupt();
    
    /* Enable WDT interrupt in NVIC to pass it to CPU */
    CyIntEnable(WDT_IRQ_NUMBER);
}

/*******************************************************************************
* Function Name: enterDeepSleepLowPowerMode
********************************************************************************
* Summary:
*  Puts the device in DeepSleep power mode. Reconfigures the Components for 
*  normal operation after wake-up. 
*
* Parameters:
*  void
*
* Return:
*  void
*
* Theory: Before going to deep sleep, the API checks for any I2C activity.
*         If I2C is active, the API returns without forcing the system to DeepSleep.
*         If I2C is inactive, device is put to DeepSleep and is woken up by WDT or
*         I2C interrupt.
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
void enterDeepSleepLowPowerMode(void)
{
    /* EZI2C_Sleep routine should be called only after on-going I2C 
    *  transaction is complete
    *  Enter critical section to lock slave state 
    */
    
    /* Variable to store interrupt state */
    uint32 interruptState = 0u;
    
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
* Function Name: anyWidgetActive
********************************************************************************
*
* Summary:
*  This API checks if any widget is active
*
* Parameters:
*  None.
*
* Theory: This API checks if any of the LeftClick, RightClick or trackpad sensors
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
       CapSense_IsWidgetActive(CapSense_LEFTCLICK_WDGT_ID) || \
       CapSense_IsWidgetActive(CapSense_RIGHTCLICK_WDGT_ID)
      )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
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
    
    /* Contains ILO Trimmed value */
    uint32 desiredDelay = 0;
    
    /* Desired delay for ILO Trimming */
    if(deviceScanMode == FAST_SCAN_MODE)
    {
        desiredDelay = WDT_TIMEOUT_FAST_SCAN;
    }
    else
    {
        desiredDelay = WDT_TIMEOUT_SLOW_SCAN;
    }
    
    /* Get the ILO compensated counts i.e. the actual counts for the desired ILO frequency 
	*  Trimming is done to improve ILO accuracy using IMO; ILO default accuracy is +/- 60% 
    */
    if(CYRET_SUCCESS == CySysClkIloCompensate(desiredDelay, &tempIloCounts))
    {    
        wdtMatchValue = (uint32)tempIloCounts;
    }    
}

/* [] END OF FILE */
