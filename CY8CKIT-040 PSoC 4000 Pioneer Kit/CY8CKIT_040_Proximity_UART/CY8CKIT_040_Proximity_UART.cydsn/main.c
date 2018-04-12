/******************************************************************************
* Project Name		: CY8CKIT_040_Proximity_UART_Design
* File Name			: main.c
* Version 			: 1.0
* Device Used		: CY8C4014LQI-422
* Software Used		: PSoC Creator 3.1
* Compiler    		: ARM GCC 4.8.4, ARM MDK Generic, ARM RVDS Generic
* Related Hardware	: CY8CKIT-040 PSoC 4000 Pioneer Kit 
*
********************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation. All Rights Reserved.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress)
* and is protected by and subject to worldwide patent protection (United
* States and foreign), United States copyright laws and international treaty
* provisions. Cypress hereby grants to licensee a personal, non-exclusive,
* non-transferable license to copy, use, modify, create derivative works of,
* and compile the Cypress Source Code and derivative works for the sole
* purpose of creating custom software in support of licensee product to be
* used only in conjunction with a Cypress integrated circuit as specified in
* the applicable agreement. Any reproduction, modification, translation,
* compilation, or representation of this software except as specified above 
* is prohibited without the express written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH 
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the 
* materials described herein. Cypress does not assume any liability arising out 
* of the application or use of any product or circuit described herein. Cypress 
* does not authorize its products for use as critical components in life-support 
* systems where a malfunction or failure may reasonably be expected to result in 
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of 
* such use and in doing so indemnifies Cypress against all charges. 
*
* Use of this Software may be limited by and subject to the applicable Cypress
* software license agreement. 
*******************************************************************************/

/********************************************************************************
*                           THEORY OF OPERATION
*********************************************************************************
* The project will demonstrate CapSense Proximity and Software UART TX interface
* for viewing CapSense data. As the hand approaches the proximity wire-loop,
* the brightness of the LED increases (as the hand comes in proximity range). 
* The LED brightness depends on the hand's distance from the loop.
*
* Hardware connection required for testing -
* LED (RED)	- P3[2] (hard-wired in the board)
* PROX pin 	- P2[0] (requires an external wire/wire-loop connection)
* Cmod pin	- P0[4] (hard-wired in the board itself)
* TX pin	- P3[0] (hard-wired in the board itself, selected using TX_PORT/TX_PIN
*				macros present in 'main.h')
*
* Note: Debug is disabled by default in the project and UART TX line is enabled 
*		on SWD IO line (P3[0]). If debug is desired, the same can be enabled in
*		under 'System' tab in cywdr file. If debug is enabled, either TX should 
*		be disabled (by commenting out TX_ENABLE macro in ‘main.h’) or should be 
*		routed to another pin (by modifying TX_PORT and TX_PIN macros).
********************************************************************************/

#include "main.h"

/* The calculatedDuty parameter used for setting PWM dutycycle gives a value between 0 - 32768. */
/* The slider component in uc/Probe works in range of 0 - 255. The calculatedDuty variable is */
/* downgraded to range of 0 - 100 for appropriate display in uc/Probe. To downgrade the value */
/* the calculatedDuty parameter need to be divided by (32768/100 = 327.68) In hex 0x147. */
#define DOWNSCALE_FACTOR 0x147

/* Function prototypes */
uint32 PRSm_CalculateDuty(uint16 signal);
void UART_SendTxData(void);

void Input_Init(void);
void Output_Init(void);

void Input_Process(void);
void Output_Process(void);

CY_ISR_PROTO(WatchDog_ISR);

/* PRSm duty variable for sending over UART */
uint16 calculatedDuty;
uint16 sleepTimerCount;

uint8 ucSlider;
uint8 ucARGB = 0;

/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary:
*  main() performs following functions:
*  1: Initialize Input and Output blocks
*  2: Scans and process Proximity sensor
*  3: Updates the PRSm duty based on sensor position and sends data over TX
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
int main()
{    
	/* Enable Global interrupts for CapSense operation */
	CyGlobalIntEnable; 
	
	/* Initialize the input - CapSense block */
	Input_Init();
	
	/* Initialize the output blocks - PRSm, UART TX */
	Output_Init();
	
    for(;;)
    {
        /* Process input side - Proximity sensor */
		Input_Process();
		
		/* Process output based on input signal */
		Output_Process();				
    }
}

/******************************************************************************
* Function Name: Input_Init
*******************************************************************************
*
* Summary:
*  Input_Init() performs following functions:
*  1: Enables Proximity widget and starts CapSense Block
*  2: Calibrates the Proximity sensor and Initializes its baseline
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
void Input_Init(void)
{	
	/* Enable Proximity sensor */
	CapSense_EnableWidget(CapSense_PROXIMITYSENSOR0__PROX);
	
	/* Start CapSense block */
	CapSense_Start();
		
	/* Initialize Proximity baseline */
	CapSense_InitializeSensorBaseline(PROXIMITY_SENSOR);	
	
	/* Update the match register for generating a periodic WDT ISR. */
	/*Note: In order to do a periodic ISR using WDT, match value needs to be
		updated every WDT ISR with the desired time value added on top of the
		existing match value */
	CySysWdtWriteMatch(WATCHDOG_TIMER_COUNT);
	
	/* Enable the WDT interrupt in SRSS INTR mask register */
	CySysWdtUnmaskInterrupt();
	
	/* Map the WatchDog_ISR vector to the WDT_ISR */
	WDT_ISR_StartEx(WatchDog_ISR);	
}

/******************************************************************************
* Function Name: Output_Init
*******************************************************************************
*
* Summary:
*  Output_Init() performs following functions:
*  1: Starts PRSm
*  2: Starts Software UART TX component
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
void Output_Init(void)
{
	/* Start PRSm block */
	PRSm_Start();
	
	#ifdef TX_ENABLE
		/* Start software TX */
		TX_StartEx(TX_PORT, TX_PIN);
	#endif
}

/******************************************************************************
* Function Name: Input_Process
*******************************************************************************
*
* Summary:
*  Input_Process() performs following functions:
*  1: Scans the Proximity sensor.
*  2: Process the Proximity sensor signal - Apply filter, Update baseline
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
void Input_Process(void)
{
	/* Check if CapSense block is busy or not and enable the next scan accordingly */
	if(CapSense_IsBusy() == NOT_BUSY)
	{
		/* Enter sleep if number of scans without any activity crossed threshold */
		if((sleepTimerCount >= ENTER_SLEEP_COUNTS))
		{
			/* Put CapSense block to sleep */
			CapSense_Sleep();
			
			/* Put PRSm block to sleep*/
			PRSm_Sleep();
			
			/* Put device to sleep */
			CySysPmDeepSleep();
			
			/* Wakeup PRSm block once device is up */
			PRSm_Wakeup();
			
			/* Wakeup CapSense block */
			CapSense_Wakeup();
		}
		
			/* Update proximity sensor baseline */
			CapSense_UpdateSensorBaseline(PROXIMITY_SENSOR);
			
			/* Scan proximity sensor */
			CapSense_ScanSensor(PROXIMITY_SENSOR);		
		
	}
}

/******************************************************************************
* Function Name: Output_Process
*******************************************************************************
*
* Summary:
*  Output_Process() performs following functions:
*  1: Calculates the PRSm duty based on Proximity signal
*  2: Updates the duty to PRSm compare register and sends the data over TX
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
void Output_Process(void)
{
	/* Obtain the duty based on proximity sensor */
	calculatedDuty = PRSm_CalculateDuty(CapSense_SensorSignal[PROXIMITY_SENSOR]);	
	
	/* Write the calculated duty to the PRSm compare register */
	PRSm_WriteCompare(calculatedDuty);

	ucSlider = calculatedDuty/DOWNSCALE_FACTOR;
	if (ucSlider == 100)
	{
		ucARGB = 1;	
	}
	else
	{
		ucARGB = 0;	
	}
	
	/* Send data over UART if enabled */
	#ifdef	TX_ENABLE
		UART_SendTxData();
	#endif	
}

/******************************************************************************
* Function Name: PRSm_CalculateDuty
*******************************************************************************
*
* Summary:
*  PRSm_CalculateDuty() calculates and returns the PRSm duty based on signal value 
*	passed.
*
* Parameters:
*  uint16 signal - Signal value with which the limits are compared and 
*					PRSm duty is calculated.
*
* Return:
*  uint32 pwmDuty - Calculated duty value.
*
******************************************************************************/
uint32 PRSm_CalculateDuty(uint16 signal)
{
	uint32 pwmDuty;
	
	/* Check if sensor signal value is above PROXIMITY_LOWER_LIMIT */
	if(CapSense_SensorSignal[PROXIMITY_SENSOR] > PROXIMITY_LOWER_LIMIT)
	{
		/* Check if sensor signal value is above PROXIMITY_UPPER_LIMIT */
		if(CapSense_SensorSignal[PROXIMITY_SENSOR] > PROXIMITY_UPPER_LIMIT)
		{
			/* PRSm duty = MAX Brightness */
			pwmDuty = PRSm_PWM_PERIOD_VALUE + 1;
		}
		else
		{
			/* Calculate the duty relative to signal's lower limit and upper limit */			
			pwmDuty = 1 + ((PRSm_PWM_PERIOD_VALUE * (signal - PROXIMITY_LOWER_LIMIT )) /(PROXIMITY_UPPER_LIMIT - PROXIMITY_LOWER_LIMIT));			
		}
		
		/* Clear sleep timer count - Disable the interrupt to prevent race condition because of interrupt access */
		
		CyIntDisable(4);
		sleepTimerCount = 0;
		CyIntEnable(4);
	}
	else
	{
		/* If signal is below signal lower limit, turn OFF the PRSm (duty = 0) */
		pwmDuty = 1;
	}
	
	/* return the calculated duty */
	return(pwmDuty);
}

#ifdef TX_ENABLE
/******************************************************************************
* Function Name: UART_SendTxData
*******************************************************************************
*
* Summary:
*  UART_SendTxData() sends the sensor data over UART TX line
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
void UART_SendTxData(void)
{	
	/* Starting delimiter of a packet */
	TX_PutCRLF();
	
	/* Send debug data in MSB first fashion */
	TX_PutChar((uint8)(CapSense_rawFilterData1[PROXIMITY_SENSOR_INDEX]>>8));
	TX_PutChar((uint8)(CapSense_rawFilterData1[PROXIMITY_SENSOR_INDEX]));
	
	TX_PutChar((uint8)(CapSense_SensorBaseline[PROXIMITY_SENSOR_INDEX]>>8));
	TX_PutChar((uint8)(CapSense_SensorBaseline[PROXIMITY_SENSOR_INDEX]));
	
	TX_PutChar((uint8)(CapSense_SensorSignal[PROXIMITY_SENSOR_INDEX]>>8));
	TX_PutChar((uint8)(CapSense_SensorSignal[PROXIMITY_SENSOR_INDEX]));
	
	TX_PutChar((uint8)(calculatedDuty>>8));
	TX_PutChar((uint8)(calculatedDuty));
	
	/* End delimiter of a packet */
	TX_PutChar(0);
	TX_PutChar(0xFF);
	TX_PutChar(0xFF);
	
}
#endif
/******************************************************************************
* Function Name: WatchDog_ISR
*******************************************************************************
*
* Summary:
*  Watchdog Timer match ISR - Used for timing and sleep wake-up source
*
* Parameters:
*  None.
*
* Return:
*  None.
*
******************************************************************************/
CY_ISR(WatchDog_ISR)
{
	/* Increment the sleepTimerCount variable and make sure it doesn't overflow beyond ENTER_SLEEP_COUNTS */
	if(sleepTimerCount < ENTER_SLEEP_COUNTS)
	{
		sleepTimerCount++;
	}
	
	/* Update match register to generate a periodic ISR  */
	CySysWdtWriteMatch((uint16)CySysWdtReadMatch() + WATCHDOG_TIMER_COUNT);	
	
	/* Clear WDT to prevent WDT reset */
	CySysWdtClearInterrupt();
}

/* [] END OF FILE */
