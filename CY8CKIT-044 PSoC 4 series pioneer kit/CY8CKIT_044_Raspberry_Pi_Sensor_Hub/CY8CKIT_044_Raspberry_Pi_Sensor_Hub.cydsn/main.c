/*****************************************************************************
* Project Name		: CY8CKIT_044_Raspberry_Pi_Sensor_Hub
* Version			: 1.0
* Device Used		: CY8C4247AZI-M485     
* Software Used		: PSoC Creator 4.2
* Compiler Used		: ARM GCC 5.4.1 
* Related Hardware	: CY8CKIT-044 PSoC 4 M-Series Pioneer Kit 
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
/********************************************************************************
* Theory of Operation: The PSoC 4 M-Series Pioneer Kit has an onboard accelerometer,
* an ambient light sensor, and a PWM temperature sensor. This project collects 
* data from all these sensors and stores the data in global variables. The project
* also has a Real Time Clock (RTC) which is used to keep track of time. 
* The PSoC 4200M stores the latest sensor data (temperature and acceleration)
* along with the timestamp data collected from the RTC to the onboard F-RAM every 
* one second. The data is stored in the order : [hours, minutes, seconds, 
* temperature, x axis acceleration, y axis acceleration, z axis acceleration]. 
* The same data is read back every one second and sent through UART along with the 
* location information of data in F-RAM. The data sent through UART can be directly 
* accessed from a Raspberry Pi connected to the Raspberry Pi header on CY8CKIT-044.
*
* The accelerometer is configured to generate an interrupt whenever an 
* acceleration is detected. PSoC 4200M reads the acceleration data when 
* accelerometer triggers the interrupt. The acceleration data, ambient light 
* sensor data, and the temperature value are stored in global variables and can
* be accessed using the uC/Probe tool. 
* Note that the ambient light sensor data is logged to the global variable
* "uC_Probe_ambientLight" automatically using the PSoC 4200M DMA. 
*
* This project also allows the uC/Probe tool to change the color of onboard RGB
* LED according to the value set by RGB color palette in the uC/Probe.
*****************************************************************************/
#include "project.h"
#include "main.h"
#include "accelerometer.h"
#include "FRAM_Control.h"
#include "stdio.h"

/*****************************************************************************
* Local Function Prototypes
*****************************************************************************/
static void UpdateLEDColor(void);
CY_ISR_PROTO(Accelerometer_Interrupt);

/*****************************************************************************
* Global Variable Declarations
*****************************************************************************/
/************************************ 
 * Micrium uC/Probe Global Variables
************************************/
int16 uC_Probe_ambientLight = ZERO;
int8 uC_Probe_temperatureInteger = ZERO;
int16 uC_Probe_x_dirMovement = ACC_NO_MOVEMENT;
int16 uC_Probe_y_dirMovement = ACC_NO_MOVEMENT;
int16 uC_Probe_z_dirMovement = ACC_NO_MOVEMENT;
uint32 uC_Probe_RGBPalette = WHITE;
uint8 uC_Probe_Hours = ZERO;
uint8 uC_Probe_Minutes = ZERO;
uint8 uC_Probe_Seconds = ZERO;

/**********************************/
uint8 temperatureFractional = ZERO;
uint8 acclerometer_configured = FALSE;
uint8 accRequest = FALSE;
uint8 oneSecondTrigger = FALSE;
uint8 displayErrorOnce = FALSE;

int main()
{
    /* Local Variables */
	uint8 tempError = FALSE;
	int16 temperatureReturnVal = ZERO;
	uint32 ledColorVal = ZERO;
	char uartString[MAX_STRING_SIZE] = {' '};

	/* Dummy variable to read the interrupt release register of accelerometer. */
	uint8 dummy = ZERO;
	
	/* Start and initialize all the peripherals. Enable the global interrupt. */
	Initialize_Project();
	
	/* Initialize the accelerometer after a self-test. */
	InitializeAccelerometer();

    for(;;)
    {
     	/* accRequest is set to TRUE when accelerometer generates an interrupt.
		 * Configured in isr_Accelerometer */
		if(accRequest == TRUE && acclerometer_configured == TRUE && stopDataAccess == FALSE)
		{
			accRequest = FALSE;
			
			/* Read the accelerometer data and update the global variables. */
			ReadAccelerometer(&uC_Probe_x_dirMovement, &uC_Probe_y_dirMovement, &uC_Probe_z_dirMovement);	
			
			/* Read interrupt release register to clear the latched interrupt. */
			Accelerometer_RegRead(ACC_INT_REL, &dummy);
		}
		
		/* Check if temperature measurement is complete. */
		tempError = Temperature_Sense_ConversionStatus();
		
		/* If conversion is complete, update the temperature variable. */
		if((tempError & Temperature_Sense_STATUS_COMPLETE) == Temperature_Sense_STATUS_COMPLETE)
		{
			/* Read the temperature value. */
			temperatureReturnVal = Temperature_Sense_GetTemperature(DEFAULT_SENSOR);
			
			/* Temperature measured is in 1/100th of a degree Celsius. 
			 * Scale the value for comfortable viewing in uC/Probe. */
			uC_Probe_temperatureInteger = temperatureReturnVal/DIVISOR_HUNDRED; 
			
			/* Find the fractional part of temperature data for logging in the F-RAM. */
			if(temperatureReturnVal < ZERO)
			{
				temperatureFractional = DIVISOR_HUNDRED - temperatureReturnVal%DIVISOR_HUNDRED; 
			}
			else
			{
				temperatureFractional = temperatureReturnVal%DIVISOR_HUNDRED; 
			}
			
			/* Trigger next conversion when the current conversion is complete. */
			Temperature_Sense_Trigger();
		}
		
		/* Update the PWMs if the global variable controlled by uC/Probe has changed. */
		if(ledColorVal != uC_Probe_RGBPalette)
		{
			/* Save the current value to the local variable. */
			ledColorVal = uC_Probe_RGBPalette;
			
			/* Update the LED brightness corresponding to the changes in uC/Probe interface. */
			UpdateLEDColor();
		}
		
		/* Update the F-RAM with sensor data every one second. Update the RTC */
		if(oneSecondTrigger == TRUE && stopDataAccess == FALSE)
		{
			oneSecondTrigger = FALSE;
			FRAM_SaveSensorData();
			FRAM_ReadSensorData();
		}
		
		if(displayErrorOnce == TRUE)
		{
			sprintf(uartString, "\n \rI2C Arbitration lost. Data logging stopped.");
			UART_Comm_UartPutString(uartString);
			
			sprintf(uartString, "\n \rPress SW1 to reset the device.");
			UART_Comm_UartPutString(uartString);
			displayErrorOnce = FALSE;
		}
    }
}

/*******************************************************************************
* Function Name: Initialize_Project
********************************************************************************
* Summary:
*  Starts and initializes all the Components in the project. Enables 
   global interrupt. 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Initialize_Project(void)
{
	/* Enable global interrupt. */
	CyGlobalIntEnable; 

	/* Start the Opamp hardware. */
	Opamp_Start();
	
	/* Enable and start ADC block. */
	ADC_Start();
	
	/* Enable the DMA Component. This API also sets the 
	 * source and destination addresses. */
	DMA_Start((void *)ADC_SAR_CHAN0_RESULT_PTR, (void *)&uC_Probe_ambientLight);
	
	/* Start ADC conversions. */
	ADC_StartConvert();
	
	/* Enable and start accelerometer I2C block. */	
	I2C_Start();
	
	/* Enable the interrupt from accelerometer. */
	isr_Accelerometer_Interrupt_StartEx(Accelerometer_Interrupt);
	
	/* Start the TMP05 temperature sensor Component. */
	Temperature_Sense_Start();
	
	/* Trigger the temperature sensor Component to start measurement. */
	Temperature_Sense_Trigger();
	
	/* Enable and start the PWM blocks. */
	PWM_Red_Start();
	PWM_Green_Start();
	PWM_Blue_Start();
	
	/* Enable and start UART communication block. */
	UART_Comm_Start();
	
	/* Start RTC. The RTC is configured to use the Watch Dog Timer0. */
	RTC_Start();    

	/* Link the WatchDog Timer Isr to the function which updates F-RAM. 
	 * This will allow the F-RAM to be updated as per the settings in the
	 * CY8CKIT_044_Raspberry_Pi_Sensor_Hub.cydwr. The WatchDog Timer2 is 
	 * configured to generate an interrupt every 1 second. */
	CySysWdtSetIsrCallback(CY_SYS_WDT_COUNTER2, SetTriggerFlag);
	
	/* Wait for 50ms for the accelerometer to start up. */
	CyDelay(ACC_STARTUP_TIME);
}

/*******************************************************************************
* Function Name: UpdateLEDColor
********************************************************************************
* Summary:
* 	Updates the LED color based on the uC/Probe color palette value.
*
* Parameters:
*  void
*
* Return:
* void
*
*******************************************************************************/
static void UpdateLEDColor(void)
{	
	uint8 alpha = ZERO;
	uint8 red = ZERO, green = ZERO, blue = ZERO;
	uint16 pwm_red = ZERO, pwm_green = ZERO, pwm_blue = ZERO;
	
	/* Convert the 32-bit Red, Green, Blue, Alpha (RGBA) 
	 * value to individual 8-bit numbers. */
	alpha = HI8(HI16(uC_Probe_RGBPalette));
	red = LO8(HI16(uC_Probe_RGBPalette));
	green = HI8(LO16(uC_Probe_RGBPalette));
	blue = LO8(LO16(uC_Probe_RGBPalette));
	
	/* Convert the RGBA value received from uC/Probe to RGB value. */
	pwm_red = (((0xFF - alpha) * 0xFF) + (alpha * red)) >> BRIGHTNESS_SATURATION_SHIFT;
	pwm_green = (((0xFF - alpha) * 0xFF) + (alpha * green)) >> BRIGHTNESS_SATURATION_SHIFT;
	pwm_blue = (((0xFF - alpha) * 0xFF) + (alpha * blue)) >> BRIGHTNESS_SATURATION_SHIFT;
	
	/* Update the LED brightness. */
	PWM_Red_WriteCompare(pwm_red);	
	PWM_Green_WriteCompare(pwm_green);	
	PWM_Blue_WriteCompare(pwm_blue);	
}

/*******************************************************************************
* Function Name: SetTriggerFlag
********************************************************************************
* Summary:
* 	Sets the global variable oneSecondTrigger to TRUE. This function is called 
*   every one second automatically by the WDT interrupt.
*
* Parameters:
* void
*
* Return:
* void
*
*******************************************************************************/
void SetTriggerFlag(void)
{
	oneSecondTrigger = TRUE;
}

/*******************************************************************************
* Function Name: isr_Accelerometer_Interrupt_Interrupt
********************************************************************************
* Summary:
* 	Interrupt routine for accelerometer interrupt. Sets the accRequset flag,
*   clears the GPIO interrupt and updates the RTC.
*
* Parameters:
* void
*
* Return:
* void
*
*******************************************************************************/
CY_ISR(Accelerometer_Interrupt)
{
   	accRequest = TRUE;
	Pin_Accelerometer_Interrupt_ClearInterrupt();
}

/* [] END OF FILE */

