/*****************************************************************************
* Project Name		: CY8CKIT_044_Accelerometer
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
/*******************************************************************************
* Theory of Operation: The PSoC 4 M-Series Pioneer Kit has an onboard accelerometer
* device. The accelerometer is configured to generate an interrupt whenever an 
* acceleration is detected. PSoC 4200M reads the acceleration data when 
* accelerometer triggers the interrupt. The onboard RGB LED color is modulated
* to reflect the direction of the motion as given below. 
* ___________________________________________________________________________
* Direction of Motion 	|  RGB LED Color									|
* ---------------------------------------------------------------------------
* x-direction			|  RED												|
* y-direction			|  GREEN											|
* z-direction			|  BLUE												|
* ---------------------------------------------------------------------------
* The intensity of each of the colors is directly proportional to the magnitude
* of acceleration experienced in the corresponding axis.
*
* The PSoC 4200M device remains in Sleep low power mode until the accelerometer 
* triggers an interrupt. The PSoC 4200M returns to Sleep mode after changing 
* the RGB LED color.
*****************************************************************************/
#include "project.h"
#include "main.h"
#include "accelerometer.h"

/*****************************************************************************
* Local Function Prototypes
*****************************************************************************/
CY_ISR_PROTO(Accelerometer_Interrupt);

/*****************************************************************************
* Global Variable Declarations
*****************************************************************************/
/* Variables to store the acceleration data in 3 dimensions. */
int16 uC_Probe_x_dirMovement = ACC_NO_MOVEMENT;
int16 uC_Probe_y_dirMovement = ACC_NO_MOVEMENT;
int16 uC_Probe_z_dirMovement = ACC_NO_MOVEMENT;

/* If the accelerometer is configured, this variable is set to TRUE. */
uint8 acclerometer_configured = FALSE;

/* This global variable is set every time the accelerometer triggers an interrupt. */
uint8 accRequest = FALSE;

int main()
{
	/* Dummy Variable to read from the interrupt release register of accelerometer. */
	uint8 dummy = ZERO;

	/* Start all the Components and enable the global interrupt. */
	Initialize_Project();
	
	/* Initialize the accelerometer after a self-test. */
	InitializeAccelerometer();

    for(;;)
    {
     	/* accRequest is set to TRUE when accelerometer generates an interrupt.
		 * Configured in isr_Accelerometer. */
		if(accRequest == TRUE && acclerometer_configured == TRUE)
		{
			/* Reset the flag. */
			accRequest = FALSE;
			
			/* Read the accelerometer data. */
			ReadAccelerometer(&uC_Probe_x_dirMovement, &uC_Probe_y_dirMovement, &uC_Probe_z_dirMovement);
			
			/* Release the latched interrupt. */
			Accelerometer_RegRead(ACC_INT_REL, &dummy);
			
			/* Update the LED color based on the acceleration data. */
			UpdateLEDColor();
		}

		/* Put the CPU to sleep until the accelerometer wakes up the device. */
		CySysPmSleep();
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

	/* Enable and start accelerometer I2C block. */	
	I2C_Start();
	
	/* Enable the interrupt from accelerometer. */
	isr_Accelerometer_Interrupt_StartEx(Accelerometer_Interrupt);
	
	/* Enable and start the PWM blocks. */
	PWM_Red_Start();
	PWM_Green_Start();
	PWM_Blue_Start();
	
	/* Switch off all LEDs. */
	PWM_Red_WriteCompare(OFF);
	PWM_Green_WriteCompare(OFF);
	PWM_Blue_WriteCompare(OFF);
	
	/* Wait for 700ms for the accelerometer to start up. */
	CyDelay(ACC_STARTUP_TIME);
}

/*******************************************************************************
* Function Name: UpdateLEDColor
********************************************************************************
* Summary:
* 	Updates the LED color based on the accelerometer value.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void UpdateLEDColor(void)
{
	/* Variables to store previous state between function calls. */
	static int16 prev_x_value = ZERO;
	static int16 prev_y_value = ZERO;
	static int16 prev_z_value = ZERO;	
	
	/* Variables to calculate the absolute acceleration difference. */
	int16 diffVal1 = ZERO;
	int16 diffVal2 = ZERO;
	uint16 positiveDiffVal = ZERO;
	
	/* Determine the acceleration difference and store the current 
	 * acceleration data in the static variable. */
	diffVal1 = uC_Probe_x_dirMovement - prev_x_value;
	diffVal2 = prev_x_value - uC_Probe_x_dirMovement;
	prev_x_value = uC_Probe_x_dirMovement;
	
	/* Determine the absolute value. */
	if(diffVal1 > diffVal2)
	{
		positiveDiffVal = diffVal1;
	}
	else
	{
		positiveDiffVal = diffVal2;
	}
	
	/* Modulate the PWM corresponding to the magnitude of acceleration data
	 if acceleration is greater than threshold value. */
	if(positiveDiffVal > ACC_THRESHOLD_VALUE)
	{
		PWM_Red_WriteCompare(positiveDiffVal << BRIGHTNESS_MULTIPLIER);
	}
	else
	{
		PWM_Red_WriteCompare(LOW);
	}
	
	/* Determine the acceleration difference and store the current 
	 * acceleration data in the static variable. */
	diffVal1 = uC_Probe_y_dirMovement - prev_y_value;
	diffVal2 = prev_y_value - uC_Probe_y_dirMovement;
	prev_y_value = uC_Probe_y_dirMovement;
	
	/* Determine the absolute value. */
	if(diffVal1 > diffVal2)
	{
		positiveDiffVal = diffVal1;
	}
	else
	{
		positiveDiffVal = diffVal2;
	}
	
	/* Modulate the PWM corresponding to the magnitude of acceleration data
	 if acceleration is greater than threshold value. */
	if(positiveDiffVal > ACC_THRESHOLD_VALUE)
	{
		PWM_Green_WriteCompare(positiveDiffVal << BRIGHTNESS_MULTIPLIER);
	}
	else
	{
		PWM_Green_WriteCompare(LOW);
	}
	
	/* Determine the acceleration difference and store the current 
	 * acceleration data in the static variable. */
	diffVal1 = uC_Probe_z_dirMovement - prev_z_value;
	diffVal2 = prev_z_value - uC_Probe_z_dirMovement;
	prev_z_value = uC_Probe_z_dirMovement;
	
	/* Determine the absolute value. */
	if(diffVal1 > diffVal2)
	{
		positiveDiffVal = diffVal1;
	}
	else
	{
		positiveDiffVal = diffVal2;
	}
	
	/* Modulate the PWM corresponding to the magnitude of acceleration data
	 if acceleration is greater than threshold value. */
	if(positiveDiffVal > ACC_THRESHOLD_VALUE)
	{
		PWM_Blue_WriteCompare(positiveDiffVal << BRIGHTNESS_MULTIPLIER);
	}
	else
	{
		PWM_Blue_WriteCompare(LOW);
	}
}

/*******************************************************************************
* Function Name: isr_Accelerometer_Interrupt
********************************************************************************
* Summary:
* 	Interrupt routine for accelerometer interrupt. Sets the accRequset flag and 
*   clears the GPIO interrupt.
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
