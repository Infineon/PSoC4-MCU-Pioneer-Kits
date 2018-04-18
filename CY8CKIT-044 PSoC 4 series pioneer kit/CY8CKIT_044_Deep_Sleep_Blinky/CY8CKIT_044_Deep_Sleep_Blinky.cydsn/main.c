/******************************************************************************
* Project Name		: CY8CKIT_044_Deep_Sleep_Blinky
* Version			: 1.0
* Device Used		: CY8C4247AZI-M485     
* Software Used		: PSoC Creator 3.2
* Compiler Used		: ARM GCC 4.8.4 
* Related Hardware	: CY8CKIT-044 PSoC 4 M-Series Pioneer Kit 
*******************************************************************************
* Copyright (2015), Cypress Semiconductor Corporation.
*******************************************************************************
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
*******************************************************************************
* Theory of Operation: This project demonstrates Deep-Sleep power mode operation
* of PSoC 4200M device. The PSoC 4200M device is put to Deep-Sleep power mode 
* and wakes up every 1 second. The PSoC 4200M device wakes up from Deep-Sleep 
* power mode when the Watchdog Timer0 (WDT0) count reaches the match value. This 
* is configured in the "Clocks" tab of the file CY8CKIT_044_Deep_Sleep_Blinky.cydwr.
* The WDT0 is clocked using the Low Frequency Clock (LFCLK) which in turn is 
* sourced by ILO. The ILO can be inaccurate up to 60%. This project uses the 
* ILO Trim Component to trim the ILO to the accuracy of the IMO (+/-2%). 
*
* The project also has a pin Component which is connected to the green LED on the
* PSoC 4 M-Series Pioneer Kit. The PSoC 4200M toggles the state of this pin each time
* the device wakes up from Deep-Sleep power mode. This blinks the green LED at a
* frequency of 0.5Hz.
*
* Note: This project sets the "Debug Select" option in the System tab of 
* CY8CKIT_044_Deep_Sleep_Blinky.cydwr file as GPIO instead of SWD. This allows 
* the PSoC 4200M to save more power.
*******************************************************************************/
#include "project.h"

#define ZERO				0x00
#define DELAY_MS			1000
#define CLOCK_FOR_DELAY		32*DELAY_MS

/* 4 ILO periods expressed in microseconds. */
#define ILOX4 				((4 * 1000)/32)

int main()
{
	/* Variables used to trim ILO. */
	uint16 clocksComp = ZERO;
    uint16 currentCounterVal = ZERO;
	
	CyGlobalIntEnable;
	
	/* Start the ILO Trim Component. */
	ILO_Trim_Start();
	
    for(;;)
    {
		/* Read the pin state and then set it to the opposite value. */
		Pin_GreenLED_Write(!Pin_GreenLED_Read());
		
		/* Calculate the number of clock cycles required 
		 * to compensate for ILO error for a timer value of 1 second. */
		clocksComp = ILO_Trim_Compensate(CLOCK_FOR_DELAY);
		
		/* Update the Watchdog Timer with the compensated match value. */
        CySysWdtWriteMatch(CY_SYS_WDT_COUNTER0, clocksComp - 1);
		
		/* Read the current count value of the WDT. */
        currentCounterVal = CySysWdtReadCount(CY_SYS_WDT_COUNTER0);
		
		/* Reset the counter if the count value is greater than
		 * the match value. */
        if (currentCounterVal > (clocksComp - 1))
        {
            CySysWdtResetCounters(CY_SYS_WDT_COUNTER0_RESET);
        }
		
		/* Provide a delay to allow the changes on WDT register to take 
		 * effect. This is approximately 4 LFCLK cycles. */
		CyDelayUs(ILOX4);
			
		/* Enter Deep-Sleep power mode. The WDT is set to wakeup the
		 * device every 1s in the "Clock" tab of CY8CKIT_044_Deep_Sleep_Blinky.cydwr
		 * settings for WDT0. */
		CySysPmDeepSleep();	
    }
}

/* [] END OF FILE */
