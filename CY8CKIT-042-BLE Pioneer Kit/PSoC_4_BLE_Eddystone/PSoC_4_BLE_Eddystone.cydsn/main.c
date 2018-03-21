/******************************************************************************
* Project Name      : PSoC_4_BLE_Eddystone
* File Name         : main.c
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

/*******************************************************************************
*                           THEORY OF OPERATION
********************************************************************************
* This project implements a BLE beacon based on Google’s Eddystone protocol.
* To view the beacon on an android device use Locate Beacon (or any other beacon
* scanning app).
* Refer CY8CKIT-042-BLE Pioneer Kit user guide for more details.
********************************************************************************
* Hardware connection required for testing -
* SARMUX[0] - P3[0] (Connect this pin from J2 header to VREF on J3 header in the
*                    BLE Pioneer Kit)
*******************************************************************************/
#include <project.h>
#include <stdbool.h>
#include "WatchdogTimer.h"
#include "main.h"
#include "Eddystone.h"
#include "Temperature.h"
#include "Battery.h"

/*******************************************************************************
* Global Variable Declaration
*******************************************************************************/
/* This counter keeps track of total number of advertisement packets
*  transmitted. */
static uint32  advPacketCount = 0;

/* This flag when true stops BLE for advertising. */
static bool stopAdv = false;

/* To decide whether to have connectable or non-connectable adv */
static bool startConnAdv = false;

/*******************************************************************************
* Function Name: interruptHandlerSW2
********************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for SW2.
*
*******************************************************************************/
CY_ISR(interruptHandlerSW2)
{
    /* Clear the button interrupt */
    SW2_ClearInterrupt();
    /* Set the stop advertisment flag */
    stopAdv = true;
    /* Set the start connection advertisement flag */
    startConnAdv = true;
}

/*******************************************************************************
* Function Name: GetMeasuredBatteryVoltage
********************************************************************************
*
* Summary:
*   This function returns if connectable adv needs to be started.
*
* Parameters:
*  None
*
* Return:
*  bool: 0 - do not start connectable adv
*        1 - start connectable adv
*
*******************************************************************************/
bool IsConnAdvStart(void)
{
    /* Save the flag */
    bool temp = startConnAdv;

    /* Reset the flag */
    startConnAdv = false;

    /* Return the saved flag state */
    return temp;
}

/*******************************************************************************
* Function Name: WCO_ECO_LowPowerStart
********************************************************************************
*
* Summary:
*   This function is used to handle the WCO and ECO clock during the
*   stabilization process.
*
* Parameters:
*   none
*
* Return:
*   none
*******************************************************************************/
void WCO_ECO_LowPowerStart(void)
{
    /* Setup counter0 for WCO startup */
    WDT_Initialize(CY_SYS_WDT_COUNTER0, WCO_STARTUP_PERIOD);

    /* Start the WCO clock */
    CySysClkWcoStart();

    /* WCO takes 500ms to stabilize. Enable counter 0 to trigger an interrupt
    *  after 500ms. This helps to keep the device in low power mode for 500ms.
    */
    WDT_EnableCounter(CY_SYS_WDT_COUNTER0_MASK);

    /* Put the system in deepsleep and Wait for the WDT counter 0 interrupt to
    *  wake up the device. On wakeup WCO is up & running */
    CySysPmDeepSleep();

    /*Unlock the WDT registes*/
    CySysWdtUnlock();

    /* Switch WCO to the low power mode after startup */
    CySysClkWcoSetPowerMode(CY_SYS_CLK_WCO_LPM);

    /* LFCLK is now driven by WCO */
    CySysClkSetLfclkSource(CY_SYS_CLK_LFCLK_SRC_WCO);

    /* WCO is running, shut down the ILO */
    CySysClkIloStop();

    /*Lock the WDT registes*/
    CySysWdtLock();

    /* Disable counter 0 */
    WDT_DisableCounter(CY_SYS_WDT_COUNTER0_MASK);

    /* Setup counter0 for ECO startup */
    WDT_Initialize(CY_SYS_WDT_COUNTER0, ECO_STARTUP_PERIOD);

    /* WCO is stabliized and It's time to start ECO */
    CySysClkEcoStart(0);

    /* ECO takes ~3msec to stabilize. Enable counter 1 to trigger an interrupt
    *  after 3.5msec. This helps to keep the device in low power mode for
    *  3.5msec and don't let PMIC to discharge. If we don't do this, PMIC drains
    *  out */

    /* Enable WDT's  counter 0 to generate an interrupt after 3.5 mseconds */
    WDT_EnableCounter(CY_SYS_WDT_COUNTER0_MASK);

    /* Put the system in deepsleep and Wait for the WDT counter 0
    *  interrupt to wake up the device. On wakeup ECO is up & running */
    CySysPmDeepSleep();

    /* Both ECO and WCO are stabilized. Disable the counter 0 */
    WDT_DisableCounter(CY_SYS_WDT_COUNTER0_MASK);
}

/*******************************************************************************
* Function Name: Initialization
********************************************************************************
*
* Summary:
*   This function is used to intialize all blocks of the application
*
* Parameters:
*   none
*
* Return:
*   none
*******************************************************************************/
void Initialization(void)
{
    /* Set the divider for ECO, ECO will be used as source when IMO is switched
    *  off to save power, to drive the HFCLK */
    CySysClkWriteEcoDiv(CY_SYS_CLK_ECO_DIV8);

    /* Do the following for achieving lowest possible WCO & ECO startup current:
    *  1. Shut down the ECO (to reduce power consumption while WCO is starting)
    *  2. Enable WDT counter 0 to wakeup the system after 500ms
    *     (500ms = WCO startup time)
    *  3. Configure PSoC 4 BLE device in DeepSleep mode for the 500ms WCO
    *     startup time
    *  4. After WCO is enabled, restart the ECO so that BLESS interface can
    *     function
    *  5. Enable WDT counter 1 to wakeup the system after 1ms
    *     (1ms = ECO startup time)
    *  5. Configure PSoC 4 BLE device in DeepSleep mode for the 1ms ECO startup
    *     time */

    /* Shutdown the ECO and later re-start in low power mode after WCO is turned
    *  on. */
    CySysClkEcoStop();

    /* Initialize WDT interrupt */
    WDT_Interrupt_StartEx(WDT_Handler);

    /* Enable WCO & ECO in low power mode using WDT counter 0/1 as system wakeup
    *  sources respectively */
    WCO_ECO_LowPowerStart();

    /* Start BLE component and register the EddystoneEventHandler function. This
    *  function exposes the events from BLE component for application use. */
    CyBle_Start(EddystoneEventHandler);

    cyBle_attValuesLen[16].actualLength = DEFAULT_URL_LENGTH;

    /* Start the ADC component for temperature and battery measurement. */
    ADC_Start();

    /* Put ADC to sleep and wake it up onlu formeasurements. */
    ADC_Sleep();

    /* Start and register the SW2 ISR. */
    ConnectionAdv_ISR_StartEx(interruptHandlerSW2);
}

/*******************************************************************************
* Function Name: LowPower
********************************************************************************
*
* Summary:
*   This function is used to set the BLESS and CPU in low power mode in between
*   the advertising/connection intervals
*
* Parameters:
*   none
*
* Return:
*   none
*******************************************************************************/
void LowPower(void)
{
    CYBLE_LP_MODE_T pwrState;
    CYBLE_BLESS_STATE_T blessState;
    uint8 intStatus = 0;

    /* Configure BLESS in Deep-Sleep mode */
    pwrState = CyBle_EnterLPM(CYBLE_BLESS_DEEPSLEEP);

    /* No interrupts allowed while entering system low power modes */
    intStatus = CyEnterCriticalSection();

    /* Get current BLE state. */
    blessState = CyBle_GetBleSsState();

    /* Make sure BLESS is in Deep-Sleep before configuring system in Deep-Sleep
    */
    if(pwrState == CYBLE_BLESS_DEEPSLEEP)
    {
        if(blessState == CYBLE_BLESS_STATE_ECO_ON || blessState ==
                                                    CYBLE_BLESS_STATE_DEEPSLEEP)
        {
            /* System Deep-Sleep. 1.3uA mode */
            CySysPmDeepSleep();
        }
    }
    else if (blessState != CYBLE_BLESS_STATE_EVENT_CLOSE)
    {
        /* Change HF clock source from IMO to ECO, as IMO can be stopped to save
        *  power. */
        CySysClkWriteHfclkDirect(CY_SYS_CLK_HFCLK_ECO);

        /* Stop IMO for reducing power consumption */
        CySysClkImoStop();

        /* Put the CPU to Sleep. 1.1mA mode */
        CySysPmSleep();

        /* Starts execution after waking up, start IMO */
        CySysClkImoStart();

        /* Change HF clock source back to IMO */
        CySysClkWriteHfclkDirect(CY_SYS_CLK_HFCLK_IMO);
    }

    CyExitCriticalSection(intStatus);
}

/*******************************************************************************
* Function Name: GetAdvPacketCount
********************************************************************************
*
* Summary:
*   This function returns value of advPacketCount.
*
* Parameters:
*   none
*
* Return:
*   uint32: returns value of advPacketCount
*******************************************************************************/
uint32 GetAdvPacketCount(void)
{
    return advPacketCount;
}

/*******************************************************************************
* Function Name: SetAdvPacketCount
********************************************************************************
*
* Summary:
*   This function sets value of advPacketCount.
*
* Parameters:
*   uint32: returns value of advPacketCount
*
* Return:
*   none
*******************************************************************************/
void SetAdvPacketCount(uint32 value)
{
    advPacketCount =  value;
}

/*******************************************************************************
* Function Name: IncrementAdvPacketCount
********************************************************************************
*
* Summary:
*   This function keeps track of adv packets based on BLESS states.
*
* Parameters:
*   none
*
* Return:
*   none
*******************************************************************************/
void IncrementAdvPacketCount(void)
{
    static bool advOnGoing = false;
    CYBLE_BLESS_STATE_T blessState;

    /* Get current BLE state. */
    blessState = CyBle_GetBleSsState();

    if(CyBle_GetState() == CYBLE_STATE_ADVERTISING)
    {
        if (    (blessState == CYBLE_BLESS_STATE_ACTIVE) ||
                (blessState == CYBLE_BLESS_STATE_ECO_ON) ||
                (blessState == CYBLE_BLESS_STATE_ECO_STABLE))
        {
            advOnGoing = true;
        }
        else
        {
            /* Counts the number of packets that were advertised since power on.
            *  It misses the count(by 1) when advertisement is stopped and
            *  started again to change the frames. Increment advPacketCount only
            *  when an advertisement is completed. */
            if (advOnGoing == true)
            {
                /* Increament advertisment packet count. */
                advPacketCount++;

                if(advPacketCount == 0)
                {
                    /* SecCnt should be zero if advPacketCount is reset */
                    SetSecCnt(0);
                }
            }
            advOnGoing = false;
        }
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  Main function.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
int main()
{
    /* Enable global interrupt mask */
    CyGlobalIntEnable;

    /* This function will initialize the system resources such as BLE and ADC */
    Initialization();

    while (1)
    {
        /* BLE stack processing state machine interface */
        CyBle_ProcessEvents();

        /* Process BLESS states */
        IncrementAdvPacketCount();

        /* Update the TLM Adv packets with "advPacketCount" and
        *  "SecCnt" */
        UpdateTLMAdvPacket();

        /* Put CPU and BLESS to low power mode */
        LowPower();

        if(stopAdv && (CyBle_GetState() == CYBLE_STATE_ADVERTISING))
        {
            /* Reset the stop advertisement flag. */
            stopAdv = false;
            /* Stop advertisement. */
            CyBle_GappStopAdvertisement();
        }
    }
}

/* [] END OF FILE */
