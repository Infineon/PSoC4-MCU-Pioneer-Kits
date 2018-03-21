/******************************************************************************
* Project Name		: BLE_Dongle_Peripheral_IAS
* File Name			: BLEApplications.c
* Version 			: 1.0
* Device Used		: CYBL10162-56LQXI
* Software Used		: PSoC Creator 4.1
* Compiler    		: ARM GCC 5.4.1, ARM MDK Generic
* Related Hardware	: CySmart USB Dongle (part of CY8CKIT-042-BLE Bluetooth Low 
*                     Energy Pioneer Kit) 
* Owner				: ROIT
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

#include <main.h>

/* This variable stores the connection handle value after connection is made */
CYBLE_CONN_HANDLE_T  connectionHandle;

/*******************************************************************************
* Function Name: GenericAppEventHandler
********************************************************************************
*
* Summary:
*  This is an event callback function to receive events from the BLE Component.
*
* Parameters:  
*  uint8 event:       Event from the BLE component
*  void* eventParams: A structure instance for corresponding event type. The 
*                     list of event structure is described in the component 
*                     datasheet.
*
* Return: 
*  None
*
*******************************************************************************/
void GenericAppEventHandler(uint32 event, void *eventParam)
{
    switch(event)
	{
    /**********************************************************
    *                       General Events
    ***********************************************************/
	case CYBLE_EVT_STACK_ON: 
		/* Start BLE Advertisement after BLE Stack is ON */
		CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
		
		break;
        
    /**********************************************************
    *                       GAP Events
    ***********************************************************/
    case CYBLE_EVT_GAP_DEVICE_DISCONNECTED:
        /* Put the device to discoverable mode so that remote can search it. */
        CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
		
		/* Stop PWM as no LED status is required*/
		LED_PWM_Stop();
		
		/* Set drive mode of Alert LED pin to High-Z to shutdown LED */
		Alert_LED_SetDriveMode(Alert_LED_DM_ALG_HIZ);
        break;
    /**********************************************************
    *                       GATT Events
    ***********************************************************/
    case CYBLE_EVT_GATT_CONNECT_IND:
		/* Extract connection handle */
        connectionHandle = *(CYBLE_CONN_HANDLE_T *)eventParam;	
		
		/* Start PWM for LED status control */
		LED_PWM_Start();
        break;

    default:
        break;
	}
}

/*******************************************************************************
* Function Name: IasServiceAppEventHandler
********************************************************************************
*
* Summary:
*  This is an event callback function to receive events from the BLE Component,
*  which are specific to Immediate Alert Service.
*
* Parameters:  
*  uint8 event:       Write Command event from the BLE component.
*  void* eventParams: parameter to the respective event
*
* Return: 
*  None
*
*******************************************************************************/
void iasEventHandler(uint32 event, void *eventParam)
{
	/* Local variables */
    uint8 alertLevel;
    CYBLE_IAS_CHAR_VALUE_T *iasWrCmdValueParam;
    
    if(event == CYBLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Check if the event received was for writing Alert level */
        iasWrCmdValueParam = (CYBLE_IAS_CHAR_VALUE_T *)eventParam;
            
		/* If size of received attribute is equal to size of Alert level data */
        if(CYBLE_IAS_ALERT_LEVEL_SIZE == iasWrCmdValueParam->value->len)
        {
			/* Extract the alert level value received from Client device*/
            alertLevel = *((iasWrCmdValueParam->value->val));
            
			/* Switch LED status depending on Alert level received */
            switch (alertLevel)
    		{
	    		case CYBLE_NO_ALERT:
						Alert_LED_SetDriveMode(Alert_LED_DM_ALG_HIZ);
						LED_PWM_WriteCompare(LED_NO_ALERT);
	    			break;
	    			
	    		case CYBLE_MILD_ALERT:
						Alert_LED_SetDriveMode(Alert_LED_DM_STRONG);
						LED_PWM_WriteCompare(LED_MILD_ALERT);
	    			break;
	    		
	    		case CYBLE_HIGH_ALERT:
						Alert_LED_SetDriveMode(Alert_LED_DM_STRONG);
						LED_PWM_WriteCompare(LED_HIGH_ALERT);
	    			break;
	    			
	    		default:
	    		    break;
    		}
        }
    }
}

/* [] END OF FILE */
