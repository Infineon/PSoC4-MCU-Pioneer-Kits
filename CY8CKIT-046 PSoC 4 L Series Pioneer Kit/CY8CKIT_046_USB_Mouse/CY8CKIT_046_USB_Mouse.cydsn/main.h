/*****************************************************************************
* File Name		: main.h
* Version		: 1.0 
*
* Description:
*  This file contains the function prototypes and constants used in
*  main.c
*
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
* Cypress reserves the Right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*******************************************************************************/

#ifndef MAIN_H		/* Guard to prevent multiple inclusions */
    #define MAIN_H
    	
    #include "cytypes.h"

    /*******************************************************************************
    * Macro Definitions
    *******************************************************************************/

    /* Constants used to enable/disable tuner */
    #define ENABLED                     (1)
    #define DISABLED                    (0)    
    #define TUNER_ENABLE                (DISABLED) 

    /* Enables or disables sending debug data over UART.     
     */
    #define TXDEBUG
    
    #ifdef TXDEBUG
        #define ENABLE_UART_DBG_OUTPUT			(1)
    #else
        #define ENABLE_UART_DBG_OUTPUT			(0)
    #endif
    	
    #if(ENABLE_UART_DBG_OUTPUT)
    	#define PRINT	UART_UartPutString
    #else
    	#define PRINT(x)	
    #endif		

    /* Defines the IN end point number for the keyboard interface. */
    #define KEYBOARD_END_POINT				(1)

    /* Defines the number of bytes used to report keyboard data over USB.
     */
    #define KEY_RPT_SIZE					(8)

    /* Defines the IN end point number for the mouse interface. */
    #define MOUSE_END_POINT					(2)

    /* Defines the number of bytes used to report mouse data over USB. */
    #define MOUSE_RPT_SIZE					(4)

    #define MOUSE_DATA_SENT					(1)
    #define KEYBOARD_DATA_SENT				(2)
    #define USB_HID_DATA_SENT				(MOUSE_DATA_SENT | KEYBOARD_DATA_SENT)
    #define USB_HID_DATA_SENT_FLAG_CLEAR	(0)


    /*******************************************************************************
    *   Function Prototypes
    *******************************************************************************/

    void Print(const char8 string[]);

#endif /* #ifndef MAIN_H */


/* [] END OF FILE */
