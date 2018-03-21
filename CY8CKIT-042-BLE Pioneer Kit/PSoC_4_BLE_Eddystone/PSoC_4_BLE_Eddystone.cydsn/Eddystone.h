/******************************************************************************
* Project Name      : PSoC_4_BLE_Eddystone
* File Name         : Eddystone.h
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

#ifndef __EDDYSTONE_H__
#define __EDDYSTONE_H__

#include <project.h>
#include <stdbool.h>

/*******************************************************************************
* Macros and Constants
*******************************************************************************/
 /* 65535 = 2 seconds */
#define TWO_SECOND_INTERRUPT_COUNT      (0xFFFF)
/* Default UID/URL Frame timeout */
#define APP_UID_URL_TIMEOUT             (0x000A)
/* Default TLM Frame timeout */
#define APP_TLM_TIMEOUT                 (0x0001)
/* Minimum supported beacon period */
#define MIN_BEACON_PERIOD               (CYBLE_GAP_ADV_ADVERT_INTERVAL_NONCON_MIN * 0.625)
/* Maximum supported beacon period */
#define MAX_BEACON_PERIOD               (CYBLE_GAP_ADV_ADVERT_INTERVAL_MAX * 0.625)
/* Default beacon period */
#define DEFAULT_BEACON_PERIOD           (0x03E8)

typedef enum
{
    /* Eddystone not active */
    NO_EDDYSTONE,
    /* Eddystone UID adv */
    EDDYSTONE_UID,
    /* Eddystone URL adv */
    EDDYSTONE_URL,
    /* Eddystone TLM adv */
    EDDYSTONE_TLM
} EDDYSTONE_ROLE_T;

typedef enum
{
    /* Characteristics unlocked */
    UNLOCKED,
    /* Characteristics locked */
    LOCKED
} EDDYSTONE_CHAR_STATE_T;

/* Default URL flag value */
#define DEFAULT_URL_FLAG_VALUE          0x10

#define PWR_LVL_3_DBM                   (0x03)  /*   3 dBm */
#define PWR_LVL_0_DBM                   (0x00)  /*   0 dBm */
#define PWR_LVL_NEG_6_DBM               (0xFA)  /*  -6 dBm */
#define PWR_LVL_NEG_18_DBM              (0xEE)  /* -18 dBm */

/* Index of the Advertised TX Power Levels */
typedef enum
{
    TX_POWER_MODE_LOWEST,
    TX_POWER_MODE_LOW,
    TX_POWER_MODE_MEDIUM,
    TX_POWER_MODE_HIGH,

    /* Maximum number of power levels */
    MAX_NUM_PWR_LVL
} TX_POWER_MODE_T;

/* URL Scheme Prefix */
typedef enum
{
    HTTP_WWW,       /* http://www.  */
    HTTPS_WWW,      /* https://www. */
    HTTP,           /* http://      */
    HTTPS,          /* https://     */
    URL_PREFIX_MAX
} EDDYSTONE_URL_SCHEME_PREFIX_T;

/* Eddystone-URL HTTP URL encoding */
typedef enum
{
    DOT_COM_SLASH,      /* .com/  */
    DOT_ORG_SLASH,      /* .org/  */
    DOT_EDU_SLASH,      /* .edu/  */
    DOT_NET_SLASH,      /* .bet/  */
    DOT_INFO_SLASH,     /* .info/ */
    DOT_BIZ_SLASH,      /* .biz/  */
    DOT_GOV_SLASH,      /* .gov/  */
    DOT_COM,            /* .com   */
    DOT_ORG,            /* .org   */
    DOT_EDU,            /* .edu   */
    DOT_NET,            /* .bet   */
    DOT_INFO,           /* .info  */
    DOT_BIZ,            /* .biz   */
    DOT_GOV,            /* .gov   */
    RESERVED
} EDDYSTONE_URL_HTTP_ENCODING_T;

/* Default URL advertised */
#define DEFAULT_URL                     {                   \
                                            HTTP_WWW,       \
                                            'c',            \
                                            'y',            \
                                            'p',            \
                                            'r',            \
                                            'e',            \
                                            's',            \
                                            's',            \
                                            DOT_COM_SLASH   \
                                        }

#define DEFAULT_URL_LENGTH              9

/*******************************************************************************
* Function prototype
*******************************************************************************/
void UpdateTLMAdvPacket(void);
void EddystoneEventHandler(uint32 event, void* eventParam);

#endif  /*  __EDDYSTONE_H__ */
/* [] END OF FILE */
