/******************************************************************************
* Project Name      : PSoC_4_BLE_Eddystone
* File Name         : Eddystone.c
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

#include <project.h>
#include <stdbool.h>
#include <string.h>
#include "Eddystone.h"
#include "WatchdogTimer.h"
#include "main.h"
#include "Temperature.h"
#include "Battery.h"

#define LOCK_CODE_LENGTH                    16
#define MAX_URL_LENGTH                      19
#define DEFAULT_URL_LENGTH                  9
#define CONNECTABLE_STATE_TIMEOUT           30
#define CONNECTABLE_STATE_ADV_INTERVAL_MIN  0x00A0
#define CONNECTABLE_STATE_ADV_INTERVAL_MAX  0x00A0

/*******************************************************************************
* Global Variable Declaration
*******************************************************************************/
/* The default URL */
const uint8 DefaultURL[MAX_URL_LENGTH] = DEFAULT_URL;

/* The default beacon lock key */
const uint8 LOCK[LOCK_CODE_LENGTH] =    {
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00
                                        };

/* To decide whether to have TLM packets interleaved or not */
static bool isEddystoneTLMPresent = true;

/* Selection of UID/URL */
static EDDYSTONE_ROLE_T eddystoneImplenmentation = EDDYSTONE_URL;

/* Select between UID/URL or TLM */
static EDDYSTONE_ROLE_T beaconCurrentRole = EDDYSTONE_URL;

/* Stores the current URL being advrtised */
uint8 CurrentURL[MAX_URL_LENGTH] = DEFAULT_URL;

/* Stores the current URL length being advrtised */
uint8 URLLength = DEFAULT_URL_LENGTH;

/* Stores the current tx power mode */
uint8 currentTxmode = TX_POWER_MODE_LOW;

/* Stores the current advertised tx power levels */
uint8 currentTxPowerLevels[MAX_NUM_PWR_LVL] =   {
                                                    PWR_LVL_NEG_18_DBM,
                                                    PWR_LVL_NEG_6_DBM,
                                                    PWR_LVL_0_DBM,
                                                    PWR_LVL_3_DBM
                                                };

/* Stores the current non-connectable adv interval */
uint16 CurrentAdvPeriod = DEFAULT_BEACON_PERIOD;

/* Stores the current URL flags */
uint8 DefaultURLFlags = DEFAULT_URL_FLAG_VALUE;

/*Advertising data when GAP Reol:Peripheral (connectable)*/
CYBLE_GAPP_DISC_DATA_T CyBleConnectableAdvData =
{
    {
        0x02u, // Length of flags
        0x01u, // Flags
        0x06u, // Flag data
        0x0Du, // length of Local name
        0x09u, // Local name
        'C',
        'Y',
        ' ',
        'E',
        'd',
        'd',
        'y',
        's',
        't',
        'o',
        'n',
        'e'
    },          // uint8 advertising_data[CYBLE_MAX_ADV_DATA_LEN]
        0x11u,  // uint8 adv_data_length
};

/* True if an error response is send. */
static bool errorSent;

/*******************************************************************************
* Function Name: WriteAttributeValue
********************************************************************************
*
* Summary:
*  Writes a specified attribute value of certian length to the appropriate
*  attribute handle present in the  GATT DB
*
* Parameters:
*  CYBLE_GATT_DB_ATTR_HANDLE_T  :  GATT DB Attribute Handle Type
*  uint8                        :  length of data
*  uint8*                       :  data buffer
*  uint8                        :  flags
*
* Return:
*  CYBLE_GATT_ERR_CODE_T        : returns the write operation error code
*
*******************************************************************************/
CYBLE_GATT_ERR_CODE_T WriteAttributeValue   (
                                CYBLE_GATT_DB_ATTR_HANDLE_T attributeHandle,
                                uint8 length,
                                uint8* data,
                                uint8 flag
                                            )
{
    CYBLE_GATT_HANDLE_VALUE_PAIR_T handlevaluePair;
    CYBLE_GATT_ERR_CODE_T gattErrCode;

    handlevaluePair.attrHandle = attributeHandle;
    handlevaluePair.value.len = length;
    handlevaluePair.value.val = data;
    gattErrCode = CyBle_GattsWriteAttributeValue( &handlevaluePair, 0,
                                                &cyBle_connHandle, flag);

    return gattErrCode;
}

/*******************************************************************************
* Function Name: ResetGattDb
********************************************************************************
*
* Summary:
*  Resets all relevant characteristics in the GATT DB
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ResetGattDb(void)
{
    /*Default URI Data */
    memcpy(CurrentURL, DefaultURL, MAX_URL_LENGTH);
    WriteAttributeValue(CYBLE_EDDYSTONE_CONFIGURATION_URI_DATA_CHAR_HANDLE,
                        sizeof(DefaultURL), (uint8 *)DefaultURL,
                        CYBLE_GATT_DB_LOCALLY_INITIATED);
    URLLength = DEFAULT_URL_LENGTH;
    cyBle_attValuesLen[16].actualLength = URLLength;

    /*Default URI Flags */
    WriteAttributeValue(CYBLE_EDDYSTONE_CONFIGURATION_URI_FLAGS_CHAR_HANDLE,
                        sizeof(DefaultURLFlags), &DefaultURLFlags,
                        CYBLE_GATT_DB_LOCALLY_INITIATED);

    /*Default TX power mode */
    currentTxmode = TX_POWER_MODE_LOW;
    WriteAttributeValue(CYBLE_EDDYSTONE_CONFIGURATION_TX_POWER_MODE_CHAR_HANDLE,
                        sizeof(currentTxmode), &currentTxmode,
                        CYBLE_GATT_DB_LOCALLY_INITIATED);

    /*Default Beacon period */
    CurrentAdvPeriod = DEFAULT_BEACON_PERIOD;
    WriteAttributeValue(CYBLE_EDDYSTONE_CONFIGURATION_BEACON_PERIOD_CHAR_HANDLE,
                        sizeof(CurrentAdvPeriod), (uint8 *)&CurrentAdvPeriod,
                        CYBLE_GATT_DB_LOCALLY_INITIATED);

    /*Default LOCK key */
    WriteAttributeValue(CYBLE_EDDYSTONE_CONFIGURATION_LOCK_CHAR_HANDLE,
                        sizeof(LOCK), (uint8 *)LOCK,
                        CYBLE_GATT_DB_LOCALLY_INITIATED);
}

/*******************************************************************************
* Function Name: SendErrorResponse
********************************************************************************
*
* Summary:
*  Sends error code for a particular attribute handle
*
* Parameters:
*  CYBLE_GATT_DB_ATTR_HANDLE_T  :  attribute handle
*  CYBLE_GATT_ERR_CODE_T        :  error code to be send
*
* Return:
*  None
*
*******************************************************************************/
void SendErrorResponse( CYBLE_GATT_DB_ATTR_HANDLE_T attributeHandle,
                        CYBLE_GATT_ERR_CODE_T errorCode)
{
    CYBLE_GATTS_ERR_PARAM_T errorparam;

    errorparam.attrHandle = attributeHandle;
    errorparam.errorCode = errorCode;
    errorparam.opcode = CYBLE_GATT_WRITE_REQ;
    CyBle_GattsErrorRsp(cyBle_connHandle, &errorparam);
    errorSent = true;
}

/*******************************************************************************
* Function Name: UpdateTxPower
********************************************************************************
*
* Summary:
*  Updates advertisement transmit power based on the Tx power mode
*
* Parameters:
*  uint8:  Tx power mode to be set
*
* Return:
*  None
*
*******************************************************************************/
void UpdateTxPower(uint8 TxMode)
{
    CYBLE_BLESS_PWR_IN_DB_T txPower;

    /* Set channel type as Adv channels */
    txPower.bleSsChId = CYBLE_LL_ADV_CH_TYPE;

    /* Set the correct power levels */
    if (TxMode == TX_POWER_MODE_HIGH)   /* +3 dBm */
    {
        txPower.blePwrLevelInDbm = CYBLE_LL_PWR_LVL_3_DBM;
    }
    else if (TxMode == TX_POWER_MODE_MEDIUM)    /* 0 dBm */
    {
        txPower.blePwrLevelInDbm = CYBLE_LL_PWR_LVL_0_DBM;
    }
    else if (TxMode == TX_POWER_MODE_LOW)   /* -6 dBm */
    {
        txPower.blePwrLevelInDbm = CYBLE_LL_PWR_LVL_NEG_6_DBM;
    }
    else if (TxMode == TX_POWER_MODE_LOWEST)    /* -18 dBm */
    {
        txPower.blePwrLevelInDbm = CYBLE_LL_PWR_LVL_NEG_18_DBM;
    }

    /* Set Tx power level */
    CyBle_SetTxPowerLevel(&txPower);
}

/*******************************************************************************
* Function Name: ProcessWriteReq
********************************************************************************
*
* Summary:
*  Process all GATT level write requests and responds with appropriate status
*
* Parameters:
*  CYBLE_GATTS_WRITE_CMD_REQ_PARAM_T:  GATT write command request prameter
*
* Return:
*  None
*
*******************************************************************************/
void ProcessWriteReq(CYBLE_GATTS_WRITE_CMD_REQ_PARAM_T writeCmdReq)
{
    bool value_val;
    uint8 status = LOCKED, key_buf[LOCK_CODE_LENGTH];
    uint16 beaconPeriod = 0;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T valuePairT;

    /* Reset error send flag */
    errorSent = false;
    /* Retrieve the LOCK status from the GATT DB */
    valuePairT.attrHandle =
                        CYBLE_EDDYSTONE_CONFIGURATION_LOCK_STATE_CHAR_HANDLE;
    valuePairT.value.val = (uint8 *)&value_val;
    valuePairT.value.len = sizeof(bool);

    CyBle_GattsReadAttributeValue( &valuePairT, &cyBle_connHandle,
                                    CYBLE_GATT_DB_LOCALLY_INITIATED );

    /* Check the LOCK status */
    if(valuePairT.value.val[0] == UNLOCKED)
    {
        /*URL Data*/
        if(writeCmdReq.handleValPair.attrHandle ==
                            CYBLE_EDDYSTONE_CONFIGURATION_URI_DATA_CHAR_HANDLE)
        {
            /* First byte should be scheme prefix and length should be less than
            *  or equal to MAX_URL_LENGTH */
            if( (writeCmdReq.handleValPair.value.len <= MAX_URL_LENGTH) &&
                (writeCmdReq.handleValPair.value.val[0] < URL_PREFIX_MAX)   )
            {
                uint8 TempURL[MAX_URL_LENGTH];

                memset(TempURL, 0, MAX_URL_LENGTH);
                memcpy(
                            TempURL,
                            writeCmdReq.handleValPair.value.val,
                            writeCmdReq.handleValPair.value.len
                       );
                if( CYBLE_GATT_ERR_NONE == WriteAttributeValue   (
                            CYBLE_EDDYSTONE_CONFIGURATION_URI_DATA_CHAR_HANDLE,
                            writeCmdReq.handleValPair.value.len,
                            writeCmdReq.handleValPair.value.val,
                            CYBLE_GATT_DB_PEER_INITIATED        ) )
                {
                    /* Update the length as per the new URL data */
                    URLLength = writeCmdReq.handleValPair.value.len;
                    cyBle_attValuesLen[16].actualLength = URLLength;
                    /* Update the URL data */
                    memcpy(CurrentURL, TempURL, MAX_URL_LENGTH);
                }
            }
            else if (writeCmdReq.handleValPair.value.len > MAX_URL_LENGTH)
            {
                 /* Invalid length. Send error response */
               SendErrorResponse        (
                            CYBLE_EDDYSTONE_CONFIGURATION_URI_DATA_CHAR_HANDLE,
                            CYBLE_GATT_ERR_INVALID_ATTRIBUTE_LEN
                                        );
            }
        }
        /* Lock Characteristic */
        else if(writeCmdReq.handleValPair.attrHandle ==
                                CYBLE_EDDYSTONE_CONFIGURATION_LOCK_CHAR_HANDLE)
        {
            if(writeCmdReq.handleValPair.value.len == LOCK_CODE_LENGTH)
            {
                WriteAttributeValue (
                                CYBLE_EDDYSTONE_CONFIGURATION_LOCK_CHAR_HANDLE,
                                writeCmdReq.handleValPair.value.len,
                                writeCmdReq.handleValPair.value.val,
                                CYBLE_GATT_DB_PEER_INITIATED
                                    );

                /* Update the LOCK characteristic */
                status = LOCKED;
                WriteAttributeValue (
                        CYBLE_EDDYSTONE_CONFIGURATION_LOCK_STATE_CHAR_HANDLE,
                        sizeof(bool), &status,
                        CYBLE_GATT_DB_LOCALLY_INITIATED
                                    );
            }
            else
            {
                /* Invalid length. Send error response */
               SendErrorResponse(CYBLE_EDDYSTONE_CONFIGURATION_LOCK_CHAR_HANDLE,
                                    CYBLE_GATT_ERR_INVALID_ATTRIBUTE_LEN);
            }
        }
        /* Advertised Tx power level */
        else if(writeCmdReq.handleValPair.attrHandle ==
           CYBLE_EDDYSTONE_CONFIGURATION_ADVERTISED_TX_POWER_LEVELS_CHAR_HANDLE)
        {
            if (writeCmdReq.handleValPair.value.len == MAX_NUM_PWR_LVL)
            {
                WriteAttributeValue (
                    CYBLE_EDDYSTONE_CONFIGURATION_ADVERTISED_TX_POWER_LEVELS_CHAR_HANDLE,
                    writeCmdReq.handleValPair.value.len,
                    writeCmdReq.handleValPair.value.val,
                    CYBLE_GATT_DB_PEER_INITIATED
                                    );

                if (writeCmdReq.handleValPair.value.len == MAX_NUM_PWR_LVL)
                {
                    /* Update Tx Power levels */
                    memcpy  (
                                currentTxPowerLevels,
                                writeCmdReq.handleValPair.value.val,
                                MAX_NUM_PWR_LVL
                            );
                }
                else
                {
                    SendErrorResponse   (
                        CYBLE_EDDYSTONE_CONFIGURATION_ADVERTISED_TX_POWER_LEVELS_CHAR_HANDLE,
                        CYBLE_GATT_ERR_INVALID_ATTRIBUTE_LEN
                                        );
                }
            }
            else
            {
                /* Invalid length. Send error response */
               SendErrorResponse    (
                    CYBLE_EDDYSTONE_CONFIGURATION_ADVERTISED_TX_POWER_LEVELS_CHAR_HANDLE,
                    CYBLE_GATT_ERR_INVALID_ATTRIBUTE_LEN
                                    );
            }
        }
        /* Tx Power Mode */
        else if(writeCmdReq.handleValPair.attrHandle ==
                        CYBLE_EDDYSTONE_CONFIGURATION_TX_POWER_MODE_CHAR_HANDLE)
        {
            if(writeCmdReq.handleValPair.value.val[0] <= 0x03)
            {
                WriteAttributeValue (
                        CYBLE_EDDYSTONE_CONFIGURATION_TX_POWER_MODE_CHAR_HANDLE,
                        writeCmdReq.handleValPair.value.len,
                        writeCmdReq.handleValPair.value.val,
                        CYBLE_GATT_DB_PEER_INITIATED
                                    );
                currentTxmode = writeCmdReq.handleValPair.value.val[0];

                UpdateTxPower(currentTxmode);
            }
            else /* Invalid value. Write not permitted. */
            {
                SendErrorResponse   (
                        CYBLE_EDDYSTONE_CONFIGURATION_TX_POWER_MODE_CHAR_HANDLE,
                        CYBLE_GATT_ERR_WRITE_NOT_PERMITTED
                                    );
            }
        }
        /* Beacon Period */
        else if(writeCmdReq.handleValPair.attrHandle ==
                        CYBLE_EDDYSTONE_CONFIGURATION_BEACON_PERIOD_CHAR_HANDLE)
        {
            beaconPeriod =
                        CyBle_Get16ByPtr(writeCmdReq.handleValPair.value.val);

            /* Disable URL FRAMES */
            if(beaconPeriod == 0)
            {
                eddystoneImplenmentation = EDDYSTONE_UID;
                WriteAttributeValue (
                        CYBLE_EDDYSTONE_CONFIGURATION_BEACON_PERIOD_CHAR_HANDLE,
                        writeCmdReq.handleValPair.value.len,
                        writeCmdReq.handleValPair.value.val,
                        CYBLE_GATT_DB_PEER_INITIATED
                                    );
            }
            /* Values in valid range */
            else if((beaconPeriod >= MIN_BEACON_PERIOD) &&
                    (beaconPeriod <= MAX_BEACON_PERIOD))
            {
                WriteAttributeValue (
                        CYBLE_EDDYSTONE_CONFIGURATION_BEACON_PERIOD_CHAR_HANDLE,
                        writeCmdReq.handleValPair.value.len,
                        writeCmdReq.handleValPair.value.val,
                        CYBLE_GATT_DB_PEER_INITIATED
                                    );
                CurrentAdvPeriod = beaconPeriod / 0.625;
                eddystoneImplenmentation = EDDYSTONE_URL;
            }
            else
            {
                uint16 temp = MIN_BEACON_PERIOD;
                /* Values not supportes. Write default values */
                WriteAttributeValue(
                        CYBLE_EDDYSTONE_CONFIGURATION_BEACON_PERIOD_CHAR_HANDLE,
                        sizeof(temp),
                        (uint8 *)&temp,
                        CYBLE_GATT_DB_PEER_INITIATED);
                CurrentAdvPeriod = CYBLE_GAP_ADV_ADVERT_INTERVAL_NONCON_MIN;
                eddystoneImplenmentation = EDDYSTONE_URL;
            }
        }
        /* Reset the Configurations to default */
        else if((writeCmdReq.handleValPair.attrHandle ==
                CYBLE_EDDYSTONE_CONFIGURATION_RESET_CHAR_HANDLE) &&
                (writeCmdReq.handleValPair.value.val[0] != 0))
        {
            ResetGattDb();
        }
    }
    else if(valuePairT.value.val[0] == LOCKED)
    {
        if(writeCmdReq.handleValPair.attrHandle ==
                                CYBLE_EDDYSTONE_CONFIGURATION_LOCK_CHAR_HANDLE)
        {
            /* Accesing the lock in LOCKED state */
            SendErrorResponse(writeCmdReq.handleValPair.attrHandle,
                                    CYBLE_GATT_ERR_INSUFFICIENT_AUTHORIZATION);
        }
    }

    if(writeCmdReq.handleValPair.attrHandle ==
                        CYBLE_EDDYSTONE_CONFIGURATION_UNLOCK_CHAR_HANDLE)
    {
        if(writeCmdReq.handleValPair.value.len == LOCK_CODE_LENGTH)
        {
            if(valuePairT.value.val[0] == LOCKED)
            {
                int compareResult;
                valuePairT.attrHandle =
                                CYBLE_EDDYSTONE_CONFIGURATION_LOCK_CHAR_HANDLE;
                valuePairT.value.val = key_buf;
                valuePairT.value.len = sizeof(LOCK);
                CyBle_GattsReadAttributeValue(&valuePairT, &cyBle_connHandle,
                                            CYBLE_GATT_DB_LOCALLY_INITIATED);

                compareResult = memcmp  (
                                            valuePairT.value.val,
                                            writeCmdReq.handleValPair.value.val,
                                            LOCK_CODE_LENGTH
                                        );

                if(compareResult == 0)
                {
                    status = UNLOCKED;
                    /* Update the LOCK STATE */
                    WriteAttributeValue (
                        CYBLE_EDDYSTONE_CONFIGURATION_LOCK_STATE_CHAR_HANDLE,
                        sizeof(bool), &status,
                        CYBLE_GATT_DB_LOCALLY_INITIATED
                                        );

                    /* Reset the LOCK */
                    WriteAttributeValue (
                                CYBLE_EDDYSTONE_CONFIGURATION_LOCK_CHAR_HANDLE,
                                sizeof(LOCK), (uint8 *)LOCK,
                                CYBLE_GATT_DB_LOCALLY_INITIATED
                                        );
                }
                else /* LOCK not matched */
                {
                    SendErrorResponse(writeCmdReq.handleValPair.attrHandle,
                                    CYBLE_GATT_ERR_INSUFFICIENT_AUTHORIZATION);
                }
            }
        }
        else /* Invalid length */
        {
            SendErrorResponse(writeCmdReq.handleValPair.attrHandle,
                                        CYBLE_GATT_ERR_INVALID_ATTRIBUTE_LEN);
        }
    }

    if (errorSent == false)
    {
        CyBle_GattsWriteRsp(cyBle_connHandle);
    }
}

/*******************************************************************************
* Function Name: ConfigureAdvPacket
********************************************************************************
*
* Summary:
*  Configures the Advertising data such that it transmits UID/URL and TLM
*  frames
*
* Parameters:
*  bool:  true: configure adv parameters with UID/URL frames
*         false:configures the adv parameters to TLM frames
*
* Return:
*  None
*
*******************************************************************************/
void ConfigureAdvPacket(void)
{
    uint16 currentTemperature, currentBatteryVoltage;
    uint32 currentSecCnt, currentAdvPacketCount;

    /* Configure Eddystone packets at run-time. */
    cyBle_discoveryModeInfo.advParam->advType =
                                    CYBLE_GAPP_NON_CONNECTABLE_UNDIRECTED_ADV;

    if( (beaconCurrentRole == EDDYSTONE_UID) ||
        (beaconCurrentRole == EDDYSTONE_URL))
    {
        URL_UID_LED_Write(LED_ON);
        ConnectableState_LED_Write(LED_OFF);
        TLM_LED_Write(LED_OFF);

        /* ADV packet timeout */
        if (isEddystoneTLMPresent == true)
        {
            cyBle_discoveryModeInfo.advTo = APP_UID_URL_TIMEOUT;
        }
        else
        {
            cyBle_discoveryModeInfo.advTo = 0;  /* No timeout */
        }

        if(beaconCurrentRole == EDDYSTONE_UID)
        {
            /* Service Data */
            /* Length */
            cyBle_discoveryData.advData[7] = 0x17;
            /* Signifies Eddystone UID */
            cyBle_discoveryData.advData[11] = 0x00;
            /* Ranging data: -14dB*/
            cyBle_discoveryData.advData[12] =
                                            currentTxPowerLevels[currentTxmode];

            /* SHA-1 hash of the FQDN (cypress.com) is calculated and its
            *  first 10 bytes are placed here as the Namespace ID, MSB first. */
            cyBle_discoveryData.advData[13] = 0xCB;  /* NID[0] */
            cyBle_discoveryData.advData[14] = 0x6F;  /* NID[1] */
            cyBle_discoveryData.advData[15] = 0x15;  /* NID[2] */
            cyBle_discoveryData.advData[16] = 0xCE;  /* NID[3] */
            cyBle_discoveryData.advData[17] = 0x20;  /* NID[4] */
            cyBle_discoveryData.advData[18] = 0x2A;  /* NID[5] */
            cyBle_discoveryData.advData[19] = 0xCE;  /* NID[6] */
            cyBle_discoveryData.advData[20] = 0x15;  /* NID[7] */
            cyBle_discoveryData.advData[21] = 0x6F;  /* NID[8] */
            cyBle_discoveryData.advData[22] = 0xCB;  /* NID[9] */

            /* Instance ID - randomly created */
            cyBle_discoveryData.advData[23] = 0x01;  /* BID[0] */
            cyBle_discoveryData.advData[24] = 0x00;  /* BID[1] */
            cyBle_discoveryData.advData[25] = 0x00;  /* BID[2] */
            cyBle_discoveryData.advData[26] = 0x00;  /* BID[3] */
            cyBle_discoveryData.advData[27] = 0x00;  /* BID[4] */
            cyBle_discoveryData.advData[28] = 0x00;  /* BID[5] */
            cyBle_discoveryData.advData[29] = 0x00;  /* Reserved */
            cyBle_discoveryData.advData[30] = 0x00;  /* Reserved */

            /* ADV packet length */
            cyBle_discoveryData.advDataLen = 31;
        }
        else if(beaconCurrentRole == EDDYSTONE_URL)
        {
            /* Service Data */
            /* Length */
            cyBle_discoveryData.advData[7] = URLLength + 5;
            /* Signifies Eddystone URL */
            cyBle_discoveryData.advData[11] = 0x10;
            /* Ranging data: -14dB */
            cyBle_discoveryData.advData[12] =
                                            currentTxPowerLevels[currentTxmode];
            cyBle_discoveryData.advData[13] = CurrentURL[0];
            cyBle_discoveryData.advData[14] = CurrentURL[1];
            cyBle_discoveryData.advData[15] = CurrentURL[2];
            cyBle_discoveryData.advData[16] = CurrentURL[3];
            cyBle_discoveryData.advData[17] = CurrentURL[4];
            cyBle_discoveryData.advData[18] = CurrentURL[5];
            cyBle_discoveryData.advData[19] = CurrentURL[6];
            cyBle_discoveryData.advData[20] = CurrentURL[7];
            cyBle_discoveryData.advData[21] = CurrentURL[8];
            cyBle_discoveryData.advData[22] = CurrentURL[9];
            cyBle_discoveryData.advData[23] = CurrentURL[10];
            cyBle_discoveryData.advData[24] = CurrentURL[11];
            cyBle_discoveryData.advData[25] = CurrentURL[12];
            cyBle_discoveryData.advData[26] = CurrentURL[13];
            cyBle_discoveryData.advData[27] = CurrentURL[14];
            cyBle_discoveryData.advData[28] = CurrentURL[15];
            cyBle_discoveryData.advData[29] = CurrentURL[16];
            cyBle_discoveryData.advData[30] = CurrentURL[17];
            /* ADV packet length */
            cyBle_discoveryData.advDataLen = 8 + cyBle_discoveryData.advData[7];
        }
    }
    else if(beaconCurrentRole == EDDYSTONE_TLM)
    {
        URL_UID_LED_Write(LED_OFF);
        ConnectableState_LED_Write(LED_OFF);
        TLM_LED_Write(LED_ON);

        /* ADV packet timeout */
        cyBle_discoveryModeInfo.advTo = APP_TLM_TIMEOUT;

        if (isEddystoneTLMPresent == true)
        {
            /* Telemetry packets */

            /* Service Data */
            /* Length */
            cyBle_discoveryData.advData[7] = 0x11;
            /* Service Data */
            cyBle_discoveryData.advData[8] = 0x16;
            /* LSB - Eddystone Service */
            cyBle_discoveryData.advData[9] = 0xAA;
            /* MSB - Eddystone Service */
            cyBle_discoveryData.advData[10] = 0xFE;
            /* Signifies Eddystone TLM */
            cyBle_discoveryData.advData[11] = 0x20;
            /* TLM version */
            cyBle_discoveryData.advData[12] = 0x00;
            /* Battery voltage in mV (1 mV per bit) */
#if (BATTERY_MEASUREMENT_ENABLE == YES)
            currentBatteryVoltage = GetMeasuredBatteryVoltage();
            cyBle_discoveryData.advData[13] =
                                (uint8)((currentBatteryVoltage >> 8) & 0xFF);
            cyBle_discoveryData.advData[14] =
                                        (uint8)(currentBatteryVoltage & 0xFF);

#else

            cyBle_discoveryData.advData[13] = 0x00;
            cyBle_discoveryData.advData[14] = 0x00;
#endif  //  BATTERY_MEASUREMENT_ENABLE

            /* Beacon temperature in Celsius (8.8 fixed point notation):
            *  https://courses.cit.cornell.edu/ee476/Math/ */
#if (TEMPERATURE_SENSOR_ENABLE == YES)
            currentTemperature = GetMeasuredTemprature();
            cyBle_discoveryData.advData[15] =
                                    (uint8)((currentTemperature >> 8) & 0xFF);
            cyBle_discoveryData.advData[16] =
                                            (uint8)(currentTemperature & 0xFF);

#else
            cyBle_discoveryData.advData[15] = 0x80;
            cyBle_discoveryData.advData[16] = 0x00;

#endif  //  TEMPERATURE_SENSOR_ENABLE

            /* Advertising PDU count since power-up or reboot */
            currentAdvPacketCount = GetAdvPacketCount();
            cyBle_discoveryData.advData[17] =
                                (uint8)((currentAdvPacketCount >> 24) & 0xFF);
            cyBle_discoveryData.advData[18] =
                                (uint8)((currentAdvPacketCount >> 16) & 0xFF);
            cyBle_discoveryData.advData[19] =
                                (uint8)((currentAdvPacketCount >> 8) & 0xFF);
            cyBle_discoveryData.advData[20] =
                                (uint8)((currentAdvPacketCount) & 0xFF);

            /* Time since power-on or reboot */
            currentSecCnt = GetSecCnt();
            cyBle_discoveryData.advData[21] =
                                    (uint8)((currentSecCnt >> 24) & 0xFF);
            cyBle_discoveryData.advData[22] =
                                    (uint8)((currentSecCnt >> 16) & 0xFF);
            cyBle_discoveryData.advData[23] =
                                    (uint8)((currentSecCnt >> 8) & 0xFF);
            cyBle_discoveryData.advData[24] =
                                    (uint8)((currentSecCnt) & 0xFF);

            /* ADV packet length */
            cyBle_discoveryData.advDataLen = 25;
        }
    }

    /*Update the advertisement interval */
    cyBle_discoveryModeInfo.advParam->advIntvMin = CurrentAdvPeriod;
    cyBle_discoveryModeInfo.advParam->advIntvMax = CurrentAdvPeriod;

    cyBle_discoveryModeInfo.advData = &cyBle_discoveryData;
}

/*******************************************************************************
* Function Name: ConfigureConnAdvPacket
********************************************************************************
*
* Summary:
*  Configures the connectable Advertising data
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ConfigureConnAdvPacket(void)
{
    URL_UID_LED_Write(LED_OFF);
    ConnectableState_LED_Write(LED_ON);
    TLM_LED_Write(LED_OFF);

    /*Update the advertisement interval */
    cyBle_discoveryModeInfo.advParam->advIntvMin =
                                            CONNECTABLE_STATE_ADV_INTERVAL_MIN;
    cyBle_discoveryModeInfo.advParam->advIntvMax =
                                            CONNECTABLE_STATE_ADV_INTERVAL_MAX;

    cyBle_discoveryModeInfo.advParam->advType =
                                        CYBLE_GAPP_CONNECTABLE_UNDIRECTED_ADV;

    cyBle_discoveryModeInfo.advTo = CONNECTABLE_STATE_TIMEOUT;

    /*Advertising data*/
    cyBle_discoveryModeInfo.advData = &CyBleConnectableAdvData;
}

/*******************************************************************************
* Function Name: EddystoneEventHandler
********************************************************************************
*
* Summary:
*   This is an event callback function to receive events from the CYBLE
*   Component.
*
* Parameters:
*   uint8 event:       Event from the CYBLE component.
*   void* eventParams: A structure instance for corresponding event type. The
*                      list of event structure is described in the component
*                      datasheet.
*
* Return:
*   None
*
*******************************************************************************/
void EddystoneEventHandler(uint32 event, void* eventParam)
{
    CYBLE_API_RESULT_T apiResult;

    /* To prevent compiler warning. */
    eventParam = eventParam;

    switch (event)
    {
        /**********************************************************
         *                       General Events
         ***********************************************************/

        /* This event is received when component is Started */
        case CYBLE_EVT_STACK_ON:
            /* Configure WDT counter 0 with 2 second interval */
            WDT_Initialize(CY_SYS_WDT_COUNTER0, TWO_SECOND_INTERRUPT_COUNT);

            /* Enable WDT counter 0 */
            WDT_EnableCounter(CY_SYS_WDT_COUNTER0_MASK);

            /* Ignore the initial delay. Start counter to track the time since
            *  power ON. */
            SetEnableSecCnt(true);

            beaconCurrentRole = eddystoneImplenmentation;
            ConfigureAdvPacket();

            /* Start advertisement */
            apiResult = CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_CUSTOM);
            if(apiResult != CYBLE_ERROR_OK)
            {
                CYASSERT(0);
            }
            break;
        case CYBLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            if(CyBle_GetState() != CYBLE_STATE_ADVERTISING)
            {
                /* On advertisement timeout, restart advertisement. Before
                *  restarting previous type of advertisement, check if the other
                *  type is triggered. If so, switch to the other type of
                *  advertisement. */
                if(IsConnAdvStart() == true)
                {
                    CyBle_GattsDisableAttribute (
                                        CYBLE_EDDYSTONE_SERVICE_HANDLE
                                                );

                    ConfigureConnAdvPacket();

                    apiResult =
                        CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_CUSTOM);

                    if(apiResult != CYBLE_ERROR_OK)
                    {
                        CYASSERT(0);
                    }
                }
                else
                {
                    /* On advertisement timeout, switch between URI/URL and
                    *  TLM packets. */
                    if( (beaconCurrentRole == EDDYSTONE_UID) ||
                        (beaconCurrentRole == EDDYSTONE_URL) )
                    {
                        beaconCurrentRole = EDDYSTONE_TLM;
                    }
                    else if(beaconCurrentRole == EDDYSTONE_TLM)
                    {
                        beaconCurrentRole = eddystoneImplenmentation;
                    }

                    ConfigureAdvPacket();

                    apiResult =
                        CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_CUSTOM);

                    if(apiResult != CYBLE_ERROR_OK)
                    {
                        CYASSERT(0);
                    }
                }
            }
            else
            {
                SetAdvPacketCount(GetAdvPacketCount() + 1);
            }
            break;
        case CYBLE_EVT_GAP_DEVICE_DISCONNECTED:
            CyBle_GattsEnableAttribute(CYBLE_EDDYSTONE_SERVICE_HANDLE);
            beaconCurrentRole = eddystoneImplenmentation;
            ConfigureAdvPacket();
            CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_CUSTOM);
            break;
        case CYBLE_EVT_GATTS_WRITE_REQ:
            ProcessWriteReq(*(CYBLE_GATTS_WRITE_CMD_REQ_PARAM_T*) eventParam);
            break;
        default:
            break;
    }
}

/*******************************************************************************
* Function Name: UpdateTLMAdvPacket
********************************************************************************
*
* Summary:
*   If device is advertising TLM packets then update the TLM packets with
*   temperature, timesince power on, number of packets that are advertised
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void UpdateTLMAdvPacket(void)
{
    uint16 currentTemperature, currentBatteryVoltage;
    uint32 currentSecCnt, currentAdvPacketCount;

    /* If broadcasting TLM packets, then update the temperature, battery
    *  voltage, advPacketCount and SecCnt */
    if ((CyBle_GetState() == CYBLE_STATE_ADVERTISING) &&
        (CyBle_GetBleSsState() == CYBLE_BLESS_STATE_EVENT_CLOSE) &&
        (beaconCurrentRole == EDDYSTONE_TLM))
    {
#if (BATTERY_MEASUREMENT_ENABLE == YES)
        currentBatteryVoltage = GetMeasuredBatteryVoltage();
        cyBle_discoveryData.advData[13] =
                                (uint8)((currentBatteryVoltage >> 8) & 0xFF);
        cyBle_discoveryData.advData[14] =
                                        (uint8)(currentBatteryVoltage & 0xFF);

#else

            cyBle_discoveryData.advData[13] = 0x00;
            cyBle_discoveryData.advData[14] = 0x00;
#endif  //  BATTERY_MEASUREMENT_ENABLE

#if (TEMPERATURE_SENSOR_ENABLE == YES)
        /* Beacon temperature in Celsius (8.8 fixed point notation):
        *  https://courses.cit.cornell.edu/ee476/Math/ */
        currentTemperature = GetMeasuredTemprature();
        cyBle_discoveryData.advData[15] =
                                    (uint8)((currentTemperature >> 8) & 0xFF);
        cyBle_discoveryData.advData[16] =
                                            (uint8)(currentTemperature & 0xFF);

#else
        /*
         * Beacon temperature in Celsius (8.8 fixed point notation):
         * https://courses.cit.cornell.edu/ee476/Math/
         */
        cyBle_discoveryData.advData[15] = 0x80;
        cyBle_discoveryData.advData[16] = 0x00;

#endif

        /* Advertising PDU count since power-up or reboot */
        currentAdvPacketCount = GetAdvPacketCount();
        cyBle_discoveryData.advData[17] =
                                (uint8)((currentAdvPacketCount >> 24) & 0xFF);
        cyBle_discoveryData.advData[18] =
                                (uint8)((currentAdvPacketCount >> 16) & 0xFF);
        cyBle_discoveryData.advData[19] =
                                (uint8)((currentAdvPacketCount >> 8) & 0xFF);
        cyBle_discoveryData.advData[20] =
                                (uint8)((currentAdvPacketCount) & 0xFF);

    /* Time since power-on or reboot */
        currentSecCnt = GetSecCnt();
        cyBle_discoveryData.advData[21] =
                                        (uint8)((currentSecCnt >> 24) & 0xFF);
        cyBle_discoveryData.advData[22] =
                                        (uint8)((currentSecCnt >> 16) & 0xFF);
        cyBle_discoveryData.advData[23] =
                                        (uint8)((currentSecCnt >> 8) & 0xFF);
        cyBle_discoveryData.advData[24] =
                                        (uint8)((currentSecCnt) & 0xFF);

        /*Update Adv packet*/
        CyBle_GapUpdateAdvData(&cyBle_discoveryData,&cyBle_scanRspData);

        /* Measure battery voltage after Adv packet is updated. The battery
        *  measurement involves charging the reference cap (takes around 25ms).
        *  By performing battery measurement just after an Adv slot we can sleep
        *  uninterrupted (for the cap charge time) to save max power about. */
        MeasureBattery();
    }

    /* Sync temperature measurement with ECO_STABLE state to save power. */
    if ((CyBle_GetState() == CYBLE_STATE_ADVERTISING) &&
        (CyBle_GetBleSsState() == CYBLE_BLESS_STATE_ECO_STABLE) &&
        (beaconCurrentRole == EDDYSTONE_TLM))
    {
        /* Measure die temperature */
        MeasureTemperature();
    }
}

/* [] END OF FILE */
