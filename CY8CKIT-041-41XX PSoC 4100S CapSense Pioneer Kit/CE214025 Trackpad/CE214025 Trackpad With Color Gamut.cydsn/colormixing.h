/******************************************************************************
* File Name: colormixing.h
*
* Version: 1.00
*
* Description:  This file contains the macros and constant required for 
*               RGB color mixing
*
* Related Document: CE214025 Trackpad with Color Gamut.pdf
*
* Hardware Dependency: See code example CE214025
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
#ifndef __COLORMIXING_H
#define __COLORMIXING_H
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <project.h>   

/* RGB LED Dialight - 5988820307F specifications */    
/* Typical intensity rating in millicandelas(mcd) for 20 mA current
*    Red: 120, Green: 220, Blue: 90
*  For the current design, forward current is ~1.45mA and intensity * (2^8) is:
*    Red: 2218, Green: 2542, Blue: 1017    
*/
#define RED_MAX_LUMENS         (2227u)
#define GREEN_MAX_LUMENS       (2552u)  
#define BLUE_MAX_LUMENS        (1021u)      
    
/* Dominant Wavelength(typical value) Red: 625nm Green: 525nm Blue: 470nm */
#define RED_BASE_LAMBDA        (625u)
#define GREEN_BASE_LAMBDA      (525u)
#define BLUE_BASE_LAMBDA       (470u)

/* XY coordinates for Red, Green and Blue color for the given RGB led lambda value
*  Value is obtained from: http://www.ledtuning.nl/en/cie-convertor    
*  Value is multiplied by 10000 to avoid floating point math
*/    
#define RED_BASE_COLOR_X       (7006u)
#define RED_BASE_COLOR_Y       (2993u)
    
#define GREEN_BASE_COLOR_X     (1141u)
#define GREEN_BASE_COLOR_Y     (8262u)

#define BLUE_BASE_COLOR_X      (1241u)
#define BLUE_BASE_COLOR_Y      (578u)

/* Matrix row and column numbers */   
#define ROW0                   (0u)
#define ROW1                   (1u)
#define ROW2                   (2u)
#define COL0                   (0u)
#define COL1                   (1u)
#define COL2                   (2u)

/* Matrix is a 3 X 3 for color computing */    
#define MATRIX_3X3             (3u)

/* RGB LED Definitions */    
#define LED_RED                (0u)
#define LED_GREEN              (1u)
#define LED_BLUE               (2u)

/* Macro for color mixing function */    
#define NO_ERROR               (0x00u)
#define INVALID_COLOR          (0x80u)
#define LUMEN_ERROR            (0x40u)
    
/* Maximum value of dimming variable */    
#define DIMMER_MAX             (65535u)

/* Design has three LEDs - Red, Green and Blue */    
#define NUM_LEDS               (3u)

/* Macros for sensor status check */    
#define TRUE                   (1u)
#define FALSE                  (0u)

/* Scaling factor of 1000 */
#define SCALE_1000             (1000u)

/* Minimum and maximum compare value for PriSm component */
#define MIN_COMPAREVALUE       (1u)
#define MAX_COMPAREVALUE       (65535u)
    
/* Structure definition for RGB LED parameters */    
typedef struct 
{
    /* This variable indicates the maximum value of luminous intensity for a specific current */
    uint16 ledRatedFlux;    
    
    /* X Coordinate for a given color on the color gamut */
    int16 ledXCoordinate;
    
    /* Y Coordinate for a given color on the color gamut */
    int16 ledYCoordinate;   
} LED_COORDINATE;

/* Structure definition for current XY coordinates and required intensity */
typedef struct 
{
    /* This variable indicates required led intensity */
    uint16 ledIntensity;
    
    /* X-Touch coordinate */
    int16 currentX;  
    
    /* Y-Touch coordinate */
    int16 currentY;  
} XY_COORDINATE;

/* Function to initialize RGB color coordinates from the color gamut */
void initializeLedCoordinates(LED_COORDINATE*);

/* Function computes the compare value required to produce the selected color on the color gamut */
uint8 rgbColorMix(XY_COORDINATE, LED_COORDINATE*, uint16*);

#endif /* End of #define __COLORMIXING_H */

/* [] END OF FILE */
