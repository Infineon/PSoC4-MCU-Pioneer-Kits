/******************************************************************************
* File Name: colormixing.c
*
* Version: 1.00
*
* Description:  This file contains the code to compute the TCPWM's compare value
*               for RGB color mixing.
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
/*******************************************************************************
*   Included Headers
*******************************************************************************/

/* Include all components API definitions */
#include "project.h"

/* Include color mixing API definitions */
#include "colormixing.h"

/* Include sprintf API definitions */
#include <stdio.h>

/*******************************************************************************
*  Function Declarations
*******************************************************************************/

/* Function computes the determinant for a 3X3 matrix */
int16 determinant3X3Matrix(int16[MATRIX_3X3][MATRIX_3X3]);

/* Function computes the cofactor matrix required for color mixing */
uint8 cofactor3X3Matrix(int16[MATRIX_3X3][MATRIX_3X3], int32*);

/* Function computes the inverse of a 3X3 matrix */
uint8 inverse3X3Matrix(int16[MATRIX_3X3][MATRIX_3X3], int32*);

/******************************************************************************
* Function Name: initializeLedCoordinates
*******************************************************************************
*
* Summary: This API initializes XY coordinates for RGB color on the color gamut.
*          It also initializes the maximum lumens for Red, Green and Blue LEDs
*
* Parameters:
*  ledCoordinateMatrix - Pointer to the first element of the array which holds the 
*                       RGB led data
*
* Return:
*  None.
*
* Theory:
*  The initializeLedCoordinates() API initializes the XY coordinates for Red, Green and Blue 
*  LEDs on the color gamut. 
*  Note: The XY coordinates on the color gamut are multiplied by 10000 to avoid floating point 
*  math. This function also loads the RAM variables with the maximum rated lumens for each 
*  color in the RGB LED.
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
void initializeLedCoordinates(LED_COORDINATE* ledCoordinateMatrix)
{
    /* Initialize RED LED coordinates and maximum lumens */
    (*(ledCoordinateMatrix + LED_RED)).ledXCoordinate = RED_BASE_COLOR_X;
    (*(ledCoordinateMatrix + LED_RED)).ledYCoordinate = RED_BASE_COLOR_Y;
    (*(ledCoordinateMatrix + LED_RED)).ledRatedFlux = RED_MAX_LUMENS;
    
	/* Initialize GREEN LED coordinates and maximum lumens */
    (*(ledCoordinateMatrix + LED_GREEN)).ledXCoordinate = GREEN_BASE_COLOR_X;                    
    (*(ledCoordinateMatrix + LED_GREEN)).ledYCoordinate = GREEN_BASE_COLOR_Y;
    (*(ledCoordinateMatrix + LED_GREEN)).ledRatedFlux = GREEN_MAX_LUMENS;
    
	/* Initialize BLUE LED coordinates and maximum lumens */
    (*(ledCoordinateMatrix + LED_BLUE)).ledXCoordinate = BLUE_BASE_COLOR_X;                    
    (*(ledCoordinateMatrix + LED_BLUE)).ledYCoordinate = BLUE_BASE_COLOR_Y;
    (*(ledCoordinateMatrix + LED_BLUE)).ledRatedFlux = BLUE_MAX_LUMENS;
}

/******************************************************************************
* Function Name: rgbColorMix
*******************************************************************************
*
* Summary: This API computes the compare value of the TCPWM component required to 
*          generate a specific color which is input by the user.
*
* Parameters:
*  currentCoordinate - Structure which contains XY coordinates selected 
*                      by the user and the RGB LED brightness level
*  ledRGBCoordinate - Pointer to the structure which contains XY coordinates of the RGB
*                     color on the color gamut
*  ledDimValue - Pointer to the array where the compare value of the TCPWM component has 
*                to be stored
*
* Return:
*  errorStatus:
*               0 - No Error in the color mixing
*               1 - Invalid Color Error. A color outside of the gamut has
*                   been requested. The function will return dimming values of zero.
*               2 - Not Enough Lumens. Too many lumens have been requested
*                   for the LEDs to output. The function will return dimming
*                   values that will give the brightest possible output at
*                   the requested x,y color point.
*
* Theory:
*  The rgbColorMix() API computes the compare value for the TCPWM component required to 
*  generate the color based on the XY coordinates specified by the user. 
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
uint8 rgbColorMix(XY_COORDINATE currentCoordinate, LED_COORDINATE* ledRGBCoordinate, uint16* ledDimValue)
{  
    /* Variable to store the return status */
    uint8 returnStatus = NO_ERROR;
    
    /* Temporary variables */
    uint8 index = 0;
    int32 tempVar1 = 0, tempVar2 = 0;
    
    /* Array that stores the matrix result */
    int16 outputMatrix[MATRIX_3X3][MATRIX_3X3] = {{0,0,0}, {0,0,0}, {0,0,0}};
    
    /* Array that stores the dimming value */
    int32 outputDimValue[NUM_LEDS] = {0, 0, 0};
    
    /* Compute the difference between RGB X-coordinate and currentX coordinate */
    for(index = 0; index < NUM_LEDS; index++)
    { 
        /* Compute the difference */
        tempVar1 = (int32)((*(ledRGBCoordinate + index)).ledXCoordinate \
                            - currentCoordinate.currentX);      
        
        /* Scale the difference to avoid floating point math */
        tempVar1 *= SCALE_1000;
        
        /* Compute the divisor/2 value to Ceil the resulting divided value in the next step */
        tempVar2 = (int32)(*(ledRGBCoordinate + index)).ledYCoordinate >> 1;
        
        /* If subtraction result is negative, add the -divisor/2 value to the negative value */
        if(tempVar1 < 0)
        {                                     
            tempVar1 -= (int32)tempVar2;
        }
        /* If subtraction result is positive, add the divisor/2 value to the positive value */
        else
        {                                        
            tempVar1 += (int32)tempVar2;
        }
            
        /* Divide the subtraction result with YColor value */
        tempVar1 /= (int32)(*(ledRGBCoordinate + index)).ledYCoordinate;
            
        /* Store the result in the first row of the matrix */
        outputMatrix[ROW0][index] = (int32)tempVar1;    
    }
    
    /* Compute the difference between RGB Y-coordinate and currentY coordinate */
    for(index = 0; index < NUM_LEDS; index++)
    {     
        /* Compute the difference */
        tempVar1 = (int32)(*(ledRGBCoordinate + index)).ledYCoordinate \
                            - currentCoordinate.currentY;  
        
        /* Scale the difference to avoid floating point math */
        tempVar1 *= 1000;                  
        
        /* Compute the divisor/2 value to Ceil the resulting divided value in the next step */
        tempVar2 = (int32)(*(ledRGBCoordinate + index)).ledYCoordinate >> 1;         
        
        /* If subtraction result is negative, add the -divisor/2 value to the negative value */
        if(tempVar1 < 0)
        {
            tempVar1 -= (int32)tempVar2;
        }
        /* If subtraction result is positive, add the divisor/2 value to the positive value */
        else
        {                                        
            tempVar1 += (int32)tempVar2;
        } 
        
        /* Divide the subtraction result with YColor value */
        tempVar1 /= (int32)(*(ledRGBCoordinate + index)).ledYCoordinate;     
            
        /* Store the result in the first row of the matrix */
        outputMatrix[ROW1][index] = (int32)tempVar1;
    }
    
    /* Store '1' in the third row of the matrix */
    for(index = 0; index < NUM_LEDS; index++)        
    {
        outputMatrix[ROW2][index] = 1u;        
    }
    
    /* Do an Inversion operation on the matrix and return only 3rd column elements */
    index = inverse3X3Matrix(outputMatrix, outputDimValue);   
    
    /* If color mixing resulted in an error, return the status as error to the main function */
    if(index != INVALID_COLOR)
    {
        /* Compute the dimming values */
        for(index=0; index < NUM_LEDS; index++)
        {
             /* Multiply each element with a constant value to scale things correctly */
            outputDimValue[index] <<= 14;                
            
            /* Compute the divisor/2 value to Ceil the resulting divided value in the next step */
            tempVar2 = (int32)(*(ledRGBCoordinate + index)).ledRatedFlux >> 1;      
            
            /* If subtraction result is negative, add the -divisor/2 value to the negative value */
            if(outputDimValue[index] < 0)
            {
                outputDimValue[index] -= (int32)tempVar2;
            }
            /* If subtraction result is positive, add the divisor/2 value to the positive value */
            else
            {       
                outputDimValue[index] += (int32)tempVar2;
            }
            /* Divide the computed value by maximum rated flux */
            outputDimValue[index] /= (int32)(*(ledRGBCoordinate + index)).ledRatedFlux;
        }
        
        /* Find the maximum dimming value */   
        if(outputDimValue[LED_RED] > outputDimValue[LED_GREEN])
        {
            tempVar1 = outputDimValue[LED_RED];      
        }
        else
        {
            tempVar1 = outputDimValue[LED_GREEN];
        }

        if(tempVar1 < outputDimValue[LED_BLUE])
        {
            tempVar1 = outputDimValue[LED_BLUE];   
        }
        /* Divide the intensity value by the maximum intensity */
        for(index = 0; index < NUM_LEDS; index++)
        {
            /* Multiply the intensity value with maximum intensity */
            outputDimValue[index] *= (int32)currentCoordinate.ledIntensity;
            
            /* Compute the divisor/2 value to Ceil the resulting divided value in the next step */
            outputDimValue[index] += (tempVar1 >> 1);
            
            /* Divide the computed value by maximum dim value */
            outputDimValue[index] /= tempVar1;
            
            /* Store the dimming value in the data structure */
            *(ledDimValue + index) = (uint16)outputDimValue[index]; 

            /* If dimming value is zero, set it to minimum value */
            if(*(ledDimValue + index) == 0)                           
            {                                                               
                *(ledDimValue + index) = MIN_COMPAREVALUE;
            } 
        }
    }
    else
    {        
        returnStatus = INVALID_COLOR;
    }     
    return (returnStatus);
}

/******************************************************************************
* Function Name: determinant3X3Matrix
*******************************************************************************
*
* Summary: This API computes the determinant of a 3X3 matrix
*
* Parameters:
*  inputMatrix - A 3X3 matrix whose determinant has to be found
*
* Return:
*  determinant value.
*
* Theory:
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
int16 determinant3X3Matrix(int16 inputMatrix[MATRIX_3X3][MATRIX_3X3])
{
    /* Temporary variable */
    int32 tempVar1 = 0, tempVar2 = 0, tempVar3 = 0;
    int32 downDiagonal = 0, upDiagonal = 0, determinant = 0;

    /* Compute determinant */
    tempVar1 = inputMatrix[ROW0][COL0] * inputMatrix[ROW1][COL1]; 
    tempVar1 = tempVar1 * (int32)inputMatrix[ROW2][COL2];
    
    tempVar2 = inputMatrix[ROW0][COL1] * inputMatrix[ROW1][COL2];
    tempVar2 = tempVar2 * (int32)inputMatrix[ROW2][COL0];
    
    tempVar3 = inputMatrix[ROW0][COL2] * inputMatrix[ROW1][COL0];
    tempVar3 = tempVar3 * (int32)inputMatrix[ROW2][COL1];
    
    downDiagonal = tempVar1 + tempVar2 + tempVar3;
    
    tempVar1 = inputMatrix[ROW0][COL2] * inputMatrix[ROW1][COL1]; 
    tempVar1 = tempVar1 * (int32)inputMatrix[ROW2][COL0];
    
    tempVar2 = inputMatrix[ROW0][COL0] * inputMatrix[ROW1][COL2];
    tempVar2 = tempVar2 * (int32)inputMatrix[ROW2][COL1];
    
    tempVar3 = inputMatrix[ROW0][COL1] * inputMatrix[ROW1][COL0];
    tempVar3 = tempVar3 * (int32)inputMatrix[ROW2][COL2];
    
    upDiagonal = tempVar1 + tempVar2 + tempVar3;
    
    determinant = downDiagonal - upDiagonal;                        
    
    /* Scale the determinant because the value was multiplied by 1000 earlier */
    determinant /= SCALE_1000;
    
    /* return the determinant value */
    return (int16)determinant;
    
}

/******************************************************************************
* Function Name: cofactor3X3Matrix
*******************************************************************************
*
* Summary: This API computes the cofactor of a 3X3 matrix
*
* Parameters:
*  matrix - A 3X3 matrix whose cofactor has to be found 
*  returnVector - The array in which the cofactor is returned
*
* Return:
*  Error status.
*               0 - No Error in the color mixing
*               1 - Invalid Color Error. A color outside of the gamut has
*                   been requested. The function will return dimming values of zero.
* Theory:
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
uint8 cofactor3X3Matrix(int16 matrix[MATRIX_3X3][MATRIX_3X3], int32* returnVector)
{
    /* Temporary variable */
    int32 varTemp1 = 0, varTemp2 = 0;
    
    /* Compute cofactor matrix */
    varTemp1 = matrix[ROW0][COL1] * matrix[ROW1][COL2];      
    varTemp2 = matrix[ROW1][COL1] * matrix[ROW0][COL2];       
    *(returnVector + ROW0) = varTemp1 - varTemp2;
    
    varTemp1 = matrix[ROW0][COL0] * matrix[ROW1][COL2];         
    varTemp2 = matrix[ROW0][COL2] * matrix[ROW1][COL0];      
    *(returnVector + ROW1) = varTemp2 - varTemp1;
    
    varTemp1 = matrix[ROW0][COL0] * matrix[ROW1][COL1];           
    varTemp2 = matrix[ROW1][COL0] * matrix[ROW0][COL1];        
    *(returnVector + ROW2) = varTemp1 - varTemp2;
    
    /* If any color is negative, return INVALID_COLOR */
    if((*(returnVector + ROW0) >= 0) && (*(returnVector + ROW1) >= 0) && (*(returnVector + ROW2) >= 0))
    {
        return NO_ERROR;
    }
    else if((*(returnVector + ROW0) <= 0) && (*(returnVector + ROW1) <= 0) && (*(returnVector + ROW2) <= 0))
    {
        return NO_ERROR;
    }
    else
    {
        return INVALID_COLOR;
    }
}

/******************************************************************************
* Function Name: inverse3x3Matrix
*******************************************************************************
*
* Summary: This API computes inverse of a 3X3 matrix
*
* Parameters:
*  matrix - A 3X3 matrix whose inverse has to be computed 
*  returnVector - The array in which the inverse is returned
*
* Return:
*  Error status.
*               0 - No Error in the color mixing
*               1 - Invalid Color Error. A color outside of the gamut has
*                   been requested. The function will return dimming values of zero.
* Theory:
*
* Side Effects: None
*
* Note: None
*
*******************************************************************************/
uint8 inverse3X3Matrix(int16 matrix[MATRIX_3X3][MATRIX_3X3], int32* returnVector)
{
    /* Variable to store return status */
    uint8 returnStatus = NO_ERROR;
    
    /* Temporary variables */
    uint8 status = 0, index = 0;
    int16 tempVar = 0;
    
    /* Variable to store determinant */
    int16 determinant;
  
    /* Compute determinant of a 3X3 matrix */
    determinant = determinant3X3Matrix(matrix);
    
    /* If determinant is '0', set it to 1 to avoid divide by zero exception */
    if(determinant == 0)
    {
        determinant = 1;
    }
    
    /* Compute cofactor of a 3X3 matrix */
    status = cofactor3X3Matrix(matrix, returnVector);
    
    /* If cofactor matrix is positive, compute the inverse of matrix*/
    if(status != INVALID_COLOR)
    {   
        /* Compute determinant/2 value */
        tempVar = determinant >> 1;
        
        /* Add determinant/2 value to the cofactor matrix array */
        for(index = 0; index < NUM_LEDS; index++)
        {
            if(*(returnVector + index) < 0)
            {
                *(returnVector + index) -= (int32)tempVar;
            }
            else
            {
                *(returnVector + index) += (int32)tempVar;        
            }
            
            /* Divide cofactor matrix by determinant */
            *(returnVector + index) = *(returnVector + index) / (int32)determinant;        
        }
    }
    /* If cofactor matrix has a negative value, return INVALID_COLOR code */
    else
    {
        returnStatus = INVALID_COLOR;
    }
    return (returnStatus);
}

/* [] END OF FILE */
