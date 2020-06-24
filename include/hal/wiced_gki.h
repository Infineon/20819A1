/***************************************************************************//**
* \file <wiced_gki.h>
*
* Provides the API definitions for the Generic Kernel Interface (GKI).
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
* Cypress Semiconductor Corporation. All Rights Reserved.
*
* This software, including source code, documentation and related
* materials ("Software"), is owned by Cypress Semiconductor Corporation
* or one of its subsidiaries ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products. Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef __WICED_GKI_H__
#define __WICED_GKI_H__

#include "wiced.h"

/**
* \addtogroup GKI GKI
* \ingroup group_memory
* \{
* Defines the interfaces for Buffer , Timer and Event Management Services
*/

/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/******************************************************************************
* Function Name: wiced_bt_did_stack_overflow
***************************************************************************//**
*
* Checks if the application thread stack overflowed at some point.
*
* \return
* TRUE : on stack overflow
* FALSE : if no stack overflow
*
******************************************************************************/
uint8_t wiced_bt_did_stack_overflow(void);


/******************************************************************************
* Function Name: wiced_bt_stack_check_init
***************************************************************************//**
*
* Prepares the stack to allow the app to check for stack overflow.
*
******************************************************************************/
void wiced_bt_stack_check_init(void);


/******************************************************************************
* Function Name: wiced_bt_ble_get_available_tx_buffers
***************************************************************************//**
*
* Used to get the available number of ble tx buffers.
*
* \return
* The available number of ble tx buffers
*
******************************************************************************/
uint32_t wiced_bt_ble_get_available_tx_buffers( void );


/******************************************************************************
* Function Name: wdog_generate_hw_reset
***************************************************************************//**
*
* Generates a system reset by the watchdog timer.
*
******************************************************************************/
void wdog_generate_hw_reset(void);


/******************************************************************************
* Function Name: wiced_gki_delay_us
***************************************************************************//**
*
* This function provides a specified amount of delay in microseconds.
* The function overhead is about 18us at 16MHz. It is not possible to get
* a shorter delay than that. This function can be used before the ARM timers
* are initialized.
*
* \param delay
* Delay time in microseconds
*
******************************************************************************/
void wiced_gki_delay_us(uint32_t delay);


/** \} GKI */

#endif //__WICED_GKI_H__


/* [] END OF FILE */
