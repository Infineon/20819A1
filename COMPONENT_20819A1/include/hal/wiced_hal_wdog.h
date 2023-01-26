/***************************************************************************//**
* \file <wiced_hal_wdog.h>
*
* Provides the API definitions for the watchdog timer.
*
********************************************************************************
* \copyright
* Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
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

#ifndef __WICED_WDOG_RESET_H__
#define __WICED_WDOG_RESET_H__

/**
* \addtogroup watchdog Watchdog Timer
* \ingroup HardwareDrivers
* \{
*
* This driver manages the hardware watchdog countdown timer. By default the
* watchdog is enabled on a two second timeout. The system must reach the
* lowest priority 'idle' thread once every two seconds in order to kick the
* watchdog. If the system is being held somewhere (e.g. a busy loop in an
* application level event handler), then the idle thread will not be reached
* and the watchdog will trigger a soft reset of the chip.
*
* The timeout period of the watchdog cannot be altered, but the below functions
* allow the watchdog to be disabled (for debug purposes), kicked (reset the two
* two second countdown), or tripped (trigger a soft reset manually).
*/

/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/*******************************************************************************
* Function Name: wiced_hal_wdog_disable
****************************************************************************//**
*
* Disable the system watchdog. Useful for debugging when the watchdog would
* interfere and reset the system when not desired (e.g. when the system
* is connected to a debugger, etc).
*
* \param void
*
* \return void
*
* \note
* This is for debugging purposes only. Hence, there is no complementary
* "Enable Watchdog" function. Please do *not* use this function in production
* applications.
*
*******************************************************************************/
void wiced_hal_wdog_disable( void );

/*******************************************************************************
* Function Name: wiced_hal_wdog_reset_system
****************************************************************************//**
*
* Executes a soft reset of the chip by tripping the watchdog timer.
*
* \param void
*
* \return void
*
*******************************************************************************/
void wiced_hal_wdog_reset_system( void );

/*******************************************************************************
* Function Name: wiced_hal_wdog_restart
****************************************************************************//**
*
* Kicks the watchdog timer, resetting the 2 second timer. Used when a process
* might otherwise cause the watchdog to trigger a system reset.
*
* \param void
*
* \return void
*
*******************************************************************************/
void wiced_hal_wdog_restart( void );

/** \} watchdog */

#endif // __WICED_WDOG_RESET_H__
