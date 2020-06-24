/***************************************************************************//**
 * \file <wiced_timer.h>
 *
 * \brief Provides the API declarations for Timer services
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
#ifndef _WICED_TIMER_H_
#define _WICED_TIMER_H_


#include "wiced.h"
#include "wiced_result.h"
#ifdef __cplusplus
extern "C"
{
#endif


/**
 * \addtogroup timer Timer Management Services
 * \ingroup HardwareDrivers
 * @{
 *
 * Provides the API declarations for Timer services
 */


/******************************************************************************
 * Global Enumerations definitions
 ******************************************************************************/
/**
 * \addtogroup group_timer_enums
 * \{
 */

/** WICED Timer Types */
enum wiced_timer_type_e
{
    WICED_SECONDS_TIMER = 1,
    WICED_MILLI_SECONDS_TIMER, /* The minimum resolution supported is 1 ms */
    WICED_SECONDS_PERIODIC_TIMER,
    WICED_MILLI_SECONDS_PERIODIC_TIMER /*The minimum resolution supported is 1 ms */
};

typedef uint8_t  wiced_timer_type_t;/* (see #wiced_timer_type_e) */

/** \} group_timer_enums */

/*
 * Function prototype for the timer callbacks.
 *
 */
#if defined  _WIN32 || defined WICEDX || defined __ANDROID__ || defined __APPLE__
#define TIMER_PARAM_TYPE    void *
#else
#define TIMER_PARAM_TYPE   uint32_t
#endif

typedef void ( *wiced_timer_callback_t )( TIMER_PARAM_TYPE cb_params );

/**
 * Defines the WICED timer instance size
 */
#if defined _WIN32 || defined WICEDX || defined __ANDROID__ || defined __APPLE__
    #define WICED_TIMER_INSTANCE_SIZE_IN_WORDS      17
#else
    #define WICED_TIMER_INSTANCE_SIZE_IN_WORDS      14
#endif

/******************************************************************************
 * Global Data Structure definitions
 ******************************************************************************/
/**
 * \addtogroup group_timer_data_structures
 * \{
 */

/*
 * Defines the WICED timer structure
 * requires. Timer module doesn't use any dynamic memory and the number of timers
 * depends on the memory available to the application to define the timers
 */
typedef struct
{
	uint32_t reserved[WICED_TIMER_INSTANCE_SIZE_IN_WORDS];
}wiced_timer_t;

/** \} group_timer_data_structures */

/******************************************************************************
 * Function Name: wiced_init_timer
 ******************************************************************************
 * Initializes the timer

 *
 * \param   p_timer     Pointer to the timer structure
 * \param   p_cb        Timer callback function to be invoked on timer expiry
 * \param   cb_param    Parameter to be passed to the timer callback function which gets invoked on timer expiry, if any
 * \param   timer_type  Type of the timer
 *
 * \return  wiced_result_t
 * - WICED_SUCCESS if timer was initialized successfully
 * - Refer to WICED_RESULT_LIST for list of Error/failure codes
*******************************************************************************/
wiced_result_t wiced_init_timer( wiced_timer_t* p_timer, wiced_timer_callback_t TimerCb,
                                 TIMER_PARAM_TYPE cBackparam, wiced_timer_type_t type);

/******************************************************************************
 * Function Name: wiced_start_timer
 ******************************************************************************
 * Starts the timer
 * the next timer among the active timers expires.
 *
 * \param   wiced_timer_t   Pointer to the timer structure
 * \param   timeout			timeout value
 *
 * \return  wiced_result_t
 * - WICED_SUCCESS if timer was activated/started successfully.
 * - Refer to WICED_RESULT_LIST for list of Error/failure codes
*******************************************************************************/

wiced_result_t wiced_start_timer(wiced_timer_t* p_timer, uint32_t timeout);

/******************************************************************************
 * Function Name: wiced_stop_timer
 ******************************************************************************
 * Stops the timer
 *
 * \param   wiced_timer_t   Pointer to the timer structure
 *
 * \return  wiced_result_t
 * - WICED_SUCCESS if timer was deactivated/stopped successfully.
 * - Refer to WICED_RESULT_LIST for list of Error/failure codes
*******************************************************************************/

wiced_result_t wiced_stop_timer(wiced_timer_t* p_timer);

/******************************************************************************
 * Function Name: wiced_is_timer_in_use
 ******************************************************************************
 * Checks if the timer is in use
 *
 * \param   p_timer     Pointer to the timer structure
 *
 * \return   wiced_bool_t
 * - WICED_TRUE if the timer is in use
 * - WICED_FALSE if the timer is not in use
*******************************************************************************/
wiced_bool_t wiced_is_timer_in_use(wiced_timer_t* p_timer);

/******************************************************************************
 * Function Name: wiced_deinit_timer
 ******************************************************************************
 * Deinitialize the timer instance and stops the timer if it is running
 *
 * \param   p_timer     Pointer to the timer
 *
 * \return  wiced_result_t
 * - WICED_SUCCESS if timer was deinitialized successfully
 * - Refer to WICED_RESULT_LIST for list of Error/failure codes
*******************************************************************************/
wiced_result_t wiced_deinit_timer(  wiced_timer_t* p_timer );

/** @} */
#ifdef __cplusplus
}
#endif

#endif //_WICED_TIMER_H_

/** \} group_timer */
