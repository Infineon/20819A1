/***************************************************************************//**
* \file <wiced_bt_event.h>
*
* Enables tasks to be serialized to the pre-existing application thread.
* The application thread is used by the underlying WICED stack to trigger
* stack events and asynchronous callbacks.
*
* \note
* To implement a custom thread or worker thread (employing task queue),
* see \ref wiced_rtos.h.
*
* \defgroup appthread Application Thread Serialization
* \ingroup  rtos
*
********************************************************************************
* \copyright
* Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef __WICED_BT_EVENT_H__
#define __WICED_BT_EVENT_H__

#include "wiced.h"

/** \cond INTERNAL */
/******************************************************************************
 * Global Enumerations definitions                                            *
 ******************************************************************************/

typedef enum
{
    WICED_SERIALIZATION_EVENT = 1,
} wiced_bt_internal_events_t;

/** \endcond */

/**
* \addtogroup appthread
* \{
*/

/******************************************************************************
 * Global Data Structure definitions                                          *
 ******************************************************************************/

 /** Structure used in WICED stack to add callback and data into task queue.  The serialization queue will have these callbacks */
typedef struct
{
    int (*fn)(void*); /**< Callback invoked within the app thread context */
    void* data; /**< Any arbitrary data to be given to the callback.  wiced_app_event_serialize Caller has to allocate and free once serialized event handled */
} wiced_app_event_srzn_cb_t;


/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/


/*******************************************************************************
* Function Name: wiced_app_event_serialize
****************************************************************************//**
*
* This function lets you serialize a call onto the application thread, which
* has been instantiated by the WICED stack and is used to interact with the
* application in an event-based fashion. Once serialized, tasks are pushed onto
* a task queue, where they are pulled based on pre-defined priority of the
* application thread. The queue is 16 deep, but this is shared with the stack.
*
* \param[in] fn         function to execute once dequeued from app thread
* \param[in] data       pointer to non-local data to be sent as arg to callback
*
* \return
*  - WICED_TRUE indicates success
*  - WICED_FALSE indicates failure
*
* \note
* The data parameter must point to non-local data as the pointer will be
* accessed after the current function returns.
*
* \note
* Data will not be freed by stack. If allocated by app, must be freed by app.
*
*******************************************************************************/
wiced_bool_t wiced_app_event_serialize( int (*fn)(void*), void* data );

/** \} appthread */

#endif //__WICED_BT_EVENT_H__
