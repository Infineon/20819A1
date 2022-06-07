/**************************************************************************//**
* \file <wiced_bt_trace.h>
*
* \brief
* 	Enables  tracing for the application
* 	to particular uart for debug and logging.
*
*//*****************************************************************************
* Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
/**
* \addtogroup  wiced_utils  AIROC Trace Utilities
* \ingroup     wicedsys
*
* @{
*
* Trace Utilities
*
* \brief
* Support for applications to send debug or information messages
* to debug UART destinations.
******************************************************************************/

#ifndef _WICED_BT_TRACE_H_
#define _WICED_BT_TRACE_H_

#include <stdarg.h>
#include "string.h"
#include "wiced.h"
#include "wiced_bt_types.h"


/** Debug trace message destinations.
* Used when calling wiced_set_debug_uart().*/
typedef enum
{
    WICED_ROUTE_DEBUG_NONE,             /** < No Traces */
    WICED_ROUTE_DEBUG_TO_WICED_UART,    /**< Set to WICED UART to send debug
                                             strings over the AIROC debug interface */
    WICED_ROUTE_DEBUG_TO_HCI_UART,      /**< Use this to direct the debug traces over HCI  UART */
    WICED_ROUTE_DEBUG_TO_DBG_UART,      /**< Use this to direct the debug traces over debug UART */
    WICED_ROUTE_DEBUG_TO_PUART          /**< Use this to direct the debug traces over peripheral UART */
}wiced_debug_uart_types_t;



/******************************************************************************
* Macro         WICED_BT_TRACE
*
* The utility macro to output trace messages to the debug UART destination.
*
* Uses this printf()-style macro to print custom messages from the application code.
* Standard printf() % format arguments supported include: 'u,d,i,x,X,c,s'
*
* In addition, a custom %B format argument is provided to conveniently print
* 6-octect Bluetooth addresses.  Supply a byte array as the corresponding argument
* to match the %B format specifier.  For example:
*
* WICED_BT_TRACE("Received inquiry response from: %B\n", p_inquiry_result->remote_bd_addr);
******************************************************************************/

#ifdef WICED_BT_TRACE_ENABLE
	#define WICED_BT_TRACE(...)                 wiced_printf(NULL, 0, __VA_ARGS__)
	#define WICED_BT_TRACE_CRIT(...)            wiced_printf(NULL, 0, __VA_ARGS__)
	#define WICED_BT_TRACE_ARRAY(ptr, len, ...) wiced_printf(NULL, 0, __VA_ARGS__); wiced_trace_array(ptr, len);
#else
	#define WICED_BT_TRACE(format, ...)
	#define WICED_BT_TRACE_ARRAY(ptr, len, ...)
#endif

/*******************************************************************************
* Function Name:        wiced_trace_array
****************************************************************************//**
*
* \param[in]      array      : Pointer to array to be printed
* \param[in]      len        : Length of array to be printed
*
* \return         void
*
*******************************************************************************/
void wiced_trace_array( const uint8_t* p_array, uint16_t len);

/*******************************************************************************
* Function Name:        wiced_printf
****************************************************************************//**
*
* \param[in]      buffer     : Pointer to character buffer
* \param[in]      len        : Length of character buffer
*
* \return         void
*
*******************************************************************************/
int wiced_printf(char * buffer, int len, ...);

/*******************************************************************************
* Function Name:        wiced_bt_trace_enable
****************************************************************************//**
* Enables the trace messages to UART for the selected wiced_debug_uart_types_t
*
* \param[in]      void
*
* \return         void
*
*******************************************************************************/
void wiced_bt_trace_enable(void);

/*******************************************************************************
* Function Name:        wiced_set_debug_uart
****************************************************************************//**
* Specifies the UART to be used for debug traces.
*
* \param[in]      uart        : The UART to be used.
*
* \return          void
*
*******************************************************************************/
void wiced_set_debug_uart ( wiced_debug_uart_types_t uart );

#endif /* _WICED_BT_TRACE_H_ */
