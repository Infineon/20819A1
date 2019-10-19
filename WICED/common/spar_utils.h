/*
********************************************************************
* THIS INFORMATION IS PROPRIETARY TO Cypress Semiconductor.
*-------------------------------------------------------------------
*
*           Copyright (c) 2008 Cypress Semiconductor.
*                  ALL RIGHTS RESERVED
*
********************************************************************

********************************************************************
*    File Name: spar_utils.h
*
*    Abstract: A few utilities with a reasonable implementation for
*              SPAR.
*
********************************************************************
*/

#ifndef __SPAR_UTILS_H__
#define __SPAR_UTILS_H__

#include "brcm_fw_types.h"

#ifdef __GNUC__
#define sprintf __2sprintf
#define snprintf __2snprintf
#endif

#if ENABLE_DEBUG
#include "tx_port.h"
#include "wiced_hal_wdog.h"

/// When debugging is enabled, sets up the HW for debugging.
#define SETUP_APP_FOR_DEBUG_IF_DEBUG_ENABLED()   do{        \
        if(0 == memcmp(PLATFORM, "CYBT_213", 8)) \
        { \
            wiced_hal_gpio_select_function(WICED_P12, WICED_SWDCK); \
            wiced_hal_gpio_select_function(WICED_P13, WICED_SWDIO); \
        } \
        else if(0 == memcmp(PLATFORM, "CYW98982", 8)) \
        { \
            wiced_hal_gpio_select_function(WICED_P02, WICED_SWDCK); \
            wiced_hal_gpio_select_function(WICED_P10, WICED_SWDIO); \
        } \
        else \
        { \
            wiced_hal_gpio_select_function(WICED_P02, WICED_SWDCK); \
            wiced_hal_gpio_select_function(WICED_P03, WICED_SWDIO); \
        } \
        wiced_hal_wdog_disable(); \
        }while(0)

/// Optionally waits in a pseudo while(1) until the user allows the CPU to continue
#define BUSY_WAIT_TILL_MANUAL_CONTINUE_IF_DEBUG_ENABLED()     do{   \
        volatile UINT8 spar_debug_continue = 0;                     \
        unsigned int interrupt_save = _tx_v7m_get_and_disable_int();\
        while(!spar_debug_continue);                                \
        _tx_v7m_set_int(interrupt_save);                            \
        }while(0)
#else
#define SETUP_APP_FOR_DEBUG_IF_DEBUG_ENABLED()
#define BUSY_WAIT_TILL_MANUAL_CONTINUE_IF_DEBUG_ENABLED()
#endif

/// Allow the app to place code in retention RAM.
/// Note that there is very limited retention RAM, so choose
/// what goes into this area very carefully.
#define PLACE_CODE_IN_RETENTION_RAM    __attribute__ ((section(".code_in_retention_ram")))

/// Allow app to place this data in retention RAM.
#define PLACE_DATA_IN_RETENTION_RAM    __attribute__ ((section(".data_in_retention_ram")))

// If we panic from SPAR, we might not even have access to anything in
// the ROM or the Flash -- we suspect that we've been linked against
// the wrong image. So this.

#define SPAR_ASSERT_PANIC(expr) \
    do { if (!(expr)) while (1) ; } while(0)


#endif
