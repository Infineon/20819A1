/*******************************************************************************
 * All common definitions for this SPAR
 *******************************************************************************/

#ifndef _SPAR_COMMON_H_
#define _SPAR_COMMON_H_

#include "brcm_fw_types.h"
#include "spar_utils.h"

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C extern
#endif

EXTERN_C void (*wiced_bt_app_pre_init)(void );

EXTERN_C void application_start( void );

#define APPLICATION_START() void application_start( void )

// TODO: Other technologies.

#endif
