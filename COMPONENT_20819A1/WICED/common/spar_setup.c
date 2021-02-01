/*
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
 */
/*
********************************************************************
*    File Name: spar_setup.c
*
*           The Stackable Patch and Application Runtime
*
*    Abstract: C-runtime setup of this SPAR tier
*
********************************************************************
*/

#include "spar_utils.h"
#include "sparcommon.h"
#include "string.h"
#include "brcm_fw_types.h"
#include "wiced_platform.h"
#include "wiced_memory_pre_init.h"

// Disable startup calibrarion that can cause 900 ms delay in startup
#ifndef DO_RF_CALIBRATION
#define DISABLE_RF_CALIBRATION
#endif

#define BLE_MEM_BLOCK 1
/*****************************************************************
*   External definitions
*
*****************************************************************/
extern BYTE* g_dynamic_memory_MinAddress;
extern BYTE* g_aon_memory_manager_MinAddress;
extern void install_libs(void);
extern void application_start( void );
extern wiced_bool_t micro_bcsIsNormalModeTransition();
extern void pmu_Enable32KhzOscillator(void);

__attribute__((section(".app_init_code"))) void application_start_internal( void )
{
    wiced_platform_init();
    #if USE_32K_EXTERNAL_LPO
    // Switching to external LPO
    pmu_Enable32KhzOscillator();
    #endif
    application_start();
}

/*****************************************************************
 *   Function: spar_setup()
 *
 *   Abstract: Process the information in .secinfo, copying and
 *   clearing sections as needed.
 *
 *
 *****************************************************************/
#ifndef __GNUC__
#pragma arm section code = "spar_setup"
void SPAR_CRT_SETUP(void)
{
    typedef struct
    {
        UINT32 source;
        UINT32 target;
        UINT32 len;
    } armlink_copy_secinfo_t;

    extern void *_tx_initialize_unused_memory;
    extern UINT32 Region$$Table$$Base;
    extern UINT32 Region$$Table$$Limit;

    extern UINT32 Image$$SPAR_DRAM_ZI_AREA$$ZI$$Base;
    extern UINT32 Image$$SPAR_DRAM_ZI_AREA$$ZI$$Length;
    extern UINT32 Image$$first_free_section_in_spar_NV_RAM$$Base;
    extern UINT8 g_ble_max_connections;

    armlink_copy_secinfo_t *cpysecinfo;

    // Get the section info base of this spar slice
    UINT32 cpysecinfobase = (UINT32)&Region$$Table$$Base;
    UINT32 cpysecinfolim = (UINT32)&Region$$Table$$Limit;

    UINT32 clrsecbase = (UINT32)&Image$$SPAR_DRAM_ZI_AREA$$ZI$$Base;
    UINT32 clrseclen = (UINT32)&Image$$SPAR_DRAM_ZI_AREA$$ZI$$Length;
    UINT32 endofspar = (UINT32)&Image$$first_free_section_in_spar_NV_RAM$$Base;
    UINT8  pin_val = 7;
    // Here we ought to assert that we're linked against the right
    // image, before we call memcpy/memset in ROM/Flash...

    if(cpysecinfobase != cpysecinfolim)
    {
        // Section info length is not zero
        // which means that there is RW data
        cpysecinfo = (armlink_copy_secinfo_t *)cpysecinfobase;
        memcpy((void *)cpysecinfo->target, (void *)cpysecinfo->source, cpysecinfo->len);
    }

    // Clear ZI section
    if(clrseclen != 0)
        memset((void *)clrsecbase, 0x00, clrseclen);

    // And move avail memory above this spar if required
    // Note that if there are other spars will be placed with minimum
    // alignment (because of the linker option to IRAM_SPAR_BEGIN) and itself
    // is responsible for moving the avail mem ptr.
    g_dynamic_memory_MinAddress = (BYTE *)endofspar;

    // Install included libraries and patches if any
    install_libs();

    // Max BLE connections by default are 7. To increase number of connections
    // uncommnent below and update. Note that max connections are limited
    // by application memory.
    // g_ble_max_connections = 7 + BLE_MEM_BLOCK;

    // WAR please refer to BTSDK-59 resolution comments.
    memcpy((void *)0x200BAE, (void *)&pin_val, sizeof(UINT8));

    // Setup the application start function.
    wiced_bt_set_app_start_function(application_start_internal);
}
#pragma arm section code

#else

/**
 * this weak symbol will have the default values for memory pre-init firmware allocations
 * declare this structure again, with no attribute, in app code
 * initialize with desired values to override
 */
WICED_MEM_PRE_INIT_CONTROL g_mem_pre_init __attribute__((weak)) =
{
    WICED_MEM_PRE_INIT_IGNORE,
    WICED_MEM_PRE_INIT_IGNORE,
    WICED_MEM_PRE_INIT_IGNORE,
    WICED_MEM_PRE_INIT_IGNORE,
    WICED_MEM_PRE_INIT_IGNORE
};


__attribute__ ((section(".spar_setup")))
void SPAR_CRT_SETUP(void)
{
    extern void* spar_iram_bss_begin;
    extern unsigned spar_iram_data_length, spar_iram_bss_length;
    extern void* spar_irom_data_begin, *spar_iram_data_begin, *spar_iram_end, *aon_iram_end;

    UINT8  pin_val = 7;

#ifdef DISABLE_RF_CALIBRATION
    extern uint32_t g_rfp_config[2];
    g_rfp_config[0] = 0;
    g_rfp_config[1] = 0;
#endif

    // Initialize initialized data if .data length is non-zero and it needs to be copied from NV to RAM.
    if(((UINT32)&spar_irom_data_begin != (UINT32)&spar_iram_data_begin) && ((UINT32)&spar_iram_data_length != 0))
        memcpy((void*)&spar_iram_data_begin, (void*)&spar_irom_data_begin, (UINT32)&spar_iram_data_length);

    // // Clear the ZI section
    if((UINT32)&spar_iram_bss_length != 0)
    {
        memset((void*)&spar_iram_bss_begin, 0x00, (UINT32)&spar_iram_bss_length);
    }

    // And move avail memory above this spar if required
    // Note that if there are other spars will be placed with minimum
    // alignment (because of the linker option to IRAM_SPAR_BEGIN) and itself
    // is responsible for moving the avail mem ptr.
    g_dynamic_memory_MinAddress = (BYTE *)(((UINT32)&spar_iram_end + 32) & 0xFFFFFFF0);
    if( !micro_bcsIsNormalModeTransition() )
    {
        g_aon_memory_manager_MinAddress = (BYTE *)(&aon_iram_end);
    }

    // WAR please refer to BTSDK-59 resolution comments.
    memcpy((void *)0x200BAE, (void *)&pin_val, sizeof(UINT8));

    // Install included libraries and patches if any
    install_libs();

    // optimize memory - use WICED_MEM_PRE_INIT_IGNORE to make no change
    wiced_memory_pre_init(&g_mem_pre_init);

    // Setup the application start function.
    wiced_bt_app_pre_init = application_start_internal;
}

#endif
