/***************************************************************************//**
* \file <wiced_bt_nvram_coredump.h>
*
* Provides the API definitions for the nvram coredump interface used for the
* dbfw_lib.a library
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

#ifndef __WICED_BT_NVRAM_COREDUMP_H__
#define __WICED_BT_NVRAM_COREDUMP_H__

/**
* \addtogroup DBFW AIROC NVRAM Coredump
* \ingroup wicedsys
* \{
* Defines coredump nvram management interface
*/

/******************************************************************************
 * Defines                                                                    *
 ******************************************************************************/
#define DBFW_COREDUMP_SP_DUMP_SIZE  80


/******************************************************************************
 * Structures                                                                 *
 ******************************************************************************/
typedef struct
{
    UINT32 sp;
    UINT32 pc;
    UINT32 lr;
    UINT32 r[7];     /* R0-R6 */
} arm_cm3_regs_t;

typedef struct
{
    arm_cm3_regs_t  regs;
    UINT8           call_stack[DBFW_COREDUMP_SP_DUMP_SIZE];
} wiced_bt_nvram_coredump_t;


/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/******************************************************************************
* Function Name: wiced_bt_nvram_coredump_enable
***************************************************************************//**
*
* Enables writing the coredump to the VS section of nvram. Note: by default
* the coredump is sent as VSC events over the uart transport.
*
******************************************************************************/
void wiced_bt_nvram_coredump_enable(void);

/******************************************************************************
* Function Name: wiced_bt_nvram_coredump_is_found
***************************************************************************//**
*
* This function checks if there is coredump data stored in nvram.
*
* \param p_data
* Pointer to wiced_bt_nvram_coredump_t structure to be filled if data is found
*
* \return
* TRUE is coredump data is found, else FALSE
*
******************************************************************************/
BOOL8 wiced_bt_nvram_coredump_is_found(wiced_bt_nvram_coredump_t* p_data);

/******************************************************************************
* Function Name: wiced_bt_nvram_coredump_delete
***************************************************************************//**
*
* This function deletes the coredump data that was stored in nvram. Note: the
* coredump data will not be overwritten, ie the data must be deleted before
* any subsequent coredump data is saved.
*
******************************************************************************/
void wiced_bt_nvram_coredump_delete(void);

#endif //__WICED_BT_NVRAM_COREDUMP_H__
