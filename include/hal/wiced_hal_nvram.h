/***************************************************************************//**
* \file <wiced_hal_nvram.h>
*
* Provides the API definitions for NVRAM access.
*
*\\*****************************************************************************
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

#ifndef _WICED_HAL_NVRAM_H_
#define _WICED_HAL_NVRAM_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup nvram NVRAM
 * \ingroup HardwareDrivers
 * \{
 *
 * This driver provides simplified access to the volatile section ("VS") of
 * memory.
 *
 * The following APIs allow the application layer to easily read and write
 * into VS section so that persistent data can be stored through a power
 * cycle. The most common use of these APIs will be storing link keys
 * of peer devices in order to bond with those devices.
 *
 * The VS_ID's used as parameters to the read and write functions below are
 * merely handles used to refer to the data stored by the app. They do not
 * indicate the physical storage location of the memory. It is not necessary
 * to use sequential VS_ID's, nor is it necessary to use them in ascending
 * order. The VS_ID is nothing more than a file name and any VS_ID can be
 * used that lies in the range 0x200 - 0x3fff.
 *
 * The amount of available memory using the NVRAM APIs defaults to 4KB. This
 * can be increased if more space is needed, but it will reduce the amount
 * of memory available to the application.
 *
 * In order to fully utilize the 4K of memory available, care must be taken
 * by the developer. The 4K of memory is broken up into blocks of 512 bytes
 * and the NVRAM APIs will not handle the wrap-around between these blocks.
 * For small numbers of bytes written (1-50), this limitation is not of
 * significant concern. Larger write sizes, must exercise caution.
 * For example, if the NVRAM API is used to write eight 260 byte chunks in
 * a row, then it will end up consuming all 4K of physical memory. Because two
 * 260 byte chunks cannot fit into a single block, each one is allocated it's
 * own page.
 *
 * The max size that can be written to a single VS_ID is 500 due to the
 * limitation of the 512 block size. The remaining 12 bytes are header space.
 * The more VS_ID's used, the less overall space will be used because there
 * will be less headers to store.
 *
 * \note
 * The term "NVRAM" used in this context simply means that the data stored there
 * is persistent through power cycles. It does not refer to any particular type
 * of memory (e.g. powered RAM). For 20719 and 20819 utilizing on-chip flash,
 * while the 20706 will use its external boot flash.
 *
 * \defgroup group_nvram_enums Enumerated Types
 * \defgroup group_nvram_functions Functions
 *
 */

/******************************************************************************
 * Global Enumerations definitions                                            *
 ******************************************************************************/

/**
* \addtogroup group_nvram_enums
* \{
*/

/** Defines the valid range for vs_id's available to application. */
enum
{
    WICED_NVRAM_VSID_START              = 0x200,
    WICED_NVRAM_VSID_END                = 0x3FFF
};

/** \} group_nvram_enums */


/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/**
* \addtogroup group_nvram_functions
* \{
*/

/*******************************************************************************
* Function Name: wiced_hal_write_nvram
****************************************************************************//**
*
* Write data to NVRAM. Each VS_ID can store 500 bytes. The range of VS_IDs
* available to write to is enumerated above as the range of
* WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END. The p_status parameter is
* used to indicate additional information about type of failure if return
* code is zero. Possible values for p_status:
*  - WICED_BADARG indicates vs_id out of range
*  - WICED_SUCCESS indicates success and return code indicates num written
*  - WICED_ERROR indicates failure, returned zero bytes written
*
* \param[in]  vs_id         volatile section ID
* \param[in]  data_length   num bytes to write to NVRAM (max 500)
* \param[in]  p_data        pointer to the data to be written to NVRAM
* \param[out] p_status		result code returned
*
* \return
*  - non-zero value indicates number of bytes written
*  - 0 indicates failure
*
* \note
* This function can only be called from the application thread.
*
*******************************************************************************/
uint16_t wiced_hal_write_nvram( uint16_t vs_id, uint16_t data_length, uint8_t * p_data, wiced_result_t * p_status );

/*******************************************************************************
* Function Name: wiced_hal_read_nvram
****************************************************************************//**
*
* Reads data from NVRAM, which was previously written by the application to a
* specific VS_ID using \ref wiced_hal_write_nvram. Use the same VS_ID to restore
* the data to the input memory buffer. The p_status parameter is used to
* indicate additional information about type of failure if return code is zero.
* Possible values for p_status:
*  - WICED_BADARG indicates vs_id out of range
*  - WICED_SUCCESS indicates success and return code indicates num written
*  - WICED_ERROR indicates failure, returned zero bytes written
*
* \param[in]  vs_id         volatile section ID
* \param[in]  data_length   num bytes to read from NVRAM (max 500)
* \param[out] p_data        allocated buffer into which data can be memcpy'd
* \param[out] p_status		result code returned
*
* \return
*  - non-zero value indicates number of bytes read
*  - 0 indicates failure
*
* \note
* This function can only be called from the application thread.
*
*******************************************************************************/
uint16_t wiced_hal_read_nvram( uint16_t vs_id, uint16_t data_length, uint8_t * p_data, wiced_result_t * p_status );

/*******************************************************************************
* Function Name: wiced_hal_delete_nvram
****************************************************************************//**
*
* Deletes data that exists at a specific, previously written vs_id. It's _not_
* necessary to call this function in order to overwrite new data to a vs_id.
* The p_status parameter is used to indicate additional information about type
* of failure if return code is zero. Possible values for p_status:
*  - WICED_BADARG indicates vs_id out of range
*  - WICED_SUCCESS indicates success and return code indicates num written
*  - WICED_ERROR indicates failure, returned zero bytes written
*
* \param[in]  vs_id         volatile section ID
* \param[out] p_status		result code returned
*
* \return void
*
* \note
* This function can only be called from the application thread.
*
*******************************************************************************/
void wiced_hal_delete_nvram( uint16_t vs_id, wiced_result_t * p_status );

/** \} group_nvram_enums */

/** \} nvram */

#ifdef __cplusplus
}
#endif

#endif //_WICED_HAL_NVRAM_H_
