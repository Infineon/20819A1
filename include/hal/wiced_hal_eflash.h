/***************************************************************************//**
* \file <wiced_hal_eflash.h>
*
* Provides the API definitions for the Embedded Flash interface driver.
*
********************************************************************************
* \copyright
* Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef _WICED_HAL_EFLASH_H_
#define _WICED_HAL_EFLASH_H_

/** \addtogroup Platform config - Default flash (i.e. flash exists on WICED eval boards) configuration.
*   \ingroup HardwareDrivers
*/
/*! @{ */
/******************************************************
 *                   Defines
 ******************************************************/
/**
 *  819 uses 512 byte sector, 256KB on chip flash.
 */
#define FLASH_SECTOR_SIZE           (0x200u)
#define FLASH_SIZE                  0x40000
#define FLASH_BASE_ADDRESS          (0x500000u)

/** Number of sectors reserved from the end of the flash for the application
 *  specific purpose (for ex: to log the crash dump). By default no reservation
 Note:- 16K of flash is used for internal firmware operation.
 (Remaining of the flash - reservation) can be divided equally and used for active
 and upgradable firmware. So, application should care the OTA firmware (application+patch)
 size while reserving using below.
 */
#define APPLICATION_SPECIFIC_FLASH_RESERVATION  0
/* @} */

/**
* \addtogroup EmbeddedFlashInterfaceDriver Embedded Flash Interface
* \ingroup HardwareDrivers
* \{
* Defines a driver for the Embedded Flash interface.
*/

/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/******************************************************************************
* Function Name: wiced_hal_eflash_init
***************************************************************************//**
*
* Initializes the embedded flash.
*
* \return
*  - WICED_SUCCESS
*  - WICED_ERROR
*
******************************************************************************/
wiced_result_t wiced_hal_eflash_init( void );


/******************************************************************************
* Function Name: wiced_hal_eflash_read
***************************************************************************//**
*
* Reads the data from eflash.
*
* \param offset
* The start offset of eflash from which data to be read, offset should be word aligned
*
* \param p_buffer
* The pointer to the buffer to which data is to be read, buffer should be word aligned
*
* \param length
* The length of data in byte's to be read
*
* \return
*  - WICED_SUCCESS
*  - WICED_ERROR
*
******************************************************************************/
wiced_result_t wiced_hal_eflash_read( uint32_t offset, uint8_t* p_buffer, uint32_t length );


/******************************************************************************
* Function Name: wiced_hal_eflash_write
***************************************************************************//**
*
* Writes the data to eflash. Interrupts will be locked for the duration of the write.
* And only writes to an already erased location is valid.
*
* \param offset
* The start offset of eflash to be written, offset should be word aligned
*
* \param p_buffer
* The pointer to the buffer from which data will be written, buffer should be byte,
* half-word, or word aligned.
*
* \param length
* The length of data in byte's to be written, length should be word aligned
*
* \return
*  - WICED_SUCCESS
*  - WICED_ERROR
*
******************************************************************************/
wiced_result_t wiced_hal_eflash_write( uint32_t offset, uint8_t* p_buffer, uint32_t length );


/******************************************************************************
* Function Name: wiced_hal_eflash_erase
***************************************************************************//**
*
* Erase the eflash. Erase is performed page-wise. eflash page size is 0x200 bytes.
* eflash page count is 0x200.
*
* \param offset
* The start offset of eflash page to be erased.
*
* \param length
* The length of data in byte's to be erased, erase performed page-wise.
*
* \return
*  - WICED_SUCCESS
*  - WICED_ERROR
*
******************************************************************************/
wiced_result_t wiced_hal_eflash_erase( uint32_t offset, uint32_t length );


/******************************************************************************
* Function Name: wiced_hal_eflash_erase_whole
***************************************************************************//**
*
* Erase the whole eflash.
*
* \return
*  - WICED_SUCCESS
*  - WICED_ERROR
*
******************************************************************************/
wiced_result_t wiced_hal_eflash_erase_whole( void );


/******************************************************************************
* Function Name: wiced_hal_eflash_get_size
***************************************************************************//**
*
* Gets the size of the eflash.
*
* \return
* eflash size
*
******************************************************************************/
uint32_t wiced_hal_eflash_get_size( void );

/** \} EmbeddedFlashInterfaceDriver */

#endif // _WICED_HAL_EFLASH_H_


/* [] END OF FILE */
