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

void wiced_ofu_pet_watchdog(void);
void wiced_ofu_reset_device(void);

#define WICED_OFU_DEFAULT_SPI_CLK 12000000
void wiced_ofu_sflash_init(uint32_t spi_clk_hz);

uint32_t wiced_ofu_get_ds1_offset(void);

typedef UINT32 (*POST_CONFIG_CALLBACK)(int);
void wiced_ofu_set_post_config_callback(POST_CONFIG_CALLBACK cb);

#define OFU_CRYPT_TYPE_NONE         0
#define OFU_CRYPT_TYPE_AES_CFB128   1
#define OFU_CRYPT_TYPE_AES_CTR      2

uint32_t wiced_ofu_get_external_storage_context_size(void);
BOOL32 wiced_ofu_new_external_storage_key(wiced_bool_t encrypt, uint32_t type, void *handle);
BOOL32 wiced_ofu_store_external_storage_key(void *handle);
BOOL32 wiced_ofu_restore_external_storage_key(void *handle);
BOOL32 wiced_ofu_delete_external_storage_key(void *handle);
BOOL32 wiced_ofu_crypt( wiced_bool_t encrypt, uint32_t offset, uint32_t length, const uint8_t *input,
                        uint8_t *output, void *handle);
void wiced_ofu_store_image_length(uint32_t length, void *handle);
uint32_t wiced_ofu_get_image_length(void *handle);
void wiced_ofu_enter_eflash_write_or_erase(void);
void wiced_ofu_leave_eflash_write_or_erase(void);
