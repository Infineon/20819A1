/*
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
 */

/***************************************************************************//**
* \file <wiced_hal_i2c.h>
*
* List of parameters and defined functions needed to access the
* Inter-Integrated Circuit (I2C, IIC) driver.
*
*******************************************************************************/

#ifndef __WICED_I2C_DRIVER_H__
#define __WICED_I2C_DRIVER_H__

#include "brcm_fw_types.h"

/**
* \addtogroup I2CDriver I2C
* \ingroup HardwareDrivers
* \{
*
* Defines an I2C driver to facilitate communication with other devices on an
* I2C bus (such as a temperature sensor, etc). The driver is only capable of
* assuming a master role. Applications use this driver to obtain the status
* from and control the behavior of the I2C hardware. This driver only offers
* services for speed control and data transfer operations.
*
* Please note that even though this driver can access off-chip memory
* (if installed; EEPROM, etc), please use the drivers found in
* wiced_hal_ieeprom.h to access those modules, as those drivers include
* checks to ensure safe data handling operations. This driver is intended
* only to interface with other devices on the I2C bus, such as a motion
* sensor.
*
*/

/******************************************************************************
*** Parameters.
***
*** The following parameters are used to configure the driver or define
*** return status. They are not modifiable.
******************************************************************************/

/** Enumeration of speed options */
enum
{
    I2CM_SPEED_100KHZ = 240, /**< I2C speed is 100kHz */
    I2CM_SPEED_400KHZ = 60,  /**< I2C speed is 499kHz */
    I2CM_SPEED_800KHZ = 30,  /**< I2C speed is 800kHz */
    I2CM_SPEED_1000KHZ = 24, /**< I2C speed is 1MHz */
};

/** Enumeration of I2C transaction return values */
enum
{
    I2CM_SUCCESS,   /**< I2C transaction was successful */
    I2CM_OP_FAILED, /**< I2C transaction failed, possibly due to no ACK from slave */
    I2CM_BUSY       /**< I2C HW block is busy with another transaction */
};

/******************************************************************************
*** Function prototypes.
******************************************************************************/

/*******************************************************************************
* Function Name: wiced_hal_i2c_init
****************************************************************************//**
*
* Initializes the I2C driver and its private values. This initialization
* sets the bus speed to 100KHz by default (I2CM_SPEED_100KHZ). To make
* the bus run at another speed, call wiced_hal_i2c_setSpeed(<speed>)
* right after this call.
*
*
* \param none
*
* \return none
*
* \funcusage
* \snippet demo/hid/ble_remote
*          hal/i2c_master
*
* \note
* (!) Note that it is the user's responsibility to configure the desired
* GPIOs, since different HW platforms will have different configuration
* parameters. Please see the Kit Guide or HW User Manual for your device
* for more information.
*
*******************************************************************************/
void wiced_hal_i2c_init(void);


/*******************************************************************************
* Function Name: wiced_hal_i2c_set_speed
****************************************************************************//**
*
* Sets the I2C bus speed.
*
* \param[in] speed  The new speed to be used.
*                   Refer to the speeds in the parameter section
*
* \return none
*
* \funcusage
* \snippet demo/hid/ble_remote
*          hal/i2c_master
*
* \note
* By default, the speed after initialization is set to 100KHz
*
* \note
* The clock frequency is determined by the input parameter which is the transport
* clock counter that counts the number of reference clock cycles
*
*******************************************************************************/
void wiced_hal_i2c_set_speed(uint8_t speed);


/*******************************************************************************
* Function Name: wiced_hal_i2c_get_speed
****************************************************************************//**
*
* Gets the current I2C bus speed. Smaller numbers indicate higher speeds.
*
* \param   none
*
* \return  The SCL clock cycle counter value which corresponds to the SCL speed
* *        Refer to the speeds in the parameter section
*
* \funcusage
* \snippet demo/hid/ble_remote
*          hal/i2c_master
*
*******************************************************************************/
uint8_t wiced_hal_i2c_get_speed(void);


/*******************************************************************************
* Function Name: wiced_hal_i2c_read
****************************************************************************//**
*
* Reads data into given buffer from the I2C HW addressing a particular
* slave address. The data bytes are transparent. Though any arbitrary
* length of data may be read from the slave, atomic transactions greater
* than HW's capability are not possible - they will be split into multiple
* transactions. This is a blocking call. Interrupts will not affect timing
* within a transaction.
*
* \param[in,out] data  Pointer to a buffer that will hold the incoming data
* \param[in] length    The length in bytes of the data to read
* \param[in] slave     The 7-bit slave address
*
* \return              The status of the transaction. Refer to the
*                      transaction return values in the parameters section
*
* \funcusage
* \snippet hal/i2c_master
*
* \note
* Please see the Kit Guide or HW User Manual for your device for the
* actual limitation of the part on your platform.
*
* \note
* The <data> buffer should be large enough to <length> bytes
*
*******************************************************************************/
uint8_t wiced_hal_i2c_read(uint8_t* data, uint16_t length, uint8_t slave);


/*******************************************************************************
* Function Name: wiced_hal_i2c_write
****************************************************************************//**
*
* Writes the given data to the I2C HW addressing a particular slave address.
* The data bytes are transparent. Though any arbitrary length of data may be
* written to the slave, atomic transactions greater than HW's capability
* are not possible - they will be split into multiple transactions. This is
* a blocking call. Interrupts will not affect timing within a transaction.
*
* \param[in] data    Pointer to a buffer that will hold the outgoing data
* \param[in] length  The length in bytes of the data to write
* \param[in] slave   The 7-bit slave address
*
* \return            The status of the transaction. Refer to the
*                    transaction return values in the parameters section
*
* \funcusage
* \snippet demo/hid/ble_remote
*          hal/i2c_master
*
* \note
* Please see the Kit Guide or HW User Manual for your device for the
* actual limitation of the part on your platform.
*
*******************************************************************************/
uint8_t wiced_hal_i2c_write(uint8_t* data, uint16_t length, uint8_t slave);


/*******************************************************************************
* Function Name: wiced_hal_i2c_combined_read
****************************************************************************//**
*
* Executes two transactions back-to-back with a repeated start condition between
* the first and the second. tx_data is written to the slave in the first transaction
* while data is read back from the slave into rx_data after the repeated start.
*
* \param[in, out] rx_data  Pointer to a buffer that will hold the incoming data
* \param[in] rx_data_len   The length in bytes of the data to read
* \param[in] tx_data       Pointer to a buffer that will hold the outgoing data
* \param[in] tx_data_len   The length in bytes of the data to write
* \param[in] slave         The 7-bit slave address
*
* \return                  The status of the transaction. Refer to the
*                          transaction return values in the parameters section
*
* \funcusage
* \snippet demo/hid/ble_remote
*          hal/i2c_master
*
* \note
* The <rx_data> buffer should be large enough to hold <rx_data_len> bytes
*
*******************************************************************************/
uint8_t wiced_hal_i2c_combined_read(uint8_t* rx_data, uint8_t rx_data_len, uint8_t* tx_data, uint16_t tx_data_len, uint8_t slave);

/** \} I2C */

#endif // __WICED_I2C_DRIVER_H__
