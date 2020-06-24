/*
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
 */

/** @file
*
* This file lists the API's and structs required to access the
* Pulse-Width Modulation (PWM) driver.
*
*/

#ifndef __WICED_PWM_H__
#define __WICED_PWM_H__

#include "wiced_bt_dev.h"

/**  \addtogroup PwmDriver Pulse Width Modulation (PWM)
* \ingroup HardwareDrivers
* @{
* Defines a driver to facilitate interfacing with the Pulse-Width
* Modulation (PWM) driver.
*
* Use this driver to output a PWM signal to a GPIO pin for external use. There
* are six, 16-bit hardware PWM channels avaliable (0-5). Typical use-cases
* include fine-control over small devices such as LEDs. Please reference the
* Datasheet for your device for more information.
*
*/


/******************************************************************************
 * Global Data Structure definitions                                          *
 ******************************************************************************/

/**
* \addtogroup group_pwm_data_structures Structures
* PWM Structures
* \{
*/

/**
* PWM HW block has 6 PWM channels each with its own 16 bit counter.
* The first PWM channel is PWM0.
*/
typedef enum
{
    PWM0  = 0,
    PWM1  = 1,
    PWM2  = 2,
    PWM3  = 3,
    PWM4  = 4,
    PWM5  = 5,
    MAX_PWMS = 6
} PwmChannels;

/**
* Each PWM channel can be sourced from two clock inputs listed in PwmClockType.
* PMU_CLK is routed via ACLK1 and thus application needs to enable ACLK1 before using PMU_CLK as PWM source.
* When LHL_CLK is selected it's routed directly to PWM and input frequency is set to 32 KHz.
*/
typedef enum
{
    LHL_CLK,
    PMU_CLK
} PwmClockType;

/**
* wiced_pwm_config_t is used only in wiced_hal_pwm_get_params() utility API .
*/
typedef struct{
    UINT32 init_count;
    UINT32 toggle_count;
} pwm_config_t;

typedef pwm_config_t wiced_pwm_config_t;

/** \} group_pwm_data_structures */

/******************************************************************************
*** Global functions .
******************************************************************************/

/**
* \addtogroup group_pwm_functions Functions
* PWM Functions
* \{
*/

/******************************************************************************
* Function Name: wiced_hal_pwm_start
***************************************************************************//**
*
* Configures, enables, and starts the PWM to be active on a preconfigured GPIO pin.
*
* (!) Note that the desired GPIO pin must have already been configured
* as output. See device datasheet of your device for more information.
*
* (!) Note that if you use PMU_CLK instead of LHL_CLK, a call to
*     wiced_hal_aclk_enable() is required for configuring ACLK1.
*
* (!) Note that the maximum width or period of the PWM is 0xFFFF (16-bits).
*
* \param[in] channel      : Desired PWM channel to use [PWM0-PWM5].
*
* \param[in] PwmClockType : Clock source for this PWM channel based on desired frequency.
*
* \param[in] toggleCount  : The number of ticks to wait before toggling the signal.
*
* \param[in] initCount    : Initial value for the counter.
*
* \param[in] invert       : 1 to invert the signal.
*
* \return                 : 1 if PWM was successfully started, 0 otherwise.
*
******************************************************************************/
BOOL32 wiced_hal_pwm_start(UINT8        channel,
                                         PwmClockType clk,
                                         UINT32       toggleCount,
                                         UINT32       initCount,
                                         BOOL32       invert);


/******************************************************************************
* Function Name: wiced_hal_pwm_change_values
***************************************************************************//**
*
* Changes the PWM settings after the PWM HW has already been started.
*
*
* (!) Note that the maximum width or period of the PWM is 0xFFFF (16-bits).
*
* \param[in] channel      : Desired PWM channel to use [PWM0-PWM5].
*
* \param[in] toggleCount  : The number of ticks to wait before toggling the signal.
*
* \param[in] initCount    : Initial value for the counter.
*
* \return                 : 1 if PWM was successfully changed, 0 otherwise.
*
******************************************************************************/
BOOL32 wiced_hal_pwm_change_values(UINT8 channel,
                                                 UINT32 toggleCount,
                                                 UINT32 initCount);

/******************************************************************************
* Function Name: wiced_hal_pwm_get_toggle_count
***************************************************************************//**
*
* Returns the current toggle count setting for the corresponding PWM channel.
*
* \param[in] channel : Desired PWM channel from which to obtain the toggle count. [PWM0-PWM5]
*
* \return            : The value at which the PWM is going to toggle.
*
******************************************************************************/
UINT32 wiced_hal_pwm_get_toggle_count(UINT8 channel);

/******************************************************************************
* Function Name: wiced_hal_pwm_get_init_value
***************************************************************************//**
*
* Returns the current toggle count setting for the corresponding PWM channel.
*
* \param[in] channel : Desired PWM channel from which to obtain the initial count. [PWM0-PWM5]
*
* \return            : The initial count value of the PWM.
*
******************************************************************************/
UINT32 wiced_hal_pwm_get_init_value(UINT8 channel);

/******************************************************************************
* Function Name: wiced_hal_pwm_disable
***************************************************************************//**
*
* Disables the PWM channel.
*
* \param[in] channel : Desired PWM channel to stop/disable. [PWM0-PWM5]
*
* \return            : None
*
******************************************************************************/
void wiced_hal_pwm_disable(UINT8 channel);

/******************************************************************************
* Function Name: wiced_hal_pwm_enable
***************************************************************************//**
*
* Enables the PWM channel which is already preconfigured.
*
* \param[in] channel : Desired PWM channel to enable. [PWM0-PWM5]
*
* \return            : None
*
******************************************************************************/
void wiced_hal_pwm_enable(UINT8 channel);

/******************************************************************************
* Function Name: wiced_hal_pwm_configure_pin
***************************************************************************//**
*
* Configure any LHL GPIO to any PWM port
*
* \param[in] pin     : Desired LHL GPIO to configure as PWM from wiced_bt_gpio_numbers_t.
*
* \param[in] PWM     : Desired PWM channel to set the pin to. [PWM0-PWM5]
*
* \return            : None
*
******************************************************************************/
void wiced_hal_pwm_configure_pin( UINT8  pin, UINT8 PWM) __attribute__ ((deprecated("Please use supermux tool to configure PWM pin")));

/******************************************************************************
* Function Name: wiced_hal_pwm_get_params
***************************************************************************//**
*
* This utility API can be used calculate PWM input parameters per desired PWM output
*
* \param[in] clock_frequency_in : Input clock frequency selected for specific PWM channel.
*
* \param[in] duty_cycle         : Ducy cycle expected in percentage for specific PWM channel.
*
* \param[in] pwm_frequency_out  : Desired PWM output frequency for specific PWM channel.
*
* \param[out] params_out        : Output parameters to be used in configuring specific PWM channel.
*
* \return                       :  None
*
******************************************************************************/
void wiced_hal_pwm_get_params( uint32_t clock_frequency_in, uint32_t duty_cycle, uint32_t pwm_frequency_out, pwm_config_t * params_out);

/** \} group_pwm_functions */
/** \} PwmDriver */

#endif //__WICED_PWM_H__

/* [] END OF FILE */
