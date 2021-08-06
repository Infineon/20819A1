/***************************************************************************//**
* \file <wiced_sleep.h>
*
* Provides the API definitions for low power modes.
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

#ifndef _WICED_SLEEP_H_
#define _WICED_SLEEP_H_

/**
 * \addtogroup wiced_sleep_config       WICED Sleep Configuration
 * \ingroup HardwareDrivers
 * \{
 *
 *  This driver provides simplified access to the WICED sleep framework on the
 *  CYW20819 device.
 *
 *  The following APIs allow the application layer to easily configure sleep
 *  parameters as well as communicate with the device Power Management Unit
 *  (PMU). Using this APIs the application can allow or disallow ePDS when asked
 *  by the callback from PMU as well as asynchronously enter HID-Off mode.
 *
 *  defgroup group_sleep_defines         Defines
 *  defgroup group_sleep_enums           ENUMs
 *  defgroup group_sleep_data_structures Data Structures
 *
 */

/******************************************************************************
 *                       Global Enumerations definitions
 ******************************************************************************/

/**
* \addtogroup group_sleep_defines
* \{
*/

#define WICED_SLEEP_MAX_TIME_TO_SLEEP    ~0      /* Sleep forever. Return this in the sleep callback type
                                                    WICED_SLEEP_POLL_TIME_TO_SLEEP */

/** Wake sources.*/
#define WICED_SLEEP_WAKE_SOURCE_KEYSCAN  (1<<0)  /**< Enable wake from keyscan */
#define WICED_SLEEP_WAKE_SOURCE_QUAD     (1<<1)  /**< Enable wake from quadrature sensor */
#define WICED_SLEEP_WAKE_SOURCE_GPIO     (1<<2)  /**< Enable wake from GPIO */
#define WICED_SLEEP_WAKE_SOURCE_MASK     (WICED_SLEEP_WAKE_SOURCE_GPIO | \
                                         WICED_SLEEP_WAKE_SOURCE_KEYSCAN | \
                                         WICED_SLEEP_WAKE_SOURCE_QUAD) /**< All wake sources */

/** \} group_sleep_defines */

/**
* \addtogroup group_sleep_enums
* \{
*/

/** Defines whether the boot is cold boot or fast boot. Not used for CYW20819 */
typedef enum
{
    WICED_SLEEP_COLD_BOOT, /**< Cold boot */
    WICED_SLEEP_FAST_BOOT  /**< Fast boot */
}wiced_sleep_boot_type_t;

/** Defines whether to sleep with external HCI host present or not */
typedef enum
{
    WICED_SLEEP_MODE_NO_TRANSPORT, /** <When a external HCI host is connected (determined using
                                    the assertion of CTS line) sleep is not allowed */
    WICED_SLEEP_MODE_TRANSPORT     /** <When a external HCI host is NOT connected (determined using
                                    the assertion of CTS line) sleep is not allowed */
}wiced_sleep_mode_type_t;

/** Active interrupt level for Wake through GPIO*/
typedef enum
{
    WICED_SLEEP_WAKE_ACTIVE_LOW, /**< Active low interrupt wakes the chip */
    WICED_SLEEP_WAKE_ACTIVE_HIGH /**< Active high interrupt wakes the chip*/
}wiced_sleep_wake_type_t;

/** Defines the poll type from the PMU. The PMU can poll for either time to sleep or
 *  type of sleep */
typedef enum
{
    WICED_SLEEP_POLL_TIME_TO_SLEEP,      /**< Maximum allowed sleep duration */
    WICED_SLEEP_POLL_SLEEP_PERMISSION    /**< Permission to sleep */
} wiced_sleep_poll_type_t;

/** Defines whether to allow sleep or not
*/
typedef enum
{
    WICED_SLEEP_NOT_ALLOWED = 0,          /**< Sleep is not allowed */
    WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN, /**< Sleep in EPDS mode is allowed */
    WICED_SLEEP_ALLOWED_WITH_SHUTDOWN,    /**< Hibernate mode */
}wiced_sleep_permission_type_t;

/** Lists the reason for reset
*/
typedef enum
{
    WICED_SLEEP_WAKE_REASON_POR = 0,          /**< Reset due to POR */
    WICED_SLEEP_WAKE_REASON_HIDOFF_TIMEOUT,   /**< Sleep in EPDS mode is allowed */
    WICED_SLEEP_WAKE_REASON_HIDOFF_GPIO,      /**< Confirm if GPIO interrupt reset */
}wiced_sleep_wake_reason_t;

/** \} group_sleep_enums */

/******************************************************************************
 * Callback function definitions                                                           *
 ******************************************************************************/

/*******************************************************************************
* Function Name: *wiced_sleep_allow_check_callback
****************************************************************************//**
*
*  Prototype for the callback function that needs to be defined by the
*  application. PMU will call this function whenever it want to enter low power
*  mode (ePDS). The PMU will pass the parameter value either as
*  WICED_SLEEP_POLL_TIME_TO_SLEEP or WICED_SLEEP_POLL_SLEEP_PERMISSION and the
*  application needs to return appropriate value (see return description)
*
* \param[in]       type:  Poll type can either be WICED_SLEEP_POLL_TIME_TO_SLEEP
*                         or WICED_SLEEP_POLL_SLEEP_PERMISSION
*                         (see #wiced_sleep_poll_type_t)
*
* \return          if type == WICED_SLEEP_POLL_TIME_TO_SLEEP, application should
*                  return WICED_SLEEP_MAX_TIME_TO_SLEEP.
*                  if type == WICED_SLEEP_POLL_SLEEP_PERMISSION, application should
*                  return one of the values in wiced_sleep_permission_type_t
*
* Note:- Application shall return immediately, with the return value specifying whether
*        it allows/disallows sleep. Immediate return is required to allow the maximum
*        time to sleep. Applications shall ensure that all peripheral activity (on PUART, SPI, .. )
*        is paused before allowing sleep. Any pending activity on the peripherals
*        is lost when the device goes into sleep.
*/
typedef uint32_t (*wiced_sleep_allow_check_callback ) (wiced_sleep_poll_type_t type );

/*******************************************************************************
* Function Name: *wiced_sleep_post_sleep_callback
****************************************************************************//**
*
*  Application implements call back of this type to perform post sleep actions.
*
* @param[out]       restore_configuration:
*                   WICED_TRUE     - reconfigure configured gpios and re-initialize configured peripherals
*                   WICED_FALSE    - reconfiguration not required
*
* \return void
*
*/
typedef void (*wiced_sleep_post_sleep_callback ) ( wiced_bool_t restore_configuration );

/******************************************************************************
*                      Global Data Structure definitions
******************************************************************************/

/**
* \addtogroup group_sleep_data_structures
* \{
*/

/** This structure defines the sleep configuration parameters to be passed to
 *  wiced_sleep_configure API
*/
typedef struct
{
    wiced_sleep_mode_type_t                  sleep_mode;             /**< Defines whether to sleep with or without transport */
    wiced_sleep_wake_type_t                  host_wake_mode;         /**< Defines the active level for host wake signal */
    wiced_sleep_wake_type_t                  device_wake_mode;       /**< Defines the active level for device wake signal
                                                                          If device wake signal is not present on the device then
                                                                          GPIO defined in device_wake_gpio_num is used */
    uint8_t                                  device_wake_source;     /**< Device wake source(s). See 'wake sources' defines for more
                                                                          details. GPIO is mandatory if WICED_SLEEP_MODE_TRANSPORT
                                                                          is used as sleep_mode*/
    uint32_t                                 device_wake_gpio_num;   /**< GPIO# for device wake, mandatory for
                                                                          WICED_SLEEP_MODE_TRANSPORT */
    wiced_sleep_allow_check_callback         sleep_permit_handler;   /**< Call back to be called by sleep framework
                                                                          to poll for sleep permission */
    wiced_sleep_post_sleep_callback          post_sleep_cback_handler;  /**< Callback to application on wake from sleep */
}wiced_sleep_config_t;

/** \} group_sleep_data_structures */

/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/*******************************************************************************
* Function Name: wiced_sleep_configure
****************************************************************************//**
 * This API is used to configure various sleep mode parameters
 *  (see @wiced_sleep_config_t) such as interrupt source, GPIO to be used as
 *  interrupt etc.
 *
 * \param[in]       p_sleep_config: see @wiced_sleep_config_t
 *
 * \return          WICED_SUCCESS or WICED_ERROR
 */
wiced_result_t wiced_sleep_configure( wiced_sleep_config_t *p_sleep_config );

/*******************************************************************************
* Function Name: wiced_sleep_get_boot_mode
****************************************************************************//**
 * This API is used request the type of reboot. A device is said to cold boot if
 * it is booting from power up or HID-Off mode whereas the device is going through
 * a fast boot if it is booting from shutdown sleep
 *
 * \param None
 *
 * \return          wiced_sleep_boot_type_t
 *
 * \note
 * This API is not used for CYW20819 as it doesn't support shutdown sleep
 */
wiced_sleep_boot_type_t wiced_sleep_get_boot_mode(void);

/*******************************************************************************
* Function Name: wiced_sleep_enter_hid_off
****************************************************************************//**
 * This API is used to enter HID OFF mode. The application can call this API
 * at any time when there is not BT activity
 *
 * \param[in]       wakeup_time :       wake up time in milliseconds.
 *                                      Application can configure for both GPIO wake and/or timed wake.
 *                                      Forever sleep(i.e. no timed wake) can be configure by
 *                                      setting wakeup_time = 0. The max time is 0x3fffff LPO cycles
 *                                      which is ~128010 milliseconds considering 32.768 KHz LPO.
 *                                      LPO can have 3000ppm error in some HW configurations.
 *
 * \param[in]       wake_gpio_pin:      LHL GPIO pin number. Range WICED_P00 to WICED_P39.
 *                                      If application does not need to support wake using GPIO,
 *                                      it shall use WICED_HAL_GPIO_PIN_UNUSED.
 *                                      if pin is not LHL pin, device can wake only after wakeup_time.
 * \param[in]       wake_active_mode:   Positive edge trigger or negative edge trigger wake can be
 *                                      configured using WICED_GPIO_ACTIVE_HIGH or WICED_GPIO_ACTIVE_LOW
 *                                      respectively.
 *
 * \return          WICED_ABORTED if HID off sleep aborts.
 *                  Never returns if HID off sleep configure success.
 *
 * \note
 * API must be called without any BT activity.
 *
 */
wiced_result_t wiced_sleep_enter_hid_off( uint32_t wakeup_time,
                                          uint32_t wake_gpio_pin,
                                          wiced_sleep_wake_type_t wake_active_mode );
/**
 * API to get reset reason
 *
 * returns the reason for wakeup
 *
 * @return          wiced_reset_reason_t: Reason for wakeup
 *
 */
wiced_sleep_wake_reason_t wiced_sleep_hid_off_wake_reason(void);

/**
 * Function Name: wiced_disable_non_wakeup_interrupt_before_entering_sleep_mode
 *
 * Disable all non wake up interrupt before entering sleep mode.
 *
 * \param   none
 *
 * \return  none
 *
 */
void wiced_disable_non_wakeup_interrupt_before_entering_sleep_mode(void);

/**
 * Function Name: wiced_restore_non_wakeup_interrupt_after_exiting_sleep_mode
 *
 * Restore all non wake up interrupt after exiting sleep mode.
 *
 * \param   none
 *
 * \return  none
 *
 */
void wiced_restore_non_wakeup_interrupt_after_exiting_sleep_mode(void);

/** @} */
#endif //_WICED_SLEEP_H_
