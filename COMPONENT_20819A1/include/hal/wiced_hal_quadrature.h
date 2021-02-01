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

/** @file
*
* List of parameters and defined functions needed to access the
* Quadrature driver
*
*/

#ifndef __WICED_QUADRATURE_H__
#define __WICED_QUADRATURE_H__

#include "brcm_fw_types.h"


/**  \addtogroup Quadrature
 *  \ingroup HardwareDrivers
*/
/*! @{ */
/**
* Defines the quadrature driver that can detect and decode scroll wheel movement in X, Y, Z axis.
*
*/

/* enum value for port0PinsUsedAsQuadratureInput*/
enum
{
    ENABLE_PORT_2_PINS_AS_QUAD_INPUT    =   0,
    ENABLE_PORT_0_PINS_AS_QUAD_INPUT    =   1,
};

/* enum vlaue for QOC LED used in QOC_LEDs_output_polarity */
enum
{
    QUADRATURE_LED_CONFIG_OPEN_SOURCE   =   0x00,   //1 when on, tristate when off(open source)
    QUADRATURE_LED_CONFIG_SOURCE        =   0x01,   //1 when on, 0 when off
    QUADRATURE_LED_CONFIG_OPEN_DRAIN    =   0x02,   //tristate when on, 0 when off(open drain)
    QUADRATURE_LED_CONFIG_DRAIN         =   0x03    //0 when on, 1 when off
};

/* enum value used for channelEnableAndSamplingRate */
enum
{
    CH_XY_128_KHZ_FIXED_SAMPLE_RATE    = 0x00,
    CH_XY_64_KHZ_FIXED_SAMPLE_RATE     = 0x01,
    CH_XY_32_KHZ_FIXED_SAMPLE_RATE     = 0x02,
    CH_XY_16_KHZ_FIXED_SAMPLE_RATE     = 0x03,

    CH_XY_SEL_LHL_PWM_RATE             = 0x00,
    CH_XY_SEL_FIXED_RATE               = 0x04,

    CH_XY_DISABLE                      = 0x00,
    CH_XY_ENABLE                       = 0x08,

    CH_Z_SAMPLE_ONCE_PER_LHL_PWM       = 0x00,
    CH_Z_SAMPLE_ONCE_PER_2_LHL_PWM     = 0x10,
    CH_Z_SAMPLE_ONCE_PER_4_LHL_PWM     = 0x20,
    CH_Z_SAMPLE_ONCE_PER_8_LHL_PWM     = 0x30,
    CH_Z_SAMPLE_ONCE_PER_16_LHL_PWM    = 0x40,
    CH_Z_SAMPLE_ONCE_PER_32_LHL_PWM    = 0x50,
    CH_Z_SAMPLE_ONCE_PER_64_LHL_PWM    = 0x60,
    CH_Z_SAMPLE_ONCE_PER_128_LHL_PWM   = 0x70,

    CH_Z_DISABLE                       = 0x00,
    CH_Z_ENABLE                        = 0x80

};


///////////////////////////////////////////////////////////////////////////////
/// configure quadrature driver
///
/// Application use this to configure Quadradure driver
///
/// \param QOC_LEDs_output_polarity - QOC LEDs (QOC0, QOC1, QOC2, and QOC3) output polarity
/// \param quadratureInputGpioConfig - Configuration value used for each GPIO pin used for quadrature input.
///                                                        This allows configuration of pull up/down, etc. just like a GPIO.
/// \param port0PinsUsedAsQuadratureInput - if set to ENABLE_PORT_0_PINS_AS_QUAD_INPUT, Port 0 selected as quadrature input port (P2 as qdx0, P3 as qdx1, P4 as qdy0, P5 as qdy1, P6 as qdz0,P7 as qdz1).
///                                         if set to ENABLE_PORT_2_PINS_AS_QUAD_INPUT, Port 2 selected as quadrature input port (P32 as qdx0, P33 as qdx1, P34 as qdy0, P35 as qdy1, P36 as qdz0, P37 as qdz1)
/// \param configureP26AsQOC0 - if set to WICED_TRUE, P26 should be configured as QOC LED 0.
///                                               if set to WICED_FALSE, P26 config is not modified.
/// \param configureP27AsQOC1 - if set to WICED_TRUE, P27 should be configured as QOC LED 1.
///                                               if set to WICED_FALSE, P27 config is not modified.
/// \param configureP28AsQOC2 - if set to WICED_TRUE, P28 should be configured as QOC LED 2.
///                                               if set to WICED_FALSE, P28 config is not modified.
/// \param configureP29AsQOC3 - if set to WICED_TRUE, P29 should be configured as QOC LED 3.
///                                               if set to WICED_FALSE, P29 config is not modified.
/// \param channelEnableAndSamplingRate -Configuration for QD XY and Z enable and sampling rate control
/// \param pollXAxis -if set to WICED_TRUE, enable X Axis Data Polling. Otherwise, disable X Axis Data Polling
/// \param pollYAxis -if set to WICED_TRUE, enable Y Axis Data Polling. Otherwise, disable Y Axis Data Polling
/// \param pollZAxis -if set to WICED_TRUE, enable Z Axis Data Polling. Otherwise, disable Z Axis Data Polling
///
/// \return - none
///////////////////////////////////////////////////////////////////////////////
void wiced_hal_quadrature_configure(uint16_t QOC_LEDs_output_polarity, uint16_t quadratureInputGpioConfig, uint8_t port0PinsUsedAsQuadratureInput,
                          wiced_bool_t configureP26AsQOC0, wiced_bool_t configureP27AsQOC1, wiced_bool_t configureP28AsQOC2, wiced_bool_t configureP29AsQOC3,
                          uint8_t channelEnableAndSamplingRate, wiced_bool_t pollXAxis, wiced_bool_t pollYAxis, wiced_bool_t pollZAxis);

////////////////////////////////////////////////////////////////
/// Initialize the quadrature driver. The quadrature HW is also
/// initialized if the last reset was because of power up.
/// Clears out any accumulated motion.
/// This must be invoked once at power up before using any quadrature
/// HW services and this call MUST be after mia driver has been
/// initialized.

////////////////////////////////////////////////////////////////
void wiced_hal_quadrature_init(void);

////////////////////////////////////////////////////////////////
/// Disable the quadrature interface including wakeups.
/// Also clears any accumulated motion
////////////////////////////////////////////////////////////////
void wiced_hal_quadrature_turnOff(void);

////////////////////////////////////////////////////////////////
/// Returns currently accumulated motion for the X axis. The
/// internally accumulated motion value is set to 0.
///
/// \return
///     X axis accumulated motion
////////////////////////////////////////////////////////////////
int16_t wiced_hal_quadrature_get_X_motion(void);

////////////////////////////////////////////////////////////////
/// Returns currently accumulated motion for the Y axis. The
/// internally accumulated motion value is set to 0.
///
/// \return
///     Y axis accumulated motion
////////////////////////////////////////////////////////////////
int16_t wiced_hal_quadrature_get_Y_motion(void);

////////////////////////////////////////////////////////////////
/// Returns currently accumulated motion for the Z axis. The
/// internally accumulated motion value is set to 0.
///
/// \return
///     Z axis accumulated motion
////////////////////////////////////////////////////////////////
int16_t wiced_hal_quadrature_get_Z_motion(void);

/////////////////////////////////////////////////////////////////
/// Returns currently accumulated motion for the active axis (X, Y or Z).
/// It assumes only one of the three, pollXAxis/pollYAxis/pollZAxis, is enabled.
///
/// \return accumulated motion in the active axis(X, Y, or Z)
////////////////////////////////////////////////////////////////
int16_t wiced_hal_quadrature_get_scroll_count(void);

/////////////////////////////////////////////////////////////////////////////
/// Register for notification of changes.
/// Once registered, you CAN NOT UNREGISTER; registration is meant to
/// be a startup activity.
///
/// \param userfn points to the function to call when the interrupt
/// comes and matches one of the masks (if you pass a method, it must
/// be static). The function does not need to clear the interrupt
/// status; this will be done by KeyscanDriver::lhlInterrupt(). The
/// function will be called ONCE per interrupt.
///
/// \param userdata will be passed back to userfn as-is; it can be used to
/// carry the "this", for example.
//////////////////////////////////////////////////////////////////////////////
void wiced_hal_quadrature_register_for_event_notification(void (*userfn)(void*), void* userdata);


/* @} */


#endif
