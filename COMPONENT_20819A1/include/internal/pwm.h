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

#ifndef __PWM__H__
#define __PWM__H__



/** \addtogroup  PWM
* \ingroup HardwareDrivers
*/
/*! @{ */
/**
* Defines the BCM standard pwm driver. It is compiled and used with
* BRCM standard applications [Mouse or Keyboard].
*/

/// PWM HW block has 4 PWMs Channels(10 bit)
/// This macros enable to switch from Pwm to its mask value.
/// These mask values are used in enable/disable case.
#define pwmIdToMask(id)           (1<<id)

/// PWM HW block has 4 PWM channels each with its own 10 bit counter.
/// The first PWM id is PWM0;
enum
{
    PWM0  = 0,
    PWM1  = 1,
    PWM2  = 2,
    PWM3  = 3,
#if defined(BCM20739) || defined(BCM20735) || defined(BCM20819)
    PWM4  = 4,
    PWM5  = 5,
    MAX_PWMS = 6
#else
    MAX_PWMS = 4
#endif
};
/// Clock used for PWM. When LHL_CLK is set, 128 KHz is used. When PMU_CLK is set, 1 MHz or 8 MHz.
typedef enum
{
    LHL_CLK,
    PMU_CLK
} PwmClockType;

enum
{
    // PWM Channel Mask.
#if defined(BCM20739) || defined(BCM20735) || defined(BCM20819)
    PWM_CHANNEL_MASK        =   0x300F
#else
    PWM_CHANNEL_MASK        =   0x0F
#endif
};

enum
{
#if defined(BCM20739) || defined(BCM20735) || defined(BCM20819)
    MAX_TOGGLE_COUNT        = 0xFFFF
#else
    MAX_TOGGLE_COUNT        = 0x3FF
#endif
};


BOOL32 pwm_start( UINT8 id, PwmClockType clk, UINT32 toggleCount, UINT32 InitCount );
BOOL32 pwm_transitionToSubstituteValues(UINT8 id, UINT32 toggleCount, UINT32 InitCount );
BOOL32 pwm_startWithAlternateValues( UINT8 id, PwmClockType clk,UINT32 toggleCount, UINT32 InitCount, BOOL32 invert );
UINT32 pwm_getToggleCount(UINT8 id);
UINT32 pwm_getInitValue(UINT8 id);

/// The following methods are for advanced users only
BOOL32 pwm_setToggleCount(UINT8 id, UINT32 toggleCount);
void pwm_setInversion(UINT8 id, BOOL32 invert);
BOOL32 pwm_setValues ( UINT8 id, UINT32 toggleCount, UINT32 initCount, PwmClockType clk, BOOL32 invert);
void pwm_setClock(UINT8 id, PwmClockType clk);
void pwm_setInitValue( UINT8 id, UINT32 InitCount );
void pwm_resetInternalCount(UINT32 mask);
void pwm_enableChannel(UINT32 mask);
void pwm_disableChannel(UINT32 mask);
void pwm_setReset(UINT32 mask, BOOL32 resetEnable);
/* @} */

#endif
