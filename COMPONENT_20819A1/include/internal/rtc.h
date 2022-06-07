/***************************************************************************//**
* \file <rtc.h>
*
* \brief
* This file defines an RTC driver
*
*//*****************************************************************************
* Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef __RTC_H__
#define __RTC_H__


#include "brcm_fw_types.h"

/**
* \addtogroup RTC
* \ingroup HardwareDrivers
* @{
* Defines an RTC driver.
*
* 20730/20733 support 48 bits RTC timer from 32kHz crystal
* oscillator.
*
* Usage:
*
* (1) use Rtc->getInstance() to get the RTC driver instance
*     first.
* (2) use Rtc->setRTCTime() to set the current calender time.
* (3) user Rtc->getRTCTime() to retrieve the current calender
*     time.
* (4) Rtrc->ctime(), will convert the RTC_time to ascii user
*     friendly string.
*
*/

/**
* \addtogroup group_RTC_enum
* \{
*/

enum
{
    LHL_CTL_32K_OSC_POWER_UP = 0x04,
    LHL_CTL_32K_OSC_POWER_DN = 0x00,
    LHL_CTL_32K_OSC_POWER_MASK = 0x04
};

/*
 *  time conversion reference timebase
 *  2010/1/1
 */
typedef enum
{
    BASE_LINE_REF_YEAR      =   2010,
    BASE_LINE_REF_MONTH     =   1,
    BASE_LINE_REF_DATE      =   1,
} tRTC_REFERENCE_TIME_BASE_LINE;

/*
 * RTC enable type
 */
typedef enum
{
    LHL_CTL_RTC_ENABLE              = 1, /**< enable RTC and power up 32kHz crystal oscillator */
    LHL_CTL_RTC_DISABLE             = 0, /**< disable RTC and power down the 32kHz crystal oscillator */
}tRTC_LHL_CTL_RTC_ENABLE_MODE;

/*
 * RTC clock source selection
 */
enum
{
    RTC_REF_CLOCK_SRC_32KHZ  = 32,
    RTC_REF_CLOCK_SRC_128KHZ = 128
};

/** \} group_RTC_enum */

/**
* \addtogroup group_RTC_data_structures
* \{
*/

/**
 * LHL_CTL register
 */
typedef union
{
    UINT32          lhl_adc_rtc_ctl_reg;
    struct
    {
        UINT32                  reserved1              : 12;  /**< bit 0:11  reserved */
        UINT32                  rtcTerminalCntStatusEn  : 1;  /**< bit 12 -  Real time clock terminal count status enable */
        UINT32                  rtcResetCounter         : 1;  /**< bit 13 - RTC reset counter */
        UINT32                  rtcTimerFuncEn          : 1;  /**< Bit 14 - RTC timer function enable */
        UINT32                  rtcCounterEn            : 1;  /**< Bit 15 - RTC counter enable */
        UINT32                  reserved2               : 16; /**< reserved 31:3 */
    }bitmap;
} tRTC_LHL_ADC_RTC_CTL_REG;

/**
 * Real time clock read from hardware (48 bits)
 */
typedef union
{
    UINT64          rtc64;
    struct
    {
        UINT16      rtc16[4];
    }reg16map;
    struct
    {
        UINT32      rtc32[2];
    }reg32map;
} tRTC_REAL_TIME_CLOCK;

/**
 * broken-down calendar representation of time
 */
typedef struct
{
    UINT16  second; /**< seconds (0 - 59), not support leap seconds */
    UINT16  minute; /**< minutes (0 - 59) */
    UINT16  hour;   /**< hours (0 - 23) */
    UINT16  day;    /**< day of the month (1 - 31) */
    UINT16  month;  /**< month (0 - 11, 0=January) */
    UINT16  year;   /**< year should be larger then 2010*/
} RtcTime;

// Internal runtime-state of RTC driver
typedef struct
{
    UINT32                       userSetRtcClockInSeconds;
    tRTC_REAL_TIME_CLOCK         userSetRtcHWTimeStamp;
} RtcState;

/** \} group_RTC_data_structures */


/******************************************************************************
* Function Name: rtc_init
***************************************************************************//**
* Initialize the RTC block.
*
* \return None
*/
void rtc_init(void);

/******************************************************************************
* Function Name: rtc_getRTCRawClock
***************************************************************************//**
* Reads Real time clock value from hardware clock
*
* \param rtcClock - buffer to for the read data
*
* \return None
*/
void rtc_getRTCRawClock(tRTC_REAL_TIME_CLOCK *rtcClock);

/******************************************************************************
* Function Name: rtc_getRTCTime
***************************************************************************//**
* Reads Real time clock value from hardware clock
*
* \param timebuf
*
* \return None
*/
void rtc_getRTCTime(RtcTime *timebuf);

/******************************************************************************
* Function Name: rtc_setReferenceTime
***************************************************************************//**
* set original reference time.
*
* \param ref_time
*
* \return None
*/
void rtc_setReferenceTime(RtcTime* ref_time);

/******************************************************************************
* Function Name: rtc_setRTCTime
***************************************************************************//**
* set the current time
* This function call will assume the current time is later then
* 2010/1/1.
*
* \param newTime
*
* \return true if success else return false
*/
BOOL32 rtc_setRTCTime(RtcTime  *newTime);

/******************************************************************************
* Function Name: rtc_ctime
***************************************************************************//**
* This function call will convert the RtcTime object pointed by rtctime to
* c string containing a human-readable verion of the corresponding
* local time and data. Caller should make sure *outbuf size
* larger then 22 bytes.
*
* the returned string has the following format
*
* Mmm dd hh:mm:ss yyyy
* where
*      Mmm - the month in letters
*      dd  - the day of the month
*      hh:mm:ss -  time
*      yyyy - year
*
* \param timer - timer object pointer to be converted
* \param outbuf - buffer for the converted string
*
* \return NULL if convert fail, or will be start of the valid string
*
*/
char *rtc_ctime(RtcTime *timer, char *outbuf);

/******************************************************************************
* Function Name: rtc_sec2RtcTime
***************************************************************************//**
* Conver from UINT32 seconds to RTC_time timer object
*
* \param second -  32-bits seconds to be converted
* \param rtctime  - converted timer object pointer
*
* \return none
*/
void rtc_sec2RtcTime(UINT32 second, RtcTime *rtctime);

/******************************************************************************
* Function Name: rtc_RtcTime2Sec
***************************************************************************//**
* Conver from RTC_time format to UINT32 seconds format
*
* \param rtctime - timer object pointer to be converted
* \param second  - converted UINT32 seconds
*
* \return none
*/
void rtc_RtcTime2Sec(RtcTime *rtctime, UINT32 *second);


/**
* \addtogroup group_RTC_globals
* \{
*/

extern UINT32                userSetRtcClockInSeconds; /**< User/application set the RTC time */
                                                       /**< will conver the user/app settings to */
                                                       /**< seconds */
extern tRTC_REAL_TIME_CLOCK  userSetRtcHWTimeStamp;

/** \} group_RTC_globals */

/** @}  */
#endif
