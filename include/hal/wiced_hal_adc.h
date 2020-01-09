/***************************************************************************//**
* \file <wiced_hal_adc.h>
*
* List of parameters and defined functions needed to access the
* Analog-to-Digital Converter (ADC) driver.
*
********************************************************************************
* \copyright
* Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef __WICED_ADC_H__
#define __WICED_ADC_H__


/**
* \addtogroup AdcDriver Analog-to-Digital Converter (ADC)
* \ingroup HardwareDrivers
* \{
*
* Defines a driver to facilitate interfacing with the Analog-to-Digital
* Converter (ADC) driver. Use this driver to measure a DC voltage via
* a GPIO pin. Voltage measurement values are reported in millivolts (mV).
*
* Note that the ADC channels do not correspond directly to the GPIO pin
* numbering convention (e.g. ADC channel 15 is tied to GPIO P18, etc).
* Refer to device datasheet for your device for more information.
*
* \defgroup group_adc_functions Functions
* \defgroup group_adc_data_structures Data Structures
*
*/

/******************************************************************************
 * Global Data Structure definitions                                          *
 ******************************************************************************/

/**
* \addtogroup group_adc_data_structures
* \{
*/

/* Supported ADC input channel selection. */
typedef enum {
    ADC_INPUT_P17           =   0x0,
    ADC_INPUT_P16           =   0x1,
    ADC_INPUT_P15           =   0x2,
    ADC_INPUT_P14           =   0x3,
    ADC_INPUT_P13           =   0x4,
    ADC_INPUT_P12           =   0x5,
    ADC_INPUT_P11           =   0x6,
    ADC_INPUT_P10           =   0x7,
    ADC_INPUT_P9            =   0x8,
    ADC_INPUT_P8            =   0x9,
    ADC_INPUT_P1            =   0xA,
    ADC_INPUT_P0            =   0xB,
    ADC_INPUT_VDDIO         =   0xC,    //ADC_INPUT_VBAT_VDDIO
    ADC_INPUT_VDD_CORE      =   0xD,    //ADC_INPUT_VDDC
    ADC_INPUT_ADC_BGREF     =   0xE,
    ADC_INPUT_ADC_REFGND    =   0xF,
    ADC_INPUT_P38           =   0x10,
    ADC_INPUT_P37           =   0x11,
    ADC_INPUT_P36           =   0x12,
    ADC_INPUT_P35           =   0x13,
    ADC_INPUT_P34           =   0x14,
    ADC_INPUT_P33           =   0x15,
    ADC_INPUT_P32           =   0x16,
    ADC_INPUT_P31           =   0x17,
    ADC_INPUT_P30           =   0x18,
    ADC_INPUT_P29           =   0x19,
    ADC_INPUT_P28           =   0x1A,
    ADC_INPUT_P23           =   0x1B,
    ADC_INPUT_P22           =   0x1C,
    ADC_INPUT_P21           =   0x1D,
    ADC_INPUT_P19           =   0x1E,
    ADC_INPUT_P18           =   0x1F,
    ADC_INPUT_CHANNEL_MASK  =   0x1f,
}ADC_INPUT_CHANNEL_SEL;


/* input  voltage range selection */
typedef enum
{
    ADC_RANGE_0_3P6V = 0,
    ADC_RANGE_0_1P8V = 1,
}ADC_INPUT_RANGE_SEL;

/** \} group_adc_data_structures */

/******************************************************************************
*** Global functions .
******************************************************************************/

/**
* \addtogroup group_adc_functions
* \{
*/

/******************************************************************************
* Function Name: wiced_hal_adc_init
***************************************************************************//**
*
* Initialize the ADC hardware to its default state.
*
* \param None
*
* \return None
*
******************************************************************************/
void wiced_hal_adc_init(void);

/******************************************************************************
* Function Name: wiced_hal_adc_set_input_range
***************************************************************************//**
*
* This function will set the input range selection and calibrates ADC hardware.
*
* \param[in] range_idx : desired range selection from ADC_INPUT_RANGE_SEL
*
* \return              : None
*
******************************************************************************/
void wiced_hal_adc_set_input_range(ADC_INPUT_RANGE_SEL rangeIdx);

/******************************************************************************
* Function Name: wiced_hal_adc_read_raw_sample
***************************************************************************//**
*
* Read the raw ADC register value for the given channel. The value returned
* here is direct from the register.
*
* \param[in] channel : The input channel that corresponds to a GPIO pin.
*
* \param[in] avgCnt : Reads adc raw sample for avgCnt+4 times, and choose
*                     the middle set of avgCnt times for calculation
*
* \return            : Raw digital value read from the hardware register.
*
******************************************************************************/
int16_t wiced_hal_adc_read_raw_sample( ADC_INPUT_CHANNEL_SEL channel, uint8_t avgCnt);

/******************************************************************************
* Function Name: wiced_hal_adc_get_ground_offset
***************************************************************************//**
*
* Get the ADC ground offset.
*
* \param  : None
*
* \return : ADC ground offset.
*
******************************************************************************/
int32_t wiced_hal_adc_get_ground_offset(void);

/******************************************************************************
* Function Name: wiced_hal_adc_get_reference_reading
***************************************************************************//**
*
* Get the ADC Reference voltage conversion value obtained from calibration.
*
* \param  : None
*
* \return : Reference voltage conversion value.
*
******************************************************************************/
int32_t wiced_hal_adc_get_reference_reading(void);

/******************************************************************************
* Function Name: wiced_hal_adc_get_reference_micro_volts
***************************************************************************//**
*
* Get the ADC Reference voltage conversion value obtained from calibration.
*
* \param  : None
*
* \return : Reference micro voltage.
*
******************************************************************************/
uint32_t wiced_hal_adc_get_reference_micro_volts(void);

/******************************************************************************
* Function Name: wiced_hal_adc_read_voltage
***************************************************************************//**
*
* Read the ADC voltage value for the given channel. The value returned here
* is converted to a voltage value from the register.
*
* \param[in] channel : The input channel that corresponds to a GPIO pin.
*
* \return            : Converted digital voltage value in mV.
*
******************************************************************************/
uint32_t wiced_hal_adc_read_voltage(ADC_INPUT_CHANNEL_SEL channel);

/** \} group_adc_functions */

#endif //__WICED_ADC_H__

/** \} AdcDriver */

/* [] END OF FILE */
