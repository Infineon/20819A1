/***************************************************************************//**
* \file <wiced_hal_gpio.h>
*
* List of parameters and defined functions needed to access the
* General Purpose Input/Output (GPIO) driver.
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

#ifndef __WICED_GPIO_H__
#define __WICED_GPIO_H__

#include "brcm_fw_types.h"


/**
* \addtogroup GPIODriver GPIO
* \ingroup HardwareDrivers
* \{
*
* Defines a driver to facilitate interfacing with the GPIO pins.
*
* Use this driver to control the behavior of any desired pin, such as
* driving a 1 or a 0, or as part of other drivers such as controlling
* the chip-select (CS) line for the SPI driver.
*
*/

/******************************************************************************
*** Parameters.
***
*** The following parameters are used to configure the driver or define
*** return status. They are not modifiable.
******************************************************************************/

/** Enum used to configure pin output configuration. */
typedef enum
{
    GPIO_PIN_OUTPUT_LOW,  /**< Output low (0) */
    GPIO_PIN_OUTPUT_HIGH, /**< Outout high (1) */
} tGPIO_PIN_OUTPUT_CONFIG;

/** Defines the constant values used for configuration by the GPIO driver.
 *  Each GPIO has a set of configuration signals
 *  - bit 0 - Edge Trigger
 *  - bit 1 - Trigger polarity
 *  - bit 2 - Dual edge trigger
 *  - bit 3 - Interrupt enable
 *  - bit 6 - Global input disable
 *  - bit 9:10 - pull up/down
 *  - bit 11 - Drive strength
 *  - bit 13 - Input hysteresis
 *  - bit 14 - Output enable
 */
enum
{
    /* Trigger Type
     * GPIO configuration bit 0, Interrupt type defines
     */
    GPIO_EDGE_TRIGGER_MASK       = 0x0001, /**< GPIO configuration bit 0 mask */
    GPIO_EDGE_TRIGGER            = 0x0001, /**< Edge triggered */
    GPIO_LEVEL_TRIGGER           = 0x0000, /**< Level triggered */

    /* Negative Edge Triggering
     * GPIO configuration bit 1, Interrupt polarity defines
     */
    GPIO_TRIGGER_POLARITY_MASK   = 0x0002, /**< GPIO configuration bit 1 mask */
    GPIO_TRIGGER_NEG             = 0x0002, /**< Negative interrupt polarity (Falling Edge)*/

    /* Dual Edge Triggering
     * GPIO configuration bit 2, single/dual edge defines
     */
    GPIO_DUAL_EDGE_TRIGGER_MASK  = 0x0004, /**< GPIO configuration bit 2 mask */
    GPIO_EDGE_TRIGGER_BOTH       = 0x0004, /**< Trigger on both edges */
    GPIO_EDGE_TRIGGER_SINGLE     = 0x0000, /**< Trigger on single edge */

    /* Interrupt Enable
     * GPIO configuration bit 3, interrupt enable/disable defines
     */
    GPIO_INTERRUPT_ENABLE_MASK   = 0x0008, /**< GPIO configuration bit 3 mask */
    GPIO_INTERRUPT_ENABLE        = 0x0008, /**< Interrupt Enabled */
    GPIO_INTERRUPT_DISABLE       = 0x0000, /**< Interrupt Disabled */

    /* Interrupt Config
     * GPIO configuration bit 0:3, Summary of Interrupt enabling type
     */
    GPIO_EN_INT_MASK             = GPIO_EDGE_TRIGGER_MASK | GPIO_TRIGGER_POLARITY_MASK | GPIO_DUAL_EDGE_TRIGGER_MASK | GPIO_INTERRUPT_ENABLE_MASK,
    GPIO_EN_INT_LEVEL_HIGH       = GPIO_INTERRUPT_ENABLE | GPIO_LEVEL_TRIGGER, /**< Interrupt on level HIGH */
    GPIO_EN_INT_LEVEL_LOW        = GPIO_INTERRUPT_ENABLE | GPIO_LEVEL_TRIGGER | GPIO_TRIGGER_NEG, /**< Interrupt on level LOW */
    GPIO_EN_INT_RISING_EDGE      = GPIO_INTERRUPT_ENABLE | GPIO_EDGE_TRIGGER, /**< Interrupt on rising edge */
    GPIO_EN_INT_FALLING_EDGE     = GPIO_INTERRUPT_ENABLE | GPIO_EDGE_TRIGGER | GPIO_TRIGGER_NEG, /**< Interrupt on falling edge */
    GPIO_EN_INT_BOTH_EDGE        = GPIO_INTERRUPT_ENABLE | GPIO_EDGE_TRIGGER | GPIO_EDGE_TRIGGER_BOTH, /**< Interrupt on both edges */

    /* GPIO Output Buffer Control and Output Value Multiplexing Control
     * GPIO configuration bit 4:5, and 14 output enable control and
     * muxing control
     */
    GPIO_INPUT_ENABLE            = 0x0000, /**< Input enable */
    GPIO_OUTPUT_DISABLE          = 0x0000, /**< Output disable */
    GPIO_OUTPUT_ENABLE           = 0x4000, /**< Output enable */
    GPIO_KS_OUTPUT_ENABLE        = 0x0001, /**< Keyscan output enable*/
    GPIO_OUTPUT_FN_SEL_MASK      = 0x0000, /**< Output function select mask*/
    GPIO_OUTPUT_FN_SEL_SHIFT     = 0,


    /* Global Input Disable
     * GPIO configuration bit 6, "Global_input_disable" Disable bit
     * This bit when set to "1" , P0 input_disable signal will control
     * ALL GPIOs. Default value (after power up or a reset event) is "0".
     */
    GPIO_GLOBAL_INPUT_ENABLE     = 0x0000, /**< Global input enable */
    GPIO_GLOBAL_INPUT_DISABLE    = 0x0040, /**< Global input disable */

    /* Pull-up/Pull-down
     * GPIO configuration bit 9 and bit 10, pull-up and pull-down enable
     * Default value is [0,0]--means no pull resistor.
     */
    GPIO_PULL_UP_DOWN_NONE       = 0x0000, /**< No pull [0,0] */
    GPIO_PULL_UP                 = 0x0400, /**< Pull up [1,0] */
    GPIO_PULL_DOWN               = 0x0200, /**< Pull down [0,1] */
    GPIO_INPUT_DISABLE           = 0x0600, /**< Input disable [1,1] (input disabled the GPIO) */

    /* Drive Strength
     * GPIO configuration bit 11
     */
    GPIO_DRIVE_SEL_MASK         = 0x0800,  /**< GPIO configuration bit 11 mask */
    GPIO_DRIVE_SEL_LOWEST       = 0x0000,  /**< Drive strength lowest - 2mA @ 1.8V */
    GPIO_DRIVE_SEL_MIDDLE_0     = 0x0000,  /**< Drive strength middle_0 - 4mA @ 3.3V */
    GPIO_DRIVE_SEL_MIDDLE_1     = 0x0800,  /**< Drive strength middle_1 - 4mA @ 1.8V */
    GPIO_DRIVE_SEL_HIGHEST      = 0x0800,  /**< Drive strength highest - 8mA @ 3.3V */

    /* Input Hysteresis
     * GPIO configuration bit 13, hysteresis control
     */
    GPIO_HYSTERESIS_MASK         = 0x2000, /**< GPIO configuration bit 13 mask */
    GPIO_HYSTERESIS_ON           = 0x2000, /**< GPIO hysteresis on */
    GPIO_HYSTERESIS_OFF          = 0x0000, /**< GPIO hysteresis off */
};

/** GPIO Numbers : last 8 are ARM GPIOs and rest are LHL GPIOs */
typedef enum
{
    /* GPIO_P00 to GPIO_P39 are LHL GPIOs */
    WICED_P00 = 0,  /**< LHL GPIO 0 */
    WICED_P01,      /**< LHL GPIO 1 */
    WICED_P02,      /**< LHL GPIO 2 */
    WICED_P03,      /**< LHL GPIO 3 */
    WICED_P04,      /**< LHL GPIO 4 */
    WICED_P05,      /**< LHL GPIO 5 */
    WICED_P06,      /**< LHL GPIO 6 */
    WICED_P07,      /**< LHL GPIO 7 */
    WICED_P08,      /**< LHL GPIO 8 */
    WICED_P09,      /**< LHL GPIO 9 */
    WICED_P10,      /**< LHL GPIO 10 */
    WICED_P11,      /**< LHL GPIO 11 */
    WICED_P12,      /**< LHL GPIO 12 */
    WICED_P13,      /**< LHL GPIO 13 */
    WICED_P14,      /**< LHL GPIO 14 */
    WICED_P15,      /**< LHL GPIO 15 */
    WICED_P16,      /**< LHL GPIO 16 */
    WICED_P17,      /**< LHL GPIO 17 */
    WICED_P18,      /**< LHL GPIO 18 */
    WICED_P19,      /**< LHL GPIO 19 */
    WICED_P20,      /**< LHL GPIO 20 */
    WICED_P21,      /**< LHL GPIO 21 */
    WICED_P22,      /**< LHL GPIO 22 */
    WICED_P23,      /**< LHL GPIO 23 */
    WICED_P24,      /**< LHL GPIO 24 */
    WICED_P25,      /**< LHL GPIO 25 */
    WICED_P26,      /**< LHL GPIO 26 */
    WICED_P27,      /**< LHL GPIO 27 */
    WICED_P28,      /**< LHL GPIO 28 */
    WICED_P29,      /**< LHL GPIO 29 */
    WICED_P30,      /**< LHL GPIO 30 */
    WICED_P31,      /**< LHL GPIO 31 */
    WICED_P32,      /**< LHL GPIO 32 */
    WICED_P33,      /**< LHL GPIO 33 */
    WICED_P34,      /**< LHL GPIO 34 */
    WICED_P35,      /**< LHL GPIO 35 */
    WICED_P36,      /**< LHL GPIO 36 */
    WICED_P37,      /**< LHL GPIO 37 */
    WICED_P38,      /**< LHL GPIO 38 */
    WICED_P39,      /**< LHL GPIO 39 */
    /* GPIO_00 to GPIO_07 are ARM GPIOs */
    WICED_GPIO_00,  /**< ARM GPIO 0 - 40 */
    WICED_GPIO_01,  /**< ARM GPIO 1 - 41 */
    WICED_GPIO_02,  /**< ARM GPIO 2 - 42 */
    WICED_GPIO_03,  /**< ARM GPIO 3 - 43 */
    WICED_GPIO_04,  /**< ARM GPIO 4 - 44 */
    WICED_GPIO_05,  /**< ARM GPIO 5 - 45 */
    WICED_GPIO_06,  /**< ARM GPIO 6 - 46 */
    WICED_GPIO_07,  /**< ARM GPIO 7 - 47 */
    MAX_NUM_OF_GPIO
}wiced_bt_gpio_numbers_t;

/** possible functions to be brought out through LHL GPIO's */
typedef enum
{
    WICED_GPIO = 0,           /**< LHL GPIO (default functionality) */
    WICED_I2C_1_SCL,          /**< I2C 1 Clock */
    WICED_I2C_1_SDA,          /**< I2C 1 Data */
    WICED_I2C_2_SCL,          /**< I2C 2 Clock */
    WICED_I2C_2_SDA,          /**< I2C 2 Data */
    WICED_SPI_1_CLK,          /**< SPI 1 Clock */
    WICED_SPI_1_CS,           /**< SPI 1 Chip Select */
    WICED_SPI_1_MOSI,         /**< SPI 1 Master Out Slave In */
    WICED_SPI_1_MISO,         /**< SPI 1 Slave In Master Out */
    WICED_SPI_1_IO2,          /**< SPI 1 IO2 */
    WICED_SPI_1_IO3,          /**< SPI 1 IO3 */
    WICED_SPI_1_INT,          /**< SPI 1 INT */
    WICED_SPI_1_DCX,          /**< SPI 1 DCX */
    WICED_SPI_2_CLK,          /**< SPI 2 Clock */
    WICED_SPI_2_CS,           /**< SPI 2 Chip Select */
    WICED_SPI_2_MOSI,         /**< SPI 2 Master Out Slave In */
    WICED_SPI_2_MISO,         /**< SPI 2 Slave In Master Out */
    WICED_SPI_2_IO2,          /**< SPI 2 IO2 */
    WICED_SPI_2_IO3,          /**< SPI 2 IO2 */
    WICED_SPI_2_INT,          /**< SPI 2 INT */
    WICED_SPI_2_DCX,          /**< SPI 2 DCX */
    WICED_SPI_3_CLK,          /**< SPI 3 Clock */
    WICED_SPI_3_CS,           /**< SPI 3 Chip Select */
    WICED_SPI_3_MOSI,         /**< SPI 3 Master Out Slave In */
    WICED_SPI_3_MISO,         /**< SPI 3 Slave In Master Out */
    WICED_SPI_3_INT,          /**< SPI 3 INT */
    WICED_SWDCK,              /**< SWD Clock */
    WICED_SWDIO,              /**< SWD Data */
    WICED_UART_1_TXD,         /**< HCI UART TX */
    WICED_UART_1_RXD,         /**< HCI UART RX */
    WICED_UART_1_CTS,         /**< HCI UART CTS */
    WICED_UART_1_RTS,         /**< HCI UART RTS */
    WICED_UART_2_TXD,         /**< PUART TX */
    WICED_UART_2_RXD,         /**< PUART RX */
    WICED_UART_2_CTS,         /**< PUART CTS */
    WICED_UART_2_RTS,         /**< PUART RTS */
    WICED_AOA_0,              /**< AOA 0 */
    WICED_AOA_1,              /**< AOA 1 */
    WICED_AOA_2,              /**< AOA 2 */
    WICED_AOD_0,              /**< AOD 0 */
    WICED_I2S_MASTER_CLK,     /**< I2S Master Clock */
    WICED_I2S_MASTER_WS,      /**< I2S Master Word Select */
    WICED_I2S_MASTER_DO,      /**< I2S Master DATA OUT */
    WICED_I2S_MASTER_DI,      /**< I2S Master DATA IN */
    WICED_I2S_SLAVE_CLK,      /**< I2S Slave Clock */
    WICED_I2S_SLAVE_WS,       /**< I2S Slave Word Select */
    WICED_I2S_SLAVE_DO,       /**< I2S Slave DATA OUT */
    WICED_I2S_SLAVE_DI,       /**< I2S Slave DATA IN */
    WICED_PCM_CLK,            /**< PCM Clock */
    WICED_PCM_SYNC,           /**< PCM SYNC */
    WICED_PCM_OUT,            /**< PCM OUT */
    WICED_PCM_IN,             /**< PCM IN */
    WICED_GCI_SECI_IN,        /**< GCI */
    WICED_GCI_SECI_OUT,       /**< GCI */
    WICED_ACLK_0,             /**< ACLK_0 Clock */
    WICED_ACLK_1,             /**< ACLK_1 Clock */
    WICED_KSO0,               /**< Keyscan Output 00 */
    WICED_KSO1,               /**< Keyscan Output 01 */
    WICED_KSO2,               /**< Keyscan Output 02 */
    WICED_KSO3,               /**< Keyscan Output 03 */
    WICED_KSO4,               /**< Keyscan Output 04 */
    WICED_KSO5,               /**< Keyscan Output 05 */
    WICED_KSO6,               /**< Keyscan Output 06 */
    WICED_KSO7,               /**< Keyscan Output 07 */
    WICED_KSO8,               /**< Keyscan Output 08 */
    WICED_KSO9,               /**< Keyscan Output 09 */
    WICED_KSO10,              /**< Keyscan Output 10 */
    WICED_KSO11,              /**< Keyscan Output 11 */
    WICED_KSO12,              /**< Keyscan Output 12 */
    WICED_KSO13,              /**< Keyscan Output 13 */
    WICED_KSO14,              /**< Keyscan Output 14 */
    WICED_KSO15,              /**< Keyscan Output 15 */
    WICED_KSO16,              /**< Keyscan Output 16 */
    WICED_KSO17,              /**< Keyscan Output 17 */
    WICED_KSO18,              /**< Keyscan Output 18 */
    WICED_KSO19,              /**< Keyscan Output 19 */
    WICED_TX_PD,              /**< TX PD */
    WICED_TX_PD_TILDA,        /**< TX PD TILDA */
    WICED_PA_RAMP,            /**< PA Ramp */
    WICED_BT_GPIO_00,         /**< ARM GPIO 0 */
    WICED_BT_GPIO_01,         /**< ARM GPIO 1 */
    WICED_BT_GPIO_02,         /**< ARM GPIO 2 */
    WICED_BT_GPIO_03,         /**< ARM GPIO 3 */
    WICED_BT_GPIO_04,         /**< ARM GPIO 4 */
    WICED_BT_GPIO_05,         /**< ARM GPIO 5 */
    WICED_BT_GPIO_06,         /**< ARM GPIO 6 */
    WICED_BT_GPIO_07,         /**< ARM GPIO 7 */
    WICED_PWM0,               /**< PWM 0 */
    WICED_PWM1,               /**< PWM 1 */
    WICED_PWM2,               /**< PWM 2 */
    WICED_PWM3,               /**< PWM 3 */
    WICED_PWM4,               /**< PWM 4 */
    WICED_PWM5,               /**< PWM 5 */
    WICED_TX_FSM = 128,       /**< TX FSM */
    WICED_RX_FSM,             /**< RX FSM */
    WICED_RX_PU,              /**< RX PU */
    WICED_TX_PU,              /**< TX PU */
    WICED_UNAVAILABLE = 0xFF  /**< Invalid functionality for error check */
} wiced_bt_gpio_function_t;

/** Aliases for compatibility with UDD signal definitions */
#define WICED_PCM_IN_I2S_DI     WICED_PCM_IN
#define WICED_PCM_OUT_I2S_DO    WICED_PCM_OUT
#define WICED_PCM_SYNC_I2S_WS   WICED_PCM_SYNC
#define WICED_PCM_CLK_I2S_CLK   WICED_PCM_CLK

/** Alias for compatibility with WICED_PDM_DATA until PDM handled */
#define WICED_PDM_DATA WICED_GPIO

/** Possible return values from wiced_hal_gpio_select_function(...), Callers only need to check for the
 *  GPIO_FAILURE case since any other status means success
 */
typedef enum GPIO_STATUS_e
{
    GPIO_FAILURE, /**< The requested pin and function mapping is not supported by hardware */
    GPIO_SUCCESS, /**< The requested pin and function mapping is complete, The pin was previously not used and the function was previously not mapped */
    GPIO_REMAPPED,/**< The requested pin and function mapping is complete, The pin was previously used by another function, that function was disabled and the new function applied */
    GPIO_MOVED    /**< The requested pin and function mapping is complete, The requested function was already mapped to a different pin, that pin was disabled and the function moved to the new pin */
} wiced_bt_gpio_select_status_t;

/** GPIO Active Level HIGH */
#define WICED_GPIO_ACTIVE_HIGH      1

/** GPIO Active Level LOW */
#define WICED_GPIO_ACTIVE_LOW       0

/** Invalid GPIO pin */
#define WICED_HAL_GPIO_PIN_UNUSED   0xFF

/******************************************************************************
*** Function prototypes and defines.
******************************************************************************/


/*******************************************************************************
* Function Name: wiced_hal_gpio_init
****************************************************************************//**
*
* Initializes the GPIO driver and its private values.
* Also programs all GPIOs to be ready for use. This must be invoked before
* accessing any GPIO driver services, typically at boot.
* This is independent of other drivers and must be one of the first to
* be initialized.
*
* \param none
*
* \return none
*
*******************************************************************************/
void wiced_hal_gpio_init(void);


/*******************************************************************************
* Function Name: wiced_hal_gpio_configure_pin
****************************************************************************//**
*
* Configures a GPIO pin.
*
* For example, to enable interrupts for all edges, with a pull-down,
* you could use the config:
* GPIO_EDGE_TRIGGER | GPIO_EDGE_TRIGGER_BOTH |
* GPIO_INTERRUPT_ENABLE_MASK | GPIO_PULL_DOWN_MASK
*
* \param[in] pin        The pin number from the schematic. Range [0-39] Ex: P<pin>
* \param[in] config     GPIO configuration. See the parameters section
* \param[in] outputVal  The value of the output pin (\ref tGPIO_PIN_OUTPUT_CONFIG)
*
* \return               None
*
* \note
* Note that the GPIO output value is programmed before
* the GPIO is configured. This ensures that the GPIO will activate with the
* correct external value. Also note that the output value is always
* written to the output register regardless of whether the GPIO is configured
* as input or output.
*
* \note
* Enabling interrupts here isn't sufficient; you also need to register
* the interrupt handler with wiced_hal_gpio_register_pin_for_interrupt().
*
*******************************************************************************/
void wiced_hal_gpio_configure_pin(uint32_t pin, uint32_t config,
                                                uint32_t outputVal);


/*******************************************************************************
* Function Name: wiced_hal_gpio_get_pin_config
****************************************************************************//**
*
* Retrieve the current configuration of the specified pin.
*
* \param[in] pin  The pin number from the schematic. Range [0-39] Ex: P<pin>
*
* \return         Configuration of specified pin. See the parameters section
*                 - 0xFF if input parameter is invalid
*
* \note
* All input parameter values must be in range or the function will
* have no effect.
*
*******************************************************************************/
uint16_t wiced_hal_gpio_get_pin_config(uint32_t pin);


/*******************************************************************************
* Function Name: wiced_hal_gpio_set_pin_output
****************************************************************************//**
*
* Sets the output value of a pin.
*
* \param[in] pin  The pin number from the schematic. Range [0-39] Ex: P<pin>
* \param[in] val  The output value
*                 - 0         the pin will be set to 0
*                 - non-zero  the pin will be set to 1
*
* \return         none
*
* \note
* The pin must already be configured as output or this will
* have no visible effect.
*
* \note
* All input parameter values must be in range or the function will
* have no effect.
*
*******************************************************************************/
void wiced_hal_gpio_set_pin_output(uint32_t pin, uint32_t val);


/*******************************************************************************
* Function Name: wiced_hal_gpio_get_pin_output
****************************************************************************//**
*
* Get the programmed output value of a pin.
*
* \param[in] pin  The pin number from the schematic. Range [0-39] Ex: P<pin>
*
* \return         The programmed output value of the pin
*                 - 0     If the output port of the pin is set to 0
*                 - 1     If the output port of the pin is set to 0
*                 - 0xFF  If the input parameter is out of range
*
* \note
* This does not return the current pin value. It returns the value that the
* pin would attempt to drive if it is configured as output.
*
*******************************************************************************/
uint32_t wiced_hal_gpio_get_pin_output(uint32_t pin);


/*******************************************************************************
* Function Name: wiced_hal_gpio_get_pin_input_status
****************************************************************************//**
*
* Read the current value at a pin.
*
* \param[in] pin  The pin number from the schematic. Range [0-39] Ex: P<pin>
*
* \return         The input value of the pin
*                 - 0     If the pin is low
*                 - 1     If the pin is high
*                 - 0xFF  If the input parameter is out of range
*
* \note
* For this to be valid, the pin must be configured with input enabled.
*
*******************************************************************************/
uint32_t wiced_hal_gpio_get_pin_input_status(uint32_t pin);


/*******************************************************************************
* Function Name: wiced_hal_gpio_get_pin_interrupt_status
****************************************************************************//**
*
* Get the interrupt status of a pin.
*
* \param[in] pin  The pin number from the schematic. Range [0-39] Ex: P<pin>
*
* \return         The interrupt status of the pin
*                 - 0     If an interrupt (programmed edge) was not detected at the pin
*                 - 1     If an interrupt (programmed edge) was detected at the pin
*                 - 0xFF  If the input parameter is out of range
*
*******************************************************************************/
uint32_t wiced_hal_gpio_get_pin_interrupt_status(uint32_t pin);


/*******************************************************************************
* Function Name: wiced_hal_gpio_register_pin_for_interrupt
****************************************************************************//**
*
* Register a function for notification of changes to a pin (via interrupt).
*
* \param[in] pin       The pin number from the schematic. Range [0-39] Ex: P<pin>
* \param[in] userfn    Pointer to the function to call when the interrupt is triggered
*                      Below is the description of the arguments received by the call back.
*                      void* user_data  - User data provided when interrupt is being registered
*                                         using wiced_hal_gpio_register_pin_for_interrupt(...)
*                      uint8_t value    - Number of the pin causing the interrupt
*
* \param[in] userdata  Pointer that will be passed back to userfn as-is. Typically NULL.
*
* \return    none
*
* \note
* This function is independent of configuring the pin for interrupts;
* a call to wiced_hal_gpio_configure_pin() is also required.
*
* \note
* Also note that once registered, you CANNOT unregister; registration is
* meant to be a startup activity. To stop receiving notifications,
* re-configure the pin and disable the interrupt using wiced_hal_gpio_configure_pin()
*
* \note
* (!) Note that the function does not need to clear the interrupt
* status; this will be done automatically.
*
* \note
* (!) Note that the function will be called ONCE per interrupt, not once per
* pin (this makes a difference if multiple pins toggle at the same time).
*
* Example:\n
* void gpio_int_test_cb(void *data);\n
*
* wiced_hal_gpio_configurePin(WICED_P01,
*     (GPIO_INPUT_ENABLE|GPIO_PULL_DOWN|GPIO_EN_INT_RISING_EDGE),
*     GPIO_PIN_OUTPUT_LOW);\n
* wiced_hal_gpio_register_pin_for_interrupt(WICED_P01, gpio_int_test_cb, NULL);
*
*******************************************************************************/
void wiced_hal_gpio_register_pin_for_interrupt(uint16_t pin,
                                               void (*userfn)(void*, uint8_t), void* userdata);


/*******************************************************************************
* Function Name: wiced_hal_gpio_clear_pin_interrupt_status
****************************************************************************//**
*
* Clear the interrupt status of a pin manually.
*
* \param[in] pin  The pin number from the schematic. Range [0-39] Ex: P<pin>
*
* \return         The interrupt status of the pin
*                 - 1     If clearing the interrupt was successful
*                 - 0xFF  If the input parameter is out of range
*
*******************************************************************************/
uint32_t wiced_hal_gpio_clear_pin_interrupt_status(uint32_t pin);


/*******************************************************************************
* Function Name: wiced_hal_gpio_disable_all_inputs
****************************************************************************//**
*
* Configures all GPIOs except P26 to be INPUT DISABLED.
*
* \param   none
*
* \return  none
*
*******************************************************************************/
void wiced_hal_gpio_disable_all_inputs(void);


/*******************************************************************************
* Function Name: wiced_hal_gpio_slimboot_reenforce_cfg
****************************************************************************//**
*
* Save the LHL GPIO configuration in AON memeory to reenforce in slimboot
* This is a must if we want to wake up in SDS by external LHL GPIO interrupts
*
* \param[in] pin       The pin number from the schematic. Range [0-39] Ex: P<pin>
* \param[in] config    Gpio configuration.
*
* \return              TRUE - successful save/update; FALSE - run out of entries,
*                      not able to save or invalid pin
*
*******************************************************************************/
BOOL32 wiced_hal_gpio_slimboot_reenforce_cfg(uint8_t pin, uint16_t config);

/*******************************************************************************
* Function Name: wiced_hal_gpio_select_function
****************************************************************************//**
*
* Configure a GPIO pin to have the chosen functionality.
*
* \param[in] pin       The pin number from the schematic. Range [0-39] Ex: P<pin>
* \param[in] function  The pin functionality; refer to wiced_bt_gpio_function_t for valid options
*
* \return              The result of selecting the pin function.
*                      Refer to wiced_bt_gpio_select_status_t for possible return states
*
*******************************************************************************/
wiced_bt_gpio_select_status_t wiced_hal_gpio_select_function(wiced_bt_gpio_numbers_t pin, wiced_bt_gpio_function_t function);


/*******************************************************************************
* Function Name: wiced_hal_unassign_gpio_function
****************************************************************************//**
*
* Unassign a GPIO pin functionality
*
* \param[in] pin       The pin number from the schematic. Range [0-39] Ex: P<pin>
* \param[in] function  The pin functionality to remove; refer to wiced_bt_gpio_function_t for valid options
*
* \return              none
*
*******************************************************************************/
void wiced_hal_unassign_gpio_function(uint32_t pin, uint32_t function);

/* \} GPIO */

#endif // __WICED_GPIO_H__
