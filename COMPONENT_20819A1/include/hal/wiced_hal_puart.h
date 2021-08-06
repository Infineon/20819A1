/***************************************************************************//**
* \file <wiced_hal_puart.h>
*
* \brief
* 	Lists the parameters and defined functions needed to access the
* 	Peripheral Universal Asynchronous Receiver/Transmitter (PUART) driver.
*
*//*****************************************************************************
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

#ifndef __WICED_PUART_H__
#define __WICED_PUART_H__

#include "brcm_fw_types.h"

typedef enum{
    PARITY_ODD,
    PARITY_EVEN,
    PARITY_NONE
}parity_t;

typedef enum{
    STOP_BIT_1,
    STOP_BIT_2
}stop_bit_t;

/**
* \addtogroup  HardwareDrivers Hardware Drivers
*
* @{
*
* Defines a driver to facilitate interfacing with the UART hardware to send
* and receive bytes or a stream of bytes over the UART hardware. Typical
* use-cases involve printing messages over UART/RS-232, or to communicate
* with peripheral devices.
*
* <b>Example Usage:</b>
*
* \code{.c}
* void testPUARTDriver(void)
* {
*     UINT8 readbyte;
*     UINT8 loopCtrl = 1;
*     char printBuffer[50];
*
*     wiced_hal_puart_init();
*     wiced_hal_puart_select_uart_pads(34, 31, 0, 0)
*
*     wiced_hal_puart_flow_off();
*     wiced_hal_puart_enable_tx();
*     wiced_hal_puart_enable_rx();
*
*     while(loopCtrl)
*     {
*         while(wiced_hal_puart_read(&readbyte))
*         {
*             wiced_hal_puart_write(readbyte);
*
*             if(readbyte == 'S')
*             {
*                 wiced_hal_puart_print("\nYou typed 'S'.");
*
*                 sprintf(printBuffer, "\nThis message sprintf'ed here.");
*                 wiced_hal_puart_print(printBuffer);
*             }
*
*             if(readbyte == 'E') // End.
*             {
*                 loopCtrl = 0;
*             }
*         }
*     }
* }
* \endcode
*
*  \addtogroup  PUARTDriver Peripheral UART (PUART)
* \ingroup HardwareDrivers
*
* @{
*/

/*! @{ */

/******************************************************************************
* Function Name: wiced_hal_puart_init
***************************************************************************//**
* Initializes the Peripheral UART interface with the default configuration
* parameters. This must be invoked once at boot before using any of PUART
* services.
*
* The default baud rate is 115200 Bd. This can be changed by calling
* wiced_hal_puart_configuration() as described later, after this
* initialization function.
*
* \param     None.
*
* \return    None.
******************************************************************************/
void wiced_hal_puart_init(void);

/******************************************************************************
* Function Name: wiced_hal_puart_flow_on
***************************************************************************//**
* Enables the flow control.
*
* \param None.
*
* \return None.
******************************************************************************/
void wiced_hal_puart_flow_on(void);

/******************************************************************************
* Function Name: wiced_hal_puart_flow_off
***************************************************************************//**
* Disables the flow control.
*
* \param None.
*
* \return None.
******************************************************************************/
void wiced_hal_puart_flow_off(void);

/******************************************************************************
* Function Name: wiced_hal_puart_select_uart_pads
***************************************************************************//**
* Selects the TX/RX and optional CTS/RTS pins (P<pin>) for the UART hardware
* to use.
*
* Follow the guidelines set in the Kit Guide or HW User Manual when
* selecting pins, because not all pins are avaliable for the PUART driver;
* they depend on the specific hardware platform.
*
* \param rxdPin - RX Pin
* \param txdPin - TX Pin
* \param ctsPin - CTS Pin
* \param rtsPin - RTS Pin
*
* \return    - TRUE if pads were successfully set.
*            - FALSE if pads were not set.
*              If FALSE, make sure the input port/pin parameters are correct.
******************************************************************************/
BOOL32 wiced_hal_puart_select_uart_pads(UINT8 rxdPin, UINT8 txdPin,
                                        UINT8 ctsPin, UINT8 rtsPin);

/******************************************************************************
* Function Name: wiced_hal_puart_print
***************************************************************************//**
* Prints/sends a string of characters via the TX line.
*
* \param string - A string of characters to send.
*
* \return None.
******************************************************************************/
void wiced_hal_puart_print(char * string);

/******************************************************************************
* Function Name: wiced_hal_puart_write
***************************************************************************//**
* Prints/sends one byte via the TX line.
*
* \param byte - The byte to send on the TX line.
*
* \return None.
******************************************************************************/
void wiced_hal_puart_write(UINT8 byte);

/******************************************************************************
* Function Name: wiced_hal_puart_read
***************************************************************************//**
* Reads one byte via the RX line.
*
* \param rxByte - The destination byte to hold received data byte from RX FIFO.
*
* \return       - TRUE if data was successfully read.
*               - FALSE if not (RX FIFO was empty).
******************************************************************************/
BOOL32 wiced_hal_puart_read(UINT8* rxByte);

/******************************************************************************
* Function Name: wiced_hal_puart_disable_tx
***************************************************************************//**
* Disables the transmit capability.
*
* \param           None.
*
* \return          None.
******************************************************************************/
void wiced_hal_puart_disable_tx(void);

/******************************************************************************
* Function Name: wiced_hal_puart_enable_tx
***************************************************************************//**
* Enables the transmit capability.
*
* \param           None.
*
* \return          None.
******************************************************************************/
void wiced_hal_puart_enable_tx(void);

/******************************************************************************
* Function Name: wiced_hal_puart_enable_rx
***************************************************************************//**
* Enables the receive capability (specifically, by enabling PUART RX interrupts
* through the MIA driver).
*
* \param           None.
*
* \return          None.
******************************************************************************/
void wiced_hal_puart_enable_rx(void);

/******************************************************************************
* Function Name: wiced_hal_puart_set_baudrate
***************************************************************************//**
* Sets the baud rate (Bd) to a value other than the default 115200 Bd.
*
* \param baudrate - The desired rate in symbols per second, e.g. 9600.
*
* \return          None.
******************************************************************************/
void wiced_hal_puart_set_baudrate(UINT32 baudrate)__attribute__ ((deprecated("Please use wiced_hal_puart_configuration()")));

/******************************************************************************
* Function Name: wiced_hal_puart_synchronous_read
***************************************************************************//**
* Reads in a set of bytes sequentially.
*
* \param buffer - The destination buffer to hold incoming bytes.
* \param lenth  - The number of bytes to read from the RX FIFO.
*
* \return         None.
******************************************************************************/
void wiced_hal_puart_synchronous_read(UINT8* buffer, UINT32 length);

/******************************************************************************
* Function Name: wiced_hal_puart_synchronous_write
***************************************************************************//**
* Writes a set of bytes sequentially.
*
* \param buffer - The source buffer to hold outgoing bytes.
* \param lenth  - The number of bytes to write to the TX FIFO.
*
* \return         None.
******************************************************************************/
void wiced_hal_puart_synchronous_write(UINT8* buffer,
                                                     UINT32 length);

/******************************************************************************
* Function Name: wiced_hal_puart_rx_fifo_not_empty
***************************************************************************//**
* Checks to see if there is any data ready in the RX FIFO.
*
* \param        None.
*
* \return     - TRUE if bytes are avaliable.
*             - FALSE if the FIFO is empty.
******************************************************************************/
BOOL32 wiced_hal_puart_rx_fifo_not_empty(void);

/******************************************************************************
* Function Name: wiced_hal_puart_reset_puart_interrupt
***************************************************************************//**
* Clears and enables the PUART interrupt.
*
* \param         None.
*
* \return        None.
******************************************************************************/
void wiced_hal_puart_reset_puart_interrupt(void);

/******************************************************************************
* Function Name: wiced_hal_puart_register_interrupt
***************************************************************************//**
* Registers the Interrupt handler with the PUART.
*
* \param puart_rx_cbk - The callback function to process RX bytes.
*
* \return               None.
******************************************************************************/
void wiced_hal_puart_register_interrupt(void (*puart_rx_cbk)(void*));

/******************************************************************************
* Function Name: wiced_hal_puart_set_watermark_level
***************************************************************************//**
* Updates the watermark level to the received value.
*
* \param watermark_level - The watermark level in bytes.
*
* \return                  None.
******************************************************************************/
void wiced_hal_puart_set_watermark_level(UINT32 watermark_level);

/******************************************************************************
* Function Name: wiced_hal_puart_configuration
***************************************************************************//**
* updates the baudrate, parity and stop bits as per received value
* \param baudrate - Desired rate in symbols/sec ex. 9600
* \param parity   - PARITY_ODD for odd parity, PARITY_EVEN for even parity,
*                   PARITY_NONE for no parity.
* \param stop_bit - STOP_BIT_1 for 1 stop bit, STOP_BIT_2 for 2 stop bits.
*
* \return BOOL32  - TRUE if the configuration was successfully set, FALSE
*                   otherwise.NOTE: For baudrate greater than 2.727272 Mb/sec
*                   use of 2 stop bits is mandatory due to hardware
*                   limitation.
********************************************************************************/

BOOL32 wiced_hal_puart_configuration(UINT32 baudrate, parity_t parity, stop_bit_t stop_bit);

/**
 * Function Name: wiced_hal_puart_disable_interrupt
 *
 * Disable Peripheral UART interrupt.
 *
 * \param   none
 *
 * \return  none
 *
 */
void wiced_hal_puart_disable_interrupt(void);

/**
 * Function Name: wiced_hal_puart_deregister_interrupt
 *
 * Deregister Interrupt handler with Peripheral UART.
 *
 * \param   none
 *
 * \return  none
 *
 */
void wiced_hal_puart_deregister_interrupt(void);

/* @} PUARTDriver */
/* @} HardwareDrivers */

#endif /* _WICED_PUART_H_ */

/* [] END OF FILE */
