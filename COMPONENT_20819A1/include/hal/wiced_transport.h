/***************************************************************************//**
* \file <wiced_transport.h>
*
* \brief
* 	This file provides the API declarations for the WICED Transport driver
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

#ifndef _WICED_TRANSPORT_H_
#define _WICED_TRANSPORT_H_

#include "wiced_hal_pspi.h"

/**
* \defgroup group_Transport     WICED Transport
* \ingroup HardwareDrivers
* @{
*
* Defines the transport utilities for configuring the transport, sending data and
* receiving data. This supports the UART transport in HCI Mode
*
* The WICED Transport uses the generic pool available for receiving and sending packets
* over the transport. Following are the configurations of the generic pool.
* 1:   The buffer size - 8 bytes, buffer count - 128.
* 2:   The buffer size - 32 bytes, buffer count - 48.
* 3:   The buffer size - 96 bytes, buffer count - 50.
* 4:   The buffer size - 268 bytes, buffer count - 12.
*
* Using the described generic pools, the max payload size supportable is 252.(16 bytes for internal headers).
* If the application wants to send a packet of a size more than the max size of the buffer
* available in the generic pool, the application can do so by creating a transport buffer
* pool of the desired size. (Refer APIs wiced_transport_create_buffer_pool,
* wiced_transport_allocate_buffer,wiced_transport_send_buffer).
*
* If the application wants to recieve a packet of a size more than the max size of the buffer
* available in the generic pool, the application can do so by configuring a receive buffer pool
* of the desired size when doing the transport init.
*
* \defgroup group_transport_enums Enumerations
* \defgroup group_transport_functions Functions
* \defgroup group_transport_cback_functions Callback Functions
* \defgroup group_transport_data_structures Data Structures
*/

/**
* \addtogroup group_transport_enums
* \{
*/
/*****************************************************************************
* Enumerations
*****************************************************************************/

/** WICED Transport Types */
typedef enum
{
    WICED_TRANSPORT_UART,
    WICED_TRANSPORT_SPI,
    WICED_TRANSPORT_UNDEFINED
}wiced_transport_type_t;

/** WICED UART Transport Mode */
typedef enum
{
    WICED_TRANSPORT_UART_HCI_MODE,
    WICED_TRANSPORT_UART_RAW_MODE,
}wiced_transport_uart_mode_t;
/** \} group_transport_enums */

/** WICED Transport Internal Buffer Pool structure */
typedef struct _wiced_trans_buffer_pool_t wiced_transport_buffer_pool_t;

/**
* \addtogroup group_transport_cback_functions
* \{
*/
/** WICED Transport Status Handler.
 * The callback function registered by the application. This callback will be
 * called by the Transport driver once the transport interface has completed
 * initialization.
 *
 * \param type                   - WICED Transport type
 ******************************************************************************/
typedef void (*wiced_transport_status_handler_t)( wiced_transport_type_t type );

/** WICED Transport Data Handler.
* The callback function registered by the application to recieve data. The
* application has to free the buffer in which data is received. Use the API
* wiced_transport_free_buffer to free the RX buffer.
*
* \param p_data                 - The pointer to the received data buffer.
* \param data_len               - The length of the data pointed to by p_data in bytes.
******************************************************************************/
typedef uint32_t (*wiced_tranport_data_handler_t)( uint8_t* p_data, uint32_t data_len );

/** WICED Transport Transmit complete indication.
* The callback function registered by the application that indicates to the
* application that a packet is has been sent using a buffer in the indicated pool.
*
* \param p_pool                  - The pool pointer.
*                                      The buffer from this pool is used to send a packet.
******************************************************************************/
typedef void (*wiced_transport_tx_complete_t)( wiced_transport_buffer_pool_t* p_pool );
/** \} group_transport_cback_functions */

/**
* \addtogroup group_transport_data_structures
* \{
*/
#pragma pack(1)

/** UART Transport Configuration */
typedef PACKED struct
{
    wiced_transport_uart_mode_t     mode;                        /**<  UART mode, HCI or Raw. */
    uint32_t                        baud_rate;                   /**<  UART baud rate. */
}wiced_uart_transport_cfg_t;

/** WICED SPI Transport Configuration */
typedef PACKED struct
{
    uint8_t          dev_role;            /**< The SPI HW to play either master (1) or slave (2). */
    uint32_t         spi_gpio_cfg;        /**< The pins to use for the data and clk lines. Refer to  spiffdriver.h for details; */
    uint16_t         spi_pin_pull_config; /**< The pin pull-up or pull-down. */
    uint32_t         clock_speed;         /**< The clock speed (non-zero for master, zero for slave).*/
    SPI_ENDIAN       endian;              /**< The direction of the bit data flow (MSB or LSB first). */
    SPI_SS_POLARITY  polarity;            /**< Active high or active low for the chip select line. */
    SPI_MODE         mode;                /**< SPI mode (0-3). */
    uint8_t          cs_pin;              /**< The GPIO pin of chip select line. */
    uint8_t          slave_ready_pin;     /**< The GPIO pin to be used as the slave is ready. */
}wiced_spi_transport_cfg_t;

/** WICED Transport Interface Configuration */
typedef PACKED union
{
    wiced_uart_transport_cfg_t uart_cfg;
    wiced_spi_transport_cfg_t  spi_cfg;
}wiced_transport_interface_cfg_t;

/** WICED Transport receive buffer pool configuration.
* The application uses this to receive
* a packet of the size( i.e if payload size > 252 ) > 268 bytes.
******************************************************************************/

typedef PACKED struct
{
    uint32_t buffer_size;
    uint32_t buffer_count;
}wiced_transport_rx_buff_pool_cfg_t;

/** WICED Transport Configuration */
typedef PACKED struct
{
    wiced_transport_type_t              type;                   /**< WICED transport type. */
    wiced_transport_interface_cfg_t     cfg;                    /**< WICED transport interface config. */
    wiced_transport_rx_buff_pool_cfg_t  rx_buff_pool_cfg;       /**< WICED RX buffer pool config. */
    wiced_transport_status_handler_t    p_status_handler;       /**< WICED transport status handler.*/
    wiced_tranport_data_handler_t       p_data_handler;         /**< WICED transport receive data handler. */
    wiced_transport_tx_complete_t       p_tx_complete_cback;    /**< WICED transport TX complete callback. */
}wiced_transport_cfg_t;

#pragma pack()
/** \} group_transport_data_structures */

/**
* \addtogroup group_transport_functions
* \{
*/

/******************************************************************************
* Function Declarations
******************************************************************************/

/******************************************************************************
* Function Name: wiced_transport_init
***************************************************************************//**
* Initializes and configures the transport interface and also registers the
* callback handlers to be invoked when receiving data, init complete, etc.
*
* \param[in]    p_cfg            - The WICED transport config.
*
* \return     wiced_result_t
******************************************************************************/
wiced_result_t wiced_transport_init( const wiced_transport_cfg_t* p_cfg );

/******************************************************************************
* Function Name: wiced_transport_create_buffer_pool
***************************************************************************//**
* Creates a buffer pool for the transport usage. The application creats a buffer
* pool if it has to send a packet of the size > 268 bytes.
* The application specifies the payload length as the buffer size. The transport
* takes care of creating a pool of the desired size considering the transport
* header requirements and the application specified payload size.
*
* \param[in]    buffer_size            - The size of each buffer in the pool.
*                                        The application specifies
*                                        the payload length as the buffer size.
* \param[in]    buffer_count           - The number of buffers in the pool.
*
* \return                              - The pointer to the buffer pool on success.
*                                        NULL on a failure.
******************************************************************************/
wiced_transport_buffer_pool_t* wiced_transport_create_buffer_pool( uint32_t buffer_size, uint32_t buffer_count );

/******************************************************************************
* Function Name: wiced_transport_allocate_buffer
***************************************************************************//**
* Allocates a buffer from the pool.
*
* \param[in]    p_pool            - The pointer to the buffer pool returned from
*                                   wiced_transport_create_buffer_pool.
*
* \return                         - The pointer to the buffer on success.
*                                   NULL on a failure.
* The application writes the payload starting from this location.
******************************************************************************/
void* wiced_transport_allocate_buffer( wiced_transport_buffer_pool_t* p_pool );

/******************************************************************************
* Function Name: wiced_transport_get_buffer_size
***************************************************************************//**
* Returns the size of the buffer in the pool.

* \param[in]    p_pool         -  The pointer to the buffer pool returned from
*                                 wiced_trans_create_buffer_pool.
*
* \return                      - The size of the buffers of the pool.
******************************************************************************/
uint32_t wiced_transport_get_buffer_size( wiced_transport_buffer_pool_t *p_pool );

/******************************************************************************
* Function Name: wiced_transport_get_buffer_count
***************************************************************************//**
* Gets the number of buffers available in the pool.
*
* \param[in]    p_pool          - The pointer to the buffer pool created using
*                                 wiced_transport_create_buffer_pool.
*
* \return                       - The number of the buffers available in the pool.
******************************************************************************/
uint32_t wiced_transport_get_buffer_count( wiced_transport_buffer_pool_t *p_pool );

/******************************************************************************
* Function Name: wiced_transport_send_buffer
***************************************************************************//**
* Sends the packet to the host over the transport using the buffer allocated
* by the application. This function takes care of preparing the header and
* sending the data. The buffer is freed by the transport after sending the packet.
*
* NOTE The application has to allocate the buffer from the transport pool
* using wiced_transport_allocate_buffer and copy the payload to this buffer
* and send the payload pointer. This allows the application to use custom size
* buffers and avoid the overrunning of generic buffers shared across the firmware code.
*
* \param[in]    code                  - The group code and command code.
* \param[in]    p_buf                 - The pointer to the payload.
* \param[in]    length                - The payload length.
* \return   wiced_result_t
******************************************************************************/
wiced_result_t wiced_transport_send_buffer( uint16_t code, uint8_t* p_buf, uint16_t length );

/******************************************************************************
* Function Name: wiced_transport_free_buffer
***************************************************************************//**
* Frees the transport buffer.
*
* NOTE When receiving a packet, the application takes care of freeing the RX buffers.
*      When sending a packet, the transport takes care of freeing the buffer after the
*      packet is sent.
*
* \param[in]    p_buf                 - The pointer to the buffer to be freed.
*
* \return             None.
******************************************************************************/
void wiced_transport_free_buffer( void * p_buf );

/******************************************************************************
* Function Name: wiced_transport_send_data
***************************************************************************//**
* Sends the packet to the host over the transport interface.
* This function allocates a buffer internally and prepares the header,
* copies the payload and then sends the packet over the transport.
* The maximum size of the buffer that can be allocated is 268 bytes.
*
* The transport internally uses a buffer from the pool
* which is available for all general purposes.
* Following are the configuration of the internal pool:
* 1:   The buffer size - 8 bytes, buffer count - 128.
* 2:   The buffer size - 32 bytes, buffer count - 48.
* 3:   The buffer size - 96 bytes, buffer count - 50.
* 4:   The buffer size - 268 bytes, buffer count - 12.
*
* NOTE Using the described generic pools,
* the max supportable payload size = 252(16 bytes for internal headers).

* \param[in]    code                    - The group code and command code.
* \param[in]    p_data                  - The pointer to the payload.
* \param[in]    length                  - The payload length.
*
* \return   wiced_result_t
******************************************************************************/
wiced_result_t wiced_transport_send_data ( uint16_t code, uint8_t* p_data, uint16_t length );

/******************************************************************************
* Function Name: wiced_transport_send_hci_trace
***************************************************************************//**
* Sends the HCI trace data over the transport.
*
* \param[in]    hci_trans_pool      -
*           - Passes the pointer to the pool created by the application.
*             if the application  has created a dedicated transport pool for
*             communicating with the host.
*             Passes NULL if the application wants the stack to
*             take care of allocating the buffer for sending the data to the host.
*             The application should be able to use the transport buffer pool
*             that it allocates and trace the whole HCI packets.
*             In the case of stack allocation, the size of the trace
*             is compromised according to the buffer availability.
*
* \param[in]    type                   - The HCI trace type.
* \param[in]    p_data                 - The pointer to the data payload.
* \param[in]    length                 - The dData payload length.
*
* \return   wiced_result_t            WICED_SUCCESS on SUCCESS.
*                                     WICED_NO_MEMORY if there are no buffers available to send.
*                                     WICED_ERROR - otherwise.
******************************************************************************/
wiced_result_t wiced_transport_send_hci_trace( wiced_transport_buffer_pool_t *hci_trans_pool ,
                                                             wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );

/******************************************************************************
* Function Name: wiced_transport_send_raw_buffer
***************************************************************************//**
* Used when transport mode is WICED_TRANSPORT_UART_RAW_MODE. Available with the wiced_uart_raw_mode_lib.
* Send the packet to the host over the transport using the buffer allocated by the application.
* This function takes care of preparing the header and sending the data. The buffer must be freed
* by the application if return status is WICED_SUCCESS.
* Note: Application has to allocate buffer from transport pool using wiced_transport_allocate_buffer
* and copy the payload to this buffer and send the payload pointer.
* This allows the application to use custom size buffers and avoid overrun of generic buffers,
* which is shared across firmware code.
*
* \param[in]    p_buf                  - Pointer to the payload
* \param[in]    length                 - Payload length
*
* \return   wiced_result_t            WICED_SUCCESS on SUCCESS.
*                                     WICED_NO_MEMORY if there are no buffers available to send.
*                                     WICED_ERROR - otherwise.
******************************************************************************/
wiced_result_t wiced_transport_send_raw_buffer( uint8_t* p_buf, uint16_t length );
/** \} group_transport_functions */
/** \} group_transport */
/** @} */

#endif /* _WICED_TRANSPORT_H_ */
