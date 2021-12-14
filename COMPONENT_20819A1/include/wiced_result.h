/***************************************************************************//**
* \file <wiced_result.h>
*
* \brief
*   This file provides return result codes
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
#ifndef __WICED_RESULT_H
#define __WICED_RESULT_H
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#ifndef RESULT_ENUM
#define RESULT_ENUM( prefix, name, value )  prefix ## name = (value)
#endif /* ifndef RESULT_ENUM */

/*************************************************************************/
/**
 *  @addtogroup  Result         AIROC Result Codes
 *  @ingroup     wicedsys
 *
 *  <b> AIROC Result list </b> for Bluetooth BR/EDR and LE @b .
 *
 *  @{
 */
/*************************************************************************/

/** AIROC result list */
#define WICED_RESULT_LIST( prefix ) \
    RESULT_ENUM( prefix, SUCCESS,                       0x00 ),   /**< Success */\
    RESULT_ENUM( prefix, DELETED,                       0x01 ),   /**< Deleted */\
    RESULT_ENUM( prefix, NO_MEMORY,                     0x10 ),   /**< No memory */\
    RESULT_ENUM( prefix, POOL_ERROR,                    0x02 ),   /**< Pool error */\
    RESULT_ENUM( prefix, PTR_ERROR,                     0x03 ),   /**< PTR error */\
    RESULT_ENUM( prefix, WAIT_ERROR,                    0x04 ),   /**< Wait error */\
    RESULT_ENUM( prefix, SIZE_ERROR,                    0x05 ),   /**< Size error */\
    RESULT_ENUM( prefix, GROUP_ERROR,                   0x06 ),   /**< Group error */\
    RESULT_ENUM( prefix, NO_EVENTS,                     0x07 ),   /**< No events */\
    RESULT_ENUM( prefix, OPTION_ERROR,                  0x08 ),   /**< Option error */\
    RESULT_ENUM( prefix, QUEUE_ERROR,                   0x09 ),   /**< Queue error */\
    RESULT_ENUM( prefix, QUEUE_EMPTY,                   0x0A ),   /**< Queue empty */\
    RESULT_ENUM( prefix, QUEUE_FULL,                    0x0B ),   /**< Queue full */\
    RESULT_ENUM( prefix, SEMAPHORE_ERROR,               0x0C ),   /**< Semaphore error */\
    RESULT_ENUM( prefix, NO_INSTANCE,                   0x0D ),   /**< No instance */\
    RESULT_ENUM( prefix, THREAD_ERROR,                  0x0E ),   /**< Thread error */\
    RESULT_ENUM( prefix, PRIORITY_ERROR,                0x0F ),   /**< Prority error */\
    RESULT_ENUM( prefix, START_ERROR,                   0x10 ),   /**< Start error */\
    RESULT_ENUM( prefix, DELETE_ERROR,                  0x11 ),   /**< Delete error */\
    RESULT_ENUM( prefix, RESUME_ERROR,                  0x12 ),   /**< Resume error */\
    RESULT_ENUM( prefix, CALLER_ERROR,                  0x13 ),   /**< Caller error */\
    RESULT_ENUM( prefix, SUSPEND_ERROR,                 0x14 ),   /**< Suspend error */\
    RESULT_ENUM( prefix, TIMER_ERROR,                   0x15 ),   /**< Timer error */\
    RESULT_ENUM( prefix, TICK_ERROR,                    0x16 ),   /**< Tick error */\
    RESULT_ENUM( prefix, ACTIVATE_ERROR,                0x17 ),   /**< Activate error */\
    RESULT_ENUM( prefix, THRESH_ERROR,                  0x18 ),   /**< Threshold error */\
    RESULT_ENUM( prefix, SUSPEND_LIFTED,                0x19 ),   /**< Suspend lifted */\
    RESULT_ENUM( prefix, WAIT_ABORTED,                  0x1A ),   /**< Wait aborted */\
    RESULT_ENUM( prefix, WAIT_ABORT_ERROR,              0x1B ),   /**< Wait abort error */\
    RESULT_ENUM( prefix, MUTEX_ERROR,                   0x1C ),   /**< Mutex error */\
    RESULT_ENUM( prefix, NOT_AVAILABLE,                 0x1D ),   /**< Not available */\
    RESULT_ENUM( prefix, NOT_OWNED,                     0x1E ),   /**< Not owned */\
    RESULT_ENUM( prefix, INHERIT_ERROR,                 0x1F ),   /**< Inherit error */\
    RESULT_ENUM( prefix, NOT_DONE,                      0x20 ),   /**< Not done */\
    RESULT_ENUM( prefix, CEILING_EXCEEDED,              0x21 ),   /**< Ceiling exceeded */\
    RESULT_ENUM( prefix, INVALID_CEILING,               0x22 ),   /**< Invalid ceiling */\
    RESULT_ENUM( prefix, STA_JOIN_FAILED,               0x23 ),   /**< Join failed */\
    RESULT_ENUM( prefix, SLEEP_ERROR,                   0x24 ),   /**< Sleep error */\
    RESULT_ENUM( prefix, PENDING,                       0x25 ),   /**< Pending */\
    RESULT_ENUM( prefix, TIMEOUT,                       0x26 ),   /**< Timeout */\
    RESULT_ENUM( prefix, PARTIAL_RESULTS,               0x27 ),   /**< Partial results */\
    RESULT_ENUM( prefix, ERROR,                         0x28 ),   /**< Error */\
    RESULT_ENUM( prefix, BADARG,                        0x29 ),   /**< Bad Arguments */\
    RESULT_ENUM( prefix, BADOPTION,                     0x2A ),   /**< Mode not supported */\
    RESULT_ENUM( prefix, UNSUPPORTED,                   0x2B ),   /**< Unsupported function */\
    RESULT_ENUM( prefix, OUT_OF_HEAP_SPACE,             0x2C ),   /**< Dynamic memory space exhausted */\
    RESULT_ENUM( prefix, NOTUP,                         0x2D ),   /**< Interface is not currently Up */\
    RESULT_ENUM( prefix, UNFINISHED,                    0x2E ),   /**< Operation not finished yet */\
    RESULT_ENUM( prefix, CONNECTION_LOST,               0x2F ),   /**< Connection to server lost */\
    RESULT_ENUM( prefix, NOT_FOUND,                     0x30 ),   /**< Item not found */\
    RESULT_ENUM( prefix, PACKET_BUFFER_CORRUPT,         0x31 ),   /**< Packet buffer corrupted */\
    RESULT_ENUM( prefix, ROUTING_ERROR,                 0x32 ),   /**< Routing error */\
    RESULT_ENUM( prefix, BADVALUE,                      0x33 ),   /**< Bad value */\
    RESULT_ENUM( prefix, WOULD_BLOCK,                   0x34 ),   /**< Function would block */\
    RESULT_ENUM( prefix, ABORTED,                       0x35 ),   /**< Operation aborted */\
    RESULT_ENUM( prefix, CONNECTION_RESET,              0x36 ),   /**< Connection has been reset */\
    RESULT_ENUM( prefix, CONNECTION_CLOSED,             0x37 ),   /**< Connection is closed */\
    RESULT_ENUM( prefix, NOT_CONNECTED,                 0x38 ),   /**< Connection is not connected */\
    RESULT_ENUM( prefix, ADDRESS_IN_USE,                0x39 ),   /**< Address is in use */\
    RESULT_ENUM( prefix, NETWORK_INTERFACE_ERROR,       0x3A ),   /**< Network interface error */\
    RESULT_ENUM( prefix, ALREADY_CONNECTED,             0x3B ),   /**< Socket is already connected */\
    RESULT_ENUM( prefix, INVALID_INTERFACE,             0x3C ),   /**< Interface specified in invalid */\
    RESULT_ENUM( prefix, SOCKET_CREATE_FAIL,            0x3D ),   /**< Socket creation failed */\
    RESULT_ENUM( prefix, INVALID_SOCKET,                0x3E ),   /**< Socket is invalid */\
    RESULT_ENUM( prefix, CORRUPT_PACKET_BUFFER,         0x3F ),   /**< Packet buffer is corrupted */\
    RESULT_ENUM( prefix, UNKNOWN_NETWORK_STACK_ERROR,   0x40 ),   /**< Unknown network stack error */\
    RESULT_ENUM( prefix, NO_STORED_AP_IN_DCT,           0x41 ),   /**< DCT contains no AP credentials */\
    RESULT_ENUM( prefix, ALREADY_INITIALIZED,           0x42 ),   /**< Already initialized */\
    RESULT_ENUM( prefix, FEATURE_NOT_ENABLED,           0xFF ),   /**< Feature not enabled */\
/* AIROC result list */

/**@}  AIROC result list */

/* AIROC BT result list */
#define BT_RESULT_LIST( prefix ) \
    RESULT_ENUM( prefix, SUCCESS,                       0    ),   /**< Success */\
    RESULT_ENUM( prefix, PARTIAL_RESULTS,               3    ),   /**< Partial results */\
    RESULT_ENUM( prefix, BADARG,                        5    ),   /**< Bad Arguments */\
    RESULT_ENUM( prefix, BADOPTION,                     6    ),   /**< Mode not supported */\
    RESULT_ENUM( prefix, OUT_OF_HEAP_SPACE,             8    ),   /**< Dynamic memory space exhausted */\
    RESULT_ENUM( prefix, UNKNOWN_EVENT,                 8029 ),   /**< Unknown event is received */\
    RESULT_ENUM( prefix, LIST_EMPTY,                    8010 ),   /**< List is empty */\
    RESULT_ENUM( prefix, ITEM_NOT_IN_LIST,              8011 ),   /**< Item not found in the list */\
    RESULT_ENUM( prefix, PACKET_DATA_OVERFLOW,          8012 ),   /**< Data overflow beyond the packet end */\
    RESULT_ENUM( prefix, PACKET_POOL_EXHAUSTED,         8013 ),   /**< All packets in the pool is in use */\
    RESULT_ENUM( prefix, PACKET_POOL_FATAL_ERROR,       8014 ),   /**< Packet pool fatal error such as permanent packet leak */\
    RESULT_ENUM( prefix, UNKNOWN_PACKET,                8015 ),   /**< Unknown packet */\
    RESULT_ENUM( prefix, PACKET_WRONG_OWNER,            8016 ),   /**< Packet is owned by another entity */\
    RESULT_ENUM( prefix, BUS_UNINITIALISED,             8017 ),   /**< Bluetooth bus isn't initialised */\
    RESULT_ENUM( prefix, MPAF_UNINITIALISED,            8018 ),   /**< MPAF framework isn't initialised */\
    RESULT_ENUM( prefix, RFCOMM_UNINITIALISED,          8019 ),   /**< RFCOMM protocol isn't initialised */\
    RESULT_ENUM( prefix, STACK_UNINITIALISED,           8020 ),   /**< SmartBridge isn't initialised */\
    RESULT_ENUM( prefix, SMARTBRIDGE_UNINITIALISED,     8021 ),   /**< Bluetooth stack isn't initialised */\
    RESULT_ENUM( prefix, ATT_CACHE_UNINITIALISED,       8022 ),   /**< Attribute cache isn't initialised */\
    RESULT_ENUM( prefix, MAX_CONNECTIONS_REACHED,       8023 ),   /**< Maximum number of connections is reached */\
    RESULT_ENUM( prefix, SOCKET_IN_USE,                 8024 ),   /**< Socket specified is in use */\
    RESULT_ENUM( prefix, SOCKET_NOT_CONNECTED,          8025 ),   /**< Socket is not connected or connection failed */\
    RESULT_ENUM( prefix, ENCRYPTION_FAILED,             8026 ),   /**< Encryption failed */\
    RESULT_ENUM( prefix, SCAN_IN_PROGRESS,              8027 ),   /**< Scan is in progress */\
    RESULT_ENUM( prefix, CONNECT_IN_PROGRESS,           8028 ),   /**< Connect is in progress */\
    RESULT_ENUM( prefix, DISCONNECT_IN_PROGRESS,        8029 ),   /**< Disconnect is in progress */\
    RESULT_ENUM( prefix, DISCOVER_IN_PROGRESS,          8030 ),   /**< Discovery is in progress */\
    RESULT_ENUM( prefix, GATT_TIMEOUT,                  8031 ),   /**< GATT timeout occured*/\
    RESULT_ENUM( prefix, ATTRIBUTE_VALUE_TOO_LONG,      8032 ),   /**< Attribute value too long */\
    RESULT_ENUM( prefix, PENDING,                       8100 ),   /**< Pending */\
    RESULT_ENUM( prefix, BUSY,                          8101 ),   /**< Device busy with another command */\
    RESULT_ENUM( prefix, NO_RESOURCES,                  8102 ),   /**< No resources to issue command */\
    RESULT_ENUM( prefix, UNSUPPORTED,                   8103 ),   /**< Unsupported function */\
    RESULT_ENUM( prefix, ILLEGAL_VALUE,                 8104 ),   /**< Illegal parameter value */\
    RESULT_ENUM( prefix, WRONG_MODE,                    8105 ),   /**< Device in wrong mode for request */\
    RESULT_ENUM( prefix, UNKNOWN_ADDR,                  8106 ),   /**< Unknown remote BD address */\
    RESULT_ENUM( prefix, TIMEOUT,                       8107 ),   /**< Timeout */\
    RESULT_ENUM( prefix, BAD_VALUE_RET,                 8108 ),   /**< A bad value was received from HCI */\
    RESULT_ENUM( prefix, ERROR,                         8109 ),   /**< Error */\
    RESULT_ENUM( prefix, NOT_AUTHORIZED,                8110 ),   /**< Authorization failed */\
    RESULT_ENUM( prefix, DEV_RESET,                     8111 ),   /**< Device has been reset */\
    RESULT_ENUM( prefix, CMD_STORED,                    8112 ),   /**< request is stored in control block */\
    RESULT_ENUM( prefix, ILLEGAL_ACTION,                8113 ),   /**< state machine gets illegal command */\
    RESULT_ENUM( prefix, DELAY_CHECK,                   8114 ),   /**< delay the check on encryption */\
    RESULT_ENUM( prefix, SCO_BAD_LENGTH,                8115 ),   /**< Bad SCO over HCI data length */\
    RESULT_ENUM( prefix, SUCCESS_NO_SECURITY,           8116 ),   /**< security passed, no security set */\
    RESULT_ENUM( prefix, FAILED_ON_SECURITY,            8117 ),   /**< security failed */\
    RESULT_ENUM( prefix, REPEATED_ATTEMPTS,             8118 ),   /**< repeated attempts for LE security requests */\
    RESULT_ENUM( prefix, MODE4_LEVEL4_NOT_SUPPORTED,    8119 ),   /**< Connections Only Mode can't be supported */\
    RESULT_ENUM( prefix, USE_DEFAULT_SECURITY,          8120 ),   /**< Use default security */\
    RESULT_ENUM( prefix, KEY_MISSING,                   8121 ),   /**< Key Missing */\
    RESULT_ENUM( prefix, ENCRYPT_DISABLED,              8122 ),   /**< Encryption is disabled */\
/*  AIROC BT result list */

/*************************************************************************/
/**
 *  @addtogroup  Result       AIROC Result Codes
 *  @ingroup     wicedsys
 *
 *  <b> Result types</b>. @b
 *  Result enums, macros with prefix WICED_ to WICED_RESULT_LIST and WICED_BT_ to BT_RESULT_LIST.
 *  See wiced_result.h and wiced_bt_constants.h
 *
 *  @{
 */
/*************************************************************************/
/** AIROC result */
typedef enum
{
    WICED_RESULT_LIST(WICED_)
    BT_RESULT_LIST      (  WICED_BT_       )  /* 8000 - 8999 */
} wiced_result_t;

/**@}  AIROC Result */

/******************************************************************************
*            Structures
******************************************************************************/

/******************************************************************************
*            Function Declarations
******************************************************************************/

#ifdef __cplusplus
} /*extern "C" */
#endif
#endif // __WICED_RESULT_H
