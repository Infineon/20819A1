/***************************************************************************//**
*  \file <_wiced_bt_l2c.h>
*  \addtogroup    l2cap   Logical Link Control and Adaptation Protocol (L2CAP)
*  \ingroup     wicedbt
*
* Bluetooth L2CAP Application Programming Interface
*
* \brief Logical Link Control and Adaptation Layer Protocol,
* referred to as L2CAP, provides connection oriented and
* connectionless data services to upper layer protocols with protocol
* multiplexing capability and segmentation and reassembly operation.
*
* @{
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
#pragma once

#include "bt_target.h"
#include "l2cdefs.h"
#include "hcidefs.h"
#include "wiced_bt_types.h"
#include "wiced_bt_ble.h"


/**
*  @addtogroup  l2cap_data_types        Data Types
*  @ingroup     l2cap
*
*  <b> Data Types </b> for @b Logical Link Control and Adaptation Layer Protocol (L2CAP).
*
*  @{
*
******************************************************************************/

/******************************************************************************
*  Constants
******************************************************************************/

/* Defines the minimum offset that L2CAP needs in a buffer.
   This is made up of  HCI type(1), len(2), handle(2), L2CAP len(2) and CID(2) => 9. */

#define L2CAP_MINIMUM_OFFSET    13      /* plus control(2), SDU length(2) */

#define L2CAP_BLE_CONN_MIN_OFFSET          9    /* HCI type(1), len(2), handle(2), L2CAP len(2) and CID(2) */
#define L2CAP_DEFAULT_BLE_CB_POOL_ID    0xFF    /* Use the default HCI ACL buffer pool */
#define L2CAP_BLE_COC_SDU_OFFSET           4    /* To provide an upper layer, some minimal offset is possibly required
                                                   to process incoming packets. */
#define L2CAP_BLE_TX_CONG_START_THRESH     3
#define L2CAP_BLE_TX_CONG_STOP_THRESH      1

/* Minimum offset for broadcast needs another two bytes for the PSM */
#define L2CAP_BROADCAST_MIN_OFFSET         11

/* Ping result codes */
#define L2CAP_PING_RESULT_OK            0       /* Ping reply received OK     */
#define L2CAP_PING_RESULT_NO_LINK       1       /* Link could not be setup    */
#define L2CAP_PING_RESULT_NO_RESPONSE   2       /* Remote L2CAP did not reply */

/* Result codes for wiced_bt_l2cap_data_write() */
#define L2CAP_DATAWRITE_FAILED        FALSE
#define L2CAP_DATAWRITE_SUCCESS       TRUE
#define L2CAP_DATAWRITE_CONGESTED     2

/* Values for priority parameter to wiced_bt_l2cap_set_acl_priority */
#define L2CAP_PRIORITY_NORMAL       0
#define L2CAP_PRIORITY_HIGH         1

/* Values for direction parameter to wiced_bt_l2cap_set_acl_priority */
#define L2CAP_DIRECTION_IGNORE              0       /* Set ACL priority direction as ignore */
#define L2CAP_DIRECTION_DATA_SOURCE         1       /* Set ACL priority direction as source */
#define L2CAP_DIRECTION_DATA_SINK           2       /* Set ACL priority direction as sink */


/* Values for priority parameter to wiced_bt_l2cap_set_tx_priority */
#define L2CAP_CHNL_PRIORITY_HIGH    0
#define L2CAP_CHNL_PRIORITY_MEDIUM  1
#define L2CAP_CHNL_PRIORITY_LOW     2

typedef uint8_t wiced_bt_l2cap_chnl_priority_t;

/* Values for Tx/Rx data rate parameter to wiced_bt_l2cap_set_chnl_data_rate */
#define L2CAP_CHNL_DATA_RATE_HIGH       3
#define L2CAP_CHNL_DATA_RATE_MEDIUM     2
#define L2CAP_CHNL_DATA_RATE_LOW        1
#define L2CAP_CHNL_DATA_RATE_NO_TRAFFIC 0

typedef uint8_t wiced_bt_l2cap_chnl_data_rate_t;

/* Data Packet Flags  (bits 2-15 are reserved)
   layer-specific 14-15 bits are used for FCR SAR.
   Used in call to wiced_bt_l2cap_data_write(). */
#define L2CAP_FLUSHABLE_MASK        0x0003
#define L2CAP_FLUSHABLE_CH_BASED    0x0000
#define L2CAP_FLUSHABLE_PACKET      0x0001
#define L2CAP_NON_FLUSHABLE_PACKET  0x0002


/* Used in wiced_bt_l2cap_flush_channel num_to_flush definitions */
#define L2CAP_FLUSH_CHANNELS_ALL       0xffff
#define L2CAP_FLUSH_CHANNELS_GET       0x0000

/* Definition used for wiced_bt_l2cap_set_desire_role */
#define L2CAP_ROLE_PERIPHERAL            HCI_ROLE_PERIPHERAL
#define L2CAP_ROLE_CENTRAL           HCI_ROLE_CENTRAL
#define L2CAP_ROLE_ALLOW_SWITCH     0x80    /**< set this bit to allow switch at create conn */
#define L2CAP_ROLE_DISALLOW_SWITCH  0x40    /**< set this bit to disallow switch at create conn */
#define L2CAP_ROLE_CHECK_SWITCH     0xC0


/* Values for 'allowed_modes' field passed in structure wiced_bt_l2cap_ertm_information_t */
#define L2CAP_FCR_CHAN_OPT_BASIC    (1 << L2CAP_FCR_BASIC_MODE)
#define L2CAP_FCR_CHAN_OPT_ERTM     (1 << L2CAP_FCR_ERTM_MODE)
#define L2CAP_FCR_CHAN_OPT_STREAM   (1 << L2CAP_FCR_STREAM_MODE)

#define L2CAP_FCR_CHAN_OPT_ALL_MASK (L2CAP_FCR_CHAN_OPT_BASIC | L2CAP_FCR_CHAN_OPT_ERTM | L2CAP_FCR_CHAN_OPT_STREAM)

/* Validity check for PSM.  All PSM values must be odd and
   assigned so that the least significant bit of the most sigificant
   octet equals to zero. */
#define L2C_INVALID_PSM(psm)    (((psm) & 0x0101) != 0x0001)
#define L2C_IS_VALID_PSM(psm)   (((psm) & 0x0101) == 0x0001)

/* Validity check for LE_PSM.
   Fixed LE_PSMs are in the range 0x0001 - 0x007F.
   Dynamic LE_PSM are in the range 0x0080 - 0x00FF.

   Values 0x0000 and 0x0100 - 0xFFFF are reserved. */
#define MINIMIUM_DYNAMIC_LE_PSM 0x0080
#define MAXIMUM_LE_PSM          0x00FF
#define L2C_BLE_INVALID_PSM(le_psm) (!(le_psm) || (le_psm) > MAX_LE_PSM)
#define L2C_BLE_IS_VALID_PSM(le_psm)   (((le_psm) != 0) && ((le_psm) <= MAX_LE_PSM))


/******************************************************************************
*  Type Definitions
******************************************************************************/

/* Structure for Enhanced Retransmission Mode Options
*  Refer to Volume 3, Part A, section 5.4 of BT Core specification for details. */
typedef struct
{
#define L2CAP_FCR_BASIC_MODE    0x00
#define L2CAP_FCR_ERTM_MODE     0x03
#define L2CAP_FCR_STREAM_MODE   0x04

    uint8_t  mode;              /* Requested mode of link */
    uint8_t  tx_window_size;    /* Maximum transmit window size (1..63) */
    uint8_t  max_transmit;      /* Maximum number of trasmission attempts */
    uint16_t rtrans_timeout;    /* Retransmission timeout (msecs) */
    uint16_t monitor_timeout;   /* Monitor timeout (msecs) */
    uint16_t max_pdu_size;      /* Maximum PDU payload size */
} wiced_bt_l2cap_fcr_options_t;

/* Defines a structure to hold the configuration parameters. The
   parameters are optional, so for each parameter there is a boolean to
   use to signify its presence or absence.
   Refer to Volume 3, Part A, section 5.4 of BT Core specification for details. */
typedef struct
{
    uint16_t        result;                 /* Only used in confirm messages */
    wiced_bool_t    mtu_present;            /* TRUE if MTU option present */
    uint16_t        mtu;                    /* Maximum transmission unit size */
    wiced_bool_t    qos_present;            /* QoS configuration present */
    wiced_bt_flow_spec_t qos;               /* QoS configuration */
    wiced_bool_t    flush_timeout_present;  /* TRUE if flush option present */
    uint16_t        flush_timeout;          /* Flush timeout value (1 msec increments) */
    wiced_bool_t    fcr_present;            /* TRUE if Enhanced Retransmission & flow control option present */
    wiced_bt_l2cap_fcr_options_t fcr;       /* Enhanced flow control and retransmission parameters */
    wiced_bool_t    fcs_present;            /* TRUE if Frame check sequence option present */
    uint8_t         fcs;                    /* '0' if desire is to bypass FCS, otherwise '1' */
    uint16_t        flags;                  /* Bit 0: 0-no continuation, 1-continuation */
} wiced_bt_l2cap_cfg_information_t;

/* L2CAP channel configured field bitmap */
#define L2CAP_CH_CFG_MASK_MTU           0x0001
#define L2CAP_CH_CFG_MASK_QOS           0x0002
#define L2CAP_CH_CFG_MASK_FLUSH_TO      0x0004
#define L2CAP_CH_CFG_MASK_FCR           0x0008
#define L2CAP_CH_CFG_MASK_FCS           0x0010
#define L2CAP_CH_CFG_MASK_EXT_FLOW_SPEC 0x0020

typedef uint16_t wiced_bt_l2cap_ch_cfg_bits_t;


/* Applications use this structure to create or accept
   connections with Enhanced Retransmission mode. */
typedef struct
{
    uint8_t       preferred_mode;     /* Preferred mode: ERTM, Streaming, or Basic */
    uint8_t       allowed_modes;      /* Bitmask for allowed modes */
    uint8_t       user_rx_pool_id;    /* GKI Pool ID to use. Typically HCI_ACL_POOL_ID. */
    uint8_t       user_tx_pool_id;    /* GKI Pool ID to use. Typically HCI_ACL_POOL_ID. */
    uint8_t       fcr_rx_pool_id;     /* GKI Pool ID to use. Typically HCI_ACL_POOL_ID. */
    uint8_t       fcr_tx_pool_id;     /* GKI Pool ID to use. Typically HCI_ACL_POOL_ID. */

} wiced_bt_l2cap_ertm_information_t;

/**@}  Data Types */

/******************************************************************************
*  Callback Functions Prototypes
******************************************************************************/
/**
*  @addtogroup  l2cap_callbacks        Callback Functions
*  @ingroup     l2cap
*
*  <b> Callback functions </b> for @b Logical Link Control and Adaptation Layer Protocol (L2CAP).
*
*  @{
*
******************************************************************************/

/******************************************************************************
*
*  Connection established callback prototype.
*
*  @param context          : Caller context provided with wiced_bt_l2cap_register()
*  @param bd_addr          : BD Address of remote
*  @param local_cid        : Local CID assigned to the connection
*  @param peer_mtu         : Peer MTU
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_connected_cback_t) (void *context, wiced_bt_device_address_t bd_addr, uint16_t local_cid, uint16_t peer_mtu);

/******************************************************************************
*
*  Disconnect indication callback prototype.
*
*  @param context          : Caller context provided with wiced_bt_l2cap_register()
*  @param local_cid        : Local CID
*  @param ack              : Boolean whether upper layer should ack this
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_disconnect_indication_cback_t) (void *context, uint16_t local_cid, wiced_bool_t ack);

/******************************************************************************
*
*  Disconnect confirm callback prototype.
*
*  @param context          : Caller context provided with wiced_bt_l2cap_register()
*  @param local_cid        : Local CID
*  @param result           : Result
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_disconnect_confirm_cback_t) (void *context, uint16_t local_cid, uint16_t result);

/******************************************************************************
*
*  Data received indication callback prototype.
*
*  @param context          : Caller context provided with wiced_bt_l2cap_register()
*  @param local_cid        : Local CID
*  @param p_addr_buff      : Address of buffer
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_data_indication_cback_t) (void *context, uint16_t local_cid, uint8_t *p_buff, uint16_t buf_len);

/******************************************************************************
*
*  Congestion status callback protype.
*  This callback is optional. If an application tries to send data when the transmit
*  queue is full, the data will be dropped.
*
*  @param context          : Caller context provided with wiced_bt_l2cap_register()
*  @param local_cid        : Local CID
*  @param congested        : TRUE if congested, FALSE if uncongested
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_congestion_status_cback_t) (void *context, uint16_t local_cid, wiced_bool_t congested);

/******************************************************************************
*
*  Transmit complete callback protype.
*  This callback is optional. If set, L2CAP calls it when packets are sent
*  or flushed. If the count is 0xFFFF, it means all the packets are sent for that
*  CID (eRTM mode only).
*
*  @param context          : Caller context provided with wiced_bt_l2cap_register()
*  @param local_cid        : Local CID
*  @param num_sdu          : Number of SDUs sent or dropped
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_tx_complete_cback_t) (void *context, uint16_t local_cid, uint16_t num_sdu);

/******************************************************************************
*
*  Applications use this structure to register with
*  L2CAP. This structure includes callback functions. All functions
*  must be provided, with the exception of the "connect pending"
*  callback and "congestion status" callback.
*  Additionally, to register a client for dynamic PSM, connect_ind_cb() must
*  be NULL because dynamic PSMs use this as a flag for "virtual PSM".
*
******************************************************************************/
typedef struct
{
    wiced_bt_l2cap_connected_cback_t             *connected_cback;              /* BR/EDR connected event */
    wiced_bt_l2cap_disconnect_indication_cback_t *disconnect_indication_cback;  /* BR/EDR disconnect indication event */
    wiced_bt_l2cap_disconnect_confirm_cback_t    *disconnect_confirm_cback;     /* BR/EDR disconnect confirmation event */
    wiced_bt_l2cap_data_indication_cback_t       *data_indication_cback;        /* BR/EDR data received indication */
    wiced_bt_l2cap_congestion_status_cback_t     *congestion_status_cback;      /* Connection (un)congested event */
    wiced_bt_l2cap_tx_complete_cback_t           *tx_complete_cback;            /* BR/EDR transmit complete event */

    uint16_t                        mtu;
    wiced_bool_t                    qos_present;
    wiced_bt_flow_spec_t            qos;
    wiced_bool_t                    flush_timeout_present;
    uint16_t                        flush_timeout;
    wiced_bool_t                    fcr_present;
    wiced_bt_l2cap_fcr_options_t    fcr;
    wiced_bool_t                    fcs_present;
    uint8_t                         fcs;            /* '0' if desire is to bypass FCS, otherwise '1' */
    wiced_bool_t                    is_ob_only;     /* Set to TRUE if registration is for outbound only to a dynamic PSM */
} wiced_bt_l2cap_appl_information_t;

/******************************************************************************
*  LE Connection indication callback prototype.
*
*  @param context          : Caller context provided with wiced_bt_l2cap_le_register()
*  @param bd_addr          : BD Address of remote
*  @param local_cid        : Local CID assigned to the connection
*  @param psm              : PSM that the remote wants to connect to
*  @param id               : Identifier that the remote sent
*  @param mtu_peer         : MTU of the peer
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_le_connect_indication_cback_t) (void *context, wiced_bt_device_address_t bd_addr,
    uint16_t local_cid, uint16_t psm, uint8_t id, uint16_t mtu_peer);


/******************************************************************************
*
*  LE Connection confirmation callback prototype.
*
*  @param context          : Caller context provided with wiced_bt_l2cap_le_register()
*  @param local_cid        : Local CID
*  @param result           : Result - 0 = connected, non-zero means failure reason
*  @param mtu_peer         : MTU of the peer
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_le_connect_confirm_cback_t) (void *context, uint16_t local_cid,
    uint16_t result, uint16_t mtu_peer);


/******************************************************************************
*
*  LE TX complete callback.
*
*  (Optional) Notification after wiced_bt_l2cap_le_data_write has sent a
*  buffer to the transport layer.
*
*  @param context          : Caller context provided with wiced_bt_l2cap_le_register()
*  @param local_cid        : Local CID
*  @param buf_count        : Number of buffers sent
*
*  @return void
*
******************************************************************************/
typedef void (wiced_bt_l2cap_le_tx_complete_cback_t)(void *context, uint16_t local_cid, uint16_t buf_count);

/******************************************************************************
*
*  Applications use this structure to register with LE L2CAP.
*  This structure includes callback functions. All functions must be provided,
*  with the exception of the "connect pending" callback and "congestion status" callback.
*  Additionally, to register a client for dynamic PSM, connect_ind_cb() must
*  be NULL because dynamic PSMs use this as a flag for "virtual PSM".
*
******************************************************************************/
typedef struct
{
    wiced_bt_l2cap_le_connect_indication_cback_t  *le_connect_indication_cback; /* LE connect indication event */
    wiced_bt_l2cap_le_connect_confirm_cback_t     *le_connect_confirm_cback;    /* LE connect confirm event */
    wiced_bt_l2cap_disconnect_indication_cback_t  *disconnect_indication_cback; /* LE disconnect indication event */
    wiced_bt_l2cap_disconnect_confirm_cback_t     *disconnect_confirm_cback;    /* LE disconnect confirm event */
    wiced_bt_l2cap_data_indication_cback_t        *data_indication_cback;       /* LE data received indication */
    wiced_bt_l2cap_congestion_status_cback_t      *congestion_status_cback;     /* LE congestion status change*/
    wiced_bt_l2cap_le_tx_complete_cback_t         *le_tx_complete_cback;        /* LE tx complete (if using private tx pool) */
} wiced_bt_l2cap_le_appl_information_t;

/**@} l2cap_callbacks */

/******************************************************************************
*  External Function Declarations
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/**
*  \addtogroup  l2cap_api_functions       API Functions
*  \ingroup     l2cap
*
*  <b> API Functions </b> module for @b L2CAP.
*
*  @{
******************************************************************************/

/******************************************************************************
*
*  Function Name:     wiced_bt_l2cap_get_current_config
*
***************************************************************************//**
*
*              This function returns configurations of the L2CAP channel
*              over a BR/EDR link
*
*  @param[in]      lcid: Local CID
*  @param[in]      pp_our_cfg: pointer of our saved configuration options
*  @param[in]      p_our_cfg_bits: valid config in bitmap
*  @param[in]      pp_peer_cfg: pointer of peer's saved configuration options
*  @param[in]      p_peer_cfg_bits : valid config in bitmap
*
*  @return     TRUE if successful, else FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_get_current_config (uint16_t lcid,
    wiced_bt_l2cap_cfg_information_t **pp_our_cfg,  wiced_bt_l2cap_ch_cfg_bits_t *p_our_cfg_bits,
    wiced_bt_l2cap_cfg_information_t **pp_peer_cfg, wiced_bt_l2cap_ch_cfg_bits_t *p_peer_cfg_bits);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_register
*
***************************************************************************//**
*
*                  Other layers call this function to register for L2CAP
*                  services over a BR/EDR link.
*
*  @param[in]      psm: PSM value
*  @param[in]      p_cb_info: L2CAP cb info
*  @param[in]      context: Caller context to return in callbacks
*
*  @return         PSM to use or zero if error. Typically, the PSM returned
*                  is the same as was passed in, but for an outgoing-only
*                  connection to a dynamic PSM, a "virtual" PSM is returned
*                  and should be used in the calls to wiced_bt_l2cap_connect_req().
*
******************************************************************************/
uint16_t wiced_bt_l2cap_register (uint16_t psm, wiced_bt_l2cap_appl_information_t *p_cb_information,  void *context);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_deregister
*
***************************************************************************//**
*
*                  Other layers call this function to deregister for L2CAP
*                  services.
*
*  @param[in]      psm: PSM value
*
*  @return         void
*
******************************************************************************/
void wiced_bt_l2cap_deregister (uint16_t psm);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_allocate_psm
*
***************************************************************************//**
*
*                  Other layers call this function to find an unused PSM for
*                  L2CAP services.
*
*  @return         PSM to use.
*
******************************************************************************/
uint16_t wiced_bt_l2cap_allocate_psm (void);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_connect_req
*
***************************************************************************//**
*
*       Higher layers call this function to create an L2CAP connection over
*       a BR/EDR link. Note that the connection is not established at this
*       time, but connection establishment gets started. The callback
*       function is invoked when connection establishes or fails.
*
*  @param[in]      psm                 : PSM value
*  @param[in]      p_bd_addr           : BD Address
*  @param[in]      p_ertm_information  : ERTM info
*
*  @return         the CID of the connection, or 0 if it failed to start
*
******************************************************************************/
uint16_t wiced_bt_l2cap_connect_req (uint16_t psm, wiced_bt_device_address_t p_bd_addr, wiced_bt_l2cap_ertm_information_t *p_ertm_information);


/******************************************************************************
*
* Function Name:         wiced_bt_l2cap_ertm_enable
*
***************************************************************************//**
*
*                  Enables ERTM over a BR/EDR link.
*
*                  Calling this function causes the linker to include
*                  ERTM-related functions.
*
* @param[in]       void
*
* @return          void
*
******************************************************************************/
void wiced_bt_l2cap_ertm_enable (void);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_ertm_connect_req
*
***************************************************************************//**
*
*       Higher layers call this function to create an L2CAP connection
*       that needs to use Enhanced Retransmission Mode.
*       Note that the connection is not established at this time, but
*       connection establishment gets started. The callback function
*       is invoked when connection establishes or fails.
*
*  @param[in]      psm: PSM value
*  @param[in]      p_bd_addr: BD Address
*  @param[in]      p_ertm_info: ERTM info
*
*  @return         The CID of the connection, or 0 if it failed to start.
*
******************************************************************************/
uint16_t wiced_bt_l2cap_ertm_connect_req (uint16_t psm, wiced_bt_device_address_t p_bd_addr,
    wiced_bt_l2cap_ertm_information_t *p_ertm_information);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_disconnect_req
*
***************************************************************************//**
*
*                  Higher layers call this function to disconnect a channel.
*
*  @param[in]      cid: CID value
*
*  @return         TRUE if disconnect request sent, else FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_disconnect_req (uint16_t cid);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_disconnect_rsp
*
***************************************************************************//**
*
*                  Higher layers call this function to acknowledge the
*                  disconnection of a channel.
*
*  @param[in]      cid: CID value
*
*  @return         TRUE if disconnect response sent, else FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_disconnect_rsp (uint16_t cid);


/******************************************************************************
*
*  Function Name:        wiced_bt_l2cap_data_write
*
***************************************************************************//**
*
*     Higher layers call this function to write data with extended parameters.
*
*  @param[in]      cid: CID value
*  @param[in]      p_data: Input buffer
*  @param[in]      flags: L2CAP_FLUSHABLE_CH_BASED
*                         L2CAP_FLUSHABLE_PACKET
*                         L2CAP_NON_FLUSHABLE_PACKET
*
*  @return         L2CAP_DATAWRITE_SUCCESS, if data accepted, else FALSE
*                  L2CAP_DATAWRITE_CONGESTED, if data accepted and the channel is congested
*                  L2CAP_DATAWRITE_FAILED, if error
*
******************************************************************************/
uint8_t wiced_bt_l2cap_data_write (uint16_t cid, uint8_t *p_buf, uint16_t buf_len, uint16_t flags);

/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_set_idle_timeout
*
***************************************************************************//**
*
*       Higher layers call this function to set the idle timeout for
*       a BR/EDR connection, or for all future connections. The "idle
*       timeout" is the amount of time that a connection can remain up
*       with no L2CAP channels on it.
*       A timeout of zero means that the connection will be torn down
*       immediately after the last channel is removed.
*       A timeout of 0xFFFF means no timeout. Values are in seconds.
*
*  @param[in]      cid: CID value
*  @param[in]      timeout: Timeout value
*  @param[in]      is_global: TRUE, if global
*
*  @return         TRUE if command succeeded, FALSE if failed
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_set_idle_timeout (uint16_t cid, uint16_t timeout,
    wiced_bool_t is_global);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_set_idle_timeout_by_bd_addr
*
***************************************************************************//**
*
*       Higher layers call this function to set the idle timeout for
*       a connection. The "idle timeout" is the amount of time that
*       a connection can remain up with no L2CAP channels on it.
*       A timeout of zero means that the connection is torn
*       down immediately after the last channel is removed.
*       A timeout of 0xFFFF means no timeout. Values are in seconds.
*       bd_addr is the remote BD address. If bd_addr = BT_BD_ANY,
*       the idle timeouts for all active l2cap links is changed.
*
*  @param[in]      bd_addr: BD Address
*  @param[in]      timeout: Timeout value
*
*  @return         TRUE if command succeeded, FALSE if failed
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_set_idle_timeout_by_bd_addr (wiced_bt_device_address_t bd_addr, uint16_t timeout,
                                                  tBT_TRANSPORT transport);

/******************************************************************************
*
*  Function Name:     wiced_bt_l2cap_set_desire_role
*
***************************************************************************//**
*
*              This function sets the desired role for L2CAP on a BR/EDR link.
*              If the new role is L2CAP_ROLE_ALLOW_SWITCH, allows switching to
*              HciCreateConnection.
*              If the new role is L2CAP_ROLE_DISALLOW_SWITCH, does not allow switching to
*              HciCreateConnection.
*
*              If the new role is a valid role (HCI_ROLE_CENTRAL or HCI_ROLE_PERIPHERAL),
*              the desired role is set to a new value. Otherwise, not changed.
*
*  @param[in]      new_role: New role value
*
*  @return     the new (current) role
*
******************************************************************************/
uint8_t wiced_bt_l2cap_set_desire_role (uint8_t new_role);


/******************************************************************************
*
***************************************************************************//**
*
*  Function Name:     wiced_bt_l2cap_flush_channel
*
*              This function flushes none, some or all buffers queued up
*              for transmission for a particular CID. If called with
*              L2CAP_FLUSH_CHANNELS_GET (0), it simply returns the number
*              of buffers queued for that CID L2CAP_FLUSH_CHANNELS_ALL (0xffff)
*              flushes all buffers.  All other values specify the maximum
*              buffers to flush.
*
*  @param[in]      lcid: LCID value
*  @param[in]      num_to_flush: Number of items for flushing
*
*  @return     Number of buffers left queued for that CID.
*
******************************************************************************/
uint16_t   wiced_bt_l2cap_flush_channel (uint16_t lcid, uint16_t num_to_flush);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_set_acl_priority
*
***************************************************************************//**
*
*                  Sets the priority for an ACL channel.
*
*  @param[in]      bd_addr: BD Address
*  @param[in]      priority: [L2CAP_PRIORITY_NORMAL | L2CAP_PRIORITY_HIGH]
*
*  @return         TRUE if a valid channel, else FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_set_acl_priority (wiced_bt_device_address_t bd_addr, uint8_t priority);

/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_set_acl_priority_ext
*
***************************************************************************//**
*
*                  Sets the priority for an ACL channel
*                  with extended parameters.
*
*  @param[in]      bd_addr: BD Address
*  @param[in]      priority: [L2CAP_PRIORITY_NORMAL | L2CAP_PRIORITY_HIGH]
*  @param[in]      direction: [L2CAP_DIRECTION_DATA_SOURCE | L2CAP_DIRECTION_DATA_SINK]
*
*  @return         TRUE if a valid channel, else FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_set_acl_priority_ext (wiced_bt_device_address_t bd_addr, uint8_t priority, uint8_t direction);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_flow_control
*
***************************************************************************//**
*
*                  Higher layers call this function to flow control a BR/EDR channel.
*
*                  data_enabled - TRUE data flows, FALSE data is stopped.
*
*  @param[in]      cid: CID value
*  @param[in]      data_enabled: data enabled
*
*  @return         TRUE if valid channel, else FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_flow_control (uint16_t cid, wiced_bool_t data_enabled);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_set_tx_priority
*
***************************************************************************//**
*
*                  Sets the transmission priority for a  BR/EDR channel. (FCR Mode)
*
*  @param[in]      cid: CID
*  @param[in]      priority: [L2CAP_PRIORITY_NORMAL | L2CAP_PRIORITY_HIGH]
*
*
*  @return         TRUE if a valid channel, else FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_set_tx_priority (uint16_t cid, wiced_bt_l2cap_chnl_priority_t priority);

/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_set_flush_timeout
*
***************************************************************************//**
*
*                  This function sets an automatic flush timeout in Baseband
*                  for ACL-U packets on a BR/EDR link.
*
*  @param[in]      bd_addr: The remote BD address of ACL link. If it is BT_DB_ANY
*                           then the flush time out will be applied to all ACL link.
*  @param[in]      flush_timeout: flush time out in ms
*                           0x0000 : No automatic flush
*                           L2CAP_NO_RETRANSMISSION : No retransmission
*                           0x0002 - 0xFFFE : flush time out, if (flush_timeout*8)+3/5)
*                                    <= HCI_MAX_AUTO_FLUSH_TOUT (in 625us slot).
*                                    Otherwise, return FALSE.
*                           L2CAP_NO_AUTOMATIC_FLUSH : No automatic flush
*
*  @return         TRUE if command succeeded, FALSE if failed
*
*  \note           This flush timeout applies to all logical channels active on the
*                  ACL link.
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_set_flush_timeout (wiced_bt_device_address_t bd_addr, uint16_t flush_timeout);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_get_peer_features
*
***************************************************************************//**
*
*                   Gets the peer features and fixed channel map over a BR/EDR link.
*
*  @param[in]      bd_addr: Peer Bd Address
*  @param[in]      p_ext_feat: features
*  @param[in]      p_chnl_mask: mask storage area
*
*  @return:    TRUE if peer is connected
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_get_peer_features (wiced_bt_device_address_t bd_addr, uint32_t *p_ext_feat, uint8_t *p_chnl_mask);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_get_chnl_fcr_mode
*
***************************************************************************//**
*
*                   Gets the channel FCR mode of a BR/EDR link.
*
*  @param[in]      lcid: Local CID
*
*  @return:    Channel mode
*
******************************************************************************/
uint8_t wiced_bt_l2cap_get_chnl_fcr_mode (uint16_t lcid);


/******************************************************************************
*
*  Function Name:        wiced_bt_l2cap_cancel_ble_connect_req
*
***************************************************************************//**
*
*                  Cancels a pending connection attempt to a BLE device.
*
*  @param[in]      rem_bda: BD Address of remote
*
*  @return:   TRUE if connection was cancelled
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_cancel_ble_connect_req (wiced_bt_device_address_t rem_bda);


/******************************************************************************
*
*  Function Name:        wiced_bt_l2cap_update_ble_conn_params
*
***************************************************************************//**
*
*                  Updates BLE connection parameters.
*
*  @param[in]      rem_bdRa: Remote BD Address
*  @param[in]      min_int: Min interval ( In frames )
*  @param[in]      max_int: Max interval ( In frames )
*  @param[in]      latency: Latency value ( LL connection events )
*  @param[in]      timeout: Timeout value ( Timeout multiplier * 10 ms )
*
*  @return:   TRUE if update started, else FALSE
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_update_ble_conn_params (wiced_bt_device_address_t rem_bdRa, uint16_t min_int, uint16_t max_int, uint16_t latency, uint16_t timeout);


/******************************************************************************
*
*  Function Name:        wiced_bt_l2cap_enable_update_ble_conn_params
*
***************************************************************************//**
*
*                  Enable or disable update based on the request from the peer
*
*  @param[in]      rem_bda: Remote Bd Address
*  @param[in]      enable: TRUE for Enable, FALSE for Disable
*
*  @return:   TRUE for success, FALSE for failure
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_enable_update_ble_conn_params (wiced_bt_device_address_t rem_bda, wiced_bool_t enable);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_get_ble_conn_role
*
***************************************************************************//**
*
*                  This function returns the connection role.
*
*  @param[in]      bd_addr: BD Address
*
*  @return         link role.
*
******************************************************************************/
uint8_t wiced_bt_l2cap_get_ble_conn_role (wiced_bt_device_address_t bd_addr);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_get_disconnect_reason
*
***************************************************************************//**
*
*                  This function returns the disconnect reason code.
*
*  @param[in]      remote_bda: Remote BD Address
*  @param[in]      transport: Transport (BR-EDR or LE)
*
*  @return         disconnect reason
*
******************************************************************************/
uint16_t wiced_bt_l2cap_get_disconnect_reason (wiced_bt_device_address_t remote_bda, tBT_TRANSPORT transport);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_register
*
***************************************************************************//**
*
*                  Other layers call this function to register L2CAP services
*                  for LE_PSM.
*
*  @param[in]      le_psm: LE PSM value
*  @param[in]      p_cb_info: L2CAP cb info
*  @param[in]      context: Caller context to return in callbacks
*
*  @return         LE_PSM to use or zero if error. Typically the LE_PSM returned
*                  is the same as was passed in, but for an outgoing-only
*                  connection a "virtual" LE_PSM is returned  and should be used
*                  in the calls to wiced_bt_l2cap_le_connect_req() and wiced_bt_l2cap_le_deregister().
*
******************************************************************************/
uint16_t wiced_bt_l2cap_le_register (uint16_t le_psm, wiced_bt_l2cap_le_appl_information_t *p_cb_information, void *context);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_deregister
*
***************************************************************************//**
*
*                  Other layers call this function to deregister L2CAP services
*                  for LE_PSM.
*
*  @param[in]      le_psm: LE PSM value
*
*  @return         TRUE for success, FALSE for failure
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_le_deregister (uint16_t le_psm);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_connect_req
*
***************************************************************************//**
*
*                  Higher layers call this function to create an L2CAP connection
*                  for LE_PSM.
*                  Note that the connection is not established at this time, but
*                  connection establishment gets started. The callback function
*                  is invoked when the connection establishes or fails.
*
*  @param[in]      le_psm              : LE PSM value
*  @param[in]      p_bd_addr           : Peer BD Address
*  @param[in]      bd_addr_type        : BLE_ADDR_PUBLIC or BLE_ADDR_RANDOM
*  @param[in]      conn_mode           : BLE_CONN_MODE_HIGH_DUTY or BLE_CONN_MODE_LOW_DUTY
*  @param[in]      rx_mtu              : Rx MTU value (must be <= ACL_POOL_SIZE)
*  @param[in]      rx_sdu_pool_id      : Rx SDU pool ID (typically L2CAP_DEFAULT_BLE_CB_POOL_ID)
*  @param[in]      req_security        : Security required
*  @param[in]      req_encr_key_size   : key size
*
*  @return         The CID of the connection, or 0 if it failed to start.
*
******************************************************************************/
uint16_t wiced_bt_l2cap_le_connect_req (uint16_t le_psm, wiced_bt_device_address_t p_bd_addr,
    wiced_bt_ble_address_type_t bd_addr_type,
    wiced_bt_ble_conn_mode_t conn_mode,
    uint16_t rx_mtu, uint8_t rx_sdu_pool_id,
    uint8_t req_security, uint8_t req_encr_key_size);


/******************************************************************************
*
***************************************************************************//**
*
*  Function Name:         wiced_bt_l2cap_le_connect_rsp
*
*                  Higher layers call this function to accept an incoming
*                  LE L2CAP connection, for which they received an connect
*                  indication callback.
*
*  @param[in]      p_bd_addr      : Peer BD Address
*  @param[in]      id             : Identifier that the remote sent
*  @param[in]      lcid           : LCID value
*  @param[in]      result         : L2CAP connection result code
*  @param[in]      rx_mtu         : Rx MTU value
*  @param[in]      rx_sdu_pool_id : Rx SDU pool ID (typically L2CAP_DEFAULT_BLE_CB_POOL_ID)
*
*  @return         TRUE for success, FALSE for failure
*
******************************************************************************/
wiced_bool_t  wiced_bt_l2cap_le_connect_rsp (wiced_bt_device_address_t p_bd_addr, uint8_t id, uint16_t lcid,
    uint16_t result, uint16_t rx_mtu, uint8_t rx_sdu_pool_id);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_disconnect_req
*
***************************************************************************//**
*
*                  Higher layers call this function to disconnect an LE COC
*                  channel.
*
*  @param[in]      lcid: LCID value
*
*  @return         TRUE if disconnect sent, else FALSE.
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_le_disconnect_req (uint16_t lcid);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_disconnect_rsp
*
***************************************************************************//**
*
*                  Higher layers call this function to acknowledge the
*                  disconnection of a LE COC channel.
*
*  @param[in]      lcid: LCID value
*
*  @return         TRUE for success, FALSE for failure
*
******************************************************************************/
wiced_bool_t wiced_bt_l2cap_le_disconnect_rsp (uint16_t lcid);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_data_write
*
***************************************************************************//**
*
*                  Sends data over the LE connection-oriented channel.
*
*  @param[in]      cid: Channel ID
*  @param[in]      p_data: Input buffer
*  @param[in]      buf_len: p_data buffer size
*
*  @return         L2CAP_DATAWRITE_SUCCESS, if data accepted, else FALSE
*                  L2CAP_DATAWRITE_CONGESTED, if data accepted and the channel is congested
*                  L2CAP_DATAWRITE_FAILED, if error
*
******************************************************************************/
uint8_t wiced_bt_l2cap_le_data_write (uint16_t cid, uint8_t *p_data, uint16_t buf_len, uint16_t flags);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_set_user_congestion
*
***************************************************************************//**
*
*                  Higher layers call this function to tell if the connection
*                  is congested or not.
*
*  @param[in]      lcid: LCID value
*  @param[in]      is_congested: TRUE, if congested
*
*  @return         TRUE if command processed OK
*
******************************************************************************/
wiced_bool_t  wiced_bt_l2cap_le_set_user_congestion (uint16_t lcid, wiced_bool_t is_congested);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_get_peer_mtu
*
***************************************************************************//**
*
*                  Higher layers call this function to get peer MTU.
*
*  @param[in]      lcid: LCID value
*
*  @return         Peer MTU or 0.
*
******************************************************************************/
uint16_t wiced_bt_l2cap_le_get_peer_mtu (uint16_t lcid);


/******************************************************************************
*
*  Function Name:         wiced_bt_l2cap_le_determ_secur_rsp
*
***************************************************************************//**
*
*                  Higher layers call this function to check if the current
*                  device security settings are sufficient to continue with
*                  call establishment.
*                  It is called by the call acceptor on reception of n LE Credit
*                  Based Connection Request.
*
*  @param[in]      bd_addr: BD Address
*  @param[in]      req_secur: Security required flags (see #wiced_bt_ble_sec_flags_e)
*  @param[in]      req_encr_key_size: Key size
*
*  @return         L2CAP_CONN_OK/L2CAP_BLE_CONN_BAD_AUTHENT/
*                  L2CAP_BLE_CONN_BAD_KEY_SIZE/L2CAP_BLE_CONN_BAD_ENCRYPT/
*                  L2CAP_CONN_NO_RESOURCES.
*
******************************************************************************/
uint16_t wiced_bt_l2cap_le_determ_secur_rsp (wiced_bt_device_address_t bd_addr, uint8_t req_secur, uint8_t req_encr_key_size);


/**@} l2cap_api_functions */

#ifdef __cplusplus
}
#endif /* _WICED_BT_L2C_H_ */
/**@} l2cap */
