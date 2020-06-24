/***************************************************************************//**
* \file <wiced_bt_ble.h>
*
* Provides the API declarations for BLE host stack management.
*
*//*****************************************************************************
* Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef __WICED_BT_BLE_H__
#define __WICED_BT_BLE_H__

#include "wiced_bt_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup  wiced_bt_ble BLE (Bluetooth Low Energy)
 * \ingroup     wiced_bt_dev
 * \{
 *
 * BLE (Bluetooth Low Energy) Functions.
 *
 * \defgroup group_ble_macro Macro
 * \defgroup group_ble_data_structures Data Structures
 * \defgroup group_ble_enums Enumerated Types
 * \defgroup group_ble_functions Functions
 * \{
 *     \defgroup group_ble_functions_adv Advertising
 *     \defgroup group_ble_functions_scan Scanning
 *     \defgroup group_ble_functions_bg Background Connections
 *     \defgroup group_ble_functions_sec Security
 *     \defgroup group_ble_functions_multi Multi Advertising
 *     \defgroup group_ble_functions_ctrl LE Controller Settings
 * \}
 *
 */

/******************************************************************************
 * Macro definitions                                                          *
 ******************************************************************************/

/**
* \addtogroup group_ble_macro
* \{
*/

/** Default advertising filter policy */
#define BTM_BLE_ADVERT_FILTER_DEFAULT                                      BTM_BLE_ADVERT_FILTER_ALL_CONNECTION_REQ_ALL_SCAN_REQ

/** ADV data flag bit definition used for \ref BTM_BLE_ADVERT_TYPE_FLAG */
#define BTM_BLE_NON_LIMITED_DISCOVERABLE_FLAG                              (0x00)
#define BTM_BLE_LIMITED_DISCOVERABLE_FLAG                                  (0x01 << 0)
#define BTM_BLE_GENERAL_DISCOVERABLE_FLAG                                  (0x01 << 1)
#define BTM_BLE_BREDR_NOT_SUPPORTED                                        (0x01 << 2)

/**
 * LE advertisement flags introduced in 4.1 to indicate
 * simultaneous BR/EDR+LE connections
 */
#define BTM_BLE_SIMULTANEOUS_DUAL_MODE_TO_SAME_DEVICE_CONTROLLER_SUPPORTED (0x01 << 3)
#define BTM_BLE_SIMULTANEOUS_DUAL_MODE_TO_SAME_DEVICE_HOST_SUPPORTED       (0x01 << 4)

#define BTM_BLE_ADVERT_FLAG_MASK                                           (BTM_BLE_LIMITED_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED | BTM_BLE_GENERAL_DISCOVERABLE_FLAG)
#define BTM_BLE_LIMITED_DISCOVERABLE_MASK                                  (BTM_BLE_LIMITED_DISCOVERABLE_FLAG)

/** default advertising channel map */
#define BTM_BLE_DEFAULT_ADVERT_CHNL_MAP                                    (BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39)

/** BLE data signature length 8 Bytes + 4 bytes counter */
#define BTM_BLE_AUTH_SIGNATURE_SIZE                                        12

/** AFH channel map size for BR/EDR transport */
#define BTM_AFH_CHNL_MAP_SIZE                                              HCI_AFH_CHANNEL_MAP_LEN

/** AFH channel map size for BLE transport */
#define BTM_BLE_CHNL_MAP_SIZE                                              HCI_BLE_CHNL_MAP_SIZE

/**
 * Max number of advertisement instances
 * (not including regular adv, instance 0)
 */
#define MULTI_ADV_MAX_NUM_INSTANCES                                        16

/**
 * Bounds of TX power used in Multi ADV. Values are
 * _not_ dbm, but rather indexes into chip-specific table
 */
#define MULTI_ADV_TX_POWER_MIN                                             0
#define MULTI_ADV_TX_POWER_MAX                                             4

/** SIG-define minimum/maximums for BLE parameters */
#define BTM_BLE_ADVERT_INTERVAL_MIN                                 0x0020
#define BTM_BLE_ADVERT_INTERVAL_MAX                                 0x4000

#define BTM_BLE_SCAN_INTERVAL_MIN                                   0x0004
#define BTM_BLE_SCAN_INTERVAL_MAX                                   0x4000
#define BTM_BLE_SCAN_WINDOW_MIN                                     0x0004
#define BTM_BLE_SCAN_WINDOW_MAX                                     0x4000

#define BTM_BLE_CONN_INTERVAL_MIN                                   0x0006
#define BTM_BLE_CONN_INTERVAL_MAX                                   0x0C80
#define BTM_BLE_CONN_LATENCY_MAX                                    500
#define BTM_BLE_CONN_SUP_TOUT_MIN                                   0x000A
#define BTM_BLE_CONN_SUP_TOUT_MAX                                   0x0C80
#define BTM_BLE_CONN_PARAM_UNDEF                                    0xffff

/** \} group_ble_macro */


/******************************************************************************
 * Global Enumerations definitions                                            *
 ******************************************************************************/

/**
* \addtogroup group_ble_enums
* \{
*/

/** BLE AFH Channel map */
typedef uint8_t wiced_bt_ble_chnl_map_t[BTM_AFH_CHNL_MAP_SIZE];

/** Auth signature \ref BTM_BLE_AUTH_SIGNATURE_SIZE */
typedef uint8_t wiced_dev_ble_signature_t[BTM_BLE_AUTH_SIGNATURE_SIZE];

/** Transmit Power in dBm (\ref MULTI_ADV_TX_POWER_MIN to \ref MULTI_ADV_TX_POWER_MAX) */
typedef int8_t wiced_bt_ble_adv_tx_power_t;

/** Advertising filter policy */
enum wiced_bt_ble_advert_filter_policy_e
{
    BTM_BLE_ADVERT_FILTER_ALL_CONNECTION_REQ_ALL_SCAN_REQ                  = 0x00, /**< Process scan and connection requests from all devices (i.e., the White List is not in use) (default) */
    BTM_BLE_ADVERT_FILTER_ALL_CONNECTION_REQ_WHITELIST_SCAN_REQ            = 0x01, /**< Process connection requests from all devices and only scan requests from devices that are in the White List. */
    BTM_BLE_ADVERT_FILTER_WHITELIST_CONNECTION_REQ_ALL_SCAN_REQ            = 0x02, /**< Process scan requests from all devices and only connection requests from devices that are in the White List */
    BTM_BLE_ADVERT_FILTER_WHITELIST_CONNECTION_REQ_WHITELIST_SCAN_REQ      = 0x03, /**< Process scan and connection requests only from devices in the White List. */
    BTM_BLE_ADVERT_FILTER_MAX
};
typedef uint8_t wiced_bt_ble_advert_filter_policy_t;                               /**< \ref wiced_bt_ble_advert_filter_policy_e */

enum
{
    /// Ready to send out an adv in the next few mS. App can change ADV data if required.
    /// Typically invoked about 2.5mS before te ADV. If there are other higher priority
    /// tasks or other events in the app thread event queue, this will be delayed.
    /// Notification is best effort.
    WICED_BT_ADV_NOTIFICATION_READY,

    /// Just completed transmitting an ADV packet.
    WICED_BT_ADV_NOTIFICATION_DONE
};


/** Advertising channel map */
enum wiced_bt_ble_advert_chnl_map_e
{
    BTM_BLE_ADVERT_CHNL_37                                                 = 0x01, /**< Channel 37 */
    BTM_BLE_ADVERT_CHNL_38                                                 = 0x02, /**< Channel 38 */
    BTM_BLE_ADVERT_CHNL_39                                                 = 0x04  /**< Channel 39 */
};
typedef uint8_t wiced_bt_ble_advert_chnl_map_t;                                    /**< \ref wiced_bt_ble_advert_chnl_map_e */

/** Advertisement data types */
enum wiced_bt_ble_advert_type_e
{
    BTM_BLE_ADVERT_TYPE_FLAG                                               = 0x01, /**< Advertisement flags */
    BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL                                      = 0x02, /**< List of supported services - 16 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE                                     = 0x03, /**< List of supported services - 16 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_32SRV_PARTIAL                                      = 0x04, /**< List of supported services - 32 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_32SRV_COMPLETE                                     = 0x05, /**< List of supported services - 32 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_128SRV_PARTIAL                                     = 0x06, /**< List of supported services - 128 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE                                    = 0x07, /**< List of supported services - 128 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_NAME_SHORT                                         = 0x08, /**< Short name */
    BTM_BLE_ADVERT_TYPE_NAME_COMPLETE                                      = 0x09, /**< Complete name */
    BTM_BLE_ADVERT_TYPE_TX_POWER                                           = 0x0A, /**< TX Power level  */
    BTM_BLE_ADVERT_TYPE_DEV_CLASS                                          = 0x0D, /**< Device Class */
    BTM_BLE_ADVERT_TYPE_SIMPLE_PAIRING_HASH_C                              = 0x0E, /**< Simple Pairing Hash C */
    BTM_BLE_ADVERT_TYPE_SIMPLE_PAIRING_RAND_C                              = 0x0F, /**< Simple Pairing Randomizer R */
    BTM_BLE_ADVERT_TYPE_SM_TK                                              = 0x10, /**< Security manager TK value */
    BTM_BLE_ADVERT_TYPE_SM_OOB_FLAG                                        = 0x11, /**< Security manager Out-of-Band data */
    BTM_BLE_ADVERT_TYPE_INTERVAL_RANGE                                     = 0x12, /**< Slave connection interval range */
    BTM_BLE_ADVERT_TYPE_SOLICITATION_SRV_UUID                              = 0x14, /**< List of solicitated services - 16 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_128SOLICITATION_SRV_UUID                           = 0x15, /**< List of solicitated services - 128 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_SERVICE_DATA                                       = 0x16, /**< Service data - 16 bit UUID */
    BTM_BLE_ADVERT_TYPE_PUBLIC_TARGET                                      = 0x17, /**< Public target address */
    BTM_BLE_ADVERT_TYPE_RANDOM_TARGET                                      = 0x18, /**< Random target address */
    BTM_BLE_ADVERT_TYPE_APPEARANCE                                         = 0x19, /**< Appearance */
    BTM_BLE_ADVERT_TYPE_ADVERT_INTERVAL                                    = 0x1a, /**< Advertising interval */
    BTM_BLE_ADVERT_TYPE_LE_BD_ADDR                                         = 0x1b, /**< LE device bluetooth address */
    BTM_BLE_ADVERT_TYPE_LE_ROLE                                            = 0x1c, /**< LE role */
    BTM_BLE_ADVERT_TYPE_256SIMPLE_PAIRING_HASH                             = 0x1d, /**< Simple Pairing Hash C-256 */
    BTM_BLE_ADVERT_TYPE_256SIMPLE_PAIRING_RAND                             = 0x1e, /**< Simple Pairing Randomizer R-256 */
    BTM_BLE_ADVERT_TYPE_32SOLICITATION_SRV_UUID                            = 0x1f, /**< List of solicitated services - 32 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_32SERVICE_DATA                                     = 0x20, /**< Service data - 32 bit UUID */
    BTM_BLE_ADVERT_TYPE_128SERVICE_DATA                                    = 0x21, /**< Service data - 128 bit UUID */
    BTM_BLE_ADVERT_TYPE_CONN_CONFIRM_VAL                                   = 0x22, /**< LE Secure Connections Confirmation Value */
    BTM_BLE_ADVERT_TYPE_CONN_RAND_VAL                                      = 0x23, /**< LE Secure Connections Random Value */
    BTM_BLE_ADVERT_TYPE_URI                                                = 0x24, /**< URI */
    BTM_BLE_ADVERT_TYPE_INDOOR_POS                                         = 0x25, /**< Indoor Positioning */
    BTM_BLE_ADVERT_TYPE_TRANS_DISCOVER_DATA                                = 0x26, /**< Transport Discovery Data */
    BTM_BLE_ADVERT_TYPE_SUPPORTED_FEATURES                                 = 0x27, /**< LE Supported Features */
    BTM_BLE_ADVERT_TYPE_UPDATE_CH_MAP_IND                                  = 0x28, /**< Channel Map Update Indication */
    BTM_BLE_ADVERT_TYPE_PB_ADV                                             = 0x29, /**< PB-ADV */
    BTM_BLE_ADVERT_TYPE_MESH_MSG                                           = 0x2A, /**< Mesh Message */
    BTM_BLE_ADVERT_TYPE_MESH_BEACON                                        = 0x2B, /**< Mesh Beacon */
    BTM_BLE_ADVERT_TYPE_3D_INFO_DATA                                       = 0x3D, /**< 3D Information Data */
    BTM_BLE_ADVERT_TYPE_MANUFACTURER                                       = 0xFF  /**< Manufacturer data */
};
typedef uint8_t wiced_bt_ble_advert_type_t;                                        /**< \ref wiced_bt_ble_advert_type_e */

/** Multi-advertisement start/stop */
enum wiced_bt_ble_multi_advert_start_e
{
    MULTI_ADVERT_STOP                                                      = 0x00, /**< Stop Multi-advertisement */
    MULTI_ADVERT_START                                                     = 0x01  /**< Start Multi-advertisement */
};

/** Multi-advertisement type */
enum wiced_bt_ble_multi_advert_type_e
{
    MULTI_ADVERT_CONNECTABLE_UNDIRECT_EVENT                                = 0x00, /**< Connectable undirected */
    MULTI_ADVERT_CONNECTABLE_DIRECT_EVENT                                  = 0x01, /**< Connectable directed */
    MULTI_ADVERT_DISCOVERABLE_EVENT                                        = 0x02, /**< Discoverable */
    MULTI_ADVERT_NONCONNECTABLE_EVENT                                      = 0x03, /**< Non-connectable */
    MULTI_ADVERT_LOW_DUTY_CYCLE_DIRECT_EVENT                               = 0x04  /**< Low-duty directed */
};
typedef uint8_t wiced_bt_ble_multi_advert_type_t;                                  /**< \ref wiced_bt_ble_multi_advert_type_e */

/** Multi-advertisement Filtering policy */
enum wiced_bt_ble_multi_advert_filtering_policy_e
{
    MULTI_ADVERT_FILTER_POLICY_WHITE_LIST_NOT_USED                         = 0x00, /**< white list not used */
    MULTI_ADVERT_WHITE_LIST_POLICY_ADV_ALLOW_UNKNOWN_CONNECTION            = 0x01, /**< white list for scan request */
    MULTI_ADVERT_WHITE_LIST_POLICY_ADV_ALLOW_UNKNOWN_SCANNING              = 0x02, /**< white list for connection request */
    MULTI_ADVERT_FILTER_POLICY_WHITE_LIST_USED_FOR_ALL                     = 0x03
};
typedef uint8_t wiced_bt_ble_multi_advert_filtering_policy_t;                      /**< \ref wiced_bt_ble_multi_advert_filtering_policy_e */

/** Privacy mode introduced in 5.0 */
enum wiced_bt_ble_privacy_mode_e
{
    BTM_BLE_PRIVACY_MODE_NETWORK                                           = 0x00, /**< network privacy mode*/
    BTM_BLE_PRIVACY_MODE_DEVICE                                            = 0x01  /**< device privacy mode*/
};
typedef uint8_t wiced_bt_ble_privacy_mode_t;                                       /**< \ref wiced_bt_ble_privacy_mode_e */

/** PHY Settings (bits 3-7 reserved) */
enum wiced_bt_ble_host_phy_preferences_e
{
    BTM_BLE_PREFER_1M_PHY                                                  = 0x01, /**< Baseline PHY used by all LE devices */
    BTM_BLE_PREFER_2M_PHY                                                  = 0x02, /**< 2M PHY optional in 5.0 */
    BTM_BLE_PREFER_LELR_PHY                                                = 0x04  /**< Coded PHY optional in 5.0 */
};
typedef uint8_t wiced_bt_ble_host_phy_preferences_t;                               /**< \ref wiced_bt_ble_host_phy_preferences_e*/

/** Coded PHY Sub-setting (bits 2-15 reserved) */
enum wiced_bt_ble_lelr_phy_preferences_e
{
    BTM_BLE_PREFER_CODED_PHY_NONE                                          = 0x00, /**< Coded PHY not used */
    BTM_BLE_PREFER_CODED_125K                                              = 0x01, /**< S8 coding, 128K rate  */
    BTM_BLE_PREFER_CODED_512K                                              = 0x02  /**< S2 Coding, 512K rate  */
};
typedef uint16_t wiced_bt_ble_coded_phy_preferences_t;                              /**< \ref wiced_bt_ble_coded_phy_preferences_e*/

/** Scan modes */
#ifndef BTM_BLE_SCAN_MODES
#define BTM_BLE_SCAN_MODES
enum wiced_bt_ble_scan_mode_e
{
    BTM_BLE_SCAN_MODE_PASSIVE                                              = 0x00, /**< Passive does not send scan request */
    BTM_BLE_SCAN_MODE_ACTIVE                                               = 0x01, /**< Active sends scan request to advertiser */
    BTM_BLE_SCAN_MODE_NONE                                                 = 0xff  /**< Disable scans */
};
#endif
typedef uint8_t wiced_bt_ble_scan_mode_t;                                          /**< \ref wiced_bt_ble_scan_mode_e */

/**
 * Whitelist filter policy
 */
enum wiced_bt_ble_scanner_filter_policy_e
{
    BTM_BLE_SCANNER_FILTER_ALL_ADV_RSP                                     = 0x00, /**< Whitelist OFF (directed ADV to mismatched BD_ADDR ignored) */
    BTM_BLE_SCANNER_FILTER_WHITELIST_ADV_RSP                               = 0x01, /**< Whtelist ON (directed ADV to mismatched BD_ADDR ignored) */
    BTM_BLE_SCANNER_FILTER_ALL_RPA_DIR_ADV_RSP                             = 0x02, /**< Whitelist OFF (accepts directed ADV to mismatched BD_ADDR address if RPA)*/
    BTM_BLE_SCANNER_FILTER_WHITELIST_RPA_DIR_ADV_RSP                       = 0x03, /**< Whitelist ON (accepts directed ADV to mismatched BD_ADDR address if RPA) */
    BTM_BLE_SCANNER_FILTER_MAX
};
typedef uint8_t wiced_bt_ble_scanner_filter_policy_t;                              /**< \ref wiced_bt_ble_scanner_filter_policy_e */

/** Scan result event type */
enum wiced_bt_dev_ble_evt_type_e
{
    BTM_BLE_EVT_CONNECTABLE_ADVERTISEMENT                                  = 0x00, /**< Connectable advertisement */
    BTM_BLE_EVT_CONNECTABLE_DIRECTED_ADVERTISEMENT                         = 0x01, /**< Connectable Directed advertisement */
    BTM_BLE_EVT_SCANNABLE_ADVERTISEMENT                                    = 0x02, /**< Scannable advertisement */
    BTM_BLE_EVT_NON_CONNECTABLE_ADVERTISEMENT                              = 0x03, /**< Non connectable advertisement */
    BTM_BLE_EVT_SCAN_RSP                                                   = 0x04  /**< Scan response */
};
typedef uint8_t wiced_bt_dev_ble_evt_type_t;                                       /**< \ref wiced_bt_dev_ble_evt_type_e */

/** Background connection type */
#ifndef BTM_BLE_CONN_TYPES
#define BTM_BLE_CONN_TYPES
enum wiced_bt_ble_conn_type_e
{
    BTM_BLE_CONN_NONE                               = 0x00,                  /**< No background connection */
    BTM_BLE_CONN_AUTO                               = 0x01,                  /**< Auto connection */
    BTM_BLE_CONN_SELECTIVE                          = 0x02                  /**< Selective connection */
};
#endif
typedef uint8_t wiced_bt_ble_conn_type_t;                                          /**< \ref wiced_bt_ble_conn_type_e */

/** Security settings used with L2CAP LE COC */
enum wiced_bt_ble_sec_flags_e
{
    BTM_SEC_LE_LINK_ENCRYPTED                                              = 0x01, /**< Link encrypted */
    BTM_SEC_LE_LINK_PAIRED_WITHOUT_MITM                                    = 0x02, /**< Paired without man-in-the-middle protection */
    BTM_SEC_LE_LINK_PAIRED_WITH_MITM                                       = 0x04  /**< Link with man-in-the-middle protection */
};

/** LE encryption method */
#ifndef BTM_BLE_SEC_ACTION_TYPES
#define BTM_BLE_SEC_ACTION_TYPES
enum
{
    BTM_BLE_SEC_NONE                    = 0x00,                             /**< No encryption */
    BTM_BLE_SEC_ENCRYPT                 = 0x01,                             /**< encrypt the link using current key */
    BTM_BLE_SEC_ENCRYPT_NO_MITM         = 0x02,                             /**< encryption without MITM */
    BTM_BLE_SEC_ENCRYPT_MITM            = 0x03                              /**< encryption with MITM*/
};
#endif
typedef uint8_t wiced_bt_ble_sec_action_type_t;

/** \} group_ble_enums */


/******************************************************************************
 * Global Data Structure definitions                                          *
 ******************************************************************************/

/**
* \addtogroup group_ble_data_structures
* \{
*/

/** LE Advertisement element*/
typedef struct
{
    uint8_t                                *p_data;                  /**< Advertisement data */
    uint16_t                                len;                     /**< Advertisement length */
    wiced_bt_ble_advert_type_t              advert_type;             /**< Advertisement data type */
} wiced_bt_ble_advert_elem_t;

/** LE scan result type */
typedef struct
{
    wiced_bt_device_address_t               remote_bd_addr;          /**< Device address */
    uint8_t                                 ble_addr_type;           /**< LE Address type */
    wiced_bt_dev_ble_evt_type_t             ble_evt_type;            /**< Scan result event type */
    int8_t                                  rssi;                    /**< Set to \ref BTM_INQ_RES_IGNORE_RSSI (0x7f), if not valid */
    uint8_t                                 flag;
} wiced_bt_ble_scan_results_t;

/** Host PHY preferences */
typedef struct
{
    wiced_bt_device_address_t               remote_bd_addr;          /**< Peer Device address */
    wiced_bt_ble_host_phy_preferences_t     tx_phys;                 /**< Host preference among the TX PHYs */
    wiced_bt_ble_host_phy_preferences_t     rx_phys;                 /**< Host preference among the RX PHYs */
    wiced_bt_ble_coded_phy_preferences_t    phy_opts;                /**< Host preference on LE coded PHY */
    uint8_t                                 resrved;                 /**< Reserved for future use */
} wiced_bt_ble_phy_preferences_t;

/** BLE connection parameters */
typedef struct
{
    uint8_t                                 role;                    /**< Master=0, slave=0*/
    uint16_t                                conn_interval;           /**< Connection interval */
    uint16_t                                conn_latency;            /**< Connection latency */
    uint16_t                                supervision_timeout;     /**< Supervision Timeout */
} wiced_bt_ble_conn_params_t;

/** LE Multi advertising parameter */
typedef struct
{
    uint16_t                                adv_int_min;             /**< Minimum adv interval (\ref BTM_BLE_ADVERT_INTERVAL_MIN to \ref BTM_BLE_ADVERT_INTERVAL_MAX) */
    uint16_t                                adv_int_max;             /**< Maximum adv interval (\ref BTM_BLE_ADVERT_INTERVAL_MIN to \ref BTM_BLE_ADVERT_INTERVAL_MAX) */
    wiced_bt_ble_multi_advert_type_t        adv_type;                /**< Adv event type */
    wiced_bt_ble_advert_chnl_map_t          channel_map;             /**< Adv channel map */
    wiced_bt_ble_advert_filter_policy_t     adv_filter_policy;       /**< Advertising filter policy */
    wiced_bt_ble_adv_tx_power_t             adv_tx_power;            /**< Adv tx power as index into power table (\ref MULTI_ADV_TX_POWER_MIN - \ref MULTI_ADV_TX_POWER_MAX) */
    wiced_bt_device_address_t               peer_bd_addr;            /**< Peer Device address */
    wiced_bt_ble_address_type_t             peer_addr_type;          /**< Peer LE Address type */
    wiced_bt_device_address_t               own_bd_addr;             /**< Own LE address */
    wiced_bt_ble_address_type_t             own_addr_type;           /**< Own LE Address type */
} wiced_bt_ble_multi_adv_params_t;

/** Callback used to indicate selective connection event (set by \ref wiced_bt_ble_set_background_connection_type) */
typedef wiced_bool_t (wiced_bt_ble_selective_conn_cback_t)(wiced_bt_device_address_t remote_bda, uint8_t *p_remote_name);

/** Callback used to return asynchronous results from \ref wiced_bt_ble_scan */
typedef void (wiced_bt_ble_scan_result_cback_t) (wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);

/** \} group_ble_data_structures */

/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/**
* \addtogroup group_ble_functions
* \{
*/

/**
* \addtogroup group_ble_functions_adv
* \{
*/

/*******************************************************************************
* Function Name: wiced_bt_start_advertisements
****************************************************************************//**
*
* Sets the advertisement mode of the host device based on the advertisement
* parameters initialized in \ref wiced_bt_cfg_settings_t upon calling the
* stack initialization API (\ref wiced_bt_stack_init). Before calling this API,
* set the raw ADV data using \ref wiced_bt_ble_set_raw_advertisement_data.
*
* \param[in] advert_mode                         \ref wiced_bt_ble_advert_mode_t
*  - BTM_BLE_ADVERT_OFF
*  - BTM_BLE_ADVERT_DIRECTED_HIGH
*  - BTM_BLE_ADVERT_DIRECTED_LOW
*  - BTM_BLE_ADVERT_UNDIRECTED_HIGH
*  - BTM_BLE_ADVERT_UNDIRECTED_LOW
*  - BTM_BLE_ADVERT_NONCONN_HIGH
*  - BTM_BLE_ADVERT_NONCONN_LOW
*  - BTM_BLE_ADVERT_DISCOVERABLE_HIGH
*  - BTM_BLE_ADVERT_DISCOVERABLE_LOW
* \param[in] directed_advertisement_bdaddr_type  Peer addr type for directed ADV
*  - BLE_ADDR_PUBLIC
*  - BLE_ADDR_RANDOM
* \param[in] directed_advertisement_bdaddr_ptr   Peer addr for directed ADV
*
* \return
*  - WICED_SUCCESS
*  - WICED_ERROR
*
*******************************************************************************/
wiced_result_t wiced_bt_start_advertisements( wiced_bt_ble_advert_mode_t advert_mode,
                                              wiced_bt_ble_address_type_t directed_advertisement_bdaddr_type,
                                              wiced_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr );

/*******************************************************************************
* Function Name: wiced_bt_ble_get_current_advert_mode
****************************************************************************//**
*
* Get advertisement mode from host state machine, set by the API
* \ref wiced_bt_start_advertisements.
*
* \param void
*
* \returns advertisement mode enumerate in \ref wiced_bt_ble_advert_mode_t
*  - BTM_BLE_ADVERT_OFF
*  - BTM_BLE_ADVERT_DIRECTED_HIGH
*  - BTM_BLE_ADVERT_DIRECTED_LOW
*  - BTM_BLE_ADVERT_UNDIRECTED_HIGH
*  - BTM_BLE_ADVERT_UNDIRECTED_LOW
*  - BTM_BLE_ADVERT_NONCONN_HIGH
*  - BTM_BLE_ADVERT_NONCONN_LOW
*  - BTM_BLE_ADVERT_DISCOVERABLE_HIGH
*  - BTM_BLE_ADVERT_DISCOVERABLE_LOW
*
*******************************************************************************/
wiced_bt_ble_advert_mode_t wiced_bt_ble_get_current_advert_mode( void );

/*******************************************************************************
* Function Name: wiced_bt_ble_set_raw_advertisement_data
****************************************************************************//**
*
* Set raw advertisement data of main advertising instance. Advertising elements
* (\ref wiced_bt_ble_advert_elem_t) are used to construct the raw ADV data that
* will be sent down to the controller. Multiple advertising elements can be used
* as long as the total length remains below 31, the max packet length for LE
* ADV packets. Below is an example of calling this API with a single element
* which transmits the name of the device a "HELLO":
*
* \code
* {
*     static char name[] = "HELLO";
*
*     wiced_bt_ble_advert_elem_t adv_elem =
*     {
*         .advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
*         .len          = strlen( name );
*         .p_data       = ( uint8_t* )name;
*     };
*
*     wiced_bt_ble_set_raw_advertisement_data(1, adv_elem);
* }
* \endcode
*
* \param[in] num_elem     num elements in p_data (_not_ bytes)
* \param[in] p_data       pointer to one or more \ref wiced_bt_ble_advert_elem_t
*
* \return
*  - WICED_SUCCESS success
*  - BTM_NO_RESOURCES dynamic memory allocation fail
*  - BTM_WRONG_MODE reserved error code
*
*******************************************************************************/
wiced_result_t wiced_bt_ble_set_raw_advertisement_data( uint8_t num_elem,
                                                        wiced_bt_ble_advert_elem_t *p_data );

/*******************************************************************************
* Function Name: wiced_bt_ble_set_raw_scan_response_data
****************************************************************************//**
*
* Set raw scan response data. Identical functionality as setting regular ADV
* data using \ref wiced_bt_ble_set_raw_advertisement_data. The only difference
* is that this data will be sent out only upon scan request (when the scanner is
* performing an 'active scan').
*
* \param[in] num_elem     num elements in p_data (_not_ bytes)
* \param[in] p_data       pointer to one or more \ref wiced_bt_ble_advert_elem_t
*
* \return
*  - WICED_SUCCESS success
*  - BTM_NO_RESOURCES dynamic memory allocation fail
*  - BTM_WRONG_MODE reserved error code
*
*******************************************************************************/
wiced_bt_dev_status_t wiced_bt_ble_set_raw_scan_response_data( uint8_t num_elem,
                                                               wiced_bt_ble_advert_elem_t *p_data );

/*******************************************************************************
* Function Name: wiced_bt_ble_update_advertising_white_list
****************************************************************************//**
*
* Add or remove a device to/from the advertising white list. The purpose of this
* advertisement whitelist is to filter out connection requests and scan requests
* from other devices before bubbling them up to the application layer.
* After adding devices to the white list, the filter policy must be set using
* \ref wiced_btm_ble_update_advertisement_filter_policy.
*
* \param[in] add                 WICED_TRUE==add, WICED_FALSE==remove
* \param[in] remote_bda          address of device to add/remove from white list
*
* \return
*  - WICED_TRUE indicates success
*  - WICED_FALSE indicates white list full or wrong ADV mode
*
* \note
* This API cannot be used while the white list is in use. If the device is
* currently advertising using the white list, disable it before using this API.
*
*******************************************************************************/
wiced_bool_t wiced_bt_ble_update_advertising_white_list( wiced_bool_t add,
                                                         wiced_bt_device_address_t remote_bda );

/*******************************************************************************
* Function Name: wiced_btm_ble_update_advertisement_filter_policy
****************************************************************************//**
*
* Updates the filter policy when the local device is the advertiser. The purpose
* of this advertisement white list is to filter out connection requests and scan
* requests from other devices before bubbling them up to the application layer.
* Before enabling the advertisement white list, the white list must be populated
* using \ref wiced_bt_ble_update_advertising_white_list.
*
* \param[in] clientCallback           \ref wiced_bt_ble_advert_filter_policy_t
*  - BTM_BLE_ADVERT_FILTER_ALL_CONNECTION_REQ_ALL_SCAN_REQ
*  - BTM_BLE_ADVERT_FILTER_ALL_CONNECTION_REQ_WHITELIST_SCAN_REQ
*  - BTM_BLE_ADVERT_FILTER_WHITELIST_CONNECTION_REQ_ALL_SCAN_REQ
*  - BTM_BLE_ADVERT_FILTER_WHITELIST_CONNECTION_REQ_WHITELIST_SCAN_REQ
*
* \return
*  - WICED_TRUE indicates success
*  - WICED_FALSE indicates feature not supported or wrong ADV mode
*
* \note
* The element ble_white_list_size of the struct \ref wiced_bt_cfg_settings_t
* must be set to a positive value in order for this API to function. This
* config struct is to be passed at stack init (\ref wiced_bt_stack_init).
*
*******************************************************************************/
wiced_bool_t wiced_btm_ble_update_advertisement_filter_policy( wiced_bt_ble_advert_filter_policy_t advertising_policy );

/*******************************************************************************
* Function Name: wiced_bt_ble_set_adv_tx_power
****************************************************************************//**
*
* Sets the transmit power used for the main advertising instance triggered by
* \ref wiced_bt_start_advertisements. The power is specified in dbm. The device
* is only able to alter its tx power within the bounds of the power table set
* in the low-level FW. Different hardware will have different possible transmit
* powers.
*
* Out of bound values will be dropped with no action taken. For example, if the
* current TX power is 0 and the max allowed is 4, inputting 5 or above will
* result in the power staying at 0 (not rounded down to 4).
*
* \param[in] power                           advertisement transmit power in dbm
*
* \return
*  - WICED_SUCCESS command successfully sent to controller
*  - BTM_ERR_PROCESSING transport buffer allocation failed
*
*******************************************************************************/
wiced_result_t wiced_bt_ble_set_adv_tx_power( int8_t power );

/*******************************************************************************
* Function Name: wiced_bt_ble_read_adv_tx_power
****************************************************************************//**
*
* Sends an HCI command to the controller to read the transmit power level used
* in LE ADV packets of the main advertising instance (0). The read value is
* returned in a callback called asynchronously once the controller has a
* response for the host. The below callback can be used to receive the data by
* passing:
*
* \code
* void read_adv_tx_power_callback( wiced_bt_tx_power_result_t *p_tx_power )
* {
*     if ( ( p_tx_power->status == WICED_BT_SUCCESS ) &&
*          ( p_tx_power->hci_status == HCI_SUCCESS ) )
*     {
*         WICED_BT_TRACE("ADV TX power = %i\r\n", p_tx_power->tx_power);
*     }
* }
*
* {
*     wiced_bt_ble_read_adv_tx_power( read_adv_tx_power_callback );
* }
* \endcode
*
* \param[in] p_cback                      callback to receive ADV transmit power
*
* \return
*  - WICED_BT_PENDING if command issued to controller.
*  - WICED_BT_NO_RESOURCES if couldn't allocate memory to issue command
*  - WICED_BT_BUSY if command is already in progress
*
*******************************************************************************/
wiced_result_t wiced_bt_ble_read_adv_tx_power( wiced_bt_dev_cmpl_cback_t *p_cback );

/** \} group_ble_functions_adv */

/**
* \addtogroup group_ble_functions_scan
* \{
*/
/*******************************************************************************
* Function Name: wiced_bt_ble_scan
****************************************************************************//**
*
* This API allows the device to register a callback to receive _connectable_
* ADV packets from slave devices. It is fundamentally the same as the API
* \ref wiced_bt_ble_observe, except that the scan results are filtered to only
* allow connectable packets. The scan interval, window, duration, and type
* (active or passive), are initialized in \ref wiced_bt_cfg_settings_t upon
* calling the stack initialization API (\ref wiced_bt_stack_init). The packets
* received in the callback can be limited to a small, known set of devices
* using \ref wiced_bt_ble_update_scanner_filter_policy.
*
* Below is an example of how to start receiving scan results in a callback
* and print the name of the advertised device if found in the received
* packet:
*
* \code
* void scan_cback( wiced_bt_ble_scan_results_t *scan_res, uint8_t *adv_data )
* {
*     uint8_t length;
*     uint8_t *p_data;
*
*     if ( p_scan_result )
*     {
*         p_data = wiced_bt_ble_check_advertising_data( p_adv_data,
*             BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &length );
*
*         if(length)
*         {
*             WICED_BT_TRACE("%B %s RSSI: %i\r\n",
*                 p_scan_result->remote_bd_addr, (char *)p_data,
*                 p_scan_result->rssi );
*         }
*         else
*         {
*             WICED_BT_TRACE("%B UNK RSSI: %i\r\n",
*                 p_scan_result->remote_bd_addr, p_scan_result->rssi );
*         }
*     }
*     else
*     {
*         WICED_BT_TRACE( "Scan completed\r\n" );
*     }
* }
*
* {
*     wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, scan_cback);
* }
* \endcode
*
* \param[in] scan_type
*  - BTM_BLE_SCAN_TYPE_NONE: disable scan
*  - BTM_BLE_SCAN_TYPE_HIGH_DUTY: use scan params prefixed high_duty_conn_scan_*
*  - BTM_BLE_SCAN_TYPE_LOW_DUTY: use scan params prefixed low_duty_conn_scan_*
* \param[in] duplicate_filter_enable  WICED_TRUE==enable, WICED_FALSE==disable
* \param[in] p_scan_result_cback      callback to receive packets asynchronously
*
* \return
*  - WICED_BT_PENDING if successfully initiated
*  - WICED_BT_BUSY if already in progress
*  - WICED_BT_ILLEGAL_VALUE if parameter(s) are out of range
*  - WICED_BT_NO_RESOURCES if could not allocate resources to start the command
*  - WICED_BT_WRONG_MODE if the device is not up.
*
* \note
* This API should not be used at the same time as \ref wiced_bt_ble_observe.
*
*******************************************************************************/
wiced_result_t  wiced_bt_ble_scan( wiced_bt_ble_scan_type_t scan_type,
                                   wiced_bool_t duplicate_filter_enable,
                                   wiced_bt_ble_scan_result_cback_t *p_scan_result_cback );

/*******************************************************************************
* Function Name: wiced_bt_ble_observe
****************************************************************************//**
*
* This API allows the device to register a callback to receive both connectable
* and non-connectable ADV packets. It is fundamentally the same as the API
* \ref wiced_bt_ble_scan, except that the scan results are filtered to allow
* non-connectable packets. Furthermore, the scan parameters used by the observe
* API are the 'low_duty' parameters set in \ref wiced_bt_cfg_settings_t, which
* is passed to \ref wiced_bt_stack_init.
*
* Refer to \ref wiced_bt_ble_scan for an example of the callback to be used.
*
* \param[in] start                    WICED_TRUE==start, WICED_FALSE==stop
* \param[in] duration                 num_seconds to scan
* \param[in] p_scan_result_cback      callback to receive packets asynchronously
*
* \return
*  - WICED_SUCCESS
*  - WICED_ERROR
*
* \note
* This API should not be used at the same time as \ref wiced_bt_ble_scan.
*
*******************************************************************************/
wiced_bt_dev_status_t wiced_bt_ble_observe( wiced_bool_t start,
                                            uint8_t duration,
                                            wiced_bt_ble_scan_result_cback_t *p_scan_result_cback );

/*******************************************************************************
* Function Name: wiced_bt_ble_get_current_scan_state
****************************************************************************//**
*
* Returns the current scan state that was set using \ref wiced_bt_ble_scan.
*
* \param void
*
* \return
*  - BTM_BLE_SCAN_TYPE_NONE no scan running
*  - BTM_BLE_SCAN_TYPE_HIGH_DUTY high duty cycle scan
*  - BTM_BLE_SCAN_TYPE_LOW_DUTY low duty cycle scan
*
*******************************************************************************/
wiced_bt_ble_scan_type_t wiced_bt_ble_get_current_scan_state( void );

/*******************************************************************************
* Function Name: wiced_bt_ble_check_advertising_data
****************************************************************************//**
*
* After receiving ADV data from another device using \ref wiced_bt_ble_scan or
* \ref wiced_bt_ble_observe, this API can optionally be used to parse the
* received ADV data. For example, if the application wants to check for the
* existence of an ASCII string name contained in the ADV packet, it would pass
* the value BTM_BLE_ADVERT_TYPE_NAME_COMPLETE into the 'type' param. If the
* ADV packet contains the proper field, the function will return a pointer to
* the ASCII string within the given ADV packet.
*
* Refer to \ref wiced_bt_ble_scan header for sample code using this API.
*
* \param[in] p_adv                           pointer to raw ADV data received
* \param[in] type                            ADV data type to parse
* \param[out] p_length                       if found, length of data, else NULL
*
* \return
*  - If found, pointer to start of requested advertisement data.
*  - NULL if requested data type not found.
*
*******************************************************************************/
uint8_t* wiced_bt_ble_check_advertising_data( uint8_t *p_adv, wiced_bt_ble_advert_type_t type, uint8_t *p_length );

/*******************************************************************************
* Function Name: wiced_bt_ble_update_scanner_white_list
****************************************************************************//**
*
* Add or remove a device from the scanner white list. Behavior of white list is
* controlled by \ref wiced_bt_ble_update_scanner_filter_policy.
*
* \param[in] add                           WICED_TRUE==add, WICED_FALSE==remove
* \param[in] remote_bda                    address of device to add/remove
* \param[in] addr_type                     \ref wiced_bt_ble_address_type_t
*  - BLE_ADDR_PUBLIC
*  - BLE_ADDR_RANDOM
*  - BLE_ADDR_PUBLIC_ID
*  - BLE_ADDR_RANDOM_ID
*
* \return
*  - WICED_TRUE success
*  - WICED_FALSE error, transport buffer allocation failure
*
* \note
* This API cannot be used while the white list is in use. If the device is
* currently scanning using the white list, disable it before using this API.
*
*******************************************************************************/
wiced_bool_t wiced_bt_ble_update_scanner_white_list( wiced_bool_t add,
                                                     wiced_bt_device_address_t remote_bda,
                                                     wiced_bt_ble_address_type_t addr_type );

/*******************************************************************************
* Function Name: wiced_bt_ble_update_scanner_filter_policy
****************************************************************************//**
*
* Controls the use of the scanner white list. Using \ref wiced_bt_ble_scan and
* \ref wiced_bt_ble_observe will by default allow any device's advertising
* packets to be received (dropping directed ADV packets whose target address
* does not match our local address). Enabling the whitelist allows the callback
* to only be triggered in the event that an ADV packet is scanned from a small
* list of devices.
*
* Directed packets using an RPA are optionally included in the search results.
* The default behavior is that directed ADV will always be dropped if the
* target address does not match the local address. When RPA (rotating address)
* is in use, we must then add the option of allowing directed packets to an
* RPA through the filter so that the host can resolve the address (if the
* address resolution is occurring on the host side).
*
* \param[in] scanner_policy            \ref wiced_bt_ble_scanner_filter_policy_t
*  - BTM_BLE_SCANNER_FILTER_ALL_ADV_RSP: (default) WL OFF, drops dir to RPA
*  - BTM_BLE_SCANNER_FILTER_WHITELIST_ADV_RSP: WL ON, drops dir to RPA
*  - BTM_BLE_SCANNER_FILTER_ALL_RPA_DIR_ADV_RSP: WL OFF, passes dir to RPA
*  - BTM_BLE_SCANNER_FILTER_WHITELIST_RPA_DIR_ADV_RSP: WL ON, passes dir to RPA
*
* \return void
*
*******************************************************************************/
void wiced_bt_ble_update_scanner_filter_policy( wiced_bt_ble_scanner_filter_policy_t scanner_policy );

/** \} group_ble_functions_scan */

/**
* \addtogroup group_ble_functions_bg
* \{
*/
/*******************************************************************************
* Function Name: wiced_bt_ble_set_background_connection_type
****************************************************************************//**
*
* Sets the local devices background connection procedure. The setting applies to
* central devices that want to auto-reconnect to a small list of peripherals.
* Peripheral devices are added into the white list using the API
* \ref wiced_bt_ble_update_background_connection_device. Once added, this API
* can be called to start the connection procedure.
*
* Using the parameter BTM_BLE_CONN_AUTO will result in the controller
* automatically connecting to any device it finds that is in the BG list.
*
* Using the parameter BTM_BLE_CONN_SELECTIVE will result in any device found via
* a passive scan procedure being bubbled up to the p_select_cback callback. The
* callback simply needs to return WICED_TRUE in order to indicate to the stack
* that it should stop the scan and initiate a direct connection.
*
* If a BG connection is already active, this API will not initiate a new
* connection.
*
* While using this API will automatically start the background connection
* procedure, the internal flags it sets are also used by the API
* \ref wiced_bt_gatt_le_connect. The 'is_direct' parameter of this function
* can be set to WICED_FALSE in order for it to utilize the background
* connection procedure.
*
* \param[in] conn_type
*  - BTM_BLE_CONN_NONE  disable background connection procedure
*  - BTM_BLE_CONN_AUTO initiate auto connection procedure
*  - BTM_BLE_CONN_SELECTIVE selective connection procedure
* \param[in] p_select_cback   connection evt callback for BTM_BLE_CONN_SELECTIVE
*
* \return
*  - WICED_TRUE success
*  - WICED_FALSE command not supported
*
* \note
* For more information on background connections, please refer to BT Spec
* Vers. 5.0, Vol. 3, Part C, 9.3.7 and 9.3.5.
*
* \note
* Background connections are primarily used in HID devices. They are rare
* otherwise.
*
*******************************************************************************/
wiced_bool_t wiced_bt_ble_set_background_connection_type( wiced_bt_ble_conn_type_t conn_type,
                                                          wiced_bt_ble_selective_conn_cback_t *p_select_cback );

/*******************************************************************************
* Function Name: wiced_bt_ble_update_background_connection_device
****************************************************************************//**
*
* Add or remove a device into the list of background connections maintained.
* Once added, the background connection procedure is controlled using the API
* \ref wiced_bt_ble_set_background_connection_type.
*
* \param[in] add_remove                  WICED_TRUE==add, WICED_FALSE==remove
* \param[in] remote_bda                  BD_ADDR of peer device to add or remove
*
* \return
*  - WICED_TRUE success
*  - WICED_FALSE removed from empty list, add to full list, feature unsupported
*
* \note
* Devices that allow SDS sleep are a special case regarding the size of the
* background connections list. Due to memory constraints during sleep, the size
* of the white list is limited to 2 slots. Otherwise, the max number of devices
* is equal to the size of the white list in \ref wiced_bt_cfg_settings_t.
*
*******************************************************************************/
wiced_bool_t wiced_bt_ble_update_background_connection_device( wiced_bool_t add_remove,
                                                               wiced_bt_device_address_t remote_bda );

/*******************************************************************************
* Function Name: wiced_bt_ble_clear_white_list
****************************************************************************//**
*
* Clears the ADV and scan white lists, as well as the background connections
* list. This API will while the white list is in use. Any ADV and scans that use
* the white list, must first be stopped. Any background connections must be
* disconnected before this API can be used.
*
* \param void
*
* \return
*  - WICED_TRUE success
*  - WICED_FALSE error, white list in use
*
*******************************************************************************/
wiced_bool_t wiced_bt_ble_clear_white_list( void );

/*******************************************************************************
* Function Name: wiced_bt_ble_get_white_list_size
****************************************************************************//**
*
* Get the number of devices that can be held by the controller's scanner
* white list. The size of the white list is set by the ble_white_list_size
* element of \ref wiced_bt_cfg_settings_t.
*
* \param void
*
* \return number of slots in the white list
*
*******************************************************************************/
uint8_t wiced_bt_ble_get_white_list_size( void );

/** \} group_ble_functions_bg */

/**
* \addtogroup group_ble_functions_sec
* \{
*/
/*******************************************************************************
* Function Name: wiced_bt_ble_security_grant
****************************************************************************//**
*
* Upon receiving a \ref BTM_SECURITY_REQUEST_EVT in the BT stack management
* callback (\ref wiced_bt_management_cback_t), this API is used to either
* grant or deny a pairing attempt by the peer device.
*
* \param[in] bd_addr                   address of peer device requesting pairing
* \param[in] res
*  - BTM_SUCCESS to grant access
*  - BTM_MODE_UNSUPPORTED to deny pairing
*  - BTM_REPEATED_ATTEMPTS already paired
*
* \return void
*
*******************************************************************************/
void wiced_bt_ble_security_grant( wiced_bt_device_address_t bd_addr, uint8_t res );

/*******************************************************************************
* Function Name: wiced_bt_ble_data_signature
****************************************************************************//**
*
* Generates an authentication signature using AES128 CMAC algorithm. The BD_ADDR
* given as a parameter is used to find the keys stored by WICED stack, then
* the keys are used to generate an authentication signature. This functionality
* is used to perform host-side authentication in the WICED stack. Rarely used
* at the application layer.
*
* \param[in] bd_addr            peer device to sign data for (using stored keys)
* \param[in] p_text             pointer to data being signed
* \param[in] len                length of data in p_text
* \param[out] signature         auth signature output by algorithm
*
* \return
*  - WICED_TRUE success, dereference 'signature' param for output
*  - WICED_FALSE error (peer addr not found or memory allocation failed)
*
*******************************************************************************/
wiced_bool_t wiced_bt_ble_data_signature( wiced_bt_device_address_t bd_addr,
                                          uint8_t *p_text,
                                          uint16_t len,
                                          wiced_dev_ble_signature_t signature );

/*******************************************************************************
* Function Name: wiced_bt_ble_verify_signature
****************************************************************************//**
*
* This function is used to check a given data signature against the original
* unsigned data. The function uses the stored keys of the given LE peer device
* to verify the signature. This functionality is used to perform host-side
* authentication in the WICED stack. Rarely used at the application layer.
*
* \param[in] bd_addr            peer device to sign data for (using stored keys)
* \param[in] p_orig             original data to check signature for
* \param[in] len                length of p_orig
* \param[in] counter            counter input of CCM algo (refer to BT spec)
* \param[in] p_comp             sign. to verify \ref wiced_dev_ble_signature_t
*
* \return
*  - WICED_TRUE success, signature matches
*  - WICED_FALSE signature mismatch, peer keys not found, or buffer alloc. fail
*
*******************************************************************************/
wiced_bool_t wiced_bt_ble_verify_signature( wiced_bt_device_address_t bd_addr,
                                            uint8_t *p_orig,
                                            uint16_t len,
                                            uint32_t counter,
                                            uint8_t *p_comp );

/*******************************************************************************
* Function Name: wiced_bt_ble_get_security_state
****************************************************************************//**
*
* Get security mode 1 flags and encryption key size for LE peer. The GATT layer
* of the WICED stack will check the link's encryption status before triggering
* GATT read/write events on auth_readable/auth_writable characteristics. For
* further MITM checking, the below API should be used to verify the encryption
* status of the link before responding to GATT operations from the peer. This
* additional checking would occur in the callback \ref wiced_bt_gatt_cback_t
* registered in \ref wiced_bt_gatt_register.
*
* If BTM_SEC_LE_LINK_PAIRED_WITHOUT_MITM and BTM_SEC_LE_LINK_PAIRED_WITH_MITM
* are both enabled in the flag, then this indicates that there is an LTK for
* this connection.
*
* \param[in] bd_addr                    address of peer device
* \param[out] p_le_sec_flags            enumerated \ref wiced_bt_ble_sec_flags_e
*  - BTM_SEC_LE_LINK_ENCRYPTED link: encrypted
*  - BTM_SEC_LE_LINK_PAIRED_WITHOUT_MITM: paired w/o man-in-the-middle protect.
*  - BTM_SEC_LE_LINK_PAIRED_WITH_MITM: paired with man-in-the-middle protect.
* \param[out] p_le_key_size            length of LTK or STK
*
* \return
*  - WICED_TRUE success, output found in p_le_sec_flags and p_le_key_size
*  - WICED_FALSE peer device not found
*
*******************************************************************************/
wiced_bool_t wiced_bt_ble_get_security_state( wiced_bt_device_address_t bd_addr,
                                              uint8_t *p_le_sec_flags,
                                              uint8_t *p_le_key_size );

/** \} group_ble_functions_sec */


/**
* \addtogroup group_ble_functions_ctrl
* \{
*/
/*******************************************************************************
* Function Name: wiced_bt_ble_set_channel_classification
****************************************************************************//**
*
* Sends an HCI command to the controller to set the host preferences for
* BLE AFH channel map. The controller will block any channels indicated by the
* map, but may also block additional channels based on its channel assessment.
* In other words, this API can be used to force a channel to be blocked, but
* cannot force a channel to remain open if the controller deems it a bad
* channel--for this reason, channels not marked bad are called 'unknown'.
* Only channels 0-36 can be indicated, other channels are for ADV. At least 1
* channel must be left (open). Example channel map to disable channel 24,
* rest unknown:
*
* \code
* uint8_t channel_map[] = { 0xFF, 0xF7, 0xFF, 0xFF, 0xF8 };
* \endcode
*
* \param[in] ble_channel_map                         bitmap of the channels 0-36
*
* \return
*  - WICED_SUCCESS command successfully sent to controller
*  - WICED_BT_NO_RESOURCES transport buffer allocation failed
*
* \note
* The host preferences remain set in the controller until overwritten by sending
* this command a second time (but do not send sooner than 1 second after first),
* or by HCI reset.
*
*******************************************************************************/
wiced_result_t wiced_bt_ble_set_channel_classification( const wiced_bt_ble_chnl_map_t ble_channel_map );

/*******************************************************************************
* Function Name: wiced_bt_ble_set_phy
****************************************************************************//**
*
* Sends an HCI command to the controller to set the host preferences for
* the BLE PHY used on a specific pre-established LE connection. The response to
* this event is triggered asynchronously via the event BTM_BLE_PHY_UPDATE_EVT,
* sent to the BT stack management handler \ref wiced_bt_management_cback_t
* registered in \ref wiced_bt_stack_init. The handler will receive a flags
* to indicate the current PHY(s) in use (\ref wiced_bt_ble_phy_update_t).
*
* PHY preferences must be set with knowledge of the underlying hardware. Not all
* PHYs are supported by all 20x19 devices. For example, 207819 only supports
* LE2M, but not LELR. Setting preferences for a PHY that does not exist in the
* hardware will have no effect, but still result in PHY update event in the
* management handler.
*
* PHY preferences for LELR in wiced_bt_ble_phy_preferences_t.phy_opts can be set
* even if the LELR PHY is not preferred. This is because the controller can
* override the host's PHY preferences set by this API in favor of LELR (_if_
* the hardware supports LELR).
*
* Example of setting LE2M:
* \code
* {
*     wiced_bt_ble_phy_preferences_t phy_preferences =
*     {
*         .rx_phys = BTM_BLE_PREFER_2M_PHY,
*         .tx_phys = BTM_BLE_PREFER_2M_PHY,
*         .phy_opts = BTM_BLE_PREFER_CODED_PHY_NONE
*     }
*
*     uint8_t peer_bda[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
*     memcpy(phy_preferences.remote_bd_addr, peer_bda, BD_ADDR_LEN);
*
*     wiced_bt_ble_set_phy(&phy_preferences);
* }
* \endcode
*
* \param[in] phy_preferences               \ref wiced_bt_ble_phy_preferences_t
*
* \return
*  - WICED_SUCCESS command successfully sent to controller
*  - BTM_NO_RESOURCES transport buffer allocation failed
*  - BTM_ILLEGAL_VALUE bad param (NULL pointer to phy_preferences)
*  - BTM_UNKNOWN_ADDR could not find peer using given address
*
* \note
* For more information, refer to the BT spec Verion 5.0, Vol. 2, Part E, 7.8.49
* ("LE Set PHY Command").
*
*******************************************************************************/
wiced_bt_dev_status_t wiced_bt_ble_set_phy( wiced_bt_ble_phy_preferences_t *phy_preferences );

/*******************************************************************************
* Function Name: wiced_bt_ble_get_connection_parameters
****************************************************************************//**
*
* Reads the current connection parameters being used for an active LE
* connection. To receive an asynchronous event every time the connections params
* change (instead of polling manually using this API), handle the event
* BTM_BLE_CONNECTION_PARAM_UPDATE in the \ref wiced_bt_management_cback_t, which
* is registered in \ref wiced_bt_stack_init.
*
* Example use of this API to poll params:
* \code
* {
*     wiced_bt_ble_conn_params_t conn_params;
*
*     //addr grabbed from connection_up handler
*     uint8_t peer_bda[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
*
*     wiced_bt_ble_get_connection_parameters(peer_bda, &conn_params);
*
*     WICED_BT_TRACE("Role: %s ", (conn_params.role) ? ("Master") : ("Slave"));
*     WICED_BT_TRACE("Interval: 0x%04x ", conn_params.conn_interval);
*     WICED_BT_TRACE("Latency: 0x%04x ", conn_params.conn_latency);
*     WICED_BT_TRACE("Timeout: 0x%04x\r\n", conn_params.supervision_timeout);
* }
* \endcode
*
* \param[in] bda                           peer address to change parameters for
* \param[out] p_conn_parameters            \ref wiced_bt_ble_conn_params_t
*
* \return
*  - WICED_BT_ILLEGAL_VALUE if p_conn_parameters is NULL.
*  - WICED_BT_UNKNOWN_ADDR  if device address is bad.
*  - WICED_BT_SUCCESS otherwise.
*
* \note
* For information on setting these parameters, refer to
* \ref wiced_bt_l2cap_update_ble_conn_params.
*
*******************************************************************************/
wiced_result_t wiced_bt_ble_get_connection_parameters( wiced_bt_device_address_t bda,
                                                       wiced_bt_ble_conn_params_t *p_conn_parameters );

/*******************************************************************************
* Function Name: wiced_bt_ble_set_privacy_mode
****************************************************************************//**
*
* Sends an HCI command to the controller to tell it whether to use device
* privacy or network privacy for a specific pre-existing LE connection. These
* privacy modes were introduced in BT 5.0. WICED devices will default to device
* privacy mode for all links.
*
* These privacy modes are used for peer devices that use an RPA. The modes
* specify what type of address the peer is allowed to use. In device privacy
* mode, the peer device is allowed to advertise with its identity address. In
* network privacy, the identity address of the peer device will not be accepted.
*
* \param[in] remote_bda                     peer address to set privacy mode for
* \param[in] privacy_mode                   \ref wiced_bt_ble_privacy_mode_t
*  - BTM_BLE_PRIVACY_MODE_DEVICE
*  - BTM_BLE_PRIVACY_MODE_NETWORK
*
* \return
*  - WICED_BT_ILLEGAL_VALUE invalid privacy type
*  - WICED_BT_UNSUPPORTED privacy mode not supported by LE controller
*  - WICED_BT_UNKNOWN_ADDR peer bd_addr cannot be found
*  - WICED_BT_ILLEGAL_ACTION peer not added to resolving list or IRK is invalid
*  - WICED_BT_ERROR error while processing
*  - WICED_BT_SUCCESS command started
*
* \note
* For more information, refer to BT Spec version 5.0, Vol. 6, Part B, 4.7.
*
*******************************************************************************/
wiced_bt_dev_status_t wiced_bt_ble_set_privacy_mode( wiced_bt_device_address_t remote_bda,
                                                     wiced_bt_ble_privacy_mode_t privacy_mode );

/** \} group_ble_functions_ctrl */

/**
* \addtogroup group_ble_functions_multi
* \{
*
* Multi advertisements allow the device to advertise as if it is multiple BLE
* broadcast devices. The device makes available 16 advertising instances.
* Instance 0 is controlled by the default ADV functionality using
* \ref wiced_bt_ble_start_advertisements and should not be controlled using the
* *_multi_advertisements APIs. Each instance has associated ADV parameters, raw
* data, scan response data, transmit power, local address, as well as the
* ability to register for events immediately before and after packet TX.
*/

/*******************************************************************************
* Function Name: wiced_start_multi_advertisements
****************************************************************************//**
*
* Enable or disable advertisements of a specific instance. Prior to calling this
* function, the instance should be instantiated and populated using the APIs
* below. At a minimum, \ref wiced_set_multi_advertisement_params and
* \ref wiced_set_multi_advertisement_data should be used prior.
*
* \param[in] advertising_enable          MULTI_ADVERT_START or MULTI_ADVERT_STOP
* \param[in] adv_instance                1 to MULTI_ADV_MAX_NUM_INSTANCES
*
* \return
*  - BTM_CMD_STARTED
*  - BTM_SUCCESS
*  - BTM_NO_RESOURCES
*
*******************************************************************************/
 wiced_bt_dev_status_t wiced_start_multi_advertisements( uint8_t advertising_enable, uint8_t adv_instance );

 /*******************************************************************************
 * Function Name: wiced_set_multi_advertisement_params
 ****************************************************************************//**
 *
 * Sets the advertising parameters of a specific advertising instance. The
 * parameters are populated into a struct \ref wiced_bt_ble_multi_adv_params_t.
 * The parameters must be set according to the constraints placed by the BT
 * spec, otherwise the parameters will be rejected with error code
 * BTM_ILLEGAL_VALUE.
 *
 * \code
 * {
 *     wiced_bt_ble_multi_adv_params_t params =
 *     {
 *         .adv_int_min = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,
 *         .adv_int_max = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,
 *         .adv_type = MULTI_ADVERT_NONCONNECTABLE_EVENT,
 *         .channel_map = BTM_BLE_DEFAULT_ADVERT_CHNL_MAP,
 *         .adv_filter_policy = MULTI_ADVERT_FILTER_POLICY_WHITE_LIST_NOT_USED,
 *         .adv_tx_power = MULTI_ADV_TX_POWER_MAX,
 *         .peer_addr_type = BLE_ADDR_PUBLIC, //valid only for directed type
 *         .peer_bd_addr = NULL, //valid only for directed type
 *         .own_addr_type = BLE_ADDR_RANDOM
 *     };
 *
 *     uint8_t local_bda[BD_ADDR_LEN] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
 *     memcpy(params.own_bd_addr, local_bda, BD_ADDR_LEN);
 *
 *     wiced_set_multi_advertisement_params(adv_instance, &params);
 * }
 * \endcode
 *
 * \param[in] adv_instance                  1 to MULTI_ADV_MAX_NUM_INSTANCES
 * \param[in] p_param                       \ref wiced_bt_ble_multi_adv_params_t
 *
 * \return
 *  - BTM_ILLEGAL_VALUE element of p_param is out of bounds
 *  - BTM_CMD_STARTED
 *  - BTM_SUCCESS
 *  - BTM_NO_RESOURCES
 *
 *******************************************************************************/
wiced_bt_dev_status_t wiced_set_multi_advertisement_params( uint8_t adv_instance,
                                                            wiced_bt_ble_multi_adv_params_t *p_param );

/*******************************************************************************
* Function Name: wiced_set_multi_advertisement_data
****************************************************************************//**
*
* Refer to \ref wiced_bt_ble_set_raw_advertisement_data. These two functions
* are functionally equivalent other than the fact that this API can specify
* a multi ADV instance for which to set the advertisement data.
*
* \param[in] p_data                             raw adv data
* \param[in] data_len                           num bytes of p_data (max 31)
* \param[in] adv_instance                       1 to MULTI_ADV_MAX_NUM_INSTANCES
*
* \return
*  - BTM_ILLEGAL_VALUE data_len too long or p_data null
*  - BTM_CMD_STARTED success
*  - BTM_SUCCESS success
*  - BTM_NO_RESOURCES transport buffer allocation failed
*
*******************************************************************************/
wiced_bt_dev_status_t wiced_set_multi_advertisement_data( uint8_t *p_data, uint8_t data_len, uint8_t adv_instance );

/*******************************************************************************
* Function Name: wiced_set_multi_advertisement_scan_response_data
****************************************************************************//**
*
* Refer to \ref wiced_bt_ble_set_raw_scan_response_data. These two functions
* are functionally identical other than the fact that this API can specify
* a multi ADV instance for which to set the scan response data.
*
* \param[in] p_data                             raw scan response data
* \param[in] data_len                           num bytes of p_data (max 31)
* \param[in] adv_instance                       1 to MULTI_ADV_MAX_NUM_INSTANCES
*
* \return
*  - BTM_ILLEGAL_VALUE data_len too long or p_data null
*  - BTM_CMD_STARTED success
*  - BTM_SUCCESS success
*  - BTM_NO_RESOURCES transport buffer allocation failed
*
*******************************************************************************/
wiced_bt_dev_status_t wiced_set_multi_advertisement_scan_response_data( uint8_t *p_data,
                                                                        uint8_t data_len,
                                                                        uint8_t adv_instance );

/*******************************************************************************
* Function Name: wiced_set_multi_advertisements_random_address
****************************************************************************//**
*
* Sets the local BD_ADDR that will be used strictly for the given advertising
* instance.
*
* \param[in] randomAddr             local address used for specific ADV instance
* \param[in] adv_instance           1 to MULTI_ADV_MAX_NUM_INSTANCES
*
* \return
*  - BTM_ILLEGAL_VALUE adv_instance out of bounds
*  - BTM_CMD_STARTED success
*  - BTM_SUCCESS success
*  - BTM_NO_RESOURCES transport buffer allocation failed
*
*******************************************************************************/
wiced_bt_dev_status_t wiced_set_multi_advertisements_random_address( wiced_bt_device_address_t randomAddr,
                                                                     uint8_t adv_instance );

/*******************************************************************************
* Function Name: wiced_bt_notify_multi_advertisement_packet_transmissions
****************************************************************************//**
*
* Refer to \ref wiced_bt_notifyAdvPacketTransmissions. These two functions
* are functionally equivalent other than the fact that this API can specify
* a specific multi ADV instance for which to trigger the callback.
*
* \param[in] adv_instance                     1 to MULTI_ADV_MAX_NUM_INSTANCES
* \param[in] clientCallback                   radio evt callback
* \param[in] advanceNoticeInMicroSeconds      num microseconds to preempt ADV tx
*
* \return
*  - WICED_TRUE success
*  - WICED_FALSE adv_instance out of bounds
*
*******************************************************************************/
wiced_bool_t wiced_bt_notify_multi_advertisement_packet_transmissions( uint8_t adv_instance,
                                                                       void (*clientCallback)( uint32_t ),
                                                                       uint32_t advanceNoticeInMicroSeconds );

/** \} group_ble_functions_multi */

/** \} group_ble_functions */

/** \} wiced_bt_ble */

#ifdef __cplusplus
}
#endif

#endif // __WICED_BT_BLE_H__
