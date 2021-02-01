/***************************************************************************//**
* \file <wiced_bt_dev.h>
*
* Provides the API declarations for device management.
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

#ifndef __WICED_BT_DEV_H__
#define __WICED_BT_DEV_H__

#include "wiced.h"
#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "hcidefs.h"
#include "bt_target.h"
#include "brcm_fw_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup  wiced_bt_dev Device Management
 * \ingroup     wicedbt
 * \{
 *
 * WICED Device Management APIs
 *
 * \defgroup wiced_bt_dev_classic BT Classic (BR/EDR)
 * \{
 *
 * \defgroup group_classic_macro Macro
 * \defgroup group_dev_data_structures Data Structures
 * \defgroup group_dev_enums Enumerated Types
 * \defgroup group_dev_functions Functions
 * \{
 *     \defgroup group_dev_functions_ctrl Device Control
 *     \defgroup group_dev_functions_classic BT Classic Host Stack Management
 *     \defgroup group_dev_functions_sec Security
 * \}
 */

/******************************************************************************
 * Macro definitions                                                          *
 ******************************************************************************/

/**
* \addtogroup group_classic_macro
* \{
*/

/** SCO link type */
#define BTM_LINK_TYPE_SCO               HCI_LINK_TYPE_SCO                      /**< Link type SCO */
#define BTM_LINK_TYPE_ESCO              HCI_LINK_TYPE_ESCO                     /**< Link type eSCO */

/** BR channel map*/
#define BTM_AFH_CHNL_MAP_SIZE           HCI_AFH_CHANNEL_MAP_LEN

/** Define scan types */
#define BTM_SCAN_TYPE_STANDARD          0
#define BTM_SCAN_TYPE_INTERLACED        1

/** Inquiry results mode */
#define BTM_INQ_RESULT                  0
#define BTM_INQ_RESULT_WITH_RSSI        1
#define BTM_INQ_RESULT_EXTENDED         2

#define BTM_INQ_RES_IGNORE_RSSI         0x7f                                   /**< RSSI value not supplied (ignore it) */
#define BTM_SCAN_PARAM_IGNORE           0                                      /**< Passed to BTM_SetScanConfig() to ignore */

/**
 * State of the remote name retrieval during inquiry operations.
 * Used in the wiced_bt_dev_inq_info_t structure, and returned in the
 * BTM_InqDbRead, BTM_InqDbFirst, and BTM_InqDbNext functions.
 * The name field is valid when the state returned is
 * BTM_INQ_RMT_NAME_DONE.
 */
#define BTM_INQ_RMT_NAME_EMPTY          0
#define BTM_INQ_RMT_NAME_PENDING        1
#define BTM_INQ_RMT_NAME_DONE           2
#define BTM_INQ_RMT_NAME_FAILED         3

/** Determines the number of uint32_t's necessary for services */
#define BTM_EIR_MAX_SERVICES            46                                     /**< max number of UUID_SERVCLASS's defined in sdpdefs.h */
#define BTM_EIR_ARRAY_BITS              32                                     /**< Number of bits in each array element */
#define BTM_EIR_SERVICE_ARRAY_SIZE      (((uint32_t)BTM_EIR_MAX_SERVICES / BTM_EIR_ARRAY_BITS) + \
                                        (((uint32_t)BTM_EIR_MAX_SERVICES % BTM_EIR_ARRAY_BITS) ? 1 : 0))

/** PIN types */
#define BTM_PIN_TYPE_VARIABLE           HCI_PIN_TYPE_VARIABLE
#define BTM_PIN_TYPE_FIXED              HCI_PIN_TYPE_FIXED

/** Size of security keys */
#define BTM_SECURITY_KEY_DATA_LEN       132                                    /**< Security key data length (used by wiced_bt_device_link_keys_t structure) */
#define BTM_SECURITY_LOCAL_KEY_DATA_LEN 65                                     /**< Local security key data length (used by wiced_bt_local_identity_keys_t structure) */

/** \} group_dev_macros */


/******************************************************************************
 * Global Enumerations definitions                                            *
 ******************************************************************************/

/**
* \addtogroup group_dev_enums
* \{
*/

/** Link Quality statistics action type */
enum wiced_bt_dev_link_quality_stats_param_e
{
    WICED_CLEAR_LINK_QUALITY_STATS,                                            /**< clear link quality stats */
    WICED_READ_LINK_QUALITY_STATS,                                             /**< read link quality stats */
    WICED_READ_THEN_CLEAR_LINK_QUALITY_STATS,                                  /**< read then clear link quality stats */
};
typedef uint8_t wiced_bt_link_quality_stats_param_t;                           /**< Link Quality Statistic Action (see #wiced_bt_dev_link_quality_stats_param_e) */

/** HCI trace types  */
typedef enum
{
    HCI_TRACE_EVENT,                                                           /**< HCI event data from controller to the host */
    HCI_TRACE_COMMAND,                                                         /**< HCI command data from host to controller */
    HCI_TRACE_INCOMING_ACL_DATA,                                               /**< HCI incoming acl data */
    HCI_TRACE_OUTGOING_ACL_DATA                                                /**< HCI outgoing acl data */
} wiced_bt_hci_trace_type_t;

/** Power Management status codes */
enum wiced_bt_dev_power_mgmt_status_e
{
    WICED_POWER_STATE_ACTIVE          = HCI_MODE_ACTIVE,                       /**< Active */
    WICED_POWER_STATE_SNIFF           = HCI_MODE_SNIFF,                        /**< Sniff */
    WICED_POWER_STATE_SSR             = WICED_POWER_STATE_SNIFF + 2,           /**< Sniff subrating notification */
    WICED_POWER_STATE_PENDING,                                                 /**< Pending (waiting for status from controller) */
    WICED_POWER_STATE_ERROR,                                                   /**< Error (controller returned error) */
};
typedef uint8_t wiced_bt_dev_power_mgmt_status_t;                              /**< Power management status (see #wiced_bt_dev_power_mgmt_status_e) */

/** BR/EDR Discoverable modes */
enum wiced_bt_discoverability_mode_e
{
    BTM_NON_DISCOVERABLE              = 0,                                     /**< Non discoverable */
    BTM_LIMITED_DISCOVERABLE          = 1,                                     /**< Limited BR/EDR discoverable */
    BTM_GENERAL_DISCOVERABLE          = 2                                      /**< General BR/EDR discoverable */
};
#define BTM_DISCOVERABLE_MASK           (BTM_LIMITED_DISCOVERABLE|BTM_GENERAL_DISCOVERABLE)
#define BTM_MAX_DISCOVERABLE            BTM_GENERAL_DISCOVERABLE

/** BR/EDR Connectable modes */
enum wiced_bt_connectability_mode_e
{
    BTM_NON_CONNECTABLE               = 0,                                     /**< Not connectable */
    BTM_CONNECTABLE                   = 1                                      /**< BR/EDR connectable */
};
#define BTM_CONNECTABLE_MASK        (BTM_NON_CONNECTABLE | BTM_CONNECTABLE)

/** Inquiry modes NOTE These modes are associated with the inquiry active values (BTM_*ACTIVE). */
enum wiced_bt_inquiry_mode_e
{
    BTM_INQUIRY_NONE                  = 0,                                     /**< Stop inquiry */
    BTM_GENERAL_INQUIRY               = 0x01,                                  /**< General inquiry */
    BTM_LIMITED_INQUIRY               = 0x02,                                  /**< Limited inquiry */
    BTM_BR_INQUIRY_MASK               = (BTM_GENERAL_INQUIRY | BTM_LIMITED_INQUIRY)
};

/** Inquiry Filter Condition types (see wiced_bt_dev_inq_parms_t) */
enum wiced_bt_dev_filter_cond_e
{
    BTM_CLR_INQUIRY_FILTER            = 0,                                     /**< No inquiry filter */
    BTM_FILTER_COND_DEVICE_CLASS      = HCI_FILTER_COND_DEVICE_CLASS,          /**< Filter on device class */
    BTM_FILTER_COND_BD_ADDR           = HCI_FILTER_COND_BD_ADDR,               /**< Filter on device addr */
};

/** Type of OOB data required  */
enum wiced_bt_dev_oob_data_req_type_e
{
    BTM_OOB_INVALID_TYPE,
    BTM_OOB_PEER,                                                              /**< Peer OOB data requested */
    BTM_OOB_LOCAL,                                                             /**< Local OOB data requested */
    BTM_OOB_BOTH                                                               /**< Both local and peer OOB data requested */
};
typedef UINT8 wiced_bt_dev_oob_data_req_type_t;                                /**< OOB data type requested (see #wiced_bt_dev_oob_data_req_type_t) */

/** SMP Pairing status codes */
#ifndef SMP_ERROR_CODES
#define SMP_ERROR_CODES
enum wiced_bt_smp_status_e
{
    SMP_SUCCESS                 = 0,                    /**< Success */
    SMP_PASSKEY_ENTRY_FAIL      = 0x01,                 /**< Passkey entry failed */
    SMP_OOB_FAIL                = 0x02,                 /**< OOB failed */
    SMP_PAIR_AUTH_FAIL          = 0x03,                 /**< Authentication failed */
    SMP_CONFIRM_VALUE_ERR       = 0x04,                 /**< Value confirmation failed */
    SMP_PAIR_NOT_SUPPORT        = 0x05,                 /**< Not supported */
    SMP_ENC_KEY_SIZE            = 0x06,                 /**< Encryption key size failure */
    SMP_INVALID_CMD             = 0x07,                 /**< Invalid command */
    SMP_PAIR_FAIL_UNKNOWN       = 0x08,                 /**< Unknown failure */
    SMP_REPEATED_ATTEMPTS       = 0x09,                 /**< Repeated attempts */
    SMP_INVALID_PARAMETERS      = 0x0A,                 /**< Invalid parameters  */
    SMP_DHKEY_CHK_FAIL          = 0x0B,                 /**< DH Key check failed */
    SMP_NUMERIC_COMPAR_FAIL     = 0x0C,                 /**< Numeric comparison failed */
    SMP_BR_PAIRING_IN_PROGR     = 0x0D,                 /**< BR paIring in progress */
    SMP_XTRANS_DERIVE_NOT_ALLOW = 0x0E,                 /**< Cross transport key derivation not allowed */
    SMP_MAX_FAIL_RSN_PER_SPEC   = SMP_XTRANS_DERIVE_NOT_ALLOW,

    /* bte smp status codes */
    SMP_PAIR_INTERNAL_ERR       = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x01), /**< Internal error */
    SMP_UNKNOWN_IO_CAP          = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x02), /**< unknown IO capability, unable to decide associatino model */
    SMP_INIT_FAIL               = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x03), /**< Initialization failed */
    SMP_CONFIRM_FAIL            = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x04), /**< Confirmation failed */
    SMP_BUSY                    = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x05), /**< Busy */
    SMP_ENC_FAIL                = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x06), /**< Encryption failed */
    SMP_STARTED                 = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x07), /**< Started */
    SMP_RSP_TIMEOUT             = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x08), /**< Response timeout */
    SMP_FAIL                    = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x09), /**< Generic failure */
    SMP_CONN_TOUT               = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x0A), /**< Connection timeout */
};
#endif
typedef uint8_t wiced_bt_smp_status_t;                                         /**< SMP Pairing status (see #wiced_bt_smp_status_e) */

/** Security Service Levels [bit mask]. Encryption should not be used without authentication. */
enum wiced_bt_sec_level_e
{
    BTM_SEC_NONE                      = 0x0000,                                /**< Nothing required */
    BTM_SEC_IN_AUTHENTICATE           = 0x0002,                                /**< Inbound call requires authentication */
    BTM_SEC_OUT_AUTHENTICATE          = 0x0010,                                /**< Outbound call requires authentication */
    BTM_SEC_ENCRYPT                   = 0x0024,                                /**< Requires encryption (inbound and outbound) */
    BTM_SEC_SECURE_CONNECTION         = 0x0040                                 /**< Secure Connections Mode (P-256 based Secure Simple Pairing and Authentication) */
};

/** Security flags for current BR/EDR link */
enum wiced_bt_sec_flags_e
{
    BTM_SEC_LINK_ENCRYPTED            = 0x01,                                  /**< Link encrypted */
    BTM_SEC_LINK_PAIRED_WITHOUT_MITM  = 0x02,                                  /**< Paired without man-in-the-middle protection */
    BTM_SEC_LINK_PAIRED_WITH_MITM     = 0x04                                   /**< Link with man-in-the-middle protection */
};

/** Pairing IO Capabilities */
enum wiced_bt_dev_io_cap_e
{
    BTM_IO_CAPABILITIES_DISPLAY_ONLY,                                          /**< Display Only */
    BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT,                              /**< Display Yes/No */
    BTM_IO_CAPABILITIES_KEYBOARD_ONLY,                                         /**< Keyboard Only */
    BTM_IO_CAPABILITIES_NONE,                                                  /**< No Input, No Output */
    BTM_IO_CAPABILITIES_BLE_DISPLAY_AND_KEYBOARD_INPUT,                        /**< Keyboard display (For BLE SMP) */
    BTM_IO_CAPABILITIES_MAX
};
typedef uint8_t wiced_bt_dev_io_cap_t;                                         /**< IO capabilities (see #wiced_bt_dev_io_cap_e) */

/** BR/EDR Authentication requirement */
enum wiced_bt_dev_auth_req_e
{
    BTM_AUTH_SINGLE_PROFILE_NO,                                                /**< MITM Protection Not Required - Single Profile/non-bonding. Numeric comparison with automatic accept allowed */
    BTM_AUTH_SINGLE_PROFILE_YES,                                               /**< MITM Protection Required - Single Profile/non-bonding. Use IO Capabilities to determine authentication procedure */
    BTM_AUTH_ALL_PROFILES_NO,                                                  /**< MITM Protection Not Required - All Profiles/dedicated bonding. Numeric comparison with automatic accept allowed */
    BTM_AUTH_ALL_PROFILES_YES,                                                 /**< MITM Protection Required - All Profiles/dedicated bonding. Use IO Capabilities to determine authentication procedure */
    BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO,                                /**< MITM Protection Not Required - Single Profiles/general bonding. Numeric comparison with automatic accept allowed */
    BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_YES,                               /**< MITM Protection Required - Single Profiles/general bonding. Use IO Capabilities to determine authentication procedure */
};
typedef uint8_t wiced_bt_dev_auth_req_t;                                       /**< BR/EDR authentication requirement (see #wiced_bt_dev_auth_req_e) */

/** LE Authentication requirement */
#ifndef BTM_BLE_AUTH_REQ_TYPES
#define BTM_BLE_AUTH_REQ_TYPES
enum wiced_bt_dev_le_auth_req_e
{
    BTM_LE_AUTH_REQ_NO_BOND =       0x00,                                               /**< Not required - No Bond */
    BTM_LE_AUTH_REQ_BOND =          0x01,                                               /**< Required - General Bond */
    BTM_LE_AUTH_REQ_MITM =          0x04,                                               /**< MITM required - Auth Y/N*/
    BTM_LE_AUTH_REQ_SC_ONLY =       0x08,                                               /**< LE Secure Connection, no MITM, no Bonding */
    BTM_LE_AUTH_REQ_SC_BOND =       (BTM_LE_AUTH_REQ_SC_ONLY|BTM_LE_AUTH_REQ_BOND),     /**< LE Secure Connection, no MITM, Bonding */
    BTM_LE_AUTH_REQ_SC_MITM =       (BTM_LE_AUTH_REQ_SC_ONLY|BTM_LE_AUTH_REQ_MITM),     /**< LE Secure Connection, MITM, no Bonding */
    BTM_LE_AUTH_REQ_SC_MITM_BOND =  (BTM_LE_AUTH_REQ_SC_ONLY|BTM_LE_AUTH_REQ_MITM|BTM_LE_AUTH_REQ_BOND),    /**< LE Secure Connection, MITM, Bonding */
    BTM_LE_AUTH_REQ_MASK =          0x1D
};
#endif
typedef uint8_t wiced_bt_dev_le_auth_req_t;                                    /**< BLE authentication requirement (see #wiced_bt_dev_le_auth_req_e) */

/** OOB Data status */
enum wiced_bt_dev_oob_data_e
{
    BTM_OOB_NONE,                                                              /**< No OOB data */
    BTM_OOB_PRESENT_192,                                                       /**< OOB data present (from the P-192 public key) */
    BTM_OOB_PRESENT_256,                                                       /**< OOB data present (from the P-256 public key) */
    BTM_OOB_PRESENT_192_256,                                                   /**< OOB data present (from the P-192 and P-256 public keys) */
    BTM_OOB_UNKNOWN                                                            /**< OOB data unknown */
};
typedef uint8_t wiced_bt_dev_oob_data_t;                                       /**< OOB data (see #wiced_bt_dev_oob_data_e) */

/** Pairing keypress types */
enum wiced_bt_dev_passkey_entry_type_e
{
    BTM_PASSKEY_ENTRY_STARTED,                                                 /**< Passkey entry started */
    BTM_PASSKEY_DIGIT_ENTERED,                                                 /**< Passkey digit entered */
    BTM_PASSKEY_DIGIT_ERASED,                                                  /**< Passkey digit erased */
    BTM_PASSKEY_DIGIT_CLEARED,                                                 /**< Passkey cleared */
    BTM_PASSKEY_ENTRY_COMPLETED                                                /**< Passkey entry completed */
};
typedef uint8_t wiced_bt_dev_passkey_entry_type_t;                             /**< Bluetooth pairing keypress value (see #wiced_bt_dev_passkey_entry_type_e) */

/** LE Key type */
enum wiced_bt_dev_le_key_type_e
{
    BTM_LE_KEY_PENC                   = (1 << 0),                              /**< Encryption information of peer device */
    BTM_LE_KEY_PID                    = (1 << 1),                              /**< Identity key of the peer device */
    BTM_LE_KEY_PCSRK                  = (1 << 2),                              /**< Peer SRK */
#if SMP_LE_SC_INCLUDED == TRUE
    BTM_LE_KEY_PLK                    = (1 << 3),
    BTM_LE_KEY_LENC                   = (1 << 4),                              /**< Master role security information:div */
    BTM_LE_KEY_LID                    = (1 << 5),                              /**< Master device ID key */
    BTM_LE_KEY_LCSRK                  = (1 << 6),                              /**< Local CSRK has been deliver to peer */
    BTM_LE_KEY_LLK                    = (1 << 7),
#else
    BTM_LE_KEY_LENC                   = (1 << 3),                              /**< Master role security information:div */
    BTM_LE_KEY_LID                    = (1 << 4),                              /**< Master device ID key */
    BTM_LE_KEY_LCSRK                  = (1 << 5)                               /**< Local CSRK has been deliver to peer */
#endif
};
typedef uint8_t wiced_bt_dev_le_key_type_t;                                    /**< LE key type (see #wiced_bt_dev_le_key_type_e) */

/** Scan mode used in initiating */
enum wiced_bt_ble_conn_mode_e
{
    BLE_CONN_MODE_OFF,                                                         /**< Stop initiating */
    BLE_CONN_MODE_LOW_DUTY,                                                    /**< Slow connection scan parameter */
    BLE_CONN_MODE_HIGH_DUTY                                                    /**< Fast connection scan parameter */
};
typedef uint8_t wiced_bt_ble_conn_mode_t;                                      /**< Conn mode \ref wiced_bt_ble_conn_mode_e */

/** advertisement type (used when calling \ref wiced_bt_start_advertisements) */
enum wiced_bt_ble_advert_mode_e
{
    BTM_BLE_ADVERT_OFF,                                                        /**< Stop advertising */
    BTM_BLE_ADVERT_DIRECTED_HIGH,                                              /**< Directed advertisement (high duty cycle) */
    BTM_BLE_ADVERT_DIRECTED_LOW,                                               /**< Directed advertisement (low duty cycle) */
    BTM_BLE_ADVERT_UNDIRECTED_HIGH,                                            /**< Undirected advertisement (high duty cycle) */
    BTM_BLE_ADVERT_UNDIRECTED_LOW,                                             /**< Undirected advertisement (low duty cycle) */
    BTM_BLE_ADVERT_NONCONN_HIGH,                                               /**< Non-connectable advertisement (high duty cycle) */
    BTM_BLE_ADVERT_NONCONN_LOW,                                                /**< Non-connectable advertisement (low duty cycle) */
    BTM_BLE_ADVERT_DISCOVERABLE_HIGH,                                          /**< discoverable advertisement (high duty cycle) */
    BTM_BLE_ADVERT_DISCOVERABLE_LOW                                            /**< discoverable advertisement (low duty cycle) */
};
typedef uint8_t wiced_bt_ble_advert_mode_t;                                    /**< Advertisement type \ref wiced_bt_ble_advert_mode_e */

/** Scan duty cycle (used for BTM_BLE_SCAN_STATE_CHANGED_EVT and wiced_bt_dev_create_connection) */
enum wiced_bt_ble_scan_type_e
{
    BTM_BLE_SCAN_TYPE_NONE,                                                    /**< Stop scanning */
    BTM_BLE_SCAN_TYPE_HIGH_DUTY,                                               /**< High duty cycle scan */
    BTM_BLE_SCAN_TYPE_LOW_DUTY                                                 /**< Low duty cycle scan */
};
typedef uint8_t wiced_bt_ble_scan_type_t;                                      /**< scan type (see #wiced_bt_ble_scan_type_e) */

/** Bluetooth Management events */
enum wiced_bt_management_evt_e
{
    /** Bluetooth status events */
    BTM_ENABLED_EVT,                                                           /**< Bluetooth controller and host stack enabled. Event data: wiced_bt_dev_enabled_t */
    BTM_DISABLED_EVT,                                                          /**< Bluetooth controller and host stack disabled. Event data: NULL */
    BTM_POWER_MANAGEMENT_STATUS_EVT,                                           /**< Power management status change. Event data: wiced_bt_power_mgmt_notification_t */

    /** Security events */
    BTM_PIN_REQUEST_EVT,                                                       /**< PIN request (used only with legacy devices). Event data: #wiced_bt_dev_name_and_class_t */
    BTM_USER_CONFIRMATION_REQUEST_EVT,                                         /**< received USER_CONFIRMATION_REQUEST event (respond using #wiced_bt_dev_confirm_req_reply). Event data: #wiced_bt_dev_user_cfm_req_t */
    BTM_PASSKEY_NOTIFICATION_EVT,                                              /**< received USER_PASSKEY_NOTIFY event. Event data: #wiced_bt_dev_user_key_notif_t */
    BTM_PASSKEY_REQUEST_EVT,                                                   /**< received USER_PASSKEY_REQUEST event (respond using #wiced_bt_dev_pass_key_req_reply). Event data: #wiced_bt_dev_user_key_req_t */
    BTM_KEYPRESS_NOTIFICATION_EVT,                                             /**< received KEYPRESS_NOTIFY event. Event data: #wiced_bt_dev_user_keypress_t */
    BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT,                            /**< Requesting IO capabilities for BR/EDR pairing. Event data: #wiced_bt_dev_bredr_io_caps_req_t */
    BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT,                           /**< Received IO capabilities response for BR/EDR pairing. Event data: #wiced_bt_dev_bredr_io_caps_rsp_t */
    BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT,                               /**< Requesting IO capabilities for BLE pairing. Slave can check peer io capabilities in event data before updating with local io capabilities. Event data: #wiced_bt_dev_ble_io_caps_req_t */
    BTM_PAIRING_COMPLETE_EVT,                                                  /**< received SIMPLE_PAIRING_COMPLETE event. Event data: #wiced_bt_dev_pairing_cplt_t */
    BTM_ENCRYPTION_STATUS_EVT,                                                 /**< Encryption status change. Event data: #wiced_bt_dev_encryption_status_t */
    BTM_SECURITY_REQUEST_EVT,                                                  /**< Security request (respond using #wiced_bt_ble_security_grant). Event data: #wiced_bt_dev_security_request_t */
    BTM_SECURITY_FAILED_EVT,                                                   /**< Security procedure/authentication failed. Event data: #wiced_bt_dev_security_failed_t */
    BTM_SECURITY_ABORTED_EVT,                                                  /**< Security procedure aborted locally, or unexpected link drop. Event data: #wiced_bt_dev_name_and_class_t */

    BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT,                                      /**< Result of reading local OOB data (wiced_bt_dev_read_local_oob_data). Event data: #wiced_bt_dev_local_oob_t */
    BTM_REMOTE_OOB_DATA_REQUEST_EVT,                                           /**< OOB data from remote device (respond using #wiced_bt_dev_remote_oob_data_reply). Event data: #wiced_bt_dev_remote_oob_t */

    BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT,                                    /**< Updated remote device link keys (store device_link_keys to  NV memory). This is the place to
verify that the correct link key has been generated. Event data: #wiced_bt_device_link_keys_t */
    BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT,                                   /**< Request for stored remote device link keys (restore device_link_keys from NV memory). If successful, return WICED_BT_SUCCESS. Event data: #wiced_bt_device_link_keys_t */

    BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT,                                        /**< Update local identity key (stored local_identity_keys NV memory). Event data: #wiced_bt_local_identity_keys_t */
    BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT,                                       /**< Request local identity key (get local_identity_keys from NV memory). If successful, return WICED_BT_SUCCESS. Event data: #wiced_bt_local_identity_keys_t */

    BTM_BLE_SCAN_STATE_CHANGED_EVT,                                            /**< BLE scan state change. Event data: #wiced_bt_ble_scan_type_t */
    BTM_BLE_ADVERT_STATE_CHANGED_EVT,                                          /**< BLE advertisement state change. Event data: #wiced_bt_ble_advert_mode_t */

    /** BLE Secure Connection events */
    BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT,                                       /**< SMP remote oob data request. Reply using wiced_bt_smp_oob_data_reply. Event data: wiced_bt_smp_remote_oob_req_t  */
    BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT,                                    /**< LE secure connection remote oob data request. Reply using wiced_bt_smp_sc_oob_reply. Event data: #wiced_bt_smp_sc_remote_oob_req_t */
    BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT,                                /**< LE secure connection local OOB data (wiced_bt_smp_create_local_sc_oob_data). Event data: #wiced_bt_smp_sc_local_oob_t*/

    BTM_SCO_CONNECTED_EVT,                                                     /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
    BTM_SCO_DISCONNECTED_EVT,                                                  /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
    BTM_SCO_CONNECTION_REQUEST_EVT,                                            /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
    BTM_SCO_CONNECTION_CHANGE_EVT,                                             /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
    BTM_BLE_CONNECTION_PARAM_UPDATE,                                           /**< BLE connection parameter update. Event data: #wiced_bt_ble_connection_param_update_t */
    BTM_BLE_PHY_UPDATE_EVT                                                     /**< BLE Physical link update. Event data: wiced_bt_ble_phy_update_t */
};
typedef uint8_t wiced_bt_management_evt_t;                                     /**< Bluetooth management events (see #wiced_bt_management_evt_e) */

/** \} group_dev_enums */


/******************************************************************************
 * Global Data Structure definitions                                          *
 ******************************************************************************/

/**
* \addtogroup group_dev_data_structures
* \{
*/

typedef uint8_t wiced_bt_br_chnl_map_t[BTM_AFH_CHNL_MAP_SIZE];
typedef uint8_t wiced_bt_sco_type_t;                                           /**< BTM_LINK_TYPE_SCO or BTM_LINK_TYPE_ESCO */

/** Result/Status for wiced_bt_dev */
typedef wiced_result_t  wiced_bt_dev_status_t;                                 /**< Result/Status for wiced_bt_dev */

/** Device enabled (used by BTM_ENABLED_EVT) */
typedef struct
{
    wiced_result_t                      status;                                /**< Status */
} wiced_bt_dev_enabled_t;

/** Device disabled (used by BTM_DISABLED_EVT) */
typedef struct
{
    uint8_t                             reason;                                /**< Reason for BTM Disable */
} wiced_bt_dev_disabled_t;

/** Structure returned with Vendor Specific Command complete callback */
typedef struct
{
    uint16_t                            opcode;                                /**< Vendor specific command opcode */
    uint16_t                            param_len;                             /**< Return parameter length        */
    uint8_t                             *p_param_buf;                          /**< Return parameter buffer        */
} wiced_bt_dev_vendor_specific_command_complete_params_t;

/** RSSI Result (in response to #wiced_bt_dev_read_rssi) */
typedef struct
{
    wiced_result_t                      status;                                /**< Status of the operation */
    uint8_t                             hci_status;                            /**< Status from controller */
    int8_t                              rssi;                                  /**< RSSI */
    wiced_bt_device_address_t           rem_bda;                               /**< Remote BD address */
} wiced_bt_dev_rssi_result_t;

/** TX Power Result (in response to #wiced_bt_dev_read_tx_power) */
typedef struct
{
    wiced_result_t                      status;                                /**< Status of the operation */
    uint8_t                             hci_status;                            /**< Status from controller */
    int8_t                              tx_power;                              /**< TX power */
    wiced_bt_device_address_t           rem_bda;                               /**< Remote BD address */
} wiced_bt_tx_power_result_t;

#pragma pack(1)
/** BR/EDR packet types detail statistics */
typedef PACKED union
{
    uint16_t                            array[10];
    PACKED struct
    {
        uint16_t                        null_count;                            /**< No.of NULL packets received/transmitted */
        uint16_t                        pool_count;                            /**< No.of POLL packets received/transmitted */
        uint16_t                        dm1_count;                             /**< No.of DM1 packets received/transmitted */
        uint16_t                        dh1_count;                             /**< No.of DH1 packets received/transmitted */
        uint16_t                        dv_count;                              /**< No.of DV packets received/transmitted */
        uint16_t                        aux1_count;                            /**< No.of AUX1 packets received/transmitted */
        uint16_t                        dm3_count;                             /**< No.of DM3 packets received/transmitted */
        uint16_t                        dh3_count;                             /**< No.of DH3 packets received/transmitted */
        uint16_t                        dm5_count;                             /**< No.of DM5 packets received/transmitted */
        uint16_t                        dh5_count;                             /**< No.of DH5 packets received/transmitted */
    } br_packet_types;
    PACKED struct
    {
        uint16_t                        null_count;                            /**< No.of NULL packets received/transmitted */
        uint16_t                        pool_count;                            /**< No.of POLL packets received/transmitted */
        uint16_t                        dm1_count;                             /**< No.of DM1 packets received/transmitted */
        uint16_t                        _2_dh1_count;                          /**< No.of 2DH1 packets received/transmitted */
        uint16_t                        _3_dh1_count;                          /**< No.of 3DH1 packets received/transmitted */
        uint16_t                        _2_dh3_count;                          /**< No.of 2DH3 packets received/transmitted */
        uint16_t                        _3_dh3_count;                          /**< No.of 3DH3 packets received/transmitted */
        uint16_t                        _2_dh5_count;                          /**< No.of 2DH5 packets received/transmitted */
        uint16_t                        _3_dh5_count;                          /**< No.of 3DH5 packets received/transmitted */
        uint16_t                        not_used;                              /**< Not Used */
    } edr_packet_types;
} wiced_bt_br_edr_pkt_type_stats;
#pragma pack()

#pragma pack(1)
/** BR/EDR link statistics */
typedef PACKED struct
{
    wiced_bt_br_edr_pkt_type_stats      rxPkts;                                /**< Received packets details */
    wiced_bt_br_edr_pkt_type_stats      txPkts;                                /**< Transmitted packets details */
    uint32_t                            rxAclBytes;                            /**< Total Received ACL bytes */
    uint32_t                            txAclBytes;                            /**< Total Transmitted ACL bytes */
    uint16_t                            hecErrs;                               /**< hecErrs packet count */
    uint16_t                            crcErrs;                               /**< crcErrs packet count */
    uint16_t                            seqnRepeat;                            /**< seqnRepeat packet count */
} wiced_bt_lq_br_edr_stats;
#pragma pack()

#pragma pack(1)
/** LE link statistics */
typedef PACKED struct
{
    uint32_t                            tx_pkt_cnt;                            /**< Transmit packet count */
    uint32_t                            tx_acked_cnt;                          /**< Transmit packet acknowledged count */
    uint32_t                            rx_good_pkt_cnt;                       /**< Received good packet count */
    uint32_t                            rx_good_bytes;                         /**< Received good byte count */
    uint32_t                            rx_all_pkt_sync_to;                    /**< All received packet sync timeout count */
    uint32_t                            rx_all_pkt_crc_err;                    /**< All received packet crc error count */
} wiced_bt_lq_le_stats;
#pragma pack()

#pragma pack(1)
/** LQ Quality Result (in response to wiced_bt_dev_lq_stats) */
typedef PACKED struct
{
    uint8_t                             status;                                /**< eEvent status */
    uint16_t                            conn_handle;                           /**< Connection handle of link quality stats */
    uint8_t                             action;                                /**< see wiced_bt_dev_link_quality_stats_param for options */
    PACKED union
    {
        wiced_bt_lq_br_edr_stats        br_edr_stats;                          /**< br edr statistics */
        wiced_bt_lq_le_stats            le_stats;                              /**< le statistics */
    } wiced_bt_lq_stats;
} wiced_bt_lq_stats_result_t;
#pragma pack()

/** SCO connected event related data */
typedef struct
{
    uint16_t                            sco_index;                             /**< SCO index */
} wiced_bt_sco_connected_t;

/** SCO disconnected event related data */
typedef struct
{
    uint16_t                            sco_index;                             /**< SCO index */
} wiced_bt_sco_disconnected_t;

/**  SCO connect request event related data */
typedef struct
{
    uint16_t                            sco_index;                             /**< SCO index */
    wiced_bt_device_address_t           bd_addr;                               /**< Peer bd address */
    wiced_bt_dev_class_t                dev_class;                             /**< Peer device class */
    wiced_bt_sco_type_t                 link_type;                             /**< SCO link type */
} wiced_bt_sco_connection_request_t;

/** SCO connection change event related data */
typedef struct
{
    uint16_t                            sco_index;                             /**< SCO index */
    uint16_t                            rx_pkt_len;                            /**< RX packet length */
    uint16_t                            tx_pkt_len;                            /**< TX packet length */
    wiced_bt_device_address_t           bd_addr;                               /**< Peer bd address */
    uint8_t                             hci_status;                            /**< HCI status */
    uint8_t                             tx_interval;                           /**< TX interval */
    uint8_t                             retrans_windows;                       /**< Retransmission windows */
} wiced_bt_sco_connection_change_t;

/** Class of Device inquiry filter */
typedef struct
{
    wiced_bt_dev_class_t                dev_class;                             /**< class of device */
    wiced_bt_dev_class_t                dev_class_mask;                        /**< class of device filter mask */
} wiced_bt_dev_cod_cond_t;

/** Inquiry filter */
typedef union
{
    wiced_bt_device_address_t           bdaddr_cond;                           /**< bluetooth address filter */
    wiced_bt_dev_cod_cond_t             cod_cond;                              /**< class of device filter */
} wiced_bt_dev_inq_filt_cond_t;

/** Inquiry Parameters */
typedef struct
{
    uint8_t                             mode;                                  /**< Inquiry mode (see #wiced_bt_inquiry_mode_e) */
    uint8_t                             duration;                              /**< Inquiry duration (1.28 sec increments) */
    uint8_t                             filter_cond_type;                      /**< Inquiry filter type  (see #wiced_bt_dev_filter_cond_e) */
    wiced_bt_dev_inq_filt_cond_t        filter_cond;                           /**< Inquiry filter */
} wiced_bt_dev_inq_parms_t;

/** Inquiry Results */
typedef struct
{
    uint16_t                            clock_offset;                          /**< Clock offset */
    wiced_bt_device_address_t           remote_bd_addr;                        /**< Device address */
    wiced_bt_dev_class_t                dev_class;                             /**< Class of device */
    uint8_t                             page_scan_rep_mode;                    /**< Page scan repetition mode */
    uint8_t                             page_scan_per_mode;                    /**< Page scan per mode */
    uint8_t                             page_scan_mode;                        /**< Page scan mode */
    int8_t                              rssi;                                  /**< Receive signal strength index (#BTM_INQ_RES_IGNORE_RSSI, if not available) */
    uint32_t                            eir_uuid[BTM_EIR_SERVICE_ARRAY_SIZE];  /**< Array or EIR UUIDs */
    wiced_bool_t                        eir_complete_list;                     /**< TRUE if EIR array is complete */
} wiced_bt_dev_inquiry_scan_result_t;

/** Structure returned with remote name  request */
typedef struct
{
    uint16_t                            status;                                /**< Status of the operation */
    wiced_bt_device_address_t           bd_addr;                               /**< Remote BD address */
    uint16_t                            length;                                /**< Device name Length */
    wiced_bt_remote_name_t              remote_bd_name;                        /**< Remote device name */
} wiced_bt_dev_remote_name_result_t;

/* Structure returned with switch role request */
typedef struct
{
    uint8_t                         status;             /**< Status of the operation */
    uint8_t                         role;               /**< BTM_ROLE_MASTER or BTM_ROLE_SLAVE */
    wiced_bt_device_address_t       bd_addr;            /**< Remote BD address involved with the switch */
} wiced_bt_dev_switch_role_result_t;

/** Remote device information (used by BTM_PIN_REQUEST_EVT, BTM_SECURITY_ABORTED_EVT) */
typedef struct
{
    wiced_bt_device_address_t           *bd_addr;                              /**< BD Address of remote */
    wiced_bt_dev_class_t                *dev_class;                            /**< peer class of device   */
    uint8_t                             *bd_name;                              /**< BD Name of remote */
} wiced_bt_dev_name_and_class_t;

/** Change in power management status  (used by BTM_POWER_MANAGEMENT_STATUS_EVT notication) */
typedef struct
{
    uint8_t                             *bd_addr;                              /**< BD Address of remote */
    wiced_bt_dev_power_mgmt_status_t    status;                                /**< PM status */
    uint16_t                            value;                                 /**< Additional mode data */
    uint8_t                             hci_status;                            /**< HCI status */
} wiced_bt_power_mgmt_notification_t;

/** bonding device information from wiced_bt_dev_get_bonded_devices */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< Peer address */
    wiced_bt_ble_address_type_t         addr_type;                             /**< Peer address type : BLE_ADDR_PUBLIC/BLE_ADDR_RANDOM */
    wiced_bt_device_type_t              device_type;                           /**< Peer device type : BT_DEVICE_TYPE_BREDR/BT_DEVICE_TYPE_BLE/BT_DEVICE_TYPE_BREDR_BLE  */
} wiced_bt_dev_bonded_device_info_t;

/** data type for BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< Peer address */
} wiced_bt_smp_remote_oob_req_t;

/** data type for BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< Peer address */
    wiced_bt_dev_oob_data_req_type_t    oob_type;                              /**< Requested oob data types (BTM_OOB_PEER, BTM_OOB_LOCAL, or BTM_OOB_BOTH) */
} wiced_bt_smp_sc_remote_oob_req_t;

/** Public key */
typedef struct
{
    BT_OCTET32                          x;
    BT_OCTET32                          y;
} wiced_bt_public_key_t;

/** Data for BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT */
typedef struct
{
    wiced_bool_t                        present;                               /**< TRUE if local oob is present */
    BT_OCTET16                          randomizer;                            /**< Randomizer */
    BT_OCTET16                          commitment;                            /**< Commitment */
    wiced_bt_ble_address_t              addr_sent_to;                          /**< Peer address sent to */
    BT_OCTET32                          private_key_used;                      /**< Private key */
    wiced_bt_public_key_t               public_key_used;                       /**< Public key */
} wiced_bt_smp_sc_local_oob_t;

/** Data type for IO capabalities response (BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< Peer address */
    wiced_bt_dev_io_cap_t               io_cap;                                /**< Peer IO capabilities */
    wiced_bt_dev_oob_data_t             oob_data;                              /**< OOB data present at peer device for the local device */
    wiced_bt_dev_auth_req_t             auth_req;                              /**< Authentication required for peer device */
} wiced_bt_dev_bredr_io_caps_rsp_t;

/** Data for pairing confirmation request (BTM_USER_CONFIRMATION_REQUEST_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< Peer address */
    uint32_t                            numeric_value;                         /**< Numeric value for comparison (if "just_works", do not show this number to UI) */
    wiced_bool_t                        just_works;                            /**< TRUE, if using "just works" association model */
    wiced_bt_dev_auth_req_t             local_authentication_requirements;     /**< Authentication requirement for local device */
    wiced_bt_dev_auth_req_t             remote_authentication_requirements;    /**< Authentication requirement for peer device */
} wiced_bt_dev_user_cfm_req_t;

/** Pairing user passkey request  (BTM_USER_PASSKEY_REQUEST_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< Peer address */
} wiced_bt_dev_user_key_req_t;

/** Data for pairing passkey notification (BTM_USER_PASSKEY_NOTIFICATION_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< Peer address */
    uint32_t                            passkey;                               /**< Passkey */
} wiced_bt_dev_user_key_notif_t;

/** Pairing keypress notification (BTM_USER_KEYPRESS_NOTIFICATION_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< peer address */
    wiced_bt_dev_passkey_entry_type_t   keypress_type;                         /**< type of keypress */
} wiced_bt_dev_user_keypress_t;

/** BR/EDR pairing complete infomation */
typedef struct
{
    uint8_t         status;                                                    /**< The status of the simple pairing process (see defintions for HCI status codes). */
} wiced_bt_dev_br_edr_pairing_info_t;

/** BLE pairing complete infomation */
typedef struct
{
    wiced_result_t                      status;                                /**< Status of the simple pairing process   */
    uint8_t                             reason;                                /**< Failure reason (see #wiced_bt_smp_status_t) */
    uint8_t                             sec_level;                             /**< 0 - None, 1- Unauthenticated Key, 4-Authenticated Key  */
    wiced_bool_t                        is_pair_cancel;                        /**< True if cancelled, else False   */
    wiced_bt_device_address_t           resolved_bd_addr;                      /**< Resolved address (if remote device using private address) */
    wiced_bt_ble_address_type_t         resolved_bd_addr_type;                 /**< Resolved addr type of bonded device */
} wiced_bt_dev_ble_pairing_info_t;

/** Transport dependent pairing complete infomation */
typedef union
{
    wiced_bt_dev_br_edr_pairing_info_t  br_edr;                                /**< BR/EDR pairing complete infomation */
    wiced_bt_dev_ble_pairing_info_t     ble;                                   /**< BLE pairing complete infomation */
} wiced_bt_dev_pairing_info_t;

/** Pairing complete notification (BTM_PAIRING_COMPLETE_EVT event data type) */
typedef struct
{
    uint8_t                             *bd_addr;                              /**< Peer address           */
    wiced_bt_transport_t                transport;                             /**< BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE */
    wiced_bt_dev_pairing_info_t         pairing_complete_info;                 /**< Transport dependent pairing complete infomation */
    wiced_result_t                      bonding_status;                        /**< Current status of bonding process to notify app of completion status of storing keys */
} wiced_bt_dev_pairing_cplt_t;

/** Security/authentication failure status  (used by BTM_SECURITY_FAILED_EVT notication). */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< [in]  Peer address */
    wiced_result_t                      status;                                /**< Status of the operation */
    uint8_t                             hci_status;                            /**< Status from controller */
} wiced_bt_dev_security_failed_t;

/** Security request (BTM_SECURITY_REQUEST_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< Peer address */
} wiced_bt_dev_security_request_t;

/** Local OOB data BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT */
typedef struct
{
    wiced_result_t                      status;                                /**< Status */
    wiced_bool_t                        is_extended_oob_data;                  /**< TRUE if extended OOB data */
    BT_OCTET16                          c_192;                                 /**< Simple Pairing Hash C derived from the P-192 public key */
    BT_OCTET16                          r_192;                                 /**< Simple Pairing Randomnizer R associated with the P-192 public key */
    BT_OCTET16                          c_256;                                 /**< Simple Pairing Hash C derived from the P-256 public key (valid only if is_extended_oob_data=TRUE) */
    BT_OCTET16                          r_256;                                 /**< Simple Pairing Randomnizer R associated with the P-256 public key (valid only if is_extended_oob_data=TRUE) */
} wiced_bt_dev_local_oob_t;

/** BTM_REMOTE_OOB_DATA_REQUEST_EVT */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< BD Address of remote */
    wiced_bool_t                        extended_oob_data;                     /**< TRUE if requesting extended OOB (P-256) */
} wiced_bt_dev_remote_oob_t;

/** BR/EDR Pairing IO Capabilities (to be filled by application callback on BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< [in] BD Address of remote */
    wiced_bt_dev_io_cap_t               local_io_cap;                          /**< local IO capabilities (to be filled by application callback) */
    wiced_bt_dev_oob_data_t             oob_data;                              /**< OOB data present at peer device for the local device */
    wiced_bt_dev_auth_req_t             auth_req;                              /**< Authentication required for peer device */
    wiced_bool_t                        is_orig;                               /**< TRUE, if local device initiated the pairing process */
} wiced_bt_dev_bredr_io_caps_req_t;

/** BLE Pairing IO Capabilities (to be filled by application callback on BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< [in] BD Address of remote */
    wiced_bt_dev_io_cap_t               local_io_cap;                          /**< local IO capabilities (to be filled by application callback) */
    uint8_t                             oob_data;                              /**< OOB data present (locally) for the peer device */
    wiced_bt_dev_le_auth_req_t          auth_req;                              /**< Authentication request (for local device) contain bonding and MITM info */
    uint8_t                             max_key_size;                          /**< Max encryption key size */
    wiced_bt_dev_le_key_type_t          init_keys;                             /**< Keys to be distributed, bit mask */
    wiced_bt_dev_le_key_type_t          resp_keys;                             /**< keys to be distributed, bit mask */
} wiced_bt_dev_ble_io_caps_req_t;

#pragma pack(1)
typedef PACKED struct
{
    BT_OCTET16                          irk;                                   /**< Peer diverified identity root */
#if SMP_INCLUDED == TRUE && SMP_LE_SC_INCLUDED == TRUE
    BT_OCTET16                          pltk;                                  /**< Peer long term key */
    BT_OCTET16                          pcsrk;                                 /**< Peer SRK peer device used to secured sign local data */

    BT_OCTET16                          lltk;                                  /**< Local long term key */
    BT_OCTET16                          lcsrk;                                 /**< Local SRK peer device used to secured sign local data */
#else
    BT_OCTET16                          ltk;                                   /**< Peer long term key */
    BT_OCTET16                          csrk;                                  /**< Peer SRK peer device used to secured sign local data */
#endif

    BT_OCTET8                           rand;                                  /**< Random vector for LTK generation */
    UINT16                              ediv;                                  /**< LTK diversifier of this slave device */
    UINT16                              div;                                   /**< Local DIV  to generate local LTK=d1(ER,DIV,0) and CSRK=d1(ER,DIV,1) */
    uint8_t                             sec_level;                             /**< Local pairing security level */
    uint8_t                             key_size;                              /**< Key size of the LTK delivered to peer device */
    uint8_t                             srk_sec_level;                         /**< Security property of peer SRK for this device */
    uint8_t                             local_csrk_sec_level;                  /**< security property of local CSRK for this device */

    UINT32                              counter;                               /**< Peer sign counter for verifying rcv signed cmd */
    UINT32                              local_counter;                         /**< Local sign counter for sending signed write cmd*/
} wiced_bt_ble_keys_t;
#pragma pack()

#pragma pack(1)
typedef PACKED struct
{
    /** BR/EDR key */
    uint8_t                             br_edr_key_type;                       /**<  BR/EDR Link Key type */
    wiced_bt_link_key_t                 br_edr_key;                            /**<  BR/EDR Link Key */

    /** LE Keys */
    wiced_bt_dev_le_key_type_t          le_keys_available_mask;                /**<  Mask of available BLE keys */
    wiced_bt_ble_address_type_t         ble_addr_type;                         /**<  LE device type: public or random address */
    wiced_bt_ble_address_type_t         static_addr_type;                      /**<  Static address type */
    wiced_bt_device_address_t           static_addr;                           /**<  Static address */
    wiced_bt_ble_keys_t                 le_keys;                               /**<  LE keys */
} wiced_bt_device_sec_keys_t;
#pragma pack()

/** Paired device link key notification (used by BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT notication) */
#pragma pack(1)
typedef PACKED struct
{
    wiced_bt_device_address_t           bd_addr;                               /**< [in] BD Address of remote */
    wiced_bt_device_sec_keys_t          key_data;                              /**< [in/out] Key data */
} wiced_bt_device_link_keys_t;
#pragma pack()

/** Encryption status change (used by BTM_ENCRYPTION_STATUS_EVT) */
typedef struct
{
    uint8_t                             *bd_addr;                              /**< BD Address of remote */
    wiced_bt_transport_t                transport;                             /**< BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE */
    void                                *p_ref_data;                           /**< Optional data passed in by wiced_bt_dev_set_encryption */
    wiced_result_t                      result;                                /**< Result of the operation */
} wiced_bt_dev_encryption_status_t;

/** LE identity key for local device (used by BTM_LE_LOCAL_IDENTITY_KEYS_UPDATE_EVT and BTM_LE_LOCAL_KEYS_REQUEST_EVT notification) */
typedef struct
{
    uint8_t                             local_key_data[BTM_SECURITY_LOCAL_KEY_DATA_LEN];    /**< [in/out] Local security key */
} wiced_bt_local_identity_keys_t;

/** BLE connection parameter update event related data */
typedef struct
{
    uint8_t                             status;                                /**< connection parameters update status */
    wiced_bt_device_address_t           bd_addr;                               /**< peer bd address */
    uint16_t                            conn_interval;                         /**< updated connection interval */
    uint16_t                            conn_latency;                          /**< updated connection latency */
    uint16_t                            supervision_timeout;                   /**< updated supervision timeout */
} wiced_bt_ble_connection_param_update_t;

/** BLE Physical link update event related data */
typedef struct
{
    uint8_t                              status;                               /**< LE Phy update status */
    wiced_bt_device_address_t            bd_address;                           /**< Peer BD address*/
    uint8_t                              tx_phy;                               /**< Transmitter PHY, values: 1=1M, 2=2M, 3=LE coded */
    uint8_t                              rx_phy;                               /**< Receiver PHY, values: 1=1M, 2=2M, 3=LE coded */
} wiced_bt_ble_phy_update_t;


/** Structure definitions for Bluetooth Management (wiced_bt_management_cback_t) event notifications */
typedef union
{
    /** Bluetooth status event data types*/
    wiced_bt_dev_enabled_t                  enabled;                                  /**< Data for BTM_ENABLED_EVT */
    wiced_bt_dev_disabled_t                 disabled;                                 /**< Data for BTM_DISABLED_EVT */
    wiced_bt_power_mgmt_notification_t      power_mgmt_notification;                  /**< Data for BTM_POWER_MANAGEMENT_STATUS_EVT */

    /** Security event data types */
    wiced_bt_dev_name_and_class_t           pin_request;                              /**< Data for BTM_PIN_REQUEST_EVT */
    wiced_bt_dev_user_cfm_req_t             user_confirmation_request;                /**< Data for BTM_USER_CONFIRMATION_REQUEST_EVT */
    wiced_bt_dev_user_key_notif_t           user_passkey_notification;                /**< Data for BTM_USER_PASSKEY_NOTIFICATION_EVT */
    wiced_bt_dev_user_key_req_t             user_passkey_request;                     /**< Data for BTM_USER_PASSKEY_REQUEST_EVT */
    wiced_bt_dev_user_keypress_t            user_keypress_notification;               /**< Data for BTM_USER_KEYPRESS_NOTIFICATION_EVT - See #wiced_bt_dev_user_keypress_t */
    wiced_bt_dev_bredr_io_caps_req_t        pairing_io_capabilities_br_edr_request;   /**< Data for BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT */
    wiced_bt_dev_bredr_io_caps_rsp_t        pairing_io_capabilities_br_edr_response;  /**< Data for BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT */
    wiced_bt_dev_ble_io_caps_req_t          pairing_io_capabilities_ble_request;      /**< Data for BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT */
    wiced_bt_dev_pairing_cplt_t             pairing_complete;                         /**< Data for BTM_PAIRING_COMPLETE_EVT */
    wiced_bt_dev_encryption_status_t        encryption_status;                        /**< Data for BTM_ENCRYPTION_STATUS_EVT */
    wiced_bt_dev_security_request_t         security_request;                         /**< Data for BTM_SECURITY_REQUEST_EVT */
    wiced_bt_dev_security_failed_t          security_failed;                          /**< Data for BTM_SECURITY_FAILED_EVT See #wiced_bt_dev_security_failed_t */
    wiced_bt_dev_name_and_class_t           security_aborted;                         /**< Data for BTM_SECURITY_ABORTED_EVT */

    wiced_bt_dev_local_oob_t                read_local_oob_data_complete;             /**< Data for BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT */
    wiced_bt_dev_remote_oob_t               remote_oob_data_request;                  /**< Data for BTM_REMOTE_OOB_DATA_REQUEST_EVT */

    wiced_bt_device_link_keys_t             paired_device_link_keys_update;           /**< Data for BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT */
    wiced_bt_device_link_keys_t             paired_device_link_keys_request;          /**< Data for BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT */
    wiced_bt_local_identity_keys_t          local_identity_keys_update;               /**< Data for BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT */
    wiced_bt_local_identity_keys_t          local_identity_keys_request;              /**< Data for BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT */

    wiced_bt_ble_scan_type_t                ble_scan_state_changed;                   /**< Data for BTM_BLE_SCAN_STATE_CHANGED_EVT */
    wiced_bt_ble_advert_mode_t              ble_advert_state_changed;                 /**< Data for BTM_BLE_ADVERT_STATE_CHANGED_EVT */

    wiced_bt_smp_remote_oob_req_t           smp_remote_oob_data_request;              /**< Data for BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT */
    wiced_bt_smp_sc_remote_oob_req_t        smp_sc_remote_oob_data_request;           /**< Data for BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT */
    wiced_bt_smp_sc_local_oob_t             *p_smp_sc_local_oob_data;                 /**< Data for BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT */

    wiced_bt_sco_connected_t                sco_connected;                            /**< Data for BTM_SCO_CONNECTED_EVT */
    wiced_bt_sco_disconnected_t             sco_disconnected;                         /**< Data for BTM_SCO_DISCONNECTED_EVT */
    wiced_bt_sco_connection_request_t       sco_connection_request;                   /**< Data for BTM_SCO_CONNECTION_REQUEST_EVT */
    wiced_bt_sco_connection_change_t        sco_connection_change;                    /**< Data for BTM_SCO_CONNECTION_CHANGE_EVT */
    wiced_bt_ble_connection_param_update_t  ble_connection_param_update;              /**< Data for BTM_BLE_CONNECTION_PARAM_UPDATE */
    wiced_bt_ble_phy_update_t               ble_phy_update_event;                     /**< Data for BTM_BLE_PHY_UPDATE_EVT */
} wiced_bt_management_evt_data_t;

/******************************************************************************
* Bluetooth Management callback
*
***************************************************************************//**
* Callback for Bluetooth Management event notifications.
* Registered using wiced_bt_stack_init()
*
* @param event             : Event ID
* @param p_event_data      : Event data
*
* @return Status of event handling
*
******************************************************************************/
typedef wiced_result_t (wiced_bt_management_cback_t) (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/******************************************************************************
* Callback Name: wiced_bt_connection_status_change_cback_t
***************************************************************************//**
*
* Callback for Bluetooth Management event notifications.
* Registered using \ref wiced_bt_register_connection_status_change
*
* \param[in] bd_addr       BD Address of remote
* \param[in] p_features
* \param[in] is_connected  TRUE if connected
* \param[in] handle        Connection handle
* \param[in] transport     BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
* \param[in] reason        status for ACL connection change
*  - HCI_SUCCESS
*  - HCI_ERR_PAGE_TIMEOUT
*  - HCI_ERR_MEMORY_FULL
*  - HCI_ERR_CONNECTION_TOUT
*  - HCI_ERR_PEER_USER
*  - HCI_ERR_CONN_CAUSE_LOCAL_HOST
*  - HCI_ERR_LMP_RESPONSE_TIMEOUT
*  - HCI_ERR_CONN_FAILED_ESTABLISHMENT
*
* \return void
*
******************************************************************************/
typedef void (wiced_bt_connection_status_change_cback_t) (wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason);

/******************************************************************************
* Callback Name: wiced_bt_dev_cmpl_cback_t
***************************************************************************//**
*
* Asynchronous operation complete callback.
*
* \param[in] p_data                                    Operation dependent data
*
* \return void
*
******************************************************************************/
typedef void (wiced_bt_dev_cmpl_cback_t) (void *p_data);

/******************************************************************************
* Callback Name: wiced_bt_dev_vendor_specific_command_complete_cback_t
***************************************************************************//**
*
* Vendor specific command complete
*
* \param[in] p_command_complete_params              Command complete parameters
*
* \return void
*
******************************************************************************/
typedef void (wiced_bt_dev_vendor_specific_command_complete_cback_t) (wiced_bt_dev_vendor_specific_command_complete_params_t *p_command_complete_params);

/******************************************************************************
* Callback Name: wiced_bt_hci_trace_cback_t
***************************************************************************//**
*
* Callback for HCI traces registered using \ref wiced_bt_dev_register_hci_trace
*
* \param[in] type                                      HCI packet type
* \param[in] length                                    Length of the trace data
* \param[in] p_data                                    Pointer to the data
*
* \return void
*
******************************************************************************/
typedef void ( wiced_bt_hci_trace_cback_t )( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );

/******************************************************************************
* Callback Name: wiced_bt_inquiry_result_cback_t
***************************************************************************//**
*
* Inquiry result callback.
*
* \param p_inquiry_result     Inquiry result data (NULL if inquiry is complete)
* \param p_eir_data           Extended inquiry response data
*
* \return void
*
******************************************************************************/
typedef void (wiced_bt_inquiry_result_cback_t) (wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data);

/******************************************************************************
* Callback Name: wiced_bt_remote_name_cback_t
***************************************************************************//**
*
* Remote name result callback.
*
* \param p_remote_name_result                           Remote name result data
*
* \return void
*
******************************************************************************/
typedef void (wiced_bt_remote_name_cback_t) (wiced_bt_dev_remote_name_result_t *p_remote_name_result);

/** \} group_dev_structures */


/******************************************************************************
 * Global functions                                                           *
 ******************************************************************************/

/**
* \addtogroup group_dev_functions
* \{
*/

/**
* \addtogroup group_dev_functions_ctrl
* \{
*/

/******************************************************************************
* Function Name: wiced_bt_dev_read_local_addr
***************************************************************************//**
*
* Reads the local device address.
*
* \param[out] bd_addr             Pointer to local BD_ADDR filled upon API call
*
* \return void
*
******************************************************************************/
void wiced_bt_dev_read_local_addr( wiced_bt_device_address_t bd_addr );

/******************************************************************************
* Function Name: wiced_bt_dev_vendor_specific_command
***************************************************************************//**
*
* Sends a vendor specific HCI command to the controller.
*
* \param[in] opcode                           Opcode of vendor specific command
* \param[in] param_len                        Length of parameter buffer
* \param[in] p_param_buf                      Parameters
* \param[in] p_cback                          Callback for command complete
*
* \return
*  - WICED_BT_SUCCESS Command sent. (command complete callback param is NULL)
*  - WICED_BT_PENDING Command sent. Waiting for command complete event.
*  - WICED_BT_BUSY Command not sent. Waiting for command complete event for prior command.
*
******************************************************************************/
wiced_result_t wiced_bt_dev_vendor_specific_command( uint16_t opcode,
                                                     uint8_t param_len,
                                                     uint8_t *p_param_buf,
                                                     wiced_bt_dev_vendor_specific_command_complete_cback_t *p_cback );

/******************************************************************************
* Function Name: wiced_bt_dev_register_connection_status_change
***************************************************************************//**
*
* Registers a callback for connection status change.
*
* \param[in] p_wiced_bt_connection_status_change_cback
*
* \return wiced_result_t
*  - WICED_BT_SUCCESS : on success;
*  - WICED_BT_FAILED : if an error occurred
*
******************************************************************************/
wiced_result_t wiced_bt_dev_register_connection_status_change( wiced_bt_connection_status_change_cback_t *p_wiced_bt_connection_status_change_cback );

/******************************************************************************
* Function Name: wiced_bt_dev_read_rssi
***************************************************************************//**
*
* Gets the Receive Signal Strength Index (RSSI) for the requested link.
*
* \param[in] remote_bda          BD_ADDR of connection to read RSSI
* \param[in] transport           BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
* \param[in] p_cback             Callback to receive wiced_bt_dev_rssi_result_t
*
* \return
*  - WICED_BT_PENDING if command issued to controller.
*  - WICED_BT_NO_RESOURCES if couldn't allocate memory to issue command
*  - WICED_BT_UNKNOWN_ADDR if no active link with bd addr specified
*  - WICED_BT_BUSY if command is already in progress
*
******************************************************************************/
wiced_result_t wiced_bt_dev_read_rssi( wiced_bt_device_address_t remote_bda,
                                       wiced_bt_transport_t transport,
                                       wiced_bt_dev_cmpl_cback_t *p_cback );

/******************************************************************************
* Function Name: wiced_bt_dev_read_tx_power
***************************************************************************//**
*
* Reads the transmit power for the requested link.
*
* \param[in] remote_bda  BD address of connection to read TX power
* \param[in] transport   BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
* \param[in] p_cback     Callback, which will receive wiced_bt_tx_power_result_t
*
* \return
*  - WICED_BT_PENDING if command issued to controller.
*  - WICED_BT_NO_RESOURCES if couldn't allocate memory to issue command
*  - WICED_BT_UNKNOWN_ADDR if no active link with bd addr specified
*  - WICED_BT_BUSY if command is already in progress.
*
******************************************************************************/
wiced_result_t wiced_bt_dev_read_tx_power( wiced_bt_device_address_t remote_bda,
                                           wiced_bt_transport_t transport,
                                           wiced_bt_dev_cmpl_cback_t *p_cback );

/******************************************************************************
* Function Name: wiced_bt_dev_register_hci_trace
***************************************************************************//**
*
* Registers to get the HCI traces.
*
* \param[in] p_cback                                    Callback for HCI traces
*
* \return void
*
******************************************************************************/
void wiced_bt_dev_register_hci_trace( wiced_bt_hci_trace_cback_t* p_cback );

/******************************************************************************
* Function Name: wiced_bt_set_local_bdaddr
***************************************************************************//**
*
* Set Local Bluetooth Device Address. If the application passes BLE_ADDR_RANDOM
* as an address type, the stack will setup a static random address.  For the
* static random address, the top two bits of the bd_addr should be set.
* The stack will enforce that.
*
* \param[in] bd_addr                                         new BD_ADDR to set
* \param[in] addr_type
*  - BLE_ADDR_RANDOM (only for BLE, not BR/EDR or dual mode)
*  - BLE_ADDR_PUBLIC
*
* \return void
*
******************************************************************************/
void wiced_bt_set_local_bdaddr( wiced_bt_device_address_t bda, wiced_bt_ble_address_type_t addr_type );

/******************************************************************************
* Function Name: wiced_bt_dev_get_role
***************************************************************************//**
*
* Gets the role of the local device for the ACL connection with the specified
* remote device.
*
* \param[in] remote_bd_addr             BD address of remote device
* \param[in] transport                  BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
* \param[out] p_role                    Role of the local device
*
* \return
*  - WICED_BT_UNKNOWN_ADDR if no active link with BD_ADDR specified
*
******************************************************************************/
wiced_result_t wiced_bt_dev_get_role( wiced_bt_device_address_t remote_bd_addr, UINT8 *p_role, wiced_bt_transport_t transport );

/**
 * Function         wiced_bt_dev_switch_role
 *
 *                  This function is called to switch the role between master and
 *                  slave.  If role is already set it will do nothing. If the
 *                  command was initiated, the callback function is called upon
 *                  completion.
 *
 * @param[in]       remote_bd_addr      : BD address of remote device
 * @param[in]       new_role            : New role (BTM_ROLE_MASTER or BTM_ROLE_SLAVE)
 * @param[in]       p_cback             : Result callback (wiced_bt_dev_switch_role_result_t will be passed to the callback)

 *
 * @return      wiced_result_t
 *              WICED_BT_SUCCESS if already in specified role.
 *              WICED_BT_CMD_STARTED if command issued to controller.
 *              WICED_BT_NO_RESOURCES if couldn't allocate memory to issue command
 *              WICED_BT_UNKNOWN_ADDR if no active link with bd addr specified
 *              WICED_BT_MODE_UNSUPPORTED if local device doesn't support role switching
 */
wiced_result_t BTM_SwitchRole(wiced_bt_device_address_t remote_bd_addr, uint8_t new_role, wiced_bt_dev_cmpl_cback_t *p_cb);
#define wiced_bt_dev_switch_role BTM_SwitchRole

/**
 * Function     wiced_bt_dev_set_link_supervision_timeout
 *
 *              set Link Supervision Timeout
 *
 * @param[in]   remote_bd_addr   : BD address of remote device
 * @param[in]   timeout          :
 *
 * @return      WICED_BT_PENDING if successfully initiated, otherwise error.
 *
 */
wiced_result_t BTM_SetLinkSuperTout(wiced_bt_device_address_t remote_bd_addr, uint16_t timeout);
#define wiced_bt_dev_set_link_supervision_timeout BTM_SetLinkSuperTout

/******************************************************************************
* Function Name: wiced_bt_set_tx_power
***************************************************************************//**
*
* Commands to set the TX power on link
*
* \param[in] bd_addr         peer address to set ADV TX power keep bd_addr NULL
* \param[in] power           power value in db
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_set_tx_power ( wiced_bt_device_address_t bd_addr, INT8 power );

/******************************************************************************
* Function Name: wiced_bt_dev_set_tx_power_range
***************************************************************************//**
*
* Command to set the TX power range on a link
*
* \param[in] bd_addr         peer address of the link
* \param[in] max_power       Max power value in db (Must be <= 4)
* \param[in] min_power       Min power value in db (Must be >= -24)
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_dev_set_tx_power_range ( wiced_bt_device_address_t bd_addr , INT8 max_power, INT8 min_power );

/******************************************************************************
* Function Name: wiced_bt_dev_link_quality_stats
***************************************************************************//**
*
* Gets the statistics for an ACL link.
*
* \param[in] bda         Bluetooth device address of desired link quality statistics
* \param[in] transport   BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
* \param[in] action
*  - WICED_CLEAR_LINK_QUALITY_STATS = reset the link quality statistics to 0
*  - WICED_READ_LINK_QUALITY_STATS = read link quality statistics
*  - WICED_READ_THEN_CLEAR_LINK_QUALITY_STATS = read link quality stats, then clear it
* \param[in] p_cback     Result callback
*
* \return
*  - WICED_BT_SUCCESS:        If successful
*  - WICED_BT_PENDING:        If command succesfully sent down
*  - WICED_BT_BUSY            If already in progress
*  - WICED_BT_NO_RESORCES:    If no memory/buffers available to sent down to controller
*  - WICED_BT_UNKNOWN_ADDR:   If given BD_ADDRESS is invalid
*
* \note
* Callback function argument is the pointer of type wiced_bt_lq_stats_result_t.
*
* \note
* This API works when there is only one ACL connection.
*
******************************************************************************/
wiced_result_t wiced_bt_dev_link_quality_stats( BD_ADDR bda,
                                                wiced_bt_transport_t transport,
                                                uint8_t action,
                                                wiced_bt_dev_cmpl_cback_t *p_cback );

/******************************************************************************
* Function Name: wiced_bt_coex_enable
***************************************************************************//**
*
* The application can invoke this function to enable the coex functionality.
*
* \param[in]       seci_baud_rate - SECI baud rate. Ensure to set a valid baud
*                                   rate which will be used for the SECI communication
*                                   between BT and WLAN chip. The maximum supported
*                                   value is up to 4M.
*
* \return          wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_coex_enable( uint32_t seci_baud_rate );

/******************************************************************************
* Function Name: wiced_bt_coex_disable
***************************************************************************//**
*
* The application can invoke this function to disable the coex functionality.
*
* \return void
*
******************************************************************************/
void wiced_bt_coex_disable( void );

/** \} group_dev_functions_ctrl */


/**
* \addtogroup group_dev_functions_classic
* \{
*/

/******************************************************************************
* Function Name: wiced_bt_start_inquiry
***************************************************************************//**
*
* Starts BR/EDR inquiry.
*
* \param[in] p_inqparms                           \ref wiced_bt_dev_inq_parms_t
* \param[in] p_inquiry_result_cback               inquiry results callback
*
* \return
*  - WICED_BT_PENDING if successfully initiated
*  - WICED_BT_BUSY if already in progress
*  - WICED_BT_ILLEGAL_VALUE if parameter(s) are out of range
*  - WICED_BT_NO_RESOURCES if could not allocate resources to start the command
*  - WICED_BT_WRONG_MODE if the device is not up.
*
******************************************************************************/
wiced_result_t wiced_bt_start_inquiry( wiced_bt_dev_inq_parms_t *p_inqparms, wiced_bt_inquiry_result_cback_t *p_inquiry_result_cback );

/******************************************************************************
* Function Name: wiced_bt_cancel_inquiry
***************************************************************************//**
*
* Cancels an active inquiry.
*
* \param void
*
* \return
*  - WICED_BT_SUCCESS if successful
*  - WICED_BT_NO_RESOURCES if could not allocate a message buffer
*  - WICED_BT_WRONG_MODE if the device is not up.
*
******************************************************************************/
wiced_result_t wiced_bt_cancel_inquiry( void );

/******************************************************************************
* Function Name: wiced_bt_dev_set_advanced_connection_params
***************************************************************************//**
*
* Sets advanced connection parameters for subsequent BR/EDR connections (remote
* clock offset, page scan mode,  and other information obtained during inquiry).
*
* If not called, then default connection parameters are used.
*
* \param[in] p_inquiry_scan_result          \ref wiced_bt_inquiry_result_cback_t
*
* \return
*  - WICED_BT_SUCCESS on success;
*  - WICED_BT_FAILED if an error occurred
*
******************************************************************************/
wiced_result_t wiced_bt_dev_set_advanced_connection_params( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_scan_result );

/******************************************************************************
* Function Name:  wiced_bt_dev_set_discoverability
***************************************************************************//**
*
* Sets inquiry scan.
*
* \note
* The duration must be less than or equal to the interval.
*
* \param[in] inq_mode       \ref wiced_bt_discoverability_mode_e
* \param[in] duration       Scan Duration (in 0.625 msec increments)
*                           Range: <b>0x0012 ~ 0x1000 </b> (11.25 ~ 2560 msecs)
* \param[in] interval       Scan Interval (in 0.625 msec intervals)
*                           Range: <b>0x0012 ~ 0x1000 </b> (11.25 ~ 2560 msecs)
*
* \return
*  - WICED_BT_SUCCESS        if successful
*  - WICED_BT_BUSY:          if a setting of the filter is already in progress
*  - WICED_BT_NO_RESOURCES   if couldn't get a memory pool buffer
*  - WICED_BT_ILLEGAL_VALUE  if a bad parameter was detected
*  - WICED_BT_WRONG_MODE:    if the device is not up
*
******************************************************************************/
wiced_result_t  wiced_bt_dev_set_discoverability( uint8_t inq_mode, uint16_t duration, uint16_t interval );

/******************************************************************************
* Function Name: wiced_bt_dev_set_connectability
***************************************************************************//**
*
* Sets page scan mode settings for BR/EDR.
*
* \note
* The duration (window parameter) must be less than or equal to the interval.
*
* \param[in] page_mode      \ref wiced_bt_connectability_mode_e
* \param[in] window         Scan Window (in 0.625 msec increments)
*                           Range: <b>0x0012 ~ 0x1000 </b> (11.25 ~ 2560 msecs)
* \param[in] interval       Scan Interval (in 0.625 msec intervals)
*                           Range: <b>0x0012 ~ 0x1000 </b> (11.25 ~ 2560 msecs)
*
* \return
*  - WICED_BT_SUCCESS:        if successful
*  - WICED_BT_ILLEGAL_VALUE:  if a bad parameter is detected
*  - WICED_BT_NO_RESOURCES:   if could notallocate a message buffer
*  - WICED_BT_WRONG_MODE:     if the device is not up.
*
******************************************************************************/
wiced_result_t wiced_bt_dev_set_connectability( uint8_t page_mode, uint16_t window, uint16_t interval );

/******************************************************************************
* Function Name: wiced_bt_dev_set_sniff_mode
***************************************************************************//**
*
* Sets a connection into Sniff mode.
*
* \param[in] remote_bda          Link for which to put into sniff mode
* \param[in] min_period          Minimum sniff period
* \param[in] max_period          Maximum sniff period
* \param[in] attempt             Number of attempts for switching to sniff mode
* \param[in] timeout             Timeout for attempting to switch to sniff mode
*
* \return WICED_BT_PENDING if successfully initiated, otherwise error
*
******************************************************************************/
wiced_result_t wiced_bt_dev_set_sniff_mode( wiced_bt_device_address_t remote_bda,
                                            uint16_t min_period,
                                            uint16_t max_period,
                                            uint16_t attempt,
                                            uint16_t timeout );

/******************************************************************************
* Function Name: wiced_bt_dev_cancel_sniff_mode
***************************************************************************//**
*
* Takes a connection out of Sniff mode. Checks if the connection is already in
* Sniff mode, and if not, the cancel Sniff mode is ignored.
*
* \param[in] remote_bda                         Peer BD_ADDR to cancel sniff to
*
* \return WICED_BT_PENDING if successfully initiated, otherwise - error.
*
******************************************************************************/
wiced_result_t wiced_bt_dev_cancel_sniff_mode( wiced_bt_device_address_t remote_bda );

/******************************************************************************
* Function Name: wiced_bt_dev_set_sniff_subrating
***************************************************************************//**
*
* Sets sniff-subrating parameters for an active connection.
*
* \param[in] remote_bda          Device address of desired ACL connection
* \param[in] max_latency         Maximum latency (in 0.625ms units)
*                                (range: 0x0002-0xFFFE)
* \param[in] min_remote_timeout  Minimum remote timeout
* \param[in] min_local_timeout   Minimum local timeout
*
* \return
*  - WICED_BT_SUCCESS on success
*  - WICED_BT_ILLEGAL_ACTION if an error occurred.
*
******************************************************************************/
wiced_result_t wiced_bt_dev_set_sniff_subrating( wiced_bt_device_address_t remote_bda,
                                                 uint16_t max_latency,
                                                 uint16_t min_remote_timeout,
                                                 uint16_t min_local_timeout );

/******************************************************************************
* Function Name: wiced_bt_dev_write_eir
***************************************************************************//**
*
* Writes EIR data to the controller.
*
* \param[in] p_buff                                         Pointer to EIR data
* \param[in] len                                            Length of EIR data
*
* \return
*  - WICED_BT_SUCCESS if successful
*  - WICED_BT_NO_RESOURCES if couldn't allocate memory to issue command
*  - WICED_BT_UNSUPPORTED if local device cannot support request
*
******************************************************************************/
wiced_result_t wiced_bt_dev_write_eir( uint8_t *p_buff, uint16_t len );

/******************************************************************************
* Function Name: wiced_bt_dev_set_afh_channel_classification
***************************************************************************//**
*
* Sends the HCI_SET_AFH_CHANNELS command to the BR/EDR controller.
*
* \note
*  - Channel n is bad = 0.
*  - Channel n is unknown = 1.
*  - The most significant bit is reserved and shall be set to 0.
*  - At least 20 channels shall be marked as unknown.
*
* \param[in] afh_channel_map              AFH Host Channel Classification array
*
* \return
*  - WICED_BT_UNSUPPORTED if the feature is not supported.
*  - WICED_BT_WRONG_MODE if the device is in the wrong mode.
*  - WICED_BT_NO_RESOURCES if the device does not have buffers to process the request.
*
******************************************************************************/
wiced_result_t wiced_bt_dev_set_afh_channel_classification( const wiced_bt_br_chnl_map_t afh_channel_map );

/******************************************************************************
* Function Name: wiced_bt_dev_get_remote_name
***************************************************************************//**
*
* Gets the BT Friendly name from the remote device.
*
* \param[in] bd_addr                                Peer BD address
* \param[in] p_remote_name_result_cback             remote name result callback
*
* \return
*  - WICED_BT_PENDING if successfully initiated
*  - WICED_BT_BUSY if already in progress
*  - WICED_BT_ILLEGAL_VALUE if parameter(s) are out of range
*  - WICED_BT_NO_RESOURCES if could not allocate resources to start the command
*  - WICED_BT_WRONG_MODE if the device is not up.
*
******************************************************************************/
wiced_result_t  wiced_bt_dev_get_remote_name( wiced_bt_device_address_t bd_addr,
                                              wiced_bt_remote_name_cback_t *p_remote_name_result_cback );

/** \} group_dev_functions_classic */


/**
* \addtogroup group_dev_functions_sec
* \{
*/

/******************************************************************************
* Function Name: wiced_bt_dev_pin_code_reply
***************************************************************************//**
*
* PIN code reply used in response to \ref BTM_PIN_REQUEST_EVT in \ref
* wiced_bt_management_cback_t.
*
* \param[in] bd_addr        Address of the device for which PIN was requested
* \param[in] res            result of the operation WICED_BT_SUCCESS if success
* \param[in] pin_len        length in bytes of the PIN Code
* \param[in] p_pin          pointer to array with the PIN Code
*
* \return void
*
******************************************************************************/
void wiced_bt_dev_pin_code_reply( wiced_bt_device_address_t bd_addr, wiced_result_t res, uint8_t pin_len, uint8_t *p_pin );

/******************************************************************************
* Function Name: wiced_bt_dev_sec_bond
***************************************************************************//**
*
* Bond with peer device. If the connection is already up, but not secure,
* pairing is attempted.
*
* \note
* PIN parameters are only needed when bonding with legacy devices
* (pre-2.1 Core Spec)
*
* \param[in] bd_addr         Peer device BD address to pair with.
* \param[in] bd_addr_type    BLE_ADDR_PUBLIC or BLE_ADDR_RANDOM
* \param[in] transport       BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
* \param[in] pin_len         Length of input parameter p_pin (0 if not used)
* \param[in] p_pin           Pointer to Pin Code to use (NULL if not used)
*
* \return
*  - WICED_BT_PENDING if successfully initiated,
*  - WICED_BT_SUCCESS if already paired to the device, else
*  - WICED_BT_ERROR
*
******************************************************************************/
wiced_result_t wiced_bt_dev_sec_bond( wiced_bt_device_address_t bd_addr,
                                      wiced_bt_ble_address_type_t bd_addr_type,
                                      wiced_bt_transport_t transport,
                                      uint8_t pin_len,
                                      uint8_t *p_pin );

/******************************************************************************
* Function Name: wiced_bt_dev_sec_bond_cancel
***************************************************************************//**
*
* Cancels an ongoing bonding process with the peer device.
*
* \param[in] bd_addr                        Peer device BD address to pair with
*
* \return
*  - WICED_BT_PENDING if cancel initiated,
*  - WICED_BT_SUCCESS if cancel has completed already, else error code.
*
******************************************************************************/
wiced_result_t wiced_bt_dev_sec_bond_cancel( wiced_bt_device_address_t bd_addr );

/******************************************************************************
* Function Name: wiced_bt_dev_set_encryption
***************************************************************************//**
*
* Encrypts the specified connection. Status is notified using
* <b>BTM_ENCRYPTION_STATUS_EVT </b> of \ref wiced_bt_management_cback_t.
*
* \param[in] bd_addr           Address of peer device
* \param[in] transport         BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
* \param[in] p_ref_data        Encryption type #wiced_bt_ble_sec_action_type_t
*
* \return
*  - WICED_BT_SUCCESS already encrypted
*  - WICED_BT_PENDING command will be returned in the callback
*  - WICED_BT_WRONG_MODE connection not up.
*  - WICED_BT_BUSY security procedures are currently active
*
******************************************************************************/
wiced_result_t wiced_bt_dev_set_encryption( wiced_bt_device_address_t bd_addr,
                                            wiced_bt_transport_t transport,
                                            void *p_ref_data );

/******************************************************************************
* Function Name: wiced_bt_dev_confirm_req_reply
***************************************************************************//**
* Confirms the numeric value for pairing to <b>BTM_USER_CONFIRMATION_REQUEST_EVT </b>
* of #wiced_bt_management_cback_t)
*
* \param[in] res            result of the operation WICED_BT_SUCCESS if success
* \param[in] bd_addr        Address of the peer device
*
* \return void
*
******************************************************************************/
void wiced_bt_dev_confirm_req_reply( wiced_result_t res, wiced_bt_device_address_t bd_addr );

/******************************************************************************
* Function Name: wiced_bt_dev_pass_key_req_reply
***************************************************************************//**
*
* Provides the pairing passkey. Used in response to <b>BTM_PASSKEY_REQUEST_EVT </b>
* of \ref wiced_bt_management_cback_t.
*
* \param[in] res            result of the operation WICED_BT_SUCCESS if success
* \param[in] bd_addr        Address of the peer device
* \param[in] passkey        numeric value in the range of 0 - 999999 (0xF423F).
*
* \return void
*
******************************************************************************/
void wiced_bt_dev_pass_key_req_reply( wiced_result_t res, wiced_bt_device_address_t bd_addr, uint32_t passkey );

/******************************************************************************
* Function Name: wiced_bt_dev_send_key_press_notif
***************************************************************************//**
*
* Informs a remote device of keypress during pairing. Used during the passkey
* entry by a device with KeyboardOnly IO capabilities (typically a HID keyboard
* device).
*
* \param[in] bd_addr                                 Address of the peer device
* \param[in] type                                    notification type
*
* \return void
*
******************************************************************************/
void wiced_bt_dev_send_key_press_notif( wiced_bt_device_address_t bd_addr, wiced_bt_dev_passkey_entry_type_t type );

/******************************************************************************
* Function Name: wiced_bt_dev_read_local_oob_data
***************************************************************************//**
*
* Reads the local OOB data from the controller (for sending to the peer device
* over oob message). When operation is completed, local OOB data is provided
* via BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT.
*
* \param void
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_dev_read_local_oob_data( void );

/******************************************************************************
* Function Name: wiced_bt_dev_remote_oob_data_reply
***************************************************************************//**
*
* Provides the remote OOB extended data for Simple Pairing in response to
* \ref BTM_REMOTE_OOB_DATA_REQUEST_EVT.
*
* \param[in] bd_addr               Address of the peer device
* \param[in] is_extended_oob_data  TRUE if extended OOB data (set according to
*                                  BTM_REMOTE_OOB_DATA_REQUEST_EVT request)
* \param[in] c_192                 simple pairing Hash C derived from P-192.
* \param[in] r_192                 simple pairing Randomizer R associated with
*                                  the P-192.
* \param[in] c_256                 simple pairing Hash C derived from the P-256
*                                  (if is_extended_oob_data=TRUE)
* \param[in] r_256                 simple pairing Randomizer R associated with
*                                  the P-256 (if is_extended_oob_data=TRUE)
*
* \return void
*
******************************************************************************/
void wiced_bt_dev_remote_oob_data_reply( wiced_result_t res,
                                         wiced_bt_device_address_t bd_addr,
                                         wiced_bool_t is_extended_oob_data,
                                         BT_OCTET16 c_192,
                                         BT_OCTET16 r_192,
                                         BT_OCTET16 c_256,
                                         BT_OCTET16 r_256);

/******************************************************************************
* Function Name: wiced_bt_dev_build_oob_data
***************************************************************************//**
*
* Builds the OOB data block to be used to send OOB extended data over OOB
* (non-Bluetooth) link.
*
* \param[out] p_data                 OOB data block location
* \param[in] max_len                 OOB data block size
* \param[in] is_extended_oob_data    TRUE if extended OOB data (for Secure Conn)
* \param[in] c_192                   simple pairing Hash C derived from the P-192
* \param[in] r_192                   simple pairing Randomizer R associated with
*                                    the P-192 public key
* \param[in] c_256                   simple pairing Hash C derived from the
*                                    P-256 (if is_extended_oob_data=TRUE)
* \param[in] r_256                   simple pairing Randomizer R associated with
*                                    P-256 (if is_extended_oob_data=TRUE)
*
* \return Number of bytes put into OOB data block.
*
******************************************************************************/
uint16_t wiced_bt_dev_build_oob_data( uint8_t *p_data,
                                      uint16_t max_len,
                                      wiced_bool_t is_extended_oob_data,
                                      BT_OCTET16 c_192,
                                      BT_OCTET16 r_192,
                                      BT_OCTET16 c_256,
                                      BT_OCTET16 r_256 );

/******************************************************************************
* Function Name: wiced_bt_smp_oob_data_reply
***************************************************************************//**
*
* Provides the OOB data for SMP in response to BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT
*
* \param[in] bd_addr       Address of the peer device
* \param[in] res           result of the operation WICED_BT_SUCCESS if success
* \param[in] len           OOB data length
* \param[in] p_data        OOB data
*
* \return void
*
******************************************************************************/
void wiced_bt_smp_oob_data_reply( wiced_bt_device_address_t bd_addr, wiced_result_t res, uint8_t len, uint8_t *p_data );

/******************************************************************************
* Function Name: wiced_bt_smp_create_local_sc_oob_data
***************************************************************************//**
*
* Creates local BLE SC (secure connection) OOB data. When the operation is
* completed, local OOB data is provided via \ref
* BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT.
*
* \param[in] bd_addr                   intended remote address for the OOB data
* \param[in] bd_addr_type              BLE_ADDR_PUBLIC or BLE_ADDR_PUBLIC
*
* \return TRUE if creation of local SC OOB data set started.
*
******************************************************************************/
wiced_bool_t wiced_bt_smp_create_local_sc_oob_data( wiced_bt_device_address_t bd_addr,
                                                    wiced_bt_ble_address_type_t bd_addr_type );

/******************************************************************************
* Function Name: wiced_bt_smp_sc_oob_reply
***************************************************************************//**
*
* Provide the SC OOB data for SMP in response to
* BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT
*
* \param[in] p_oob_data           pointer to OOB data requested  by peer device
*
* \return void
*
******************************************************************************/
void wiced_bt_smp_sc_oob_reply( uint8_t *p_oob_data );

/******************************************************************************
* Function Name: wiced_bt_dev_get_bonded_devices
***************************************************************************//**
*
* Gets the bonded device list.
*
* \param[out] p_paired_device_list    pointer filled with BD_ADDRs of bonded devices
* \param[in/out] p_num_devices        num devices to list[in], num listed[out]
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_dev_get_bonded_devices( wiced_bt_dev_bonded_device_info_t *p_paired_device_list,
                                                uint16_t *p_num_devices );

/******************************************************************************
* Function Name: wiced_bt_dev_delete_bonded_device
***************************************************************************//**
*
* Removes bonding with the remote device with assigned bd_addr.
*
* \param[in] bd_addr   bd_addr of remote device to be removed from bonding list
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_dev_delete_bonded_device( wiced_bt_device_address_t bd_addr );

/******************************************************************************
* Function Name: wiced_bt_dev_get_ble_keys
***************************************************************************//**
*
* Gets the LE key mask from stored key information of nv ram.
*
* \param[in] bd_addr                                          remote bd address
* \param[out] p_key_mask                                    ble key mask stored
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_dev_get_ble_keys( wiced_bt_device_address_t bd_addr, wiced_bt_dev_le_key_type_t *p_key_mask );

#if GATT_OVER_BREDR_INCLUDED == TRUE
/******************************************************************************
* Function Name: wiced_bt_dev_get_security_state
***************************************************************************//**
*
* Gets security flags for the device.
*
* \param[in] bd_addr                 peer address
* \param[out] p_sec_flags            security flags (see #wiced_bt_sec_flags_e)
*
* \return TRUE if successful
*
******************************************************************************/
wiced_bool_t wiced_bt_dev_get_security_state( wiced_bt_device_address_t bd_addr, uint8_t *p_sec_flags );
#endif // GATT_OVER_BREDR_INCLUDED

/******************************************************************************
* Function Name: wiced_bt_set_pairable_mode
***************************************************************************//**
*
* Enables or disables pairing.
*
* \param[in] allow_pairing                            WICED_TRUE or WICED_FALSE
* \param[in] connect_only_paired                      WICED_TRUE or WICED_FALSE
*
* \return void
*
******************************************************************************/
void wiced_bt_set_pairable_mode( uint8_t allow_pairing, uint8_t connect_only_paired );

/******************************************************************************
* Function Name: wiced_bt_dev_add_device_to_address_resolution_db
***************************************************************************//**
*
* Adds link key information to internal address resolution db.
*
* \param[in] p_link_keys       link keys information stored in application side
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_dev_add_device_to_address_resolution_db( wiced_bt_device_link_keys_t *p_link_keys );

/******************************************************************************
* Function Name: wiced_bt_dev_remove_device_from_address_resolution_db
***************************************************************************//**
*
* Removes the link key information from internal address resolution db.
*
* \param[in] p_link_keys       link keys information stored in application side
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_dev_remove_device_from_address_resolution_db( wiced_bt_device_link_keys_t *p_link_keys );

/******************************************************************************
* Function Name: wiced_bt_get_identity_address
***************************************************************************//**
*
* Returns the identity address of the given device.
*
* \param[in] bd_addr                         peer address
* \param[out] identity_address               identity address for given bd_addr
*
* \return TRUE if successful
*
******************************************************************************/
wiced_bool_t wiced_bt_get_identity_address( wiced_bt_device_address_t bd_addr,
                                            wiced_bt_device_address_t identity_address );

#define wiced_bt_dev_set_link_policy BTM_SetLinkPolicy
/**
 * Function         wiced_bt_dev_set_link_policy
 *
 *                  This function is called to set the Link Policy for remote device
 *
 * @param[in]       remote_bda      : remote device's address
 * @param[in/out]   settings        : pointer to the settings value.
 *                                    the policy setting is defined in hcidefs.h
 *                                    if the input setting consists of unsupport feature
 *                                    for local device, it will be cleared
 *
 * @return          wiced_result_t
*/
wiced_result_t wiced_bt_dev_set_link_policy(wiced_bt_device_address_t remote_bda,
        uint16_t *settings);

#define wiced_bt_set_device_class BTM_SetDeviceClass
/**
 * Function         wiced_bt_set_device_class
 *
 *                  This function is called to set the local device class
 *
 * @param[in]       dev_class      : local device class
 *
 * @return          wiced_result_t
*/

wiced_result_t wiced_bt_set_device_class(wiced_bt_dev_class_t dev_class);

/** \} group_dev_functions_sec */

/** \} group_dev_functions */

/** \} wiced_bt_dev_classic */

/** \} wiced_bt_dev */

#ifdef __cplusplus
}
#endif

#endif // __WICED_BT_DEV_H__
