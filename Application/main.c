#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "softdevice_handler_appsh.h"
#include "app_timer_appsh.h"
#include "device_manager.h"
#include "app_button.h"
#include "pstorage.h"
#include "app_trace.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                              /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define UART_TX_BUF_SIZE 256                                                            /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                                                              /**< UART RX buffer size. */

#define SHIFT_BUTTON_ID                  1                                              /**< Button used as 'SHIFT' Key. */

#define DEVICE_NAME                      "eapcoin"                              /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "eapcoin"                          /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_TIMER_PRESCALER              0                                              /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS             (4+BSP_APP_TIMERS_NUMBER)                      /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                              /**< Size of timer operation queues. */

#define PNP_ID_VENDOR_ID_SOURCE          0x02                                           /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                 0x1915                                         /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID                0xEEEE                                         /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION           0x0001                                         /**< Product Version. */

#define APP_ADV_FAST_INTERVAL            0x0028                                         /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL            0x0C80                                         /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_ADV_FAST_TIMEOUT             30                                             /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT             0                                     /**< The duration of the slow advertising period (in seconds). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(7.5, UNIT_1_25_MS)               /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(30, UNIT_1_25_MS)                /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                    6                                              /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(430, UNIT_10_MS)                 /**< Connection supervisory timeout (430 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)     /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)    /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                              /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                              /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                              /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                           /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                             /**< Maximum encryption key size. */

#define OUTPUT_REPORT_INDEX              0                                              /**< Index of Output Report. */
#define OUTPUT_REPORT_MAX_LEN            1                                              /**< Maximum length of Output Report. */
#define INPUT_REPORT_KEYS_INDEX    0                                              /**< Index of Input Report. */
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK 0x02                                           /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */
#define INPUT_REP_REF_ID               0                                              /**< Id of reference to Keyboard Input Report. */
#define OUTPUT_REP_REF_ID                0                                              /**< Id of reference to Keyboard Output Report. */

#define INPUT_CCONTROL_KEYS_INDEX 1
#define INPUT_CC_REPORT_KEYS_MAX_LEN 1
#define INPUT_CC_REP_REF_ID 2
#define OUT_CCONTROL_KEYS_INDEX	2

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2            /**< Reply when unsupported features are requested. */

#define MAX_BUFFER_ENTRIES               5                                              /**< Number of elements that can be enqueued */

#define BASE_USB_HID_SPEC_VERSION        0x0101                                         /**< Version number of base USB HID Specification implemented by this application. */

#define INPUT_REPORT_KEYS_MAX_LEN        8                                              /**< Maximum length of the Input Report characteristic. */

#define DEAD_BEEF                        0xDEADBEEF                                     /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE        MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                             BLE_STACK_HANDLER_SCHED_EVT_SIZE)          /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                 10                                             /**< Maximum number of events in the scheduler queue. */

#define MODIFIER_KEY_POS                 0                                              /**< Position of the modifier byte in the Input Report. */
#define SCAN_CODE_POS                    2                                              /**< This macro indicates the start position of the key scan code in a HID Report. As per the document titled 'Device Class Definition for Human Interface Devices (HID) V1.11, each report shall have one modifier byte followed by a reserved constant byte and then the key scan code. */
#define SHIFT_KEY_CODE                   0x02                                           /**< Key code indicating the press of the Shift Key. */

#define MAX_KEYS_IN_ONE_REPORT           (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS)    /**< Maximum number of key presses that can be sent in one Input Report. */

#define BUFFER_LIST_INIT()                                                                        \
  do                                                                                        \
  {                                                                                         \
    buffer_list.rp = 0;                                                                   \
    buffer_list.wp = 0;                                                                   \
    buffer_list.count = 0;                                                                \
  } while (0)

#define BUFFER_LIST_FULL()\
  ((MAX_BUFFER_ENTRIES == buffer_list.count - 1) ? true : false)

#define BUFFER_LIST_EMPTY()\
  ((0 == buffer_list.count) ? true : false)

#define BUFFER_ELEMENT_INIT(i)\
  do                                                                                        \
  {                                                                                         \
    buffer_list.buffer[(i)].p_data = NULL;                                                \
  } while (0)

typedef enum
{
  BLE_NO_ADV,               /**< No advertising running. */
  BLE_DIRECTED_ADV,         /**< Direct advertising to the latest central. */
  BLE_FAST_ADV_WHITELIST,   /**< Advertising with whitelist. */
  BLE_FAST_ADV,             /**< Fast advertising running. */
  BLE_SLOW_ADV,             /**< Slow advertising running. */
  BLE_SLEEP,                /**< Go to system-off. */
} ble_advertising_mode_t;

typedef struct hid_key_buffer
{
  uint8_t    data_offset;   /**< Max Data that can be buffered for all entries */
  uint8_t    data_len;      /**< Total length of data */
  uint8_t    * p_data;      /**< Scanned key pattern */
  ble_hids_t * p_instance;  /**< Identifies peer and service instance */
}buffer_entry_t;

STATIC_ASSERT(sizeof(buffer_entry_t) % 4 == 0);

typedef struct
{
  buffer_entry_t buffer[MAX_BUFFER_ENTRIES]; /**< Maximum number of entries that can enqueued in the list */
  uint8_t        rp;                         /**< Index to the read location */
  uint8_t        wp;                         /**< Index to write location */
  uint8_t        count;                      /**< Number of elements in the list */
}buffer_list_t;

STATIC_ASSERT(sizeof(buffer_list_t) % 4 == 0);

static ble_hids_t                        m_hids;                                        /**< Structure used to identify the HID service. */
static bool                              m_in_boot_mode = false;                        /**< Current protocol mode. */
static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;       /**< Handle of the current connection. */

static dm_application_instance_t         m_app_handle;                                  /**< Application identifier allocated by device manager. */
static dm_handle_t                       m_bonded_peer_handle;                          /**< Device reference handle to the current bonded central. */
static bool                              m_caps_on = false;                             /**< Variable to indicate if Caps Lock is turned on. */
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}};

static uint8_t m_caps_on_key_scan_str[] =                                                /**< Key pattern to be sent when the output report has been written with the CAPS LOCK bit set. */
{
  0x06, /* Key C */
  0x04, /* Key a */
  0x13, /* Key p */
  0x16, /* Key s */
  0x12, /* Key o */
  0x11, /* Key n */
};

static uint8_t m_caps_off_key_scan_str[] =                                               /**< Key pattern to be sent when the output report has been written with the CAPS LOCK bit cleared. */
{
  0x06, /* Key C */
  0x04, /* Key a */
  0x13, /* Key p */
  0x16, /* Key s */
  0x12, /* Key o */
  0x09, /* Key f */
};

static buffer_list_t buffer_list;

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void service_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

static void ble_advertising_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

static void timers_init(void)
{
  APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
}

static void gap_params_init(void)
{
  uint32_t                err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *)DEVICE_NAME,
                                        strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

static void dis_init(void)
{
  uint32_t         err_code;
  ble_dis_init_t   dis_init_obj;
  ble_dis_pnp_id_t pnp_id;

  pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
  pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
  pnp_id.product_id       = PNP_ID_PRODUCT_ID;
  pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

  memset(&dis_init_obj, 0, sizeof(dis_init_obj));

  ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
  dis_init_obj.p_pnp_id = &pnp_id;

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

  err_code = ble_dis_init(&dis_init_obj);
  APP_ERROR_CHECK(err_code);
}

static void hids_init(void)
{
  uint32_t                   err_code;
  ble_hids_init_t            hids_init_obj;
  ble_hids_inp_rep_init_t    input_report_array[2];
  ble_hids_inp_rep_init_t  * p_input_report;
  ble_hids_outp_rep_init_t   output_report_array[2];
  ble_hids_outp_rep_init_t * p_output_report;
  uint8_t                    hid_info_flags;

  memset((void *)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t));
  memset((void *)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t));
  
  static uint8_t report_map_data[] =
  {
		0x05, 0x01, 								// Usage Page (Generic Desktop)
		0x09, 0x06, 								// Usage (Keyboard)
		0xA1, 0x01, 								// Collection (Application)
//				0x85, 0x00, 								//		 Report Id (1)
		0x05, 0x07, 								//		 Usage Page (Key Codes)
		0x19, 0xe0, 								//		 Usage Minimum (224)
		0x29, 0xe7, 								//		 Usage Maximum (231)
		0x15, 0x00, 								//		 Logical Minimum (0)
		0x25, 0x01, 								//		 Logical Maximum (1)
		0x75, 0x01, 								//		 Report Size (1)
		0x95, 0x08, 								//		 Report Count (8)
		0x81, 0x02, 								//		 Input (Data, Variable, Absolute)

		0x95, 0x01, 								//		 Report Count (1)
		0x75, 0x08, 								//		 Report Size (8)
		0x81, 0x01, 								//		 Input (Constant) reserved byte(1)

		0x95, 0x05, 								//		 Report Count (5)
		0x75, 0x01, 								//		 Report Size (1)
		0x05, 0x08, 								//		 Usage Page (Page# for LEDs)
		0x19, 0x01, 								//		 Usage Minimum (1)
		0x29, 0x05, 								//		 Usage Maximum (5)
		0x91, 0x02, 								//		 Output (Data, Variable, Absolute), Led report
		0x95, 0x01, 								//		 Report Count (1)
		0x75, 0x03, 								//		 Report Size (3)
		0x91, 0x01, 								//		 Output (Data, Variable, Absolute), Led report padding

		0x95, 0x06, 								//		 Report Count (6)
		0x75, 0x08, 								//		 Report Size (8)
		0x15, 0x00, 								//		 Logical Minimum (0)
		0x25, 0x65, 								//		 Logical Maximum (101)
		0x05, 0x07, 								//		 Usage Page (Key codes)
		0x19, 0x00, 								//		 Usage Minimum (0)
		0x29, 0x65, 								//		 Usage Maximum (101)
		0x81, 0x00, 								//		 Input (Data, Array) Key array(6 bytes)

		0x09, 0x05, 								//		 Usage (Vendor Defined)
		0x15, 0x00, 								//		 Logical Minimum (0)
		0x26, 0xFF, 0x00, 					//		 Logical Maximum (255)
		0x75, 0x08, 								//		 Report Count (2)
		0x95, 0x02, 								//		 Report Size (8 bit)
		0xB1, 0x02, 								//		 Feature (Data, Variable, Absolute)
		0xC0, 											// End Collection (Application)
	
		// Report ID 2: Advanced buttons
		0x05, 0x0C, 										// Usage Page (Consumer)
		0x09, 0x01, 										// Usage (Consumer Control)
		0xA1, 0x01, 										// Collection (Application)
		0x85, 0x02, 										//		 Report Id (2)
		0x15, 0x00, 										//		 Logical minimum (0)
		0x25, 0x01, 										//		 Logical maximum (1)
		0x75, 0x01, 										//		 Report Size (1)
		0x95, 0x01, 										//		 Report Count (1)

		0x09, 0xCD, 										//		 Usage (Play/Pause)
		0x81, 0x02, 										//		 Input (Data,Value,Relative,Bit Field)
		0x0A, 0x83, 0x01, 							//		 Usage (AL Consumer Control Configuration)
		0x81, 0x02, 										//		 Input (Data,Value,Relative,Bit Field)
		0x09, 0xB5, 										//		 Usage (Scan Next Track)
		0x81, 0x02, 										//		 Input (Data,Value,Relative,Bit Field)
		0x09, 0xB6, 										//		 Usage (Scan Previous Track)
		0x81, 0x02, 										//		 Input (Data,Value,Relative,Bit Field)

		0x09, 0xEA, 										//		 Usage (Volume Down)
		0x81, 0x02, 										//		 Input (Data,Value,Relative,Bit Field)
		0x09, 0xE9, 										//		 Usage (Volume Up)
		0x81, 0x02, 										//		 Input (Data,Value,Relative,Bit Field)
		0x0A, 0x25, 0x02, 							//		 Usage (AC Forward)
		0x81, 0x02, 										//		 Input (Data,Value,Relative,Bit Field)
		0x0A, 0x23, 0x02, 							//		 Usage (AC Back)
		0x81, 0x02, 										//		 Input (Data,Value,Relative,Bit Field)
		0xC0														// End Collection 	
  };

  // Initialize HID Service
  p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
  p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
  p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
  p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

  p_output_report                      = &output_report_array[OUTPUT_REPORT_INDEX];
  p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
  p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
  p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);
	
  p_input_report                      = &input_report_array[INPUT_CCONTROL_KEYS_INDEX];
  p_input_report->max_len             = INPUT_CC_REPORT_KEYS_MAX_LEN;
  p_input_report->rep_ref.report_id   = INPUT_CC_REP_REF_ID;
  p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);	
	
  hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

  memset(&hids_init_obj, 0, sizeof(hids_init_obj));

  hids_init_obj.evt_handler                    = on_hids_evt;
  hids_init_obj.error_handler                  = service_error_handler;
  hids_init_obj.is_kb                          = true;
  hids_init_obj.is_mouse                       = false;
  hids_init_obj.inp_rep_count                  = 2;
  hids_init_obj.p_inp_rep_array                = input_report_array;
  hids_init_obj.outp_rep_count                 = 1;
  hids_init_obj.p_outp_rep_array               = output_report_array;
  hids_init_obj.feature_rep_count              = 0;
  hids_init_obj.p_feature_rep_array            = NULL;
  hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
  hids_init_obj.rep_map.p_data                 = report_map_data;
  hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
  hids_init_obj.hid_information.b_country_code = 0;
  hids_init_obj.hid_information.flags          = hid_info_flags;
  hids_init_obj.included_services_count        = 0;
  hids_init_obj.p_included_services_array      = NULL;

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
      &hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

  err_code = ble_hids_init(&m_hids, &hids_init_obj);
  APP_ERROR_CHECK(err_code);
}

static void services_init(void)
{
  dis_init();
  hids_init();
}

static void conn_params_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
  uint32_t               err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = NULL;
  cp_init.error_handler                  = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

static bool is_shift_key_pressed(void)
{
  bool result;
  uint32_t err_code = bsp_button_is_pressed(SHIFT_BUTTON_ID,&result);
  APP_ERROR_CHECK(err_code);
  return result;
}

static uint32_t send_key_scan_press_release(ble_hids_t *   p_hids,
                                            uint8_t *      p_key_pattern,
                                            uint16_t       pattern_len,
                                            uint16_t       pattern_offset,
                                            uint16_t *     p_actual_len)
{
  uint32_t err_code;
  uint16_t offset;
  uint16_t data_len;
  uint8_t  data[INPUT_REPORT_KEYS_MAX_LEN];
  
  STATIC_ASSERT((INPUT_REPORT_KEYS_MAX_LEN - 2) == 6);

  ASSERT(pattern_len <= (INPUT_REPORT_KEYS_MAX_LEN - 2));

  offset   = pattern_offset;
  data_len = pattern_len;

  do{
    memset(data, 0, sizeof(data));
    memcpy(data + SCAN_CODE_POS + offset, p_key_pattern + offset, data_len - offset);
    if (is_shift_key_pressed()){
      data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
    }

    if (!m_in_boot_mode){
      err_code = ble_hids_inp_rep_send(p_hids, 
                                       INPUT_REPORT_KEYS_INDEX,
                                       INPUT_REPORT_KEYS_MAX_LEN,
                                       data);
    }else{
      err_code = ble_hids_boot_kb_inp_rep_send(p_hids,
                                               INPUT_REPORT_KEYS_MAX_LEN,
                                               data);
    }
    if (err_code != NRF_SUCCESS){
      break;
    }
    offset++;
  } while (offset <= data_len);

  *p_actual_len = offset;

  return err_code;
}

static void buffer_init(void)
{
  uint32_t buffer_count;

  BUFFER_LIST_INIT();

  for (buffer_count = 0; buffer_count < MAX_BUFFER_ENTRIES; buffer_count++){
    BUFFER_ELEMENT_INIT(buffer_count);
  }
}

static uint32_t buffer_enqueue(ble_hids_t *            p_hids,
                               uint8_t *               p_key_pattern,
                               uint16_t                pattern_len,
                               uint16_t                offset)
{
  buffer_entry_t * element;
  uint32_t         err_code = NRF_SUCCESS;

  if (BUFFER_LIST_FULL()){
    err_code = NRF_ERROR_NO_MEM;
  }else{
    element                 = &buffer_list.buffer[(buffer_list.wp)];
    element->p_instance     = p_hids;
    element->p_data         = p_key_pattern;
    element->data_offset    = offset;
    element->data_len       = pattern_len;
    buffer_list.count++;
    buffer_list.wp++;
    if (buffer_list.wp == MAX_BUFFER_ENTRIES){
      buffer_list.wp = 0;
    }
  }

	return err_code;
}

static uint32_t buffer_dequeue(bool tx_flag)
{
  buffer_entry_t * p_element;
  uint32_t         err_code = NRF_SUCCESS;
  uint16_t         actual_len;

  if (BUFFER_LIST_EMPTY()) {
    err_code = NRF_ERROR_NOT_FOUND;
  }else{
    bool remove_element = true;
    p_element = &buffer_list.buffer[(buffer_list.rp)];
    if (tx_flag){
      err_code = send_key_scan_press_release(p_element->p_instance,
                                             p_element->p_data,
                                             p_element->data_len,
                                             p_element->data_offset,
                                             &actual_len);
      if ((err_code == BLE_ERROR_NO_TX_BUFFERS) && (actual_len <= p_element->data_len)){
        p_element->data_offset = actual_len;
        remove_element         = false;
      }
    }
    if (remove_element){
      BUFFER_ELEMENT_INIT(buffer_list.rp);
      buffer_list.rp++;
      buffer_list.count--;
      if (buffer_list.rp == MAX_BUFFER_ENTRIES){
        buffer_list.rp = 0;
      }
    }
  }
  return err_code;
}

static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern)
{
  uint32_t err_code;
  uint16_t actual_len;

  err_code = send_key_scan_press_release(&m_hids,
                                         p_key_pattern,
                                         key_pattern_len,
                                         0,
                                         &actual_len);
  if ((err_code == BLE_ERROR_NO_TX_BUFFERS) && (actual_len <= key_pattern_len)){
    UNUSED_VARIABLE(buffer_enqueue(&m_hids, p_key_pattern, key_pattern_len, actual_len));
  }
  if ((err_code != NRF_SUCCESS) &&
      (err_code != NRF_ERROR_INVALID_STATE) &&
      (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
      (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
  ){
    APP_ERROR_HANDLER(err_code);
  }
}

static void on_hid_rep_char_write(ble_hids_evt_t *p_evt)
{
  if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT){
    uint32_t err_code;
    uint8_t  report_val;
    uint8_t  report_index = p_evt->params.char_write.char_id.rep_index;

    if (report_index == 1){
      STATIC_ASSERT(OUTPUT_REPORT_MAX_LEN == 1);
      err_code = ble_hids_outp_rep_get(&m_hids,
                                       report_index,
                                       OUTPUT_REPORT_MAX_LEN,
                                       0,
                                       &report_val);
      APP_ERROR_CHECK(err_code);
      if (!m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) != 0)){
        keys_send(sizeof(m_caps_on_key_scan_str), m_caps_on_key_scan_str);
        m_caps_on = true;
      }else if (m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) == 0)){
        keys_send(sizeof(m_caps_off_key_scan_str), m_caps_off_key_scan_str);
        m_caps_on = false;
      }else{
      }
    }
  }
}

static void sleep_mode_enter(void)
{
  uint32_t err_code;
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t *p_evt)
{
  switch (p_evt->evt_type)
  {
    case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
      m_in_boot_mode = true;
      break;
    case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
      m_in_boot_mode = false;
      break;
    case BLE_HIDS_EVT_REP_CHAR_WRITE:
      on_hid_rep_char_write(p_evt);
      break;
		case BLE_HIDS_EVT_REPORT_READ:
			break;
		case BLE_HIDS_EVT_HOST_SUSP:
			break;
    case BLE_HIDS_EVT_NOTIF_ENABLED:
    {
      dm_service_context_t   service_context;
      service_context.service_type = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
      service_context.context_data.len = 0;
      service_context.context_data.p_data = NULL;
      if (m_in_boot_mode){
        if (p_evt->params.notification.char_id.uuid ==
          BLE_UUID_BOOT_KEYBOARD_INPUT_REPORT_CHAR){
          uint32_t err_code;
          err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
          if (err_code != NRF_ERROR_INVALID_STATE){
            APP_ERROR_CHECK(err_code);
          }
        }
      }
      else if (p_evt->params.notification.char_id.rep_type == BLE_HIDS_REP_TYPE_INPUT){
        uint32_t err_code;
        err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
        if (err_code != NRF_ERROR_INVALID_STATE){
          APP_ERROR_CHECK(err_code);
        }
      }else{
      }
      break;
    }
    default:
      break;
  }
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  uint32_t err_code;

  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_DIRECTED:
      break;
    case BLE_ADV_EVT_FAST:
      break;
    case BLE_ADV_EVT_SLOW:
      break;
    case BLE_ADV_EVT_FAST_WHITELIST:
      break;
    case BLE_ADV_EVT_SLOW_WHITELIST:
      break;
    case BLE_ADV_EVT_IDLE:
      sleep_mode_enter();
      break;
    case BLE_ADV_EVT_WHITELIST_REQUEST:
    {
      ble_gap_whitelist_t whitelist;
      ble_gap_addr_t    * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
      ble_gap_irk_t     * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

      whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
      whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
      whitelist.pp_addrs   = p_whitelist_addr;
      whitelist.pp_irks    = p_whitelist_irk;

      err_code = dm_whitelist_create(&m_app_handle, &whitelist);
      APP_ERROR_CHECK(err_code);

      err_code = ble_advertising_whitelist_reply(&whitelist);
      APP_ERROR_CHECK(err_code);
      break;
    }
    case BLE_ADV_EVT_PEER_ADDR_REQUEST:
    {
      ble_gap_addr_t peer_address;
      if(m_bonded_peer_handle.appl_id != DM_INVALID_ID){
        err_code = dm_peer_addr_get(&m_bonded_peer_handle, &peer_address);
				if(err_code == NRF_SUCCESS){
          err_code = ble_advertising_peer_addr_reply(&peer_address);
          APP_ERROR_CHECK(err_code);
				}
      }
      break;
    }
    default:
      break;
  }
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
  uint32_t                              err_code;
  ble_gatts_rw_authorize_reply_params_t auth_reply;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      m_conn_handle      = p_ble_evt->evt.gap_evt.conn_handle;
      break;
    case BLE_EVT_TX_COMPLETE:
      (void) buffer_dequeue(true);
      break;
    case BLE_GAP_EVT_DISCONNECTED:
      (void) buffer_dequeue(false);
      m_conn_handle = BLE_CONN_HANDLE_INVALID;
      m_caps_on = false;
      break;
    case BLE_EVT_USER_MEM_REQUEST:
      err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
      APP_ERROR_CHECK(err_code);
      break;
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
      if(p_ble_evt->evt.gatts_evt.params.authorize_request.type
         != BLE_GATTS_AUTHORIZE_TYPE_INVALID){
        if ((p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op == 
					BLE_GATTS_OP_PREP_WRITE_REQ) || 
					(p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op == 
					BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) || 
					(p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op == 
					BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)){
          if (p_ble_evt->evt.gatts_evt.params.authorize_request.type == 
						BLE_GATTS_AUTHORIZE_TYPE_WRITE){
	          auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
          }else{
            auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
          }
          auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
          err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle,&auth_reply);
          APP_ERROR_CHECK(err_code);
        }
      }
      break;
    case BLE_GATTC_EVT_TIMEOUT:
    case BLE_GATTS_EVT_TIMEOUT:
      err_code = sd_ble_gap_disconnect(m_conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;
    default:
        break;
  }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
  dm_ble_evt_handler(p_ble_evt);
//    bsp_btn_ble_on_ble_evt(p_ble_evt);
  on_ble_evt(p_ble_evt);
  ble_advertising_on_ble_evt(p_ble_evt);
  ble_conn_params_on_ble_evt(p_ble_evt);
  ble_hids_on_ble_evt(&m_hids, p_ble_evt);		
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
  pstorage_sys_event_handler(sys_evt);
  ble_advertising_on_sys_evt(sys_evt);
}

static void ble_stack_init(void)
{
  uint32_t err_code;

  SOFTDEVICE_HANDLER_APPSH_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION, true);

  ble_enable_params_t ble_enable_params;
  memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#ifdef S130
  ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
  ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
  err_code = sd_ble_enable(&ble_enable_params);
  APP_ERROR_CHECK(err_code);

  err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
  APP_ERROR_CHECK(err_code);

  err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
  APP_ERROR_CHECK(err_code);
}

static void scheduler_init(void)
{
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void bsp_event_handler(bsp_event_t event)
{
  uint32_t err_code;

  switch (event)
  {
    case BSP_EVENT_SLEEP:
      sleep_mode_enter();
      break;
    case BSP_EVENT_DISCONNECT:
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_ERROR_INVALID_STATE){
        APP_ERROR_CHECK(err_code);
      }
      break;
    case BSP_EVENT_WHITELIST_OFF:
      err_code = ble_advertising_restart_without_whitelist();
      if (err_code != NRF_ERROR_INVALID_STATE){
        APP_ERROR_CHECK(err_code);
      }
      break;
    case BSP_EVENT_KEY_0:
      if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				uint8_t cmd = 0x80;
				ble_hids_inp_rep_send(&m_hids, 1, 1, (uint8_t*)&cmd);
      }
			break;
		case BSP_EVENT_KEY_7:
      if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
				uint8_t cmd = 0x00;
				ble_hids_inp_rep_send(&m_hids, 1, 1, (uint8_t*)&cmd);
			}
      break;
  default:
    break;
  }
}

static void advertising_init(void)
{
  uint32_t       err_code;
  uint8_t        adv_flags;
  ble_advdata_t  advdata;

  memset(&advdata, 0, sizeof(advdata));

  adv_flags                       = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
  advdata.name_type               = BLE_ADVDATA_FULL_NAME;
  advdata.include_appearance      = true;
  advdata.flags                   = adv_flags;
  advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  advdata.uuids_complete.p_uuids  = m_adv_uuids;

  ble_adv_modes_config_t options =
  {
    BLE_ADV_WHITELIST_ENABLED,
    BLE_ADV_DIRECTED_ENABLED,
    BLE_ADV_DIRECTED_SLOW_DISABLED, 0,0,
    BLE_ADV_FAST_ENABLED, APP_ADV_FAST_INTERVAL, APP_ADV_FAST_TIMEOUT,
    BLE_ADV_SLOW_ENABLED, APP_ADV_SLOW_INTERVAL, APP_ADV_SLOW_TIMEOUT
  };

  err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, ble_advertising_error_handler);
  APP_ERROR_CHECK(err_code);
}

static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           ret_code_t           event_result)
{
  APP_ERROR_CHECK(event_result);

  switch(p_event->event_id){
    case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
    case DM_EVT_SECURITY_SETUP_COMPLETE:
      m_bonded_peer_handle = (*p_handle);
      break;
  }
  return NRF_SUCCESS;
}

static void device_manager_init(bool erase_bonds)
{
  uint32_t               err_code;
  dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
  dm_application_param_t  register_param;

  err_code = dm_handle_initialize(&m_bonded_peer_handle);
  APP_ERROR_CHECK(err_code);
  
  err_code = pstorage_init();
  APP_ERROR_CHECK(err_code);

  err_code = dm_init(&init_param);
  APP_ERROR_CHECK(err_code);
  
  memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

  register_param.sec_param.bond         = SEC_PARAM_BOND;
  register_param.sec_param.mitm         = SEC_PARAM_MITM;
  register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
  register_param.sec_param.oob          = SEC_PARAM_OOB;
  register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
  register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
  register_param.evt_handler            = device_manager_evt_handler;
  register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

  err_code = dm_register(&m_app_handle, &register_param);
  APP_ERROR_CHECK(err_code);
}

static void buttons_leds_init(bool * p_erase_bonds)
{
  bsp_event_t startup_event;

  uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                               APP_TIMER_TICKS(50, APP_TIMER_PRESCALER),
                               bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


static void power_manage(void)
{
  uint32_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}

int main(void)
{
  bool erase_bonds;
  uint32_t err_code;

 //   app_trace_init();
  timers_init();
  buttons_leds_init(&erase_bonds);
  ble_stack_init();
  scheduler_init();
  device_manager_init(erase_bonds);
  gap_params_init();
  advertising_init();
  services_init();
  conn_params_init();
  buffer_init();

  err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);

  for (;;){
    app_sched_execute();
    power_manage();
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
	  NVIC_SystemReset();
  }
}

