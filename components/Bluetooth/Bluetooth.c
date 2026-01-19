/**
 * @file Bluetooth.c
 * @brief ESP32 BLE GATT Central Client implementation for HC-08
 */

#include "Bluetooth.h"
#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_device.h"
#include "nvs_flash.h"

#define TAG "BLE_CENTRAL"

/* HC-08 Target Names */
static const char *remote_device_names[] = {"HC08_1", "HC08_2"};
#define TARGET_DEVICE_COUNT (sizeof(remote_device_names) / sizeof(remote_device_names[0]))

/* Internal State */
static bool scanning = false;
static ble_conn_callback_t user_conn_cb = NULL;
static ble_data_callback_t user_data_cb = NULL;

/* Device Structure */
typedef struct {
    bool connected;
    uint16_t conn_id;
    uint16_t gattc_if;
    esp_bd_addr_t remote_bda;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    bool found;
    char name[32];
} remote_device_t;

static remote_device_t remote_devices[TARGET_DEVICE_COUNT];
static esp_gattc_cb_t gattc_profile_event_handler;

/* One profile app ID */
#define PROFILE_APP_ID 0
#define INVALID_HANDLE 0

/* Forward declarations */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

/* Profile struct */
struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* We use one profile to manage all connections for simplicity in this specific task,
 * but we need to map events to our remote_device_t array.
 */
static struct gattc_profile_inst gl_profile_tab[1] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = esp_gattc_cb,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

/* Helper to find device index by conn_id */
static int find_device_index_by_conn_id(uint16_t conn_id) {
    for (int i = 0; i < TARGET_DEVICE_COUNT; i++) {
        if (remote_devices[i].connected && remote_devices[i].conn_id == conn_id) {
            return i;
        }
    }
    return -1;
}

/* Helper to find device index by address */
static int find_device_index_by_bda(esp_bd_addr_t bda) {
    for (int i = 0; i < TARGET_DEVICE_COUNT; i++) {
        if (remote_devices[i].connected && memcmp(remote_devices[i].remote_bda, bda, sizeof(esp_bd_addr_t)) == 0) {
            return i;
        }
    }
    return -1;
}

/* Helper to get empty slot or existing slot for device name */
static int get_device_index_by_name(const char* name) {
    for (int i = 0; i < TARGET_DEVICE_COUNT; i++) {
        if (strcmp(remote_device_names[i], name) == 0) {
            return i;
        }
    }
    return -1;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        // Scan parameters set, start scanning
        esp_ble_gap_start_scanning(30); // Scan for 30 seconds (or forever if loop)
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Scan started");
        scanning = true;
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (adv_name != NULL) {
                char current_name[33];
                if (adv_name_len > 32) adv_name_len = 32;
                memcpy(current_name, adv_name, adv_name_len);
                current_name[adv_name_len] = '\0';

                // Check if this device is one of our targets
                int dev_idx = get_device_index_by_name(current_name);
                if (dev_idx >= 0) {
                    if (!remote_devices[dev_idx].connected) {
                        ESP_LOGI(TAG, "Found target device: %s", current_name);
                        ESP_LOGI(TAG, "Connecting to %s...", current_name);
                        
                        // Stop scanning before connecting
                        esp_ble_gap_stop_scanning();
                        
                        // Connect
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, 
                                           scan_result->scan_rst.bda, 
                                           scan_result->scan_rst.ble_addr_type, 
                                           true);
                        
                        // Mark as found so we don't try to connect repeatedly in this event loop 
                        // (though we stopped scanning, race conditions exist)
                        remote_devices[dev_idx].found = true;
                        strncpy(remote_devices[dev_idx].name, current_name, 32);
                        memcpy(remote_devices[dev_idx].remote_bda, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_DISC_RES_EVT:
            ESP_LOGI(TAG, "Scan duration expired");
            scanning = false;
            // Optionally restart scanning if not all devices connected
            BLE_StartScanning();
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Scan stopped");
        scanning = false;
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
            BLE_StartScanning();
        } else {
            ESP_LOGI(TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
        }
        break;
    case ESP_GATTC_CONNECT_EVT: {
        ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        
        // Find which device we connected to by BDA
        int dev_idx = -1;
        for(int i=0; i<TARGET_DEVICE_COUNT; i++) {
             if(memcmp(remote_devices[i].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t)) == 0) {
                 dev_idx = i;
                 break;
             }
        }
        
        if (dev_idx != -1) {
            remote_devices[dev_idx].conn_id = p_data->connect.conn_id;
            remote_devices[dev_idx].gattc_if = gattc_if;
            remote_devices[dev_idx].connected = true;
            ESP_LOGI(TAG, "Connected to %s", remote_devices[dev_idx].name);
            
            // Send MTU exchange
            // esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
            
            // Directly start service discovery without waiting for MTU exchange
            // HC-08 might not support MTU exchange or implementation might differ
            esp_ble_gattc_search_service(gattc_if, p_data->connect.conn_id, NULL);
            
            // Notify user
            if (user_conn_cb) {
                user_conn_cb(true);
            }
        } else {
            ESP_LOGW(TAG, "Connected to unknown device?");
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "open failed, status %d", p_data->open.status);
            // If open failed, we should probably allow rescanning for this device
            // But we need to know which one failed.
        }
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        
        int dev_idx = find_device_index_by_conn_id(param->dis_srvc_cmpl.conn_id);
        if (dev_idx != -1) {
             // Search for the specific service UUID
             esp_gatt_srvc_id_t remote_service_id;
             remote_service_id.is_primary = true;
             remote_service_id.id.inst_id = 0x00;
             remote_service_id.id.uuid.len = ESP_UUID_LEN_16;
             remote_service_id.id.uuid.uuid.uuid16 = REMOTE_SERVICE_UUID;
             
             esp_gattc_service_elem_t *service_result = NULL;
             uint16_t count = 0;
             esp_gatt_status_t status = esp_ble_gattc_get_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_service_id, NULL, &count, 0);
             
             if (status == ESP_GATT_OK && count > 0) {
                 service_result = (esp_gattc_service_elem_t *)malloc(sizeof(esp_gattc_service_elem_t) * count);
                 if (service_result) {
                     status = esp_ble_gattc_get_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_service_id, service_result, &count, 0);
                     if (status == ESP_GATT_OK && count > 0) {
                         remote_devices[dev_idx].service_start_handle = service_result[0].start_handle;
                         remote_devices[dev_idx].service_end_handle = service_result[0].end_handle;
                         ESP_LOGI(TAG, "Found HC-08 Service on %s, start_handle %d", remote_devices[dev_idx].name, service_result[0].start_handle);
                         
                         // Request characteristics
                         esp_ble_gattc_get_char_by_uuid(gattc_if, 
                                                        param->dis_srvc_cmpl.conn_id, 
                                                        remote_devices[dev_idx].service_start_handle,
                                                        remote_devices[dev_idx].service_end_handle,
                                                        (esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid={.uuid16=REMOTE_NOTIFY_CHAR_UUID}},
                                                        NULL, &count);
                         if (count > 0){
                             esp_gattc_char_elem_t* char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                             if(char_elem_result){
                                 esp_ble_gattc_get_char_by_uuid(gattc_if, 
                                                                param->dis_srvc_cmpl.conn_id, 
                                                                remote_devices[dev_idx].service_start_handle,
                                                                remote_devices[dev_idx].service_end_handle,
                                                                (esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid={.uuid16=REMOTE_NOTIFY_CHAR_UUID}},
                                                                char_elem_result, &count);
                                 if (count > 0){
                                     remote_devices[dev_idx].char_handle = char_elem_result[0].char_handle;
                                     ESP_LOGI(TAG, "Found HC-08 Char on %s, handle %d", remote_devices[dev_idx].name, char_elem_result[0].char_handle);
                                     
                                     // Register for notification
                                     esp_ble_gattc_register_for_notify(gattc_if, remote_devices[dev_idx].remote_bda, char_elem_result[0].char_handle);
                                 }
                                 free(char_elem_result);
                             }
                         }
                     }
                     free(service_result);
                 }
             } else {
                 ESP_LOGW(TAG, "HC-08 Service not found on %s", remote_devices[dev_idx].name);
             }
        }
        
        // Resume scanning to find other devices
        BLE_StartScanning();
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK) {
            ESP_LOGE(TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        
        // After MTU exchange, start service discovery
        // esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        // Just logging
        // ESP_LOGI(TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        // ESP_LOGI(TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.id.inst_id);
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify) {
            // ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        } else {
            // ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        // esp_log_buffer_hex(TAG, p_data->notify.value, p_data->notify.value_len);
        
        int idx = find_device_index_by_conn_id(p_data->notify.conn_id);
        if (idx != -1 && user_data_cb) {
            user_data_cb((uint8_t)idx, p_data->notify.value, p_data->notify.value_len);
        }
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "write descr failed, error status = %x", p_data->write.status);
        }
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "write char failed, error status = %x", p_data->write.status);
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        int d_idx = find_device_index_by_conn_id(p_data->disconnect.conn_id);
        if (d_idx != -1) {
            remote_devices[d_idx].connected = false;
            remote_devices[d_idx].found = false; // Allow rescanning
            ESP_LOGI(TAG, "Disconnected from %s", remote_devices[d_idx].name);
        }
        
        // Restart scanning if we lost a connection or still have devices to find
        BLE_StartScanning();
        
        if (user_conn_cb) {
            user_conn_cb(BLE_IsConnected());
        }
        break;
    default:
        break;
    }
}

esp_err_t BLE_Init(ble_config_t *config) {
    esp_err_t ret;

    if (config) {
        user_conn_cb = config->conn_cb;
        user_data_cb = config->data_cb;
    }

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

    // Initialize device array
    memset(remote_devices, 0, sizeof(remote_devices));

    // Register callbacks
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "%s gap register failed, error code = %x", __func__, ret);
        return ret;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) {
        ESP_LOGE(TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return ret;
    }

    ret = esp_ble_gattc_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "%s gattc app register failed, error code = %x", __func__, ret);
        return ret;
    }

    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", ret);
    }

    ESP_LOGI(TAG, "BLE Central Initialized");
    return ESP_OK;
}

esp_err_t BLE_StartScanning(void) {
    if (!scanning) {
        // Scan parameters
        static esp_ble_scan_params_t ble_scan_params = {
            .scan_type              = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval          = 0x50,
            .scan_window            = 0x30,
            .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
        };
        esp_ble_gap_set_scan_params(&ble_scan_params);
        return ESP_OK; // Scanning starts in callback of set params
    }
    return ESP_OK;
}

esp_err_t BLE_Deinit(void) {
    // Simplified deinit
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    return ESP_OK;
}

esp_err_t BLE_SendData(uint8_t *data, uint16_t len) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    
    // Send to ALL connected devices
    int sent_count = 0;
    for (int i = 0; i < TARGET_DEVICE_COUNT; i++) {
        if (remote_devices[i].connected && remote_devices[i].char_handle != INVALID_HANDLE) {
            esp_ble_gattc_write_char(remote_devices[i].gattc_if,
                                     remote_devices[i].conn_id,
                                     remote_devices[i].char_handle,
                                     len,
                                     data,
                                     ESP_GATT_WRITE_TYPE_NO_RSP,
                                     ESP_GATT_AUTH_REQ_NONE);
            sent_count++;
        }
    }
    
    return sent_count > 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t BLE_SendString(const char *str) {
    if (!str) return ESP_ERR_INVALID_ARG;
    return BLE_SendData((uint8_t*)str, strlen(str));
}

bool BLE_IsConnected(void) {
    for (int i = 0; i < TARGET_DEVICE_COUNT; i++) {
        if (remote_devices[i].connected) return true;
    }
    return false;
}

bool BLE_IsDeviceConnected(uint8_t device_index) {
    if (device_index < TARGET_DEVICE_COUNT) {
        return remote_devices[device_index].connected;
    }
    return false;
}

void BLE_GetMacAddress(uint8_t *mac) {
    const uint8_t *addr = esp_bt_dev_get_address();
    if (addr && mac) {
        memcpy(mac, addr, 6);
    }
}

//实现单独发送到指定设备的函数
esp_err_t BLE_SendDataToDevice(uint8_t device_index, uint8_t *data, uint16_t len) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    if (device_index >= TARGET_DEVICE_COUNT) return ESP_ERR_INVALID_ARG;
    
    // 检查指定设备是否已连接
    if (!remote_devices[device_index].connected || 
        remote_devices[device_index].char_handle == INVALID_HANDLE) {
        ESP_LOGW(TAG, "Device %d not connected or no valid handle", device_index);
        return ESP_FAIL;
    }
    
    // 只发送给指定设备
    esp_err_t ret = esp_ble_gattc_write_char(
        remote_devices[device_index].gattc_if,
        remote_devices[device_index].conn_id,
        remote_devices[device_index].char_handle,
        len,
        data,
        ESP_GATT_WRITE_TYPE_NO_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sent %d bytes to HC08_%d", len, device_index + 1);
    } else {
        ESP_LOGE(TAG, "Failed to send to HC08_%d, error: %d", device_index + 1, ret);
    }
    
    return ret;
}

esp_err_t BLE_SendStringToDevice(uint8_t device_index, const char *str) {
    if (!str) return ESP_ERR_INVALID_ARG;
    return BLE_SendDataToDevice(device_index, (uint8_t*)str, strlen(str));
}