/**
 * @file Bluetooth.c
 * @brief ESP32 BLE GATT Central Client implementation for HC-08 (Live Capture Version)
 */

#include "Bluetooth.h"
#include <string.h>
#include <stdlib.h>
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
static const char *remote_device_names[] = {"HC08_1", "HC08_2"};       //通过名字识别设备
#define TARGET_DEVICE_COUNT (sizeof(remote_device_names) / sizeof(remote_device_names[0]))  //目标设备数量

/* Internal State */
static bool is_scanning = false;    //扫描状态
static ble_conn_callback_t user_conn_cb = NULL; //用户连接回调
static ble_data_callback_t user_data_cb = NULL; //用户数据回调

/* Device Structure */
typedef struct {
    bool connected;                 //连接状态
    uint16_t conn_id;               //连接ID
    uint16_t gattc_if;              //GATT客户端接口
    esp_bd_addr_t remote_bda;       //远程设备地址
    uint16_t service_start_handle;  //服务起始句柄
    uint16_t service_end_handle;    //服务结束句柄
    uint16_t char_handle;           //特征句柄
    bool found;                     //是否找到目标服务和特征
    char name[32];                  //设备名称
} remote_device_t;      //远程设备结构体

static remote_device_t remote_devices[TARGET_DEVICE_COUNT];     //创建同等数量的远程设备结构体数组

#define PROFILE_APP_ID 0           //配置文件应用ID
#define INVALID_HANDLE 0                //无效句柄

/* 回调函数 */
//GAP 层回调。处理扫描、发现广播、连接参数更新、停止扫描等链路层事件
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);    
//GATT Client 回调。处理连接后服务发现、特征发现、注册通知、收到通知数据等 GATT 事件。
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
/* 工具函数 */
//普通函数。由 `ESP_GATTC_REG_FOR_NOTIFY_EVT` 调用，写 0x2902(CCCD) 真正开启 notify。
static void enable_notify_cccd(esp_gatt_if_t gattc_if, uint16_t conn_id, int dev_idx);

//GATT Client 应用
struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;    //GATT客户端回调函数
    uint16_t gattc_if;              //GATT客户端接口
    uint16_t app_id;                //应用ID
    uint16_t conn_id;               //连接ID
    uint16_t service_start_handle;  //服务起始句柄
    uint16_t service_end_handle;    //服务结束句柄
    uint16_t char_handle;           //特征句柄
    esp_bd_addr_t remote_bda;       //远程设备地址
};  //

//Profile 管理结构
static struct gattc_profile_inst gl_profile_tab[1] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = esp_gattc_cb,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

static void enable_notify_cccd(esp_gatt_if_t gattc_if, uint16_t conn_id, int dev_idx) {
    // 1) Count descriptors under the remote characteristic (HC-08 FFE1).
    uint16_t count = 0;
    esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
        gattc_if,
        conn_id,
        ESP_GATT_DB_DESCRIPTOR,
        remote_devices[dev_idx].service_start_handle,
        remote_devices[dev_idx].service_end_handle,
        remote_devices[dev_idx].char_handle,
        &count);
    if (status != ESP_GATT_OK || count == 0) {
        ESP_LOGW(TAG, "[%s] CCCD not found (count=%u, status=%d)", remote_devices[dev_idx].name, count, status);
        return;
    }

    // 2) Fetch the CCCD (0x2902). This descriptor belongs to the peer (HC-08).
    esp_gattc_descr_elem_t *descr_result = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
    if (!descr_result) {
        ESP_LOGE(TAG, "CCCD alloc failed");
        return;
    }

    status = esp_ble_gattc_get_descr_by_char_handle(
        gattc_if,
        conn_id,
        remote_devices[dev_idx].char_handle,
        (esp_bt_uuid_t){.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}},
        descr_result,
        &count);

    if (status == ESP_GATT_OK && count > 0) {
        // 3) Write 0x0001 to CCCD: enable NOTIFY from the peer to ESP32.
        uint8_t notify_en[2] = {0x01, 0x00};
        esp_ble_gattc_write_char_descr(
            gattc_if,
            conn_id,
            descr_result[0].handle,
            sizeof(notify_en),
            notify_en,
            ESP_GATT_WRITE_TYPE_RSP,
            ESP_GATT_AUTH_REQ_NONE);
        ESP_LOGI(TAG, "[%s] CCCD write enable notify", remote_devices[dev_idx].name);
    } else {
        ESP_LOGW(TAG, "[%s] CCCD query failed (status=%d, count=%u)", remote_devices[dev_idx].name, status, count);
    }

    free(descr_result);
}

static int find_device_index_by_conn_id(uint16_t conn_id) {
    for (int i = 0; i < TARGET_DEVICE_COUNT; i++) {
        if (remote_devices[i].connected && remote_devices[i].conn_id == conn_id) {
            return i;
        }
    }
    return -1;
}

static int get_device_index_by_name(const char* name) {
    for (int i = 0; i < TARGET_DEVICE_COUNT; i++) {
        if (strcmp(remote_device_names[i], name) == 0) {
            return i;
        }
    }
    return -1;
}

void start_scan_safe(void) {
    if (!is_scanning) {
        esp_ble_gap_start_scanning(30);
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        start_scan_safe();
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Scan started");
        is_scanning = true;
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

                int dev_idx = get_device_index_by_name(current_name);
                if (dev_idx >= 0) {
                    if (!remote_devices[dev_idx].connected) {
                        ESP_LOGI(TAG, "Found target device: %s", current_name);
                        ESP_LOGI(TAG, "Connecting to %s...", current_name);
                        
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, 
                                           scan_result->scan_rst.bda, 
                                           scan_result->scan_rst.ble_addr_type, 
                                           true);
                        
                        remote_devices[dev_idx].found = true;
                        strncpy(remote_devices[dev_idx].name, current_name, 32);
                        memcpy(remote_devices[dev_idx].remote_bda, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_DISC_RES_EVT:
            ESP_LOGI(TAG, "Scan duration expired");
            is_scanning = false;
            start_scan_safe();
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
        is_scanning = false;
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         // 忽略参数更新警告
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
            ESP_LOGI(TAG, "reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
        }
        break;

    case ESP_GATTC_CONNECT_EVT: {
        ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d", p_data->connect.conn_id);
        
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
            
            // 清空旧的句柄，防止误用
            remote_devices[dev_idx].service_start_handle = 0;
            remote_devices[dev_idx].service_end_handle = 0;
            remote_devices[dev_idx].char_handle = INVALID_HANDLE;
            
            ESP_LOGI(TAG, "Connected to %s, starting Discovery...", remote_devices[dev_idx].name);
            esp_ble_gattc_search_service(gattc_if, p_data->connect.conn_id, NULL);
            
            if (user_conn_cb) user_conn_cb(true);
        }
        break;
    }

    // 【关键修复 1】恢复 SEARCH_RES 事件，实时捕捉服务句柄
    case ESP_GATTC_SEARCH_RES_EVT: {
        // 打印 UUID 帮助调试
        // ESP_LOGI(TAG, "SEARCH RES: conn_id=%d UUID=0x%04x", p_data->search_res.conn_id, p_data->search_res.srvc_id.uuid.uuid.uuid16);
        
        int dev_idx = find_device_index_by_conn_id(p_data->search_res.conn_id);
        if (dev_idx != -1) {
            // 检查是不是我们要找的 0xFFE0
            if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && 
                p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
                
                remote_devices[dev_idx].service_start_handle = p_data->search_res.start_handle;
                remote_devices[dev_idx].service_end_handle = p_data->search_res.end_handle;
                ESP_LOGI(TAG, "--> [%s] Found Service 0xFFE0! Handles: %d - %d", 
                         remote_devices[dev_idx].name, 
                         p_data->search_res.start_handle, 
                         p_data->search_res.end_handle);
            }
        }
        break;
    }

    // 【关键修复 2】搜索结束后，检查刚才是否捕捉到了句柄
    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (p_data->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Service search complete, conn_id %d", p_data->search_cmpl.conn_id);
        
        int dev_idx = find_device_index_by_conn_id(p_data->search_cmpl.conn_id);
        if (dev_idx != -1) {
            // 检查之前在 SEARCH_RES 里有没有抓到句柄
            if (remote_devices[dev_idx].service_start_handle != 0 && 
                remote_devices[dev_idx].service_end_handle != 0) {
                
                ESP_LOGI(TAG, "--> Getting characteristics for %s...", remote_devices[dev_idx].name);
                
                // 使用句柄范围直接获取特征值
                uint16_t count = 0;
                esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         p_data->search_cmpl.conn_id,
                                                                         ESP_GATT_DB_CHARACTERISTIC,
                                                                         remote_devices[dev_idx].service_start_handle,
                                                                         remote_devices[dev_idx].service_end_handle,
                                                                         INVALID_HANDLE,
                                                                         &count);
                if (status == ESP_GATT_OK && count > 0) {
                    esp_gattc_char_elem_t *char_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                    if (char_result) {
                        status = esp_ble_gattc_get_all_char(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            remote_devices[dev_idx].service_start_handle,
                                                            remote_devices[dev_idx].service_end_handle,
                                                            char_result,
                                                            &count,
                                                            0);
                        if (status == ESP_GATT_OK) {
                            for (int i = 0; i < count; i++) {
                                if (char_result[i].uuid.len == ESP_UUID_LEN_16 && 
                                    char_result[i].uuid.uuid.uuid16 == REMOTE_NOTIFY_CHAR_UUID) {
                                    
                                    remote_devices[dev_idx].char_handle = char_result[i].char_handle;
                                    ESP_LOGI(TAG, "   >>> [%s] TARGET LOCKED (0xFFE1)! Handle: %d", 
                                             remote_devices[dev_idx].name, char_result[i].char_handle);
                                    
                                    esp_ble_gattc_register_for_notify(gattc_if, remote_devices[dev_idx].remote_bda, char_result[i].char_handle);
                                }
                            }
                        }
                        free(char_result);
                    }
                } else {
                    ESP_LOGE(TAG, "--> Service handles valid, but NO characteristics found!");
                }
            } else {
                ESP_LOGW(TAG, "--> Service 0xFFE0 NOT found during search phase!");
            }
        }
        // 搜索结束，安全重启扫描
        start_scan_safe();
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        int d_idx = find_device_index_by_conn_id(p_data->disconnect.conn_id);
        if (d_idx != -1) {
            remote_devices[d_idx].connected = false;
            remote_devices[d_idx].found = false; 
            remote_devices[d_idx].char_handle = INVALID_HANDLE;
            ESP_LOGI(TAG, "Disconnected from %s", remote_devices[d_idx].name);
        }
        start_scan_safe();
        if (user_conn_cb) user_conn_cb(BLE_IsConnected());
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        int idx = -1;
        for (int i = 0; i < TARGET_DEVICE_COUNT; i++) {
            if (remote_devices[i].connected &&
                remote_devices[i].char_handle == p_data->reg_for_notify.handle) {
                idx = i;
                break;
            }
        }
        if (idx != -1 && p_data->reg_for_notify.status == ESP_GATT_OK) {
            enable_notify_cccd(gattc_if, remote_devices[idx].conn_id, idx);
        } else if (idx != -1) {
            ESP_LOGW(TAG, "[%s] reg notify failed, status=%d", remote_devices[idx].name, p_data->reg_for_notify.status);
        }
        break;
    }

    case ESP_GATTC_NOTIFY_EVT:
        {
            int idx = find_device_index_by_conn_id(p_data->notify.conn_id);
            if (idx != -1) {
                ESP_LOGI(TAG, "Notify from %s, len=%d", remote_devices[idx].name, p_data->notify.value_len);
                if (p_data->notify.value_len > 0) {
                    esp_log_buffer_hex(TAG, p_data->notify.value, p_data->notify.value_len);
                }
                if (user_data_cb) {
                    user_data_cb((uint8_t)idx, p_data->notify.value, p_data->notify.value_len);
                }
            } else {
                ESP_LOGW(TAG, "Notify from unknown conn_id=%d", p_data->notify.conn_id);
            }
        }
        break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT: 
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

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) return ret;

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) return ret;

    ret = esp_bluedroid_init();
    if (ret) return ret;

    ret = esp_bluedroid_enable();
    if (ret) return ret;

    memset(remote_devices, 0, sizeof(remote_devices));

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) return ret;

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) return ret;

    ret = esp_ble_gattc_app_register(PROFILE_APP_ID);
    if (ret) return ret;

    ret = esp_ble_gatt_set_local_mtu(500);
    return ESP_OK;
}

esp_err_t BLE_StartScanning(void) {
    start_scan_safe();
    return ESP_OK;
}

esp_err_t BLE_Deinit(void) {
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    return ESP_OK;
}

esp_err_t BLE_SendData(uint8_t *data, uint16_t len) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    
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

esp_err_t BLE_SendDataToDevice(uint8_t device_index, uint8_t *data, uint16_t len) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    if (device_index >= TARGET_DEVICE_COUNT) return ESP_ERR_INVALID_ARG;
    
    if (!remote_devices[device_index].connected || 
        remote_devices[device_index].char_handle == INVALID_HANDLE) {
        ESP_LOGW(TAG, "Device %d not connected or no valid handle", device_index);
        return ESP_FAIL;
    }
    
    return esp_ble_gattc_write_char(
        remote_devices[device_index].gattc_if,
        remote_devices[device_index].conn_id,
        remote_devices[device_index].char_handle,
        len,
        data,
        ESP_GATT_WRITE_TYPE_NO_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );
}

esp_err_t BLE_SendStringToDevice(uint8_t device_index, const char *str) {
    if (!str) return ESP_ERR_INVALID_ARG;
    return BLE_SendDataToDevice(device_index, (uint8_t*)str, strlen(str));
}
