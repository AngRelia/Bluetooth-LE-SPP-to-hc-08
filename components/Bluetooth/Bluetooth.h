/**
 * @file Bluetooth.h
 * @brief ESP32 BLE GATT Central (Master) Client for HC-08
 * @author Your Name
 * @date 2024
 */

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"

/* HC-08 Configuration */
#define REMOTE_SERVICE_UUID        0xFFE0
#define REMOTE_NOTIFY_CHAR_UUID    0xFFE1

/* Configuration Parameters */
#define BLE_MAX_DATA_LEN        512

/* Service ID definition - simplified for HC-08 context */
typedef enum {
    BLE_SERVICE_HC08 = 0,
    BLE_SERVICE_MAX
} ble_service_id_t;

/* Connection status callback */
typedef void (*ble_conn_callback_t)(bool connected);

/* Data receive callback */
// Added index to identify which HC-08 sent the data (0 for HC08_1, 1 for HC08_2, etc)
typedef void (*ble_data_callback_t)(uint8_t device_index, uint8_t *data, uint16_t len);

/**
 * @brief Bluetooth Configuration Structure
 */
typedef struct {
    const char *device_name;            // Local device name
    ble_conn_callback_t conn_cb;        // Connection callback
    ble_data_callback_t data_cb;        // Data receive callback
} ble_config_t;

/**
 * @brief Initialize BLE Module as Central
 * @param config Configuration parameters
 * @return ESP_OK on success
 */
esp_err_t BLE_Init(ble_config_t *config);

/**
 * @brief Deinitialize BLE Module
 * @return ESP_OK on success
 */
esp_err_t BLE_Deinit(void);

/**
 * @brief Send data to all connected HC-08 modules
 * @param data Data pointer
 * @param len Data length
 * @return ESP_OK on success
 */
esp_err_t BLE_SendData(uint8_t *data, uint16_t len);

/**
 * @brief Send string to all connected HC-08 modules
 * @param str String pointer
 * @return ESP_OK on success
 */
esp_err_t BLE_SendString(const char *str);

/**
 * @brief Check if at least one device is connected
 * @return true if at least one connected
 */
bool BLE_IsConnected(void);

/**
 * @brief Check connection status of specific device
 * @param device_index 0 for HC08_1, 1 for HC08_2
 * @return true if connected
 */
bool BLE_IsDeviceConnected(uint8_t device_index);

/**
 * @brief Get Local MAC Address
 * @param mac Buffer for 6 bytes
 */
void BLE_GetMacAddress(uint8_t *mac);

/**
 * @brief Start Scanning for HC-08 devices
 * @return ESP_OK on success
 */
esp_err_t BLE_StartScanning(void);

#endif /* __BLUETOOTH_H__ */
