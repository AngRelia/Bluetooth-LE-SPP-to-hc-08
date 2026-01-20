/**
 * @file main.c
 * @brief ESP32 BLE Central for HC-08 Transparent Transmission
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "Bluetooth.h"
#include "Serial.h"

#define TAG "MAIN"
#define BUTTON_GPIO 0  // BOOT button

static bool last_button_state = false;
uint16_t boot_debug = 0;
/**
 * @brief Connection status callback
 */
void ble_connection_callback(bool connected) {
    if (connected) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "✓ At least one HC-08 connected!");
        if (BLE_IsDeviceConnected(0)) ESP_LOGI(TAG, "  - HC08_1: Connected");
        else ESP_LOGI(TAG, "  - HC08_1: Disconnected");
        
        if (BLE_IsDeviceConnected(1)) ESP_LOGI(TAG, "  - HC08_2: Connected");
        else ESP_LOGI(TAG, "  - HC08_2: Disconnected");
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "⚠ Connection status changed (No active connections)");
        ESP_LOGI(TAG, "========================================");
    }
}

/**
 * @brief Data received callback
 * @param device_index 0 for HC08_1, 1 for HC08_2
 */
void ble_data_received_callback(uint8_t device_index, uint8_t *data, uint16_t len) {
    // Log receipt
    // ESP_LOGI(TAG, "Received %d bytes from HC08_%d", len, device_index + 1);
    
    // Transparently forward to UART
    // We could add a prefix here if we wanted to distinguish sources on the UART side,
    // e.g., Serial_Printf("[HC08_%d]: ", device_index + 1);
    //Serial_SendArray(data, len);
    Serial_Printf("%s", data);
}

/**
 * @brief Button task for testing transmission
 */
void button_task(void *arg) {
    while (1) {
        bool current_state = (gpio_get_level(BUTTON_GPIO) == 0);
        
        // 检测按键按下(下降沿)
        if (current_state && !last_button_state) {
            vTaskDelay(pdMS_TO_TICKS(50)); // 消抖
            
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                ESP_LOGI(TAG, "Button Pressed!");
                
                if (BLE_IsConnected()) {
                    // 模拟串口接收到数据
                    const char *test_data = "1hello,HC08_1";
                    Serial_RxLen = strlen(test_data);
                    memcpy(Serial_RxPacket, test_data, Serial_RxLen);
                    Serial_RxPacket[Serial_RxLen] = '\0'; // 添加字符串结束符
                    Serial_RxFlag = 1; // 触发主循环处理
                    
                    ESP_LOGI(TAG, "✓ Simulated UART data: %s", Serial_RxPacket);
                } else {
                    ESP_LOGW(TAG, "Cannot send: No devices connected");
                }
                
                // 等待松开,避免重复触发
                vTaskDelay(pdMS_TO_TICKS(300));
            }
        }
        
        last_button_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/**
 * @brief Main application entry
 */
void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "      ESP32 BLE Central -> HC-08      ");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize Serial
    Serial_Init();
    ESP_LOGI(TAG, "✓ Serial Initialized (115200 8N1)");

    // Initialize Button
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);
    
    // Initialize BLE
    ble_config_t ble_config = {
        .device_name = "ESP32_Master", // Not strictly used in Central mode for advertising
        .conn_cb = ble_connection_callback,
        .data_cb = ble_data_received_callback
    };
    
    ESP_LOGI(TAG, "Initializing BLE Central...");
    esp_err_t ret = BLE_Init(&ble_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ BLE Initialized. Scanning for HC08_1 and HC08_2...");
        
        uint8_t mac[6];
        BLE_GetMacAddress(mac);
        ESP_LOGI(TAG, "Local MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
        
    } else {
        ESP_LOGE(TAG, "✗ BLE Init Failed: %s", esp_err_to_name(ret));
        return;
    }
    
    while (1) {
        if (Serial_RxFlag) {
            if (BLE_IsConnected()) {
                // 简单协议: 首字节指定目标
                // '1' -> HC08_1, '2' -> HC08_2, 其他 -> 广播
                if (Serial_RxPacket[0] == '1') {
                    // 发送给 HC08_1 (去掉首字节'1')
                    BLE_SendDataToDevice(0, (uint8_t*)&Serial_RxPacket[1], Serial_RxLen - 1);
                    BLE_SendStringToDevice(0, "\r\n"); // 添加换行
                    ESP_LOGI(TAG, "→ HC08_1: %s", &Serial_RxPacket[1]);
                } else if (Serial_RxPacket[0] == '2') {
                    // 发送给 HC08_2 (去掉首字节'2')
                    BLE_SendDataToDevice(1, (uint8_t*)&Serial_RxPacket[1], Serial_RxLen - 1);
                    BLE_SendStringToDevice(1, "\r\n"); // 添加换行
                    ESP_LOGI(TAG, "→ HC08_2: %s", &Serial_RxPacket[1]);
                } else {
                    // 广播给所有设备
                    BLE_SendData((uint8_t*)Serial_RxPacket, Serial_RxLen);
                    ESP_LOGI(TAG, "→ Broadcast: %s", Serial_RxPacket);
                }
            } else {
                ESP_LOGW(TAG, "UART received but no BLE connection");
            }
            Serial_RxFlag = 0; // 清除标志
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
