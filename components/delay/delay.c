#include "delay.h"
#include "esp32s3/rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#define CPU_FREQ_MHZ 240  // ESP32-S3 默认主频240MHz

/**
 * @brief 毫秒级延时（任务安全）
 */
void Delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)  //xTaskGetSchedulerState() 返回当前 FreeRTOS 调度器状态,判断freertos是否启动
    {
        // FreeRTOS运行中，用vTaskDelay()
        vTaskDelay(ms / portTICK_PERIOD_MS);        //tickrate
    }
    else
    {
        // 裸机模式，用busy-wait
        for (uint32_t i = 0; i < ms; i++)
            ets_delay_us(1000);
    }
}

/**
 * @brief 微秒级延时（精确）
 */
void IRAM_ATTR Delay_us(uint32_t us)
{
    uint32_t start = (uint32_t)esp_cpu_get_cycle_count();
    uint32_t cycles = us * CPU_FREQ_MHZ;  // 微秒转为CPU周期
    while ((uint32_t)esp_cpu_get_cycle_count() - start < cycles)
        ;
}
