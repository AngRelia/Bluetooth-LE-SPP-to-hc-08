#include "led.h"
#include "driver/gpio.h"

void led_init(void)
{
    esp_err_t err;
    gpio_config_t gpio_cfg = {0};
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_cfg.mode = GPIO_MODE_INPUT_OUTPUT;
    gpio_cfg.pin_bit_mask = 1ull<<GPIO_NUM_38;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    err = gpio_config(&gpio_cfg);        //这玩意有返回值的，看定义找对应的变量类型，搞个变量名，用于测试是否gpio配置成功
    if (err != ESP_OK){
        printf("gpio init error\r\n");
    }
}

void gpio_toggle(gpio_num_t gpio_num)
{
    if (gpio_get_level(gpio_num)==1){
        gpio_set_level(gpio_num,0);
    }else if(gpio_get_level(gpio_num)==0){
        gpio_set_level(gpio_num,1);
    }
}