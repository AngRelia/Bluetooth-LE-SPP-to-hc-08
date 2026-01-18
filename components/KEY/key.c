#include "driver/gpio.h"
#include "key.h"

static uint8_t Key_Num = 0;   // 保存最终的按键值

void Key_init(void)
{
    esp_err_t err;
    gpio_config_t gpio_cfg = {0};
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_cfg.mode = GPIO_MODE_INPUT;
    gpio_cfg.pin_bit_mask = (1ull<<GPIO_NUM_4)|(1ull<<GPIO_NUM_5)|(1ull<<GPIO_NUM_6)|(1ull<<GPIO_NUM_7);
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    
    err = gpio_config(&gpio_cfg);        //这玩意有返回值的，看定义找对应的变量类型，搞个变量名，用于测试是否gpio配置成功
    if (err != ESP_OK){
        printf("gpio init error\r\n");
    }
}

static uint8_t Key_GetState(void)
{
    if (gpio_get_level(GPIO_NUM_4) == 0) return 1;
    if (gpio_get_level(GPIO_NUM_5) == 0) return 2;
    if (gpio_get_level(GPIO_NUM_6) == 0) return 3;
	if (gpio_get_level(GPIO_NUM_7) == 0) return 4;
    return 0;
}

uint8_t Key_GetNum(void)
{
    uint8_t temp = Key_Num;
    Key_Num = 0;   // 取走后清零，保证一次只响应一次
    return temp;
}

void Key_Tick(void)
{
    static uint8_t Count = 0;
    static uint8_t CurrState = 0, PrevState = 0;

    Count++;
    if (Count >= 20)   // 20ms消抖
    {
        Count = 0;
        PrevState = CurrState;
        CurrState = Key_GetState();

        // 松手时触发
        if (CurrState != 0 && PrevState == 0)
        {
            Key_Num = CurrState;
        }
    }
}
