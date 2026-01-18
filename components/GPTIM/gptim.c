#include "gptim.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "key.h"

gptimer_handle_t gptim;
uint8_t flag_timer;

bool IRAM_ATTR TimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    Key_Tick();
    return 0;

}
void gptim_init(void)
{
    gptimer_config_t gptimer_cfg = {0};
    gptimer_cfg.clk_src = GPTIMER_CLK_SRC_APB;
    gptimer_cfg.direction =GPTIMER_COUNT_UP;
    gptimer_cfg.flags.intr_shared = 0;
    gptimer_cfg.intr_priority = 0;
    gptimer_cfg.resolution_hz = 1000000;

    gptimer_new_timer(&gptimer_cfg, &gptim);

    gptimer_alarm_config_t gptimer_alarm_cfg = {0};
    gptimer_alarm_cfg.alarm_count = 1000;
    gptimer_alarm_cfg.flags.auto_reload_on_alarm = 1;
    gptimer_alarm_cfg.reload_count = 0;

    gptimer_set_alarm_action(gptim, &gptimer_alarm_cfg);

    gptimer_event_callbacks_t event_cfg = {0};
    event_cfg.on_alarm = TimerCallback;
    gptimer_register_event_callbacks(gptim, &event_cfg, NULL);

    gptimer_enable(gptim);
    gptimer_start(gptim);
}