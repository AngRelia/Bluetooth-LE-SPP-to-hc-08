#ifndef __DELAY_H__
#define __DELAY_H__

#include "esp_system.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif
