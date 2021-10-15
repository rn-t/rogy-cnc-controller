#ifndef SK6812MINIE_H
#define SK6812MINIE_H

#include <stdio.h>
#include "stm32f0xx_hal.h"
#include <stdbool.h>

void  sk6812miniE_init (TIM_HandleTypeDef* _htim, uint32_t channel);
void  sk6812miniE_start (uint8_t* data, int len, bool wait);

#endif /* SK6812MINIE_H */
