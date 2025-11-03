#ifndef __SAMPLING_TIMER_H
#define __SAMPLING_TIMER_H

#include "stm32f10x.h"

void SamplingTimer_Init(void);
uint8_t SamplingTimer_IsElapsed(void);

#endif /* __SAMPLING_TIMER_H */
