#ifndef __ADC_SAMPLING_H
#define __ADC_SAMPLING_H

#include "stm32f10x.h"

#define ADC_CHANNEL_COUNT 3

typedef struct
{
    uint16_t voltage_raw;
    uint16_t current_raw;
    uint16_t temperature_raw;
} AdcRawSample;

void ADC_Sampling_Init(void);
void ADC_Sampling_GetLatest(AdcRawSample *sample);

#endif /* __ADC_SAMPLING_H */
