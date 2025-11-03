#ifndef __BATTERY_MEASUREMENT_H
#define __BATTERY_MEASUREMENT_H

#include "adc_sampling.h"

typedef struct
{
    float voltage_V;
    float current_A;
    float temperature_C;
    float voltage_pin_V;
    float current_pin_V;
    float temperature_pin_V;
} BatteryMeasurements;

void BatteryMeasurement_Init(void);
void BatteryMeasurement_Update(void);
BatteryMeasurements BatteryMeasurement_Get(void);

#endif /* __BATTERY_MEASUREMENT_H */
