#include "stm32f10x.h"
#include "adc_sampling.h"
#include "battery_measurement.h"
#include "sampling_timer.h"
#include "Delay.h"

static void ProcessMeasurements(const BatteryMeasurements *measurements);

int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();

    Delay_Init();
    ADC_Sampling_Init();
    BatteryMeasurement_Init();
    SamplingTimer_Init();

    while (1)
    {
        if (SamplingTimer_IsElapsed())
        {
            BatteryMeasurement_Update();
            BatteryMeasurements measurements = BatteryMeasurement_Get();
            ProcessMeasurements(&measurements);
        }
    }
}

static void ProcessMeasurements(const BatteryMeasurements *measurements)
{
    (void)measurements;
    /* Placeholder for integration with CAN communication in subsequent steps. */
}
