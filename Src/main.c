#include "stm32f10x.h"

#include "battery_measurement.h"
#include "Delay.h"

static void prv_gpio_config(void);

int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();

    Delay_Init();

    prv_gpio_config();

    BatteryCalibration calibration =
    {
        .voltage_scale = 2.0f,            /* Divider gain from schematic */
        .voltage_offset = 0.0f,
        .current_scale = 10.0f,           /* From hardware documentation */
        .current_offset = 0.0f,
        .ntc_beta = 3435.0f,
        .ntc_r25 = 10000.0f,
        .ntc_series_resistor = 10000.0f,
        .reference_temperature = 273.15f + 25.0f,
        .vrefint_actual = 1.200f,
    };

    BatterySampling_Init(&calibration);
    BatterySampling_Start();

    BatteryMeasurements measurement;

    for (;;)
    {
        if (BatterySampling_GetMeasurements(&measurement))
        {
            /* Placeholder: add CAN transmission here later. */
            /* The measurements structure now contains pack voltage (V), */
            /* current (A), temperature (°C) and VDDA (V). */
        }

        Delay_ms(5);
    }
}

static void prv_gpio_config(void)
{
    /* Additional GPIO configuration can be added here if required. */
}
