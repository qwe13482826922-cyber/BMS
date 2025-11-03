#include "battery_measurement.h"
#include <math.h>

#define ADC_REFERENCE_VOLTAGE      3.300f
#define ADC_RESOLUTION             4095.0f

#define VOLTAGE_DIVIDER_RATIO      2.000f
#define VOLTAGE_OFFSET_V           0.000f

#define CURRENT_SENSOR_REFERENCE_V 1.650f
#define CURRENT_SENSOR_GAIN        10.000f
#define CURRENT_OFFSET_A           0.000f

#define THERMISTOR_BETA            3435.0f
#define THERMISTOR_R0              10000.0f
#define THERMISTOR_T0_KELVIN       298.15f
#define THERMISTOR_PULLUP_OHMS     10000.0f
#define THERMISTOR_SUPPLY_VOLTAGE  3.300f

#define FILTER_ALPHA               0.125f

typedef struct
{
    float gain;
    float offset;
    float filtered_value;
    uint8_t initialized;
} ChannelCalibration;

static BatteryMeasurements g_measurements = {0};
static ChannelCalibration g_voltage_calibration = {VOLTAGE_DIVIDER_RATIO, VOLTAGE_OFFSET_V, 0.0f, 0};
static ChannelCalibration g_current_calibration = {CURRENT_SENSOR_GAIN, CURRENT_OFFSET_A, 0.0f, 0};
static ChannelCalibration g_temperature_calibration = {1.0f, 0.0f, 25.0f, 0};

static float convert_adc_to_pin_voltage(uint16_t raw)
{
    return (float)raw * (ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION);
}

static float apply_filter(ChannelCalibration *channel, float value)
{
    if (!channel->initialized)
    {
        channel->filtered_value = value;
        channel->initialized = 1U;
    }
    else
    {
        channel->filtered_value += FILTER_ALPHA * (value - channel->filtered_value);
    }
    return channel->filtered_value;
}

static float convert_temperature_from_voltage(float voltage)
{
    float temperature_c = g_temperature_calibration.filtered_value;
    if (voltage > 0.01f && voltage < (THERMISTOR_SUPPLY_VOLTAGE - 0.01f))
    {
        float resistance = (voltage * THERMISTOR_PULLUP_OHMS) / (THERMISTOR_SUPPLY_VOLTAGE - voltage);
        float inv_t = (1.0f / THERMISTOR_T0_KELVIN) + (1.0f / THERMISTOR_BETA) * logf(resistance / THERMISTOR_R0);
        float temp_kelvin = 1.0f / inv_t;
        temperature_c = temp_kelvin - 273.15f;
    }
    return temperature_c;
}

void BatteryMeasurement_Init(void)
{
    g_measurements.voltage_V = 0.0f;
    g_measurements.current_A = 0.0f;
    g_measurements.temperature_C = 25.0f;
    g_measurements.voltage_pin_V = 0.0f;
    g_measurements.current_pin_V = CURRENT_SENSOR_REFERENCE_V;
    g_measurements.temperature_pin_V = THERMISTOR_SUPPLY_VOLTAGE / 2.0f;

    g_voltage_calibration.initialized = 0U;
    g_current_calibration.initialized = 0U;
    g_temperature_calibration.initialized = 0U;
    g_temperature_calibration.filtered_value = 25.0f;
}

void BatteryMeasurement_Update(void)
{
    AdcRawSample sample;
    float pin_voltage;
    float pin_current;
    float pin_temperature;
    float battery_voltage;
    float battery_current;
    float battery_temperature;

    ADC_Sampling_GetLatest(&sample);

    pin_voltage = convert_adc_to_pin_voltage(sample.voltage_raw);
    pin_current = convert_adc_to_pin_voltage(sample.current_raw);
    pin_temperature = convert_adc_to_pin_voltage(sample.temperature_raw);

    battery_voltage = (pin_voltage * g_voltage_calibration.gain) + g_voltage_calibration.offset;
    battery_current = ((pin_current - CURRENT_SENSOR_REFERENCE_V) * g_current_calibration.gain) + g_current_calibration.offset;
    battery_temperature = convert_temperature_from_voltage(pin_temperature) * g_temperature_calibration.gain + g_temperature_calibration.offset;

    g_measurements.voltage_pin_V = pin_voltage;
    g_measurements.current_pin_V = pin_current;
    g_measurements.temperature_pin_V = pin_temperature;

    g_measurements.voltage_V = apply_filter(&g_voltage_calibration, battery_voltage);
    g_measurements.current_A = apply_filter(&g_current_calibration, battery_current);
    g_measurements.temperature_C = apply_filter(&g_temperature_calibration, battery_temperature);
}

BatteryMeasurements BatteryMeasurement_Get(void)
{
    return g_measurements;
}
