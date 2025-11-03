#ifndef BATTERY_MEASUREMENT_H
#define BATTERY_MEASUREMENT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calibration constants used to linearize and scale battery measurements.
 */
typedef struct
{
    float voltage_scale;      /**< Additional software scale factor for pack voltage. */
    float voltage_offset;     /**< Voltage offset (in volts) applied after scaling. */
    float current_scale;      /**< Scale factor for shunt/instrumentation gain. */
    float current_offset;     /**< Offset (in amperes) subtracted from computed current. */
    float ntc_beta;           /**< Beta constant of the NTC thermistor (K). */
    float ntc_r25;            /**< Resistance of the NTC at 25 °C (ohms). */
    float ntc_series_resistor;/**< Series resistor used in the divider (ohms). */
    float reference_temperature; /**< Reference temperature in Kelvin for ntc_r25 (normally 298.15). */
    float vrefint_actual;     /**< Actual voltage of the internal reference (volts). */
} BatteryCalibration;

/**
 * @brief Processed measurement results in engineering units.
 */
typedef struct
{
    float pack_voltage;   /**< Pack voltage in volts. */
    float pack_current;   /**< Pack current in amperes. */
    float pack_temperature; /**< Pack temperature in degrees Celsius. */
    float vdda;           /**< Computed analog supply voltage in volts. */
} BatteryMeasurements;

/**
 * @brief Initializes the ADC, DMA and timer chain for periodic battery sampling.
 *
 * @param calibration Pointer to a calibration structure. The pointer content is copied
 *                    internally, therefore the caller can release the structure after
 *                    initialization.
 */
void BatterySampling_Init(const BatteryCalibration *calibration);

/**
 * @brief Starts the sampling timer and enables DMA transfers.
 */
void BatterySampling_Start(void);

/**
 * @brief Attempts to retrieve the latest measurements.
 *
 * @param[out] measurements Pointer to a structure that receives the results.
 *
 * @return true if new data was copied to @p measurements; false otherwise.
 */
bool BatterySampling_GetMeasurements(BatteryMeasurements *measurements);

/**
 * @brief Forces the software current zero offset based on the most recent raw conversion.
 *        Call this routine while the battery current is guaranteed to be zero to minimize
 *        steady-state errors.
 */
void BatterySampling_RecalibrateCurrentZero(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_MEASUREMENT_H */
