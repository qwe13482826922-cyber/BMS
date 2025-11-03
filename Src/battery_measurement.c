#include "battery_measurement.h"

#include <math.h>
#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#define BATTERY_ADC_CHANNEL_COUNT   (4U)  /* Voltage, current, temperature, Vrefint */
#define ADC_DMA_BUFFER_LENGTH       (BATTERY_ADC_CHANNEL_COUNT * 16U)
#define OVERSAMPLING_FACTOR         (10U)  /* 1 kHz sampling -> 100 Hz output */

#define ADC_FULL_SCALE              (4095.0f)
#define DEFAULT_VREFINT_VOLTAGE     (1.200f)
#define DEFAULT_REFERENCE_TEMP_K    (273.15f + 25.0f)

static uint16_t s_adc_dma_buffer[ADC_DMA_BUFFER_LENGTH];

static struct
{
    BatteryCalibration calibration;
    BatteryMeasurements filtered;
    uint32_t accumulators[BATTERY_ADC_CHANNEL_COUNT];
    uint16_t sample_count;
    float current_zero_voltage;
    float last_current_voltage;
    bool has_new_sample;
} s_ctx;

static void prv_copy_calibration(const BatteryCalibration *calibration);
static void prv_init_gpio(void);
static void prv_init_dma(void);
static void prv_init_adc(void);
static void prv_init_timer(void);
static void prv_start_conversion(void);
static void prv_process_dma_block(uint16_t start_index, uint16_t count);
static void prv_finalize_measurements(void);
static float prv_convert_ntc_temperature(float v_ntc, float vdda);

void BatterySampling_Init(const BatteryCalibration *calibration)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    prv_copy_calibration(calibration);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    prv_init_gpio();

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    prv_init_dma();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    prv_init_adc();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    prv_init_timer();

    s_ctx.current_zero_voltage = 0.0f;
    s_ctx.last_current_voltage = 0.0f;
    s_ctx.filtered.vdda = 3.300f;
    s_ctx.filtered.pack_voltage = 0.0f;
    s_ctx.filtered.pack_current = 0.0f;
    s_ctx.filtered.pack_temperature = 25.0f;
    s_ctx.has_new_sample = false;
}

void BatterySampling_Start(void)
{
    prv_start_conversion();
}

bool BatterySampling_GetMeasurements(BatteryMeasurements *measurements)
{
    bool has_new = false;
    __disable_irq();
    if (s_ctx.has_new_sample)
    {
        s_ctx.has_new_sample = false;
        has_new = true;
        if (measurements != NULL)
        {
            *measurements = s_ctx.filtered;
        }
    }
    __enable_irq();
    return has_new;
}

void BatterySampling_RecalibrateCurrentZero(void)
{
    __disable_irq();
    float vdda = s_ctx.filtered.vdda;
    if (vdda <= 0.0f)
    {
        vdda = 3.300f;
    }
    float current_voltage = s_ctx.last_current_voltage;
    if (current_voltage <= 0.0f)
    {
        current_voltage = vdda * 0.5f;
    }
    s_ctx.current_zero_voltage = current_voltage;
    __enable_irq();
}

static void prv_copy_calibration(const BatteryCalibration *calibration)
{
    BatteryCalibration defaults =
    {
        .voltage_scale = 2.0f,
        .voltage_offset = 0.0f,
        .current_scale = 10.0f,
        .current_offset = 0.0f,
        .ntc_beta = 3435.0f,
        .ntc_r25 = 10000.0f,
        .ntc_series_resistor = 10000.0f,
        .reference_temperature = DEFAULT_REFERENCE_TEMP_K,
        .vrefint_actual = DEFAULT_VREFINT_VOLTAGE,
    };

    if (calibration != NULL)
    {
        defaults = *calibration;
    }

    if (defaults.reference_temperature <= 0.0f)
    {
        defaults.reference_temperature = DEFAULT_REFERENCE_TEMP_K;
    }
    if (defaults.vrefint_actual <= 0.0f)
    {
        defaults.vrefint_actual = DEFAULT_VREFINT_VOLTAGE;
    }

    s_ctx.calibration = defaults;
}

static void prv_init_gpio(void)
{
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_Init(GPIOA, &gpio);
}

static void prv_init_dma(void)
{
    DMA_InitTypeDef dma;
    DMA_DeInit(DMA1_Channel1);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)s_adc_dma_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = ADC_DMA_BUFFER_LENGTH;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_HT | DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    DMA_Cmd(DMA1_Channel1, ENABLE);
}

static void prv_init_adc(void)
{
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_InitTypeDef adc;
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = ENABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv =
#if defined(ADC_ExternalTrigConv_T2_TRGO)
        ADC_ExternalTrigConv_T2_TRGO;
#elif defined(ADC_ExternalTrigConv_T2_CC2)
        ADC_ExternalTrigConv_T2_CC2;
#else
        ADC_ExternalTrigConv_None;
#endif
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = BATTERY_ADC_CHANNEL_COUNT;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 4, ADC_SampleTime_239Cycles5);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
    {
    }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
    {
    }
}

static void prv_init_timer(void)
{
    TIM_TimeBaseInitTypeDef tim;
    TIM_TimeBaseStructInit(&tim);

    /* 72 MHz / (7200 * 10) = 1000 Hz update event */
    tim.TIM_Prescaler = 7200 - 1;
    tim.TIM_Period = 10 - 1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tim);

#if defined(ADC_ExternalTrigConv_T2_TRGO)
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
#elif defined(ADC_ExternalTrigConv_T2_CC2)
    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_Timing;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 1;
    TIM_OC2Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_CC2);
#else
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
#endif
}

static void prv_start_conversion(void)
{
    TIM_Cmd(TIM2, ENABLE);
#if !defined(ADC_ExternalTrigConv_T2_TRGO) && !defined(ADC_ExternalTrigConv_T2_CC2)
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
#endif
}

static void prv_process_dma_block(uint16_t start_index, uint16_t count)
{
    uint32_t sum[BATTERY_ADC_CHANNEL_COUNT] = {0};
    uint16_t samples = 0;

    for (uint16_t i = 0; i < count; i += BATTERY_ADC_CHANNEL_COUNT)
    {
        for (uint16_t ch = 0; ch < BATTERY_ADC_CHANNEL_COUNT; ++ch)
        {
            sum[ch] += s_adc_dma_buffer[start_index + i + ch];
        }
        ++samples;
    }

    for (uint16_t ch = 0; ch < BATTERY_ADC_CHANNEL_COUNT; ++ch)
    {
        s_ctx.accumulators[ch] += sum[ch];
    }
    s_ctx.sample_count += samples;

    if (s_ctx.sample_count >= OVERSAMPLING_FACTOR)
    {
        prv_finalize_measurements();
        memset(s_ctx.accumulators, 0, sizeof(s_ctx.accumulators));
        s_ctx.sample_count = 0;
    }
}

static void prv_finalize_measurements(void)
{
    if (s_ctx.sample_count == 0)
    {
        return;
    }

    float inv_count = 1.0f / (float)s_ctx.sample_count;
    float raw_voltage = (float)s_ctx.accumulators[0] * inv_count;
    float raw_current = (float)s_ctx.accumulators[1] * inv_count;
    float raw_temp = (float)s_ctx.accumulators[2] * inv_count;
    float raw_vref = (float)s_ctx.accumulators[3] * inv_count;

    float vref_voltage = s_ctx.calibration.vrefint_actual;
    float vdda = (raw_vref > 0.0f) ? (vref_voltage * ADC_FULL_SCALE / raw_vref) : 3.300f;

    float adc_to_voltage = vdda / ADC_FULL_SCALE;

    float voltage_adc = raw_voltage * adc_to_voltage;
    float pack_voltage = voltage_adc * s_ctx.calibration.voltage_scale + s_ctx.calibration.voltage_offset;

    float current_voltage = raw_current * adc_to_voltage;
    float zero_ref = s_ctx.current_zero_voltage;
    if (zero_ref <= 0.0f)
    {
        zero_ref = vdda * 0.5f;
    }
    float pack_current = (current_voltage - zero_ref) * s_ctx.calibration.current_scale + s_ctx.calibration.current_offset;
    s_ctx.last_current_voltage = current_voltage;

    float temp_voltage = raw_temp * adc_to_voltage;
    float pack_temperature = prv_convert_ntc_temperature(temp_voltage, vdda);

    /* First-order low-pass filter to mitigate jitter */
    const float alpha = 0.2f;
    s_ctx.filtered.vdda = (1.0f - alpha) * s_ctx.filtered.vdda + alpha * vdda;
    s_ctx.filtered.pack_voltage = (1.0f - alpha) * s_ctx.filtered.pack_voltage + alpha * pack_voltage;
    s_ctx.filtered.pack_current = (1.0f - alpha) * s_ctx.filtered.pack_current + alpha * pack_current;
    s_ctx.filtered.pack_temperature = (1.0f - alpha) * s_ctx.filtered.pack_temperature + alpha * pack_temperature;

    s_ctx.has_new_sample = true;
}

static float prv_convert_ntc_temperature(float v_ntc, float vdda)
{
    const BatteryCalibration *cal = &s_ctx.calibration;
    float r_series = cal->ntc_series_resistor;
    float r_ntc;

    if (v_ntc <= 0.0f || v_ntc >= vdda)
    {
        return 25.0f;
    }

    r_ntc = (v_ntc * r_series) / (vdda - v_ntc);

    float beta = cal->ntc_beta;
    float r0 = cal->ntc_r25;
    float t0 = cal->reference_temperature;

    if (beta <= 0.0f || r0 <= 0.0f)
    {
        return 25.0f;
    }

    float inv_t = (1.0f / t0) + (1.0f / beta) * logf(r_ntc / r0);
    float temp_kelvin = 1.0f / inv_t;
    float temp_celsius = temp_kelvin - 273.15f;
    return temp_celsius;
}

void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_HT1))
    {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        prv_process_dma_block(0, ADC_DMA_BUFFER_LENGTH / 2U);
    }

    if (DMA_GetITStatus(DMA1_IT_TC1))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        prv_process_dma_block(ADC_DMA_BUFFER_LENGTH / 2U, ADC_DMA_BUFFER_LENGTH / 2U);
    }
}
