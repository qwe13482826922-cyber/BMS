#include "adc_sampling.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

static __IO uint16_t adc_dma_buffer[ADC_CHANNEL_COUNT];

void ADC_Sampling_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    ADC_InitTypeDef adc_init;
    DMA_InitTypeDef dma_init;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    gpio_init.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio_init);

    DMA_DeInit(DMA1_Channel1);
    dma_init.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma_init.DMA_MemoryBaseAddr = (uint32_t)adc_dma_buffer;
    dma_init.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma_init.DMA_BufferSize = ADC_CHANNEL_COUNT;
    dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma_init.DMA_Mode = DMA_Mode_Circular;
    dma_init.DMA_Priority = DMA_Priority_High;
    dma_init.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma_init);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_DeInit(ADC1);
    ADC_StructInit(&adc_init);
    adc_init.ADC_Mode = ADC_Mode_Independent;
    adc_init.ADC_ScanConvMode = ENABLE;
    adc_init.ADC_ContinuousConvMode = ENABLE;
    adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc_init.ADC_DataAlign = ADC_DataAlign_Right;
    adc_init.ADC_NbrOfChannel = ADC_CHANNEL_COUNT;
    ADC_Init(ADC1, &adc_init);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
        ;

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC_Sampling_GetLatest(AdcRawSample *sample)
{
    uint16_t voltage_raw;
    uint16_t current_raw;
    uint16_t temperature_raw;

    __disable_irq();
    voltage_raw = adc_dma_buffer[0];
    current_raw = adc_dma_buffer[1];
    temperature_raw = adc_dma_buffer[2];
    __enable_irq();

    sample->voltage_raw = voltage_raw;
    sample->current_raw = current_raw;
    sample->temperature_raw = temperature_raw;
}
